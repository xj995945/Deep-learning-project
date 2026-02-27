#include <stdio.h>
#include <mutex>
#include "rknn_api.h"

#include "postprocess.h"
#include "preprocess.h"
#include "common.h"
#include "servoControl.h"
#include "stationManager.h"
#include "station_guidance.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "coreNum.hpp"
#include "rkYolov5s.hpp"

#include <unistd.h>
#include <sys/socket.h>
#include <pthread.h>

#include <cstdio>
#include <atomic>
#include <thread>

int x_center;             // 检测目标x中心
int y_center;             // 检测目标y中心
float dis_obj;            // 摄像头到目标的距离
float Kd = 1156.84;       // 计算距离的比例
float WINDOW_X = 1280.0f; // 瞄准镜像素为1280 * 720
float WINDOW_Y = 720.0f;
int ServoSpdH = 20;            // 伺服水平转速
int ServoSpdV = 10;            // 伺服垂直转速
float Pixel_Difference_x = 30; // 摄像头和瞄准镜像素差值
float Pixel_Difference_y = 10; // 摄像头和瞄准镜像素差值

extern std::atomic<bool> g_guidance_done;

static void dump_tensor_attr(rknn_tensor_attr *attr)
{
    std::string shape_str = attr->n_dims < 1 ? "" : std::to_string(attr->dims[0]);
    for (int i = 1; i < attr->n_dims; ++i)
    {
        shape_str += ", " + std::to_string(attr->dims[i]);
    }
}

static unsigned char *load_data(FILE *fp, size_t ofst, size_t sz)
{
    unsigned char *data;
    int ret;

    data = NULL;

    if (NULL == fp)
    {
        return NULL;
    }

    ret = fseek(fp, ofst, SEEK_SET);
    if (ret != 0)
    {
        printf("blob seek failure.\n");
        return NULL;
    }

    data = (unsigned char *)malloc(sz);
    if (data == NULL)
    {
        printf("buffer malloc failure.\n");
        return NULL;
    }
    ret = fread(data, 1, sz, fp);
    return data;
}

static unsigned char *load_model(const char *filename, int *model_size)
{
    FILE *fp;
    unsigned char *data;

    fp = fopen(filename, "rb");
    if (NULL == fp)
    {
        printf("Open file %s failed.\n", filename);
        return NULL;
    }

    fseek(fp, 0, SEEK_END);
    int size = ftell(fp);

    data = load_data(fp, 0, size);

    fclose(fp);

    *model_size = size;
    return data;
}

static int saveFloat(const char *file_name, float *output, int element_size)
{
    FILE *fp;
    fp = fopen(file_name, "w");
    for (int i = 0; i < element_size; i++)
    {
        fprintf(fp, "%.6f\n", output[i]);
    }
    fclose(fp);
    return 0;
}

rkYolov5s::rkYolov5s(const std::string &model_path, int station_idx_)
    : model_path(model_path), station_idx(station_idx_)
{
    // this->model_path = model_path;
    nms_threshold = NMS_THRESH;      // 默认的NMS阈值
    box_conf_threshold = BOX_THRESH; // 默认的置信度阈值
}

int rkYolov5s::init(rknn_context *ctx_in, bool share_weight)
{
    printf("Loading model...\n");
    int model_data_size = 0;
    model_data = load_model(model_path.c_str(), &model_data_size);
    // 模型参数复用/Model parameter reuse
    if (share_weight == true)
        ret = rknn_dup_context(ctx_in, &ctx);
    else
        ret = rknn_init(&ctx, model_data, model_data_size, 0, NULL);
    if (ret < 0)
    {
        printf("rknn_init error ret=%d\n", ret);
        return -1;
    }

    // 设置模型绑定的核心/Set the core of the model that needs to be bound
    rknn_core_mask core_mask;
    switch (get_core_num())
    {
    case 0:
        core_mask = RKNN_NPU_CORE_0;
        break;
    case 1:
        core_mask = RKNN_NPU_CORE_1;
        break;
    case 2:
        core_mask = RKNN_NPU_CORE_2;
        break;
    }
    ret = rknn_set_core_mask(ctx, core_mask);
    if (ret < 0)
    {
        printf("rknn_init core error ret=%d\n", ret);
        return -1;
    }

    rknn_sdk_version version;
    ret = rknn_query(ctx, RKNN_QUERY_SDK_VERSION, &version, sizeof(rknn_sdk_version));
    if (ret < 0)
    {
        printf("rknn_init error ret=%d\n", ret);
        return -1;
    }
    printf("sdk version: %s driver version: %s\n", version.api_version, version.drv_version);

    // 获取模型输入输出参数/Obtain the input and output parameters of the model
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret < 0)
    {
        printf("rknn_init error ret=%d\n", ret);
        return -1;
    }
    printf("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

    // 设置输入参数/Set the input parameters
    input_attrs = (rknn_tensor_attr *)calloc(io_num.n_input, sizeof(rknn_tensor_attr));
    for (int i = 0; i < io_num.n_input; i++)
    {
        input_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret < 0)
        {
            printf("rknn_init error ret=%d\n", ret);
            return -1;
        }
        dump_tensor_attr(&(input_attrs[i]));
    }

    // 设置输出参数/Set the output parameters
    output_attrs = (rknn_tensor_attr *)calloc(io_num.n_output, sizeof(rknn_tensor_attr));
    for (int i = 0; i < io_num.n_output; i++)
    {
        output_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
        dump_tensor_attr(&(output_attrs[i]));
    }

    if (input_attrs[0].fmt == RKNN_TENSOR_NCHW)
    {
        printf("model is NCHW input fmt\n");
        channel = input_attrs[0].dims[1];
        height = input_attrs[0].dims[2];
        width = input_attrs[0].dims[3];
    }
    else
    {
        printf("model is NHWC input fmt\n");
        height = input_attrs[0].dims[1];
        width = input_attrs[0].dims[2];
        channel = input_attrs[0].dims[3];
    }
    printf("model input height=%d, width=%d, channel=%d\n", height, width, channel);

    memset(inputs, 0, sizeof(inputs));
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].size = width * height * channel;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].pass_through = 0;

    return 0;
}

rknn_context *rkYolov5s::get_pctx()
{
    return &ctx;
}

std::vector<rkYolov5s::DetectionResult> rkYolov5s::getLastDetections()
{
    std::lock_guard<std::mutex> lock(mtx);
    return last_detections;
}

static inline int clampi(int v, int lo, int hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

// 把角度规范到 (-180, 180]
inline float wrapAngle180(float a)
{
    // 更稳健的实现：用 fmod
    float res = fmodf(a, 360.0f);
    if (res <= -180.0f)
        res += 360.0f;
    else if (res > 180.0f)
        res -= 360.0f;
    return res;
}

cv::Mat rkYolov5s::infer(cv::Mat &orig_img)
{
    // --- 在帧开始：如果当前站是被引导目标且处于 abs move 状态，触发一次状态更新（非阻塞） ---
    // NOTE: 只有当全局一次性引导尚未完成时才执行这些与引导状态机有关的更新
    if (!g_guidance_done.load() && station_guidance::is_abs_move_in_progress())
    {
        double since_ms = station_guidance::since_abs_sent_ms();
        const double min_wait_ms = 50.0;
        if (since_ms >= min_wait_ms)
        {
            // 如果本站是被引导目标则通知状态机（personDetected=false）
            station_guidance::update_target_station_state(station_idx, false, 0, 0);
        }
    }

    // 检查本帧是否允许检测
    // 如果全局一次性引导已完成，则强制允许检测（跳过引导暂停）
    if (!g_guidance_done.load() && !station_guidance::should_run_detection(station_idx))
    {
        // 调试用打印（改为访问全局 g_guide_state）
        printf("[DBG] should_run_detection==false st=%d, detection_allowed=%d, abs_move_in_progress=%d, guiding=%d, since_abs_ms=%.0f\n",
               station_idx,
               ::g_guide_state.detection_allowed_for_station[station_idx].load(),
               ::g_guide_state.abs_move_in_progress.load(),
               ::g_guide_state.guiding_station_idx.load(),
               station_guidance::since_abs_sent_ms());

        cv::putText(orig_img, "Guided - waiting servo to reach target...",
                    cv::Point(50, 250),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 255, 0), 2);
        return orig_img; // 暂停检测
    }

    std::lock_guard<std::mutex> lock(mtx);
    cv::Mat img;
    cv::cvtColor(orig_img, img, cv::COLOR_BGR2RGB);
    img_width = img.cols;
    img_height = img.rows;

    BOX_RECT pads;
    memset(&pads, 0, sizeof(BOX_RECT));
    cv::Size target_size(width, height);
    cv::Mat resized_img(target_size.height, target_size.width, CV_8UC3);
    // 计算缩放比例/Calculate the scaling ratio
    float scale_w = (float)target_size.width / img.cols;
    float scale_h = (float)target_size.height / img.rows;

    // 图像缩放/Image scaling
    if (img_width != width || img_height != height)
    {
        // rga
        // rga_buffer_t src;
        // rga_buffer_t dst;
        // memset(&src, 0, sizeof(src));
        // memset(&dst, 0, sizeof(dst));
        // ret = resize_rga(src, dst, img, resized_img, target_size);
        // if (ret != 0)
        // {
        //     fprintf(stderr, "resize with rga error\n");
        // }
        // inputs[0].buf = resized_img.data;

        cv::resize(img, resized_img, cv::Size(width, height));
        inputs[0].buf = resized_img.data;
    }
    else
    {
        inputs[0].buf = img.data;
    }

    rknn_inputs_set(ctx, io_num.n_input, inputs);

    rknn_output outputs[io_num.n_output];
    memset(outputs, 0, sizeof(outputs));
    for (int i = 0; i < io_num.n_output; i++)
    {
        outputs[i].want_float = 0;
    }

    // 模型推理/Model inference
    ret = rknn_run(ctx, NULL);
    ret = rknn_outputs_get(ctx, io_num.n_output, outputs, NULL);

    // 获取最新的阈值
    float cur_conf = g_conf_threshold.load();
    float cur_nms = g_nms_threshold.load();

    // 后处理/Post-processing
    detect_result_group_t detect_result_group;
    memset(&detect_result_group, 0, sizeof(detect_result_group_t));
    std::vector<float> out_scales;
    std::vector<int32_t> out_zps;
    for (int i = 0; i < io_num.n_output; ++i)
    {
        out_scales.push_back(output_attrs[i].scale);
        out_zps.push_back(output_attrs[i].zp);
    }
    post_process((int8_t *)outputs[0].buf, (int8_t *)outputs[1].buf, (int8_t *)outputs[2].buf, height, width,
                 cur_conf, cur_nms, pads, scale_w, scale_h, out_zps, out_scales, &detect_result_group);

    // 绘制框体/Draw the box
    char text[256];
    for (int i = 0; i < detect_result_group.count; i++)
    {
        detect_result_t *det_result = &(detect_result_group.results[i]);
        sprintf(text, "%s %.1f%%", det_result->name, det_result->prop * 100);
        int x1 = det_result->box.left;
        int y1 = det_result->box.top;
        int x2 = det_result->box.right;
        int y2 = det_result->box.bottom;
        // 绘制检测框
        // rectangle(orig_img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0,255,0), 2);
    }

    last_detections.clear();
    for (int i = 0; i < detect_result_group.count; i++)
    {
        detect_result_t *det_result = &(detect_result_group.results[i]);
        DetectionResult res;

        res.class_id = det_result->class_id;
        res.name = det_result->name;
        res.confidence = det_result->prop;
        res.x1 = det_result->box.left;
        res.y1 = det_result->box.top;
        res.x2 = det_result->box.right;
        res.y2 = det_result->box.bottom;
        last_detections.push_back(res);
    }

    ret = rknn_outputs_release(ctx, io_num.n_output, outputs);
    if (ret != 0)
    {
        fprintf(stderr, "rknn_outputs_release failed: %d\n", ret);
    }

    return orig_img;
}

rkYolov5s::~rkYolov5s()
{
    deinitPostProcess();

    ret = rknn_destroy(ctx);

    if (model_data)
        free(model_data);

    if (input_attrs)
        free(input_attrs);
    if (output_attrs)
        free(output_attrs);
}

// ================= PID 参数配置 =================
// 水平方向 (Pan) 参数
static float KP_H = 35.0f;  // 比例: 响应速度
static float KI_H = 0.000f; // 积分: 消除稳态误差
static float KD_H = 12.0f;  // 微分: 抑制震荡

// 垂直方向 (Tilt) 参数
static float KP_V = 5.5f;
static float KI_V = 0.000f;
static float KD_V = 1.1f;

// 积分限幅 (度)，防止积分过大
static float INTEGRAL_LIMIT = 15.0f;

// ================= 状态记录 =================
// 为每个站点保留 PID 历史状态
static float last_error_x[4] = {0};
static float integral_x[4] = {0};
static float last_error_y[4] = {0};
static float integral_y[4] = {0};

// ================= 参数配置 =================
static float K_FF_H = 0.1f;    // 前馈增益 (0.01)
static float VEL_ALPHA = 0.2f; // 速度滤波系数 (越小越平滑)

// 静态变量
static float last_x_raw[4] = {-1};
static float smooth_vx[4] = {0};
static std::chrono::steady_clock::time_point last_time_pt[4] = {
    std::chrono::steady_clock::now(),
    std::chrono::steady_clock::now(),
    std::chrono::steady_clock::now(),
    std::chrono::steady_clock::now()};

void trackAndMoveServo_pos(int station_idx, float x_center, float y_center)
{
    if (station_idx < 0 || station_idx >= 4)
        return;
    // 密位
    float UNITS_PER_DEGREE = 50.0f / 3.0f;

    //============切换到1920*1080追踪=================
    //  坐标换算
    float x_logic = x_center;
    float y_logic = y_center;

    //  使用 station_idx 索引获取时间
    auto now = std::chrono::steady_clock::now();
    // 第一次运行时防止 dt 过大，可加个判断
    float dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time_pt[station_idx]).count() / 1000.0f;

    // 限制最小 dt 防止除零或过大跳变
    if (dt < 0.001f)
        dt = 0.033f;

    last_time_pt[station_idx] = now; // 只更新本站的时间

    //  计算速度 (像素/秒)
    float raw_vx = 0.0f;
    if (last_x_raw[station_idx] >= 0 && dt > 0.001f && dt < 0.5f)
    {
        // 算出原始速度
        raw_vx = (x_center - last_x_raw[station_idx]); // 没除以dt，假设帧率稳定，简化计算
    }
    last_x_raw[station_idx] = x_center;

    //  速度滤波
    smooth_vx[station_idx] = VEL_ALPHA * raw_vx + (1.0f - VEL_ALPHA) * smooth_vx[station_idx];

    //  计算前馈量
    float feedforward_h = smooth_vx[station_idx] * K_FF_H;

    //  计算误差
    float error_x = (x_logic - WINDOW_X / 2) / (WINDOW_X / 2);
    // 垂直向下为正 (若反向，改为 -y_logic + ...)
    float error_y = (y_logic - WINDOW_Y / 2) / (WINDOW_Y / 2);
    if (error_x > 0)
    {
        feedforward_h = std::abs(feedforward_h);
    }
    else
    {
        feedforward_h = -std::abs(feedforward_h);
    }

    //============切换到1920*1080追踪=================

    //  死区检测
    bool in_dead_zone_x = std::abs(error_x) <= Pixel_Difference_x / (WINDOW_X / 2);
    bool in_dead_zone_y = std::abs(error_y) <= Pixel_Difference_y / (WINDOW_Y / 2);

    if (in_dead_zone_x && in_dead_zone_y)
    {
        integral_x[station_idx] = 0;
        last_error_x[station_idx] = 0;
        integral_y[station_idx] = 0;
        last_error_y[station_idx] = 0;
        return;
    }
    // float KP_H = g_track_kp.load();
    // float KD_H = g_track_kd.load();
    //  PID 计算 (水平 + 垂直)
    float delta_angle_h = 0.0f;
    float delta_angle_v = 0.0f;

    // --- 水平 PID ---
    if (!in_dead_zone_x)
    {
        integral_x[station_idx] += error_x;
        float derivative_x = error_x - last_error_x[station_idx];
        last_error_x[station_idx] = error_x;

        delta_angle_h = (KP_H * error_x) + (KI_H * integral_x[station_idx]) + (KD_H * derivative_x) + feedforward_h;
    }
    else
    {
        if (std::abs(feedforward_h) > 0.3f)
        {
            delta_angle_h = feedforward_h;
        }
        else
        {
            delta_angle_h = 0;
        }
    }

    // --- 垂直 PID ---
    if (!in_dead_zone_y)
    {
        integral_y[station_idx] += error_y;

        if (integral_y[station_idx] > INTEGRAL_LIMIT)
            integral_y[station_idx] = INTEGRAL_LIMIT;
        if (integral_y[station_idx] < -INTEGRAL_LIMIT)
            integral_y[station_idx] = -INTEGRAL_LIMIT;

        float derivative_y = error_y - last_error_y[station_idx];
        last_error_y[station_idx] = error_y;

        delta_angle_v = (KP_V * error_y) + (KI_V * integral_y[station_idx]) + (KD_V * derivative_y);
    }
    else
    {
        integral_y[station_idx] = 0;
        last_error_y[station_idx] = 0;
    }

    //  获取当前角度
    float curH = 0.0f, curV = 0.0f;
    if (!getStationServoPos(station_idx, curH, curV))
        return;

    //  叠加得到目标角度
    float targetH = curH + delta_angle_h;
    float targetV = curV + delta_angle_v;

    // ==========================================
    //  范围与归一化处理
    // ==========================================

    // --- 水平方向 (Pan): 循环归一化 ---
    while (targetH > 180.0f)
        targetH -= 360.0f;
    while (targetH < -180.0f)
        targetH += 360.0f;

    const float MAX_V_LIMIT = 30.0f;
    const float MIN_V_LIMIT = -10.0f;

    if (targetV > MAX_V_LIMIT)
        targetV = MAX_V_LIMIT;
    if (targetV < MIN_V_LIMIT)
        targetV = MIN_V_LIMIT;

    //  发送指令
    int send_val_h = (int)(targetH * UNITS_PER_DEGREE);
    int send_val_v = (int)(targetV * UNITS_PER_DEGREE);

    setServoAbsMovForStation(station_idx, send_val_h, send_val_v);

    // printf("center(%.2f, %.2f) error(%.2f, %.2f) target_angle(%.2f, %.2f) cur_angle(%.2f, %.2f) send_angle(%d, %d)\n",
    //        x_center, y_center, error_x, error_y, targetH, targetV, curH, curV, send_val_h, send_val_v);

    // // =========================================================
    // // 1PID 数据记录 (CSV)
    // // =========================================================
    // // 使用静态数组，为每个 station 保持一个文件句柄
    // static std::ofstream pid_logs[4];
    // static bool headers_written[4] = {false, false, false, false};

    // // 如果文件没打开，则打开
    // if (!pid_logs[station_idx].is_open())
    // {
    //     std::string fname = "pid_log_station_" + std::to_string(station_idx) + ".csv";
    //     // 使用 append 模式(std::ios::app) 或 覆盖模式(std::ios::out)
    //     pid_logs[station_idx].open(fname, std::ios::out);
    // }

    // // 写入表头
    // if (pid_logs[station_idx].is_open() && !headers_written[station_idx])
    // {
    //     pid_logs[station_idx] << "Time_ms,X_Center,Y_Center,Error_X,Error_Y,Cur_H,Cur_V,Target_H,Target_V,Send_Val_H,Send_Val_V" << std::endl;
    //     headers_written[station_idx] = true;
    // }

    // // 写入数据
    // if (pid_logs[station_idx].is_open())
    // {
    //     auto now = std::chrono::steady_clock::now();
    //     auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    //     pid_logs[station_idx] << ms << ","
    //                           << x_center << "," << y_center << ","
    //                           << error_x << "," << error_y << ","
    //                           << curH << "," << curV << ","
    //                           << targetH << "," << targetV << ","
    //                           << send_val_h << "," << send_val_v
    //                           << std::endl;
    // }
    // // =========================================================
}

void trackAndMoveServo_spd(int station_idx, int x_center, int y_center)
{
    // ==========================================
    //  定义目标中心点 (1280 x 720 分辨率)
    // ==========================================
    // 水平中心：1280 / 2 = 640
    int target_x = WINDOW_X / 2;

    // 垂直中心：720 / 2 = 360
    int target_y = WINDOW_Y / 2;

    // ==========================================
    //  计算误差 (Error)
    // ==========================================
    // 误差 > 0 代表目标在右下方，云台需要向右/下转动
    int error_x = x_center - target_x;
    int error_y = y_center - target_y;

    // ==========================================
    //  PD 参数设置
    // ==========================================
    // 防止数组越界
    if (station_idx < 0 || station_idx >= 4)
        return;

    // 静态变量：记录上一次的误差，用于计算 D (微分项)
    static int last_error_x[4] = {0};
    static int last_error_y[4] = {0};

    // --- P 参数 (比例系数) ---
    // 当误差达到半个屏幕宽(640)时，速度达到满速 ServoSpdH
    float Kp_H = (float)ServoSpdH / 640.0f;
    float Kp_V = (float)ServoSpdV / 360.0f;

    // --- D 参数 (微分系数) ---
    // 当误差快速减小时，产生反向力，防止云台冲过头
    // 一般设为 Kp 的 0.3 到 0.5 倍。如果抖动就改小，如果刹不住车就改大。
    float Kd_H = Kp_H * 0.4f;
    float Kd_V = Kp_V * 0.4f;

    // ==========================================
    //  死区处理 (Dead Zone)
    // ==========================================
    // 如果已经在中心附近了，就别动了，防止舵机滋滋响
    if (std::abs(error_x) < Pixel_Difference_x)
    {
        error_x = 0;
        last_error_x[station_idx] = 0; // 进死区时清空历史，防止下次启动突跳
    }
    if (std::abs(error_y) < Pixel_Difference_y)
    {
        error_y = 0;
        last_error_y[station_idx] = 0;
    }

    // ==========================================
    //  计算最终速度 (PD公式)
    // ==========================================

    // 水平速度 = P项 + D项
    // D项 = (当前误差 - 上次误差)，即误差的变化率
    int spd_x = (int)(error_x * Kp_H + (error_x - last_error_x[station_idx]) * Kd_H);

    // 垂直速度
    int spd_y = (int)(error_y * Kp_V + (error_y - last_error_y[station_idx]) * Kd_V);

    // 更新历史误差，供下一帧使用
    last_error_x[station_idx] = error_x;
    last_error_y[station_idx] = error_y;

    // ==========================================
    //  限幅与输出
    // ==========================================

    // 限制最大速度
    spd_x = clampi(spd_x, -ServoSpdH, ServoSpdH);
    spd_y = clampi(spd_y, -ServoSpdV, ServoSpdV);

    // 强制归零
    if (error_x == 0)
        spd_x = 0;
    if (error_y == 0)
        spd_y = 0;

    // 发送指令
    // 垂直方向通常需要取反 (-spd_y)，因为屏幕坐标Y向下为正，而云台向上为正
    // 如果发现上下反了，把这个负号去掉即可
    setServoSpdMovForStation(station_idx, spd_x, -spd_y);

    // 调试打印
    // static int print_count = 0;
    // if (print_count++ % 15 == 0) {
    //     printf("[PD] St:%d Err:(%d,%d) Spd:(%d,%d)\n", station_idx, error_x, error_y, spd_x, spd_y);
    // }
}
