#include "worker_thread.h"
#include "common.h"

#include <stdio.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <chrono>

#include "rkYolov5s.hpp"
#include "BYTETracker.h"
#include "STrack.h"
#include "servoControl.h"
#include "stationManager.h"
#include "station_guidance.h"
#include "ip_config.h"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// ================== 滤波工具 ==================
struct PointSmoother
{
    float smooth_x = 0;
    float smooth_y = 0;
    bool is_init = false;
    float base_alpha = 0.4f;

    void reset() { is_init = false; }

    cv::Point update(const cv::Point &raw_point)
    {
        if (!is_init)
        {
            smooth_x = (float)raw_point.x;
            smooth_y = (float)raw_point.y;
            is_init = true;
            return raw_point;
        }

        float dx = raw_point.x - smooth_x;
        float dy = raw_point.y - smooth_y;
        float dist = std::sqrt(dx * dx + dy * dy);
        float dynamic_alpha = base_alpha;

        if (dist < 3.0f)
        {
            return cv::Point((int)std::round(smooth_x), (int)std::round(smooth_y));
        }
        else if (dist < 10.0f)
        {
            dynamic_alpha = 0.05f;
        }
        else if (dist > 50.0f)
        {
            dynamic_alpha = 0.7f;
        }
        else
        {
            dynamic_alpha = 0.05f + (dist - 10.0f) / 40.0f * 0.6f;
        }

        smooth_x = dynamic_alpha * raw_point.x + (1.0f - dynamic_alpha) * smooth_x;
        smooth_y = dynamic_alpha * raw_point.y + (1.0f - dynamic_alpha) * smooth_y;

        return cv::Point((int)std::round(smooth_x), (int)std::round(smooth_y));
    }
};

int create_udp_socket()
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
        return -1;

    // 设置发送缓冲区，防止阻塞
    int sendbuff = 65535;
    setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sendbuff, sizeof(sendbuff));

    // 广播权限
    int broadcast = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0)
    {
        perror("Error enabling broadcast in worker_thread");
    }
    return fd;
}

unsigned char calc_checksum(unsigned char *data, int len)
{
    unsigned char sum = 0;
    for (int i = 0; i < len; i++)
        sum ^= data[i];
    return sum;
}

void worker_thread(int station_idx, const std::string &model_path, const std::string &rtsp_url)
{
    const int MAX_LOST_FRAMES = 30;
    const float LOCK_CONF_THRESHOLD = 0.3f;
    static int lost_counter_arr[2] = {0, 0};
    static cv::Point last_center_arr[2] = {cv::Point(-1, -1), cv::Point(-1, -1)};

    // 初始化线程局部 Socket
    int local_sock_fd = create_udp_socket();
    struct sockaddr_in local_pc_addr;
    memset(&local_pc_addr, 0, sizeof(local_pc_addr));
    local_pc_addr.sin_family = AF_INET;
    local_pc_addr.sin_port = htons(PC_LISTEN_PORT); // 配置端口 (8080)
    local_pc_addr.sin_addr.s_addr = inet_addr(PC_IP_ADDR);

    // 初始化频率控制时间戳
    auto last_servo_cmd_time = std::chrono::steady_clock::now();
    auto last_udp_time = std::chrono::steady_clock::now();

    PointSmoother smoother;

    rkYolov5s detector(model_path.c_str(), station_idx);
    if (detector.init(nullptr, false) != 0)
    {
        fprintf(stderr, "detector.init failed for station %d\n", station_idx);
        return;
    }

    BYTETracker tracker(50, 50);

    std::string pipeline = "rtspsrc location=" + rtsp_url + " latency=0 protocols=tcp ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink sync=false drop=true max-buffers=1";

    // std::string pipeline = "rtspsrc location=" + rtsp_url + " latency=0 protocols=udp ! rtph264depay ! h264parse ! mppvideodec ! videoconvert ! appsink";
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened())
    {
        fprintf(stderr, "Failed to open RTSP %s for station %d\n", rtsp_url.c_str(), station_idx);
        return;
    }

    cv::Mat frame;

    while (!stop_flag.load())
    {
        if (!cap.read(frame))
        {
            fprintf(stderr, "[%d] read failed, reconnecting...\n", station_idx);
            cap.release();
            usleep(500000);
            cap.open(rtsp_url);
            continue;
        }

        //  引导状态检查
        if (!station_guidance::should_run_detection(station_idx))
        {
            station_guidance::update_target_station_state(station_idx, false, 0, 0);
            int guiding_idx = g_guide_state.guiding_station_idx.load();
            bool am_i_target = (guiding_idx != -1) && ((1 - guiding_idx) == station_idx);

            if (am_i_target)
                cv::putText(frame, "Guided - waiting servo...", cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
            else
                cv::putText(frame, "Detection Suspended", cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

            {
                std::lock_guard<std::mutex> lk(g_mtx[station_idx]);
                g_results[station_idx] = frame.clone();
            }
            usleep(20000);
            continue;
        }

        //  识别开关检查
        if (!g_enable_recog.load())
        {
            {
                std::lock_guard<std::mutex> lk(g_mtx[station_idx]);
                g_results[station_idx] = frame.clone();
            }
            usleep(10000);
            continue;
        }

        // YOLO 推理
        cv::Mat infer_out = detector.infer(frame);
        std::vector<rkYolov5s::DetectionResult> dets = detector.getLastDetections();

        cv::Mat draw;
        if (infer_out.empty())
            draw = frame.clone();
        else
            draw = infer_out.clone();

        // 准备 UDP 发送数据
        std::vector<STrack> output_stracks;
        bool found_target_for_pid = false;
        cv::Point pid_target_center(0, 0);

        // =========================================================
        // 开启跟踪 关闭跟踪
        // =========================================================

        // 【情况 A】：开启了跟踪 (ByteTrack + PID)
        if (g_enable_track.load())
        {
            // A1. 数据转换
            std::vector<Object> objects;
            objects.reserve(dets.size());
            for (const auto &d : dets)
            {
                if (d.class_id != 0)
                    continue;
                Object obj;
                obj.label = d.class_id;
                obj.prob = d.confidence;
                obj.rect = cv::Rect(d.x1, d.y1, d.x2 - d.x1, d.y2 - d.y1);
                objects.push_back(obj);
            }

            // A2. 运行跟踪算法
            output_stracks = tracker.update(objects);

            // A3. 绘制结果 & 寻找 PID 目标
            for (size_t i = 0; i < output_stracks.size(); ++i)
            {
                std::vector<float> tlwh = output_stracks[i].tlwh;
                int id = output_stracks[i].track_id;
                int label = output_stracks[i].label;

                // 绘制逻辑
                cv::Rect rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]);

                // 确定颜色
                int locked_id = g_locked_track_ids[station_idx].load();
                cv::Scalar box_col = cv::Scalar(0, 255, 0);
                if (g_station_status[station_idx].locked.load() && id == locked_id && locked_id != -1)
                {
                    box_col = cv::Scalar(0, 0, 255); // 红色表示锁定
                }

                cv::rectangle(draw, rect, box_col, 2);
                cv::putText(draw, "ID:" + std::to_string(id), cv::Point(rect.x, rect.y - 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, box_col, 1);

                // A4. 锁定逻辑检查
                if (g_station_status[station_idx].locked.load() && id == locked_id)
                {
                    // 找到目标，应用平滑滤波
                    cv::Point cur_center(rect.x + rect.width / 2, rect.y + rect.height / 2);

                    // 如果刚找回来，重置滤波器
                    if (lost_counter_arr[station_idx] > 0)
                        smoother.reset();

                    pid_target_center = smoother.update(cur_center);
                    found_target_for_pid = true;
                    lost_counter_arr[station_idx] = 0; // 重置丢失计数
                    last_center_arr[station_idx] = pid_target_center;

                    // 加粗框
                    cv::rectangle(draw, rect, cv::Scalar(0, 255, 255), 3);

                    // 更新一次性引导状态
                    if (!g_guidance_done.load())
                    {
                        float dis_obj = 0.0f;
                        float curH = 0.0f, curV = 0.0f; // 占位
                        if (rect.width > 0 && rect.height > 0)
                            dis_obj = 1.0f / sqrtf((float)(rect.width * rect.height));
                        station_guidance::on_station_locked_frame(draw.cols, draw.rows, cur_center, dis_obj, station_idx, curH);
                    }
                }
            } // end for output_stracks

            // 检查丢失 (如果没有找到锁定目标)
            if (!found_target_for_pid && g_station_status[station_idx].locked.load())
            {
                lost_counter_arr[station_idx]++;
                if (lost_counter_arr[station_idx] > MAX_LOST_FRAMES)
                {
                    // 丢失超时，解锁
                    g_station_status[station_idx].locked.store(false);
                    g_locked_track_ids[station_idx].store(-1);
                    lost_counter_arr[station_idx] = 0;
                    if (!g_guidance_done.load())
                        station_guidance::cancel_guidance();

                    // 丢失后停止舵机
                    setServoSpdMovForStationWrapper(station_idx, 0, 0);
                    printf("Station %d lost locked track -> unlocking\n", station_idx);
                }
            }

            // 自动锁定逻辑 (如果没有锁定，且发现了目标)
            if (!g_station_status[station_idx].locked.load() && !output_stracks.empty())
            {
                // 锁定第一个置信度足够的目标
                for (auto &t : output_stracks)
                {
                    if (t.label == 0 && t.score > LOCK_CONF_THRESHOLD)
                    {
                        g_station_status[station_idx].locked.store(true);
                        g_locked_track_ids[station_idx].store(t.track_id);
                        smoother.reset();
                        // 首次锁定先停一下
                        setServoSpdMovForStationWrapper(station_idx, 0, 0);
                        break;
                    }
                }
            }

            // A5. 执行 PID 舵机控制
            if (!station_guidance::is_guiding() && found_target_for_pid)
            {
                auto now = std::chrono::steady_clock::now();
                double elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_servo_cmd_time).count();

                // 每 60ms 发一次
                if (elapsed_ms >= 20)
                {
                    if (station_idx == 0)
                    {
                        trackAndMoveServo_spd(station_idx, pid_target_center.x, pid_target_center.y);
                    }
                    else if (station_idx == 1)
                    {
                        trackAndMoveServo_pos(station_idx, pid_target_center.x, pid_target_center.y);
                    }
                    last_servo_cmd_time = now;
                }
            }
        }

        // 【情况 B】：关闭了跟踪
        else
        {
            // 只遍历原始 YOLO 结果画绿框
            for (const auto &d : dets)
            {
                if (d.class_id != 0)
                    continue;
                cv::rectangle(draw, cv::Point(d.x1, d.y1), cv::Point(d.x2, d.y2), cv::Scalar(0, 255, 0), 2);
                char text[32];
                sprintf(text, "Prob: %.2f", d.confidence);
                cv::putText(draw, text, cv::Point(d.x1, d.y1 - 5), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);
            }
            // 确保不发任何舵机指令
        }

        // =========================================================
        // UDP 数据发送
        // =========================================================
        auto now = std::chrono::steady_clock::now();
        double time_since_last_udp = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_udp_time).count();

        // 限制 UDP 频率：每 33ms 发一次 (约 30fps)
        if (time_since_last_udp >= 33.0 && local_sock_fd >= 0)
        {
            // 如果开启了跟踪，才发 track 结果
            if (g_enable_track.load() && !output_stracks.empty())
            {
                unsigned char sendBuf[1024];
                int idx = 0;
                sendBuf[idx++] = 0x10;
                sendBuf[idx++] = 0x16;
                sendBuf[idx++] = 0xef;
                sendBuf[idx++] = 0x08; // AI 数据
                sendBuf[idx++] = (unsigned char)station_idx;
                int count = std::min((int)output_stracks.size(), 5);
                sendBuf[idx++] = (unsigned char)count;

                for (int i = 0; i < count; i++)
                {
                    int id = output_stracks[i].track_id;
                    int cls = output_stracks[i].label;
                    sendBuf[idx++] = id & 0xFF;
                    sendBuf[idx++] = (id >> 8) & 0xFF;
                    sendBuf[idx++] = (unsigned char)cls;
                }
                sendBuf[idx] = calc_checksum(sendBuf, idx);
                idx++;
                sendto(local_sock_fd, sendBuf, idx, 0, (struct sockaddr *)&local_pc_addr, sizeof(local_pc_addr));

                // 发送锁定 ID 信息 (协议 0x16)
                int current_locked_id = g_locked_track_ids[station_idx].load();
                if (current_locked_id != -1)
                {
                    float current_conf = 0.0f;
                    for (const auto &t : output_stracks)
                    {
                        if (t.track_id == current_locked_id)
                        {
                            current_conf = t.score;
                            break;
                        }
                    }
                    if (current_conf > 0)
                    { // found
                        unsigned char buf2[32];
                        int idx2 = 0;
                        buf2[idx2++] = 0x10;
                        buf2[idx2++] = 0x16;
                        buf2[idx2++] = 0xef;
                        buf2[idx2++] = 0x16; // 锁定状态
                        buf2[idx2++] = (unsigned char)station_idx;
                        buf2[idx2++] = current_locked_id & 0xFF;
                        buf2[idx2++] = (current_locked_id >> 8) & 0xFF;
                        buf2[idx2++] = (unsigned char)(current_conf * 100);
                        unsigned char sum = 0;
                        for (int k = 0; k < idx2; k++)
                            sum ^= buf2[k];
                        buf2[idx2++] = sum;
                        sendto(local_sock_fd, buf2, idx2, 0, (struct sockaddr *)&local_pc_addr, sizeof(local_pc_addr));
                    }
                }
            }
            last_udp_time = now;
        }

        //  更新全局画面
        {
            std::lock_guard<std::mutex> lk(g_mtx[station_idx]);
            g_results[station_idx] = draw.clone();
        }

        //  循环切换
        if (g_cycle_flags.size() > (size_t)station_idx && g_cycle_flags[station_idx]->load())
        {
            g_cycle_flags[station_idx]->store(false);
            if (g_enable_track.load())
            { // 只有开启跟踪才能切换 ID
                std::vector<int> visible_ids;
                for (const auto &t : output_stracks)
                    if (t.label == 0)
                        visible_ids.push_back(t.track_id);

                if (!visible_ids.empty())
                {
                    int prev = g_locked_track_ids[station_idx].load();
                    int next = visible_ids[0];
                    auto it = std::find(visible_ids.begin(), visible_ids.end(), prev);
                    if (it != visible_ids.end())
                    {
                        size_t pos = std::distance(visible_ids.begin(), it);
                        next = visible_ids[(pos + 1) % visible_ids.size()];
                    }
                    g_locked_track_ids[station_idx].store(next);
                    g_station_status[station_idx].locked.store(true);
                    smoother.reset();
                    printf("[Station %d] Cycled to ID %d\n", station_idx, next);
                }
            }
        }

        // 主动休眠
        usleep(1000);

    } // end while

    if (local_sock_fd >= 0)
        close(local_sock_fd);
    cap.release();
}