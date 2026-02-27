#pragma once

#include <chrono>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <cstdio>
#include "servoControl.h"
#include <limits>
#include <array>
// ------------------  GuideState ------------------
struct GuideState
{
    // -1 表示没有引导，0 表示站0 在引导，1 表示站1 在引导（target = 1-guiding）
    std::atomic<int> guiding_station_idx{-1};

    // 期望被引导站（target）的偏航角（度）
    std::atomic<float> desired_yaw_deg{0.0f};

    // stage: 0 idle, 1 waiting_abs_move, 2 speed_tune
    std::atomic<int> stage{0};
    std::chrono::steady_clock::time_point guide_start_time;

    // 每台站是否允许检测（被引导时会被设为 false）
    std::atomic<bool> detection_allowed_for_station[2]{{true}, {true}};

    // 是否已经发送过绝对移动指令，等待到位或超时
    std::atomic<bool> abs_move_in_progress{false};
    std::chrono::steady_clock::time_point abs_move_sent_time;
    std::atomic<int> desired_servo_units{0};
    std::atomic<int> abs_move_fail_count{0};
    std::atomic<float> last_sent_desired_yaw{std::numeric_limits<float>::quiet_NaN()};
    std::chrono::steady_clock::time_point last_abs_send_time;
    std::chrono::steady_clock::time_point last_guide_end_time; // 用于冷却

    // 新增状态（初始化为0 / NAN）
    std::atomic<int> stage1_within_count{0};                        // 连续落在容差内的计数
    std::chrono::steady_clock::time_point stage1_first_within_time; // 第一次落入容差的时间点
    // GuideState 中（已有字段处）

    std::atomic<int> guide_cooldown_ms{3000}; // 冷却时间，单位 ms，可调（例如 1500~3000）
    std::atomic<bool> guidance_disabled_until_reset{false};
};

// 全局实例
extern GuideState g_guide_state;

static float angle_diff_deg(float a, float b)
{
    // 返回 a 与 b 的最小角度差（度），范围 [0, 180]
    float d = fmodf(a - b, 360.0f);
    if (d < -180.0f)
        d += 360.0f;
    if (d > 180.0f)
        d -= 360.0f;
    return fabsf(d);
}

static inline float compute_target_yaw_for_other_station1_deg(float dis_obj_m, float bearing_guider_deg, float cam_baseline_m)
{
    float b0 = 0;
    float tx = 0;
    float ty = 0;
    float rel_x = 0;
    float rel_y = 0;
    float angle1 = 0;
    const float PI_F = 3.14159265358979323846f;
    if (bearing_guider_deg >= 0 && bearing_guider_deg <= 90)
    {
        b0 = bearing_guider_deg * PI_F / 180.0f;
        tx = dis_obj_m * cosf(b0);
        rel_x = tx;
        ty = dis_obj_m * sinf(b0);
        if (ty <= cam_baseline_m)
        {
            rel_y = cam_baseline_m - ty;
            angle1 = -atan2f(rel_y, rel_x);
        }
        if (ty > cam_baseline_m)
        {
            rel_y = ty - cam_baseline_m;
            angle1 = atan2f(rel_y, rel_x);
        }
    }
    else if (bearing_guider_deg > 90 && bearing_guider_deg <= 180)
    {
        b0 = (180 - bearing_guider_deg) * PI_F / 180.0f;
        tx = dis_obj_m * cosf(b0);
        rel_x = tx;
        ty = dis_obj_m * sinf(b0);
        if (ty <= cam_baseline_m)
        {
            rel_y = cam_baseline_m - ty;
            angle1 = atan2f(rel_y, rel_x) - PI_F;
        }
        if (ty > cam_baseline_m)
        {
            rel_y = ty - cam_baseline_m;
            angle1 = PI_F - atan2f(rel_y, rel_x);
        }
    }
    else if (bearing_guider_deg < 0 && bearing_guider_deg >= -90)
    {
        b0 = abs(bearing_guider_deg) * PI_F / 180.0f;
        tx = dis_obj_m * cosf(b0);
        rel_x = tx;
        ty = dis_obj_m * sinf(b0);
        rel_y = ty + cam_baseline_m;
        angle1 = -atan2f(rel_y, rel_x);
    }
    else if (bearing_guider_deg < -90 && bearing_guider_deg >= -180)
    {
        b0 = (180 - abs(bearing_guider_deg)) * PI_F / 180.0f;
        tx = dis_obj_m * cosf(b0);
        rel_x = tx;
        ty = dis_obj_m * sinf(b0);
        rel_y = ty + cam_baseline_m;
        angle1 = atan2f(rel_y, rel_x) - PI_F;
    }
    return angle1 * 180.0f / PI_F;
}

static inline float compute_target_yaw_for_other_station0_deg(float dis_obj_m, float bearing_guider_deg, float cam_baseline_m)
{
    float b0 = 0;
    float tx = 0;
    float ty = 0;
    float rel_x = 0;
    float rel_y = 0;
    float angle1 = 0;
    const float PI_F = 3.14159265358979323846f;
    if (bearing_guider_deg >= 0 && bearing_guider_deg <= 90)
    {
        b0 = bearing_guider_deg * PI_F / 180.0f;
        tx = dis_obj_m * cosf(b0);
        rel_x = tx;
        ty = dis_obj_m * sinf(b0);
        rel_y = cam_baseline_m + ty;
        angle1 = atan2f(rel_y, rel_x);
    }
    else if (bearing_guider_deg > 90 && bearing_guider_deg <= 180)
    {
        b0 = (180 - bearing_guider_deg) * PI_F / 180.0f;
        tx = dis_obj_m * cosf(b0);
        rel_x = tx;
        ty = dis_obj_m * sinf(b0);
        rel_y = cam_baseline_m + ty;
        angle1 = PI_F - atan2f(rel_y, rel_x);
    }
    else if (bearing_guider_deg < 0 && bearing_guider_deg >= -90)
    {
        b0 = abs(bearing_guider_deg) * PI_F / 180.0f;
        tx = dis_obj_m * cosf(b0);
        rel_x = tx;
        ty = dis_obj_m * sinf(b0);

        if (ty < cam_baseline_m)
        {
            rel_y = cam_baseline_m - ty;
            angle1 = atan2f(rel_y, rel_x);
        }
        if (ty > cam_baseline_m)
        {
            rel_y = ty - cam_baseline_m;
            angle1 = -atan2f(rel_y, rel_x);
        }
    }
    else if (bearing_guider_deg < -90 && bearing_guider_deg >= -180)
    {
        b0 = (180 - abs(bearing_guider_deg)) * PI_F / 180.0f;
        tx = dis_obj_m * cosf(b0);
        rel_x = tx;
        ty = dis_obj_m * sinf(b0);
        if (ty < cam_baseline_m)
        {
            rel_y = cam_baseline_m - ty;
            angle1 = PI_F - atan2f(rel_y, rel_x);
        }
        if (ty > cam_baseline_m)
        {
            rel_y = cam_baseline_m - ty;
            angle1 = atan2f(rel_y, rel_x) - PI_F;
        }
    }
    return angle1 * 180.0f / PI_F;
}
// 角度 -> 伺服绝对单位
static inline int yaw_deg_to_servo_units(float yaw_deg)
{
    return (int)roundf(yaw_deg);
}

namespace station_guidance
{

    // Configurable parameters (set via init or use defaults)
    struct Config
    {
        float fine_tune_speed = 10.0f;        // 被引导站 微调速度（速度控制时使用）
        float fine_tune_timeout_sec = 6.0f;   // 微调超时（秒）
        float abs_move_wait_sec = 1.6f;       // 绝对位移等待估计时间（改为舵机回报判定）
        float stable_pixel_threshold = 50.0f; // 像素稳定阈值
        int stable_frames_threshold = 8;      // 稳定帧数阈值
        float cam_baseline_m = 1.0f;          // 两站基线
        float cam_baseline_v_m = 1.2f;        // 垂直基线，默认 1.2
        float cam_diagonal_fov_deg = 65.0f;   // 对角 FOV
        float abs_move_angle_tol_deg = 1.0f;  // 如果舵机与目标角度差 <= 该值则视为到位
    };

    // 初始化（可选）
    void init(const Config &cfg);

    // 在任意站（guider_station_idx）每帧检测并锁定目标时调用，用于内部做稳定性检测并在满足条件后发起对另一台站（target） 的引导。
    //  - img_w/img_h: 当前帧分辨率，用于像素->角度换算
    //  - center: 目标中心像素
    //  - dis_obj: 目标估计距离（米），可为 0 表示未知
    //  - guider_station_idx: 调用方的站号（0 或 1）
    //  - ServoPosH: 调用方当前舵机水平角（度）
    void on_station_locked_frame(int img_w, int img_h, const cv::Point &center, float dis_obj, int guider_station_idx = 0, float ServoPosH = 0.0f);

    // 在被引导站（target_station_idx）的每帧主循环调用：处理等待绝对移动、进入速度微调、超时退出等状态机。
    //  - target_station_idx: 被引导的站号（0 或 1）
    //  - personDetected: 本帧 target_station 是否检测到目标
    //  - x_center/y_center: 如果检测到，目标中心像素坐标
    void update_target_station_state(int target_station_idx, bool personDetected, int x_center, int y_center);

    // 主动退出引导（外部可调用，例如当被引导站自己锁定目标时）
    void cancel_guidance();

    // 查询当前是否处于引导中
    bool is_guiding();

    bool should_run_detection(int station_idx); // 判断某站是否允许检测
    bool is_abs_move_in_progress();
    int get_stage();            // 返回当前 stage（0/1/2）
    double since_abs_sent_ms(); // 返回从 abs_move_sent_time 到现在的毫秒数

    // 立刻暂停被 guider 指向的目标站的检测（不会改变其他引导状态，直至 cancel_guidance 恢复）
    void suspend_detection_for_target_of_guider(int guider_station_idx);
    // 输入距离
    void update_baseline(float hor_m, float ver_m);

} // namespace station_guidance

// ------------------ 外部函数 ------------------
extern void setServoAbsMovForStation(int station_idx, int posH_units, int posV_units);
extern void setServoSpdMovForStationWrapper(int station_idx, int spdH, int spdV);
extern void trackAndMoveServo_spd(int station_idx, int x_center, int y_center);
