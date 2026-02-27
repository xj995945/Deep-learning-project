#include "station_guidance.h"
#include "stationManager.h"
#include "common.h"
#include <cstdio>
#include <cmath>

GuideState g_guide_state __attribute__((weak));

namespace station_guidance
{
    static Config cfg;

    // 本地稳定检测状态
    static cv::Point prev_center_for_stability(-10000, -10000);
    static int stable_counter = 0;

    void init(const Config &c)
    {
        cfg = c;
    }
    void update_baseline(float hor_m, float ver_m)
    {
        cfg.cam_baseline_m = hor_m;
        cfg.cam_baseline_v_m = ver_m;
        printf("[station_guidance] Updated Baseline: Hor=%.2fm, Ver=%.2fm\n", hor_m, ver_m);
    }

    bool should_run_detection(int station_idx)
    {
        if (station_idx < 0 || station_idx > 1)
            return true;

        // 显式禁止检测
        if (!g_guide_state.detection_allowed_for_station[station_idx].load())
        {
            int guiding = g_guide_state.guiding_station_idx.load();
            if (guiding == -1 && !g_guide_state.abs_move_in_progress.load())
            {
                // 如果上次引导已经结束一段时间，自动恢复检测
                auto since_end = std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now() - g_guide_state.last_guide_end_time)
                                     .count();
                if (since_end > 500)
                {
                    return true;
                }
            }
            // printf("[should_run_detection] station=%d detection_allowed=false\n", station_idx);
            return false;
        }

        // 如果正处于 abs move 且该站是被引导的站，则禁止检测
        if (g_guide_state.abs_move_in_progress.load())
        {
            int guiding = g_guide_state.guiding_station_idx.load();
            int target = 1 - guiding;
            if (target == station_idx)
            {
                double since_ms = since_abs_sent_ms();
                // 设置超时阈值
                double timeout_ms = (double)(cfg.abs_move_wait_sec + cfg.fine_tune_timeout_sec + 1000) * 1000.0;
                // 可以把 +1000 改为 +2000 等按需
                if (since_ms > timeout_ms)
                {
                    // printf("[should_run_detection] station=%d abs_move stuck for %.0f ms > %.0f ms -> allow detection to recover\n",
                    //        station_idx, since_ms, timeout_ms);
                    // 允许检测（但不直接清 cancel），让目标站尝试自行重新检测并锁定
                    return true;
                }
                else
                {
                    // printf("[should_run_detection] station=%d abs_move in progress, since=%.0f ms -> block detection\n", station_idx, since_ms);
                    return false;
                }
            }
        }

        // 允许检测
        return true;
    }

    void on_station_locked_frame(int img_w, int img_h, const cv::Point &center, float dis_obj, int guider_station_idx, float ServoPosH)
    {

        // 如果没有开启“协同引导”模式 (默认是 false)，直接退出
        if (!g_coop_mode.load())
        {
            stable_counter = 0; // 清零计数
            return;
        }
        // ===========================================================

        // 入门冷却检查
        auto now = std::chrono::steady_clock::now();
        if (g_guide_state.last_guide_end_time.time_since_epoch().count() != 0)
        {
            double since_last_end = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_guide_state.last_guide_end_time).count();
            int cooldown = g_guide_state.guide_cooldown_ms.load();
            if (since_last_end < cooldown)
            {
                // printf("[station_guidance] on_station_locked_frame: within cooldown %.0fms < %dms -> skip new guidance\n",
                //        since_last_end, cooldown);
                return;
            }
        }
        if (g_guide_state.guidance_disabled_until_reset.load())
        {
            // printf("[station_guidance] on_station_locked_frame: guidance disabled_until_reset -> skip\n");
            return;
        }

        // 稳定检测
        if (prev_center_for_stability.x < -9000)
        {
            prev_center_for_stability = center;
            stable_counter = 0;
            return;
        }

        float mv = std::hypot((float)(center.x - prev_center_for_stability.x), (float)(center.y - prev_center_for_stability.y));
        if (mv <= cfg.stable_pixel_threshold)
        {
            stable_counter++;
        }
        else
        {
            stable_counter = 0;
            prev_center_for_stability = center;
            return;
        }

        const double guide_cooldown_ms = 1500.0; // 1.5s 冷却
        const float yaw_dedup_deg = 20.0f;       // 去重阈值

        // 如果已经有引导在进行且不是本 guider 发起的，则跳过
        int cur_guiding = g_guide_state.guiding_station_idx.load();
        if (cur_guiding != -1)
        {
            // 如果当前引导者就是本 guider，继续后续处理（避免重复）；否则跳过触发
            if (cur_guiding != guider_station_idx)
            {
                stable_counter = 0;
                return;
            }
        }

        // 达到稳定帧数时才考虑发送
        if (stable_counter >= cfg.stable_frames_threshold && g_guide_state.guiding_station_idx.load() == -1)
        {
            // 如果之前已经发送过绝对移动指令但还在进行中，则不重复下发
            if (g_guide_state.abs_move_in_progress.load())
            {
                // printf("[station_guidance] abs move already in progress, skip sending new one\n");
                stable_counter = 0;
                return;
            }

            // 冷却检查
            auto now = std::chrono::steady_clock::now();
            if (g_guide_state.last_guide_end_time.time_since_epoch().count() != 0)
            {
                double since_last_end = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_guide_state.last_guide_end_time).count();
                if (since_last_end < guide_cooldown_ms)
                {
                    stable_counter = 0;
                    return;
                }
            }

            // 计算目标偏航
            float bearing_deg = ServoPosH;
            float used_dis = dis_obj;
            int target_station = 1 - guider_station_idx; // 提前计算目标站编号

            // 根据 target_station 选择对应的计算函数
            float desired_yaw_for_target = bearing_deg; // 默认直接使用当前角度
            if (used_dis > 0.01f)
            {
                if (target_station == 1)
                {
                    desired_yaw_for_target = compute_target_yaw_for_other_station1_deg(used_dis, bearing_deg, cfg.cam_baseline_m);
                }
                else
                {
                    desired_yaw_for_target = compute_target_yaw_for_other_station0_deg(used_dis, bearing_deg, cfg.cam_baseline_m);
                }
            }

            float last_yaw = g_guide_state.last_sent_desired_yaw.load();
            if (std::isfinite(last_yaw))
            {
                float yaw_diff = fabsf(desired_yaw_for_target - last_yaw);
                if (yaw_diff < yaw_dedup_deg)
                {
                    // printf("[station_guidance] desired yaw diff %.2f < dedup %.2f -> skip\n", yaw_diff, yaw_dedup_deg);
                    stable_counter = 0;
                    return;
                }
            }

            // 标记即将绝对移动，设置 guiding、禁用目标站检测
            g_guide_state.abs_move_in_progress.store(true);
            g_guide_state.abs_move_sent_time = std::chrono::steady_clock::now();
            g_guide_state.abs_move_fail_count.store(0);

            g_guide_state.guiding_station_idx.store(guider_station_idx);
            // target_station 已在上面计算
            g_guide_state.desired_yaw_deg.store(desired_yaw_for_target);
            g_guide_state.stage.store(1);
            g_guide_state.guide_start_time = std::chrono::steady_clock::now();
            g_guide_state.detection_allowed_for_station[target_station].store(false);

            g_guide_state.last_sent_desired_yaw.store(desired_yaw_for_target);
            g_guide_state.last_abs_send_time = std::chrono::steady_clock::now();

            int servo_units = yaw_deg_to_servo_units(desired_yaw_for_target);
            g_guide_state.desired_servo_units.store(servo_units);

            int send_val = servo_units * 17; // 转换为舵机指令值
            // printf("[station_guidance] guider=%d computed bearing=%.2f desired_yaw=%.2f -> send abs move to target=%d servo_units=%d send_val=%.2f\n",
            //        guider_station_idx, bearing_deg, desired_yaw_for_target, target_station, servo_units, send_val);

            // 下发给被引导站（target）
            setServoAbsMovForStation(target_station, send_val, 0);

            // 重置稳定计数
            stable_counter = 0;
            return;
        }
    }

    void update_target_station_state(int target_station_idx, bool personDetected, int x_center, int y_center)
    {
        if (target_station_idx < 0 || target_station_idx > 1)
            return;

        int guiding = g_guide_state.guiding_station_idx.load();
        if (guiding == -1)
            return; // 没有引导在进行

        int expected_target = 1 - guiding;
        if (expected_target != target_station_idx)
            return; // 本调用不是被引导的那台站

        auto now = std::chrono::steady_clock::now();
        double elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_guide_state.guide_start_time).count();
        int s = g_guide_state.stage.load();

        // printf("[station_guidance] update_target_station_state: target=%d stage=%d elapsed_ms=%.0f abs_move_in_progress=%d\n",
        //        target_station_idx, s, elapsed_ms, (int)g_guide_state.abs_move_in_progress.load());

        // 参数
        const double min_wait_ms = 50.0; // 发送后最小等待
        const double absolute_timeout_ms = (cfg.abs_move_wait_sec + cfg.fine_tune_timeout_sec) * 1000.0;
        const float tol_deg = 20.0f;            // 到位容差
        const int within_count_threshold = 500; // 连续读数门槛
        const double stable_ms = 2500.0;        // 保持时间门槛（ms）

        if (s == 1)
        {
            // 刚发 abs move，先等最小上报时间
            if (g_guide_state.abs_move_in_progress.load())
            {
                double since_abs_sent_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_guide_state.abs_move_sent_time).count();
                if (since_abs_sent_ms < min_wait_ms)
                {
                    // printf("[station_guidance] waiting min_wait_ms (%.0f ms), since_abs_sent=%.0f ms\n", min_wait_ms, since_abs_sent_ms);
                    return;
                }
            }

            // 读取被引导站舵机位置
            float curH = 0.0f, curV = 0.0f;
            bool got_pos = getStationServoPos(target_station_idx, curH, curV);
            if (!got_pos)
            {
                // printf("[station_guidance] getStationServoPos returned FALSE for station %d (no fresh data)\n", target_station_idx);
                // 失败处理与超时保持不变
                if (g_guide_state.abs_move_in_progress.load())
                {
                    g_guide_state.abs_move_fail_count.fetch_add(1);
                    double since_abs_sent_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_guide_state.abs_move_sent_time).count();
                    if (since_abs_sent_ms >= absolute_timeout_ms)
                    {
                        // 超时：取消引导并恢复目标站搜索
                        g_guide_state.abs_move_in_progress.store(false);
                        g_guide_state.abs_move_fail_count.store(0);
                        g_guide_state.guiding_station_idx.store(-1);
                        g_guide_state.stage.store(0);
                        g_guide_state.detection_allowed_for_station[target_station_idx].store(true);
                        // setServoSpdMovForStationWrapper(target_station_idx, 30, 0);
                        g_guide_state.last_guide_end_time = std::chrono::steady_clock::now();
                        // printf("[station_guidance] abs-move read timeout -> cancel guidance and restore search for station %d\n", target_station_idx);
                    }
                }
                return;
            }

            // 读取成功
            g_guide_state.abs_move_fail_count.store(0);
            // printf("[station_guidance] getStationServoPos OK: curH=%.2f curV=%.2f\n", curH, curV);

            float targetYaw = g_guide_state.desired_yaw_deg.load();
            float diff = angle_diff_deg(curH, targetYaw);
            // printf("[station_guidance] targetYaw=%.2f curH=%.2f absdiff=%.2f tol=%.2f\n", targetYaw, curH, diff, tol_deg);

            if (diff <= tol_deg)
            {
                if (g_guide_state.stage1_within_count.load() == 0)
                {
                    g_guide_state.stage1_first_within_time = now;
                }
                int cnt = g_guide_state.stage1_within_count.fetch_add(1) + 1;
                double since_within_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_guide_state.stage1_first_within_time).count();

                // printf("[station_guidance] within tol: count=%d since_within=%.0f ms\n", cnt, since_within_ms);

                if (cnt >= within_count_threshold || since_within_ms >= stable_ms)
                {
                    // 到位：进入细调（stage 2），允许目标站检测与微调
                    g_guide_state.stage.store(2);
                    g_guide_state.detection_allowed_for_station[target_station_idx].store(true);
                    g_guide_state.abs_move_in_progress.store(false);
                    g_guide_state.abs_move_fail_count.store(0);
                    g_stations[target_station_idx].AbsMove = 0;
                    // 清理计数
                    g_guide_state.stage1_within_count.store(0);
                    return;
                }

                else
                {
                    return;
                }
            }
            else
            {
                // 未到位：重置计数
                g_guide_state.stage1_within_count.store(0);
                g_guide_state.stage1_first_within_time = std::chrono::steady_clock::time_point();
                double since_abs_sent_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_guide_state.abs_move_sent_time).count();
                // printf("[station_guidance] not at target yet (absdiff %.2f). since_abs_sent=%.0f ms\n", diff, since_abs_sent_ms);

                if (since_abs_sent_ms >= absolute_timeout_ms)
                {
                    // 超时取消
                    g_guide_state.guiding_station_idx.store(-1);
                    g_guide_state.stage.store(0);
                    g_guide_state.abs_move_in_progress.store(false);
                    g_guide_state.detection_allowed_for_station[target_station_idx].store(true);
                    // setServoSpdMovForStationWrapper(target_station_idx, 30, 0);
                    g_guide_state.last_guide_end_time = std::chrono::steady_clock::now();
                    // printf("[station_guidance] abs-move timeout, cancel guidance for station %d\n", target_station_idx);
                }
                return;
            }
        }

        else if (s == 2)
        {
            // 细调阶段：允许检测、监控超时
            g_guide_state.detection_allowed_for_station[target_station_idx].store(true);

            if (elapsed_ms >= (cfg.abs_move_wait_sec + cfg.fine_tune_timeout_sec) * 1000.0)
            {
                // 细调超时：恢复
                g_guide_state.guiding_station_idx.store(-1);
                g_guide_state.stage.store(0);
                g_guide_state.abs_move_in_progress.store(false);
                // setServoSpdMovForStationWrapper(target_station_idx, 30, 0);
                g_guide_state.last_guide_end_time = std::chrono::steady_clock::now();
                // printf("[station_guidance] fine-tune timeout, cancel guidance for station %d\n", target_station_idx);
            }
        }
    }

    void cancel_guidance()
    {
        // 清理所有引导相关标志，恢复默认
        g_guide_state.guiding_station_idx.store(-1);
        g_guide_state.stage.store(0);
        g_guide_state.abs_move_in_progress.store(false);
        g_guide_state.abs_move_fail_count.store(0);
        g_guide_state.detection_allowed_for_station[0].store(true);
        g_guide_state.detection_allowed_for_station[1].store(true);
        g_guide_state.guidance_disabled_until_reset.store(true);
        g_guide_state.last_guide_end_time = std::chrono::steady_clock::now();
        // Debug: confirm values
        auto tms = std::chrono::duration_cast<std::chrono::milliseconds>(g_guide_state.last_guide_end_time.time_since_epoch()).count();

        for (int i = 0; i < g_station_count; ++i)
        {
            g_stations[i].AbsMove = 0;
        }

        // 恢复两台站的速度控制（例：先停止微调，保持0；或设为搜索速度30）
        // 这里先用 0 保持当前位置，如果想让搜索恢复可改为 30
        setServoSpdMovForStationWrapper(0, 0, 0);
        setServoSpdMovForStationWrapper(1, 0, 0);

        // 确保检测允许位已经置位
        g_guide_state.detection_allowed_for_station[0].store(true);
        g_guide_state.detection_allowed_for_station[1].store(true);
    }

    bool is_guiding()
    {
        return g_guide_state.guiding_station_idx.load() != -1;
    }
    bool is_abs_move_in_progress()
    {
        return g_guide_state.abs_move_in_progress.load();
    }

    int get_stage()
    {
        return g_guide_state.stage.load();
    }

    double since_abs_sent_ms()
    {
        if (!g_guide_state.abs_move_in_progress.load())
            return 1e9;
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(now - g_guide_state.abs_move_sent_time).count();
    }
    void suspend_detection_for_target_of_guider(int guider_station_idx)
    {
        int target = 1 - guider_station_idx;
        g_guide_state.detection_allowed_for_station[target].store(false);
    }

} // namespace station_guidance
