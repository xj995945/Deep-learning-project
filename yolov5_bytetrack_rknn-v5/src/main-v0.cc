// main.cc
// 单网段适配版 复

#include <stdio.h>
#include <memory>
#include <chrono>
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>
#include <cmath>
#include <gst/gst.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "rkYolov5s.hpp"
#include "BYTETracker.h"
#include "STrack.h"
#include "servoControl.h"
#include "stationManager.h"
#include "station_guidance.h"
#include "common.h"
#include "worker_thread.h"
#include "rtsp_streamer.h"

#include <atomic>
#include <thread>
#include <vector>
#include <string>
#include <mutex>
#include <algorithm>

extern int InitStations(const char *ips[], int count, int port);
extern void setServoSpdMovForStation(int station_idx, int SpdH, int SpdV);

// ================== 【绘制十字分划辅助函数】 ==================
void draw_crosshair(cv::Mat &img, cv::Rect roi)
{
    // 计算 ROI 的中心点
    int cx = roi.x + roi.width / 2;
    int cy = roi.y + roi.height / 2;

    // 十字线长度 (画面宽度的 1/24)
    int cross_len = std::min(roi.width, roi.height) / 12;
    int thickness = 2;
    cv::Scalar cross_color(0, 255, 0); // 绿色

    // 画横线
    cv::line(img, cv::Point(cx - cross_len, cy), cv::Point(cx + cross_len, cy), cross_color, thickness);
    // 画竖线
    cv::line(img, cv::Point(cx, cy - cross_len), cv::Point(cx, cy + cross_len), cross_color, thickness);
    // 画中心圆点
    cv::circle(img, cv::Point(cx, cy), 4, cross_color, -1);
}
// ================================================================

void sigint_handler(int)
{
    stop_flag.store(true);
}

int main(int argc, char **argv)
{
    loadConfig();

    if (argc < 2)
    {
        printf("Usage: %s <rknn model> <station1_ip> [station2_ip]\n", argv[0]);
        return -1;
    }
    const char *model_name = argv[1];

    // 解析 IP
    std::vector<std::string> ips;
    std::vector<std::string> rtsp_urls;

    for (int i = 2; i < argc && i < 4; ++i)
    {
        std::string s(argv[i]);
        if (s.rfind("rtsp://", 0) == 0)
        {
            std::string host = extract_ip_from_rtsp(s);
            ips.push_back(host.empty() ? "Unknown" : host);
            rtsp_urls.push_back(s);
        }
        else
        {
            ips.push_back(s);
            rtsp_urls.push_back(std::string("rtsp://") + s + "/live");
        }
    }

    int station_count = (int)ips.size();
    if (station_count <= 0)
    {
        fprintf(stderr, "No stations provided.\n");
        return -1;
    }

    // 自适应模式判断
    if (station_count == 1)
    {
        printf("!!! 检测到单站输入 -> 强制独立模式 !!!\n");
        g_coop_mode.store(false);
        g_enable_recog.store(false);
        g_enable_track.store(false);
    }

    if (g_coop_mode.load())
    {
        printf("!!! 启动模式: 协同引导 (速度 %d, %d) !!!\n",
               g_coop_speeds[0].load(), g_coop_speeds[1].load());
        g_enable_recog.store(true);
        g_enable_track.store(true);
    }
    else
    {
        printf("!!! 启动模式: 独立搜索/跟踪 !!!\n");
    }

    g_cycle_flags.clear();
    for (int i = 0; i < station_count; ++i)
        g_cycle_flags.emplace_back(std::make_unique<std::atomic<bool>>(false));

    std::vector<const char *> ips_c;
    for (const auto &p : ips)
        ips_c.push_back(p.c_str());

    if (InitStations(ips_c.data(), station_count, 60000) != 0)
    {
        fprintf(stderr, "InitStations failed\n");
        // return -1; // 视情况注释
    }

    g_results = std::vector<cv::Mat>(station_count);
    g_mtx = std::vector<std::mutex>(station_count);

    std::vector<std::thread> workers;
    for (int i = 0; i < station_count; ++i)
    {
        workers.emplace_back(worker_thread, i, std::string(model_name), rtsp_urls[i]);
        usleep(100000);
    }

    // 1080P 设置
    int out_width = 1920;
    int out_height = 1080;
    int out_fps = 25;

    printf("[Main] Initializing RTSP Streamer: %dx%d @ %d fps\n", out_width, out_height, out_fps);

    RtspStreamer streamer(out_width, out_height, out_fps, "/stream");
    if (streamer.start(8554))
    {
        printf("RTSP server running at rtsp://192.168.2.50:8554/stream\n");
    }
    else
    {
        fprintf(stderr, "Failed to start RTSP streamer\n");
    }

    cv::namedWindow("DualView", cv::WINDOW_NORMAL);
    cv::setWindowProperty("DualView", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    signal(SIGINT, sigint_handler);

    while (!stop_flag.load())
    {
        cv::Mat canvas(out_height, out_width, CV_8UC3, cv::Scalar(0, 0, 0));

        // -------------------------------------------------
        // 情况 A: 单站 -> 全屏显示 (Full Screen)
        // -------------------------------------------------
        if (station_count == 1)
        {
            cv::Mat img;
            {
                std::lock_guard<std::mutex> lk(g_mtx[0]);
                if (!g_results[0].empty())
                    img = g_results[0].clone();
            }

            cv::Rect full_rect(0, 0, out_width, out_height);

            if (!img.empty())
            {
                cv::resize(img, canvas, cv::Size(out_width, out_height));

                // 绘制边框
                // cv::Scalar border = g_station_status[0].locked.load() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
                cv::Scalar border = cv::Scalar(0, 255, 0);
                // cv::rectangle(canvas, full_rect, border, 5);

                // 绘制 IP
                std::string txt = "Station: " + ips[0];
                cv::putText(canvas, txt, cv::Point(50, 80), cv::FONT_HERSHEY_SIMPLEX, 1.5, border, 3);

                // 绘制十字分划
                draw_crosshair(canvas, full_rect);
            }
            else
            {
                cv::putText(canvas, "Waiting for Video...", cv::Point(out_width / 2 - 200, out_height / 2),
                            cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 0, 255), 2);
            }
        }
        // -------------------------------------------------
        // 情况 B: 双站 -> 左右分屏 (Split Screen)
        // -------------------------------------------------
        else
        {
            int cellW = 960;
            int cellH = 540;
            int vOffset = (out_height - cellH) / 2;

            // --- 左边 (Station 1) ---
            {
                cv::Mat img;
                {
                    std::lock_guard<std::mutex> lk(g_mtx[0]);
                    if (!g_results[0].empty())
                        img = g_results[0].clone();
                }
                cv::Rect roi_left(0, vOffset, cellW, cellH);

                if (!img.empty())
                {
                    cv::Mat resized;
                    cv::resize(img, resized, cv::Size(cellW, cellH));
                    resized.copyTo(canvas(roi_left));

                    cv::Scalar border = g_station_status[0].locked.load() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
                    cv::rectangle(canvas, roi_left, border, 3);
                    cv::putText(canvas, "St 1: " + ips[0], cv::Point(20, vOffset + 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, border, 2);

                    draw_crosshair(canvas, roi_left);
                }
            }

            // --- 右边 (Station 2) ---
            {
                cv::Mat img;
                {
                    std::lock_guard<std::mutex> lk(g_mtx[1]);
                    if (!g_results[1].empty())
                        img = g_results[1].clone();
                }
                cv::Rect roi_right(cellW, vOffset, cellW, cellH);

                if (!img.empty())
                {
                    cv::Mat resized;
                    cv::resize(img, resized, cv::Size(cellW, cellH));
                    resized.copyTo(canvas(roi_right));

                    cv::Scalar border = g_station_status[1].locked.load() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
                    cv::rectangle(canvas, roi_right, border, 3);
                    cv::putText(canvas, "St 2: " + ips[1], cv::Point(cellW + 20, vOffset + 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, border, 2);

                    draw_crosshair(canvas, roi_right);
                }
            }
        }

        if (!canvas.empty())
        {
            streamer.pushFrame(canvas);
            cv::imshow("DualView", canvas);
        }

        while (g_main_context_iteration(NULL, FALSE))
            ;

        // int k = cv::waitKey(10);
        // if (k == 'q' || k == 27)
        // {
        //     stop_flag.store(true);
        //     break;
        // }
    }

    for (auto &t : workers)
        if (t.joinable())
            t.join();
    for (int i = 0; i < station_count; ++i)
        setServoSpdMovForStation(i, 0, 0);
    streamer.stop();

    cv::destroyWindow("DualView");
    printf("Exited cleanly\n");
    return 0;
}