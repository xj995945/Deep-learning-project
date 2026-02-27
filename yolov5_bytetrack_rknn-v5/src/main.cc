// main.cc
// 修复版：解决内存溢出和延迟问题
// 策略：取走即清空 (Consume and Clear)，防止重复帧堆积

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
#include "ip_config.h"

#include <atomic>
#include <thread>
#include <vector>
#include <string>
#include <mutex>
#include <algorithm>

extern int InitStations(const char *ips[], int count, int port);
extern void setServoSpdMovForStation(int station_idx, int SpdH, int SpdV);

void sigint_handler(int)
{
    stop_flag.store(true);
}

// 绘制 UI
void drawOverlay(cv::Mat &img, int station_idx, const std::string &ip_str)
{
    int w = img.cols;
    int h = img.rows;
    int cx = w / 2;
    int cy = h / 2;

    std::string label = "Station " + std::to_string(station_idx + 1);

    // 绘制文字
    cv::putText(img, label, cv::Point(22, 42), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 3);
    cv::putText(img, label, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

    // 绘制锁定状态
    bool is_locked = g_station_status[station_idx].locked.load();
    if (is_locked)
    {
        std::string lock_text = "LOCKED";
        cv::putText(img, lock_text, cv::Point(22, 92), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 3);
        cv::putText(img, lock_text, cv::Point(20, 90), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
        cv::rectangle(img, cv::Rect(0, 0, w, h), cv::Scalar(0, 0, 255), 6);
    }

    // 绘制准星
    int sight_w = 80;
    int sight_h = 80;
    drawCornerRect(img, cv::Rect(cx - sight_w / 2, cy - sight_h / 2, sight_w, sight_h), cv::Scalar(0, 255, 0), 2);
    cv::circle(img, cv::Point(cx, cy), 3, cv::Scalar(0, 0, 255), -1);
}

int main(int argc, char **argv)
{
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);
    loadConfig();

    if (g_coop_mode.load())
    {
        printf("!!! 启动模式: 协同引导 !!!\n");
        g_enable_recog.store(true);
        g_enable_track.store(true);
    }
    else
    {
        g_enable_recog.store(true);
        printf("!!! 启动模式: 独立搜索 !!!\n");
    }

    if (argc < 3)
    {
        printf("Usage: %s <rknn model> <station1_ip> [station2 ...]\n", argv[0]);
        return -1;
    }
    const char *model_name = argv[1];

    // 解析 IP
    std::vector<std::string> ips;
    std::vector<std::string> rtsp_urls;
    for (int i = 2; i < argc; ++i)
    {
        std::string s(argv[i]);
        if (s.rfind("rtsp://", 0) == 0)
        {
            std::string host = extract_ip_from_rtsp(s);
            ips.push_back(host.empty() ? "" : host);
            rtsp_urls.push_back(s);
        }
        else
        {
            ips.push_back(s);
            rtsp_urls.push_back(std::string("rtsp://") + s + "/live");
        }
    }

    int station_count = (int)ips.size();

    // 初始化标志和Socket
    g_cycle_flags.clear();
    for (int i = 0; i < station_count; ++i)
        g_cycle_flags.emplace_back(std::make_unique<std::atomic<bool>>(false));

    std::vector<const char *> ips_c;
    for (const auto &p : ips)
        ips_c.push_back(p.c_str());

    int station_port = RK_LISTEN_PORT; // 使用配置 60000

    if (InitStations(ips_c.data(), station_count, station_port) != 0)
    {
        fprintf(stderr, "InitStations failed\n");
        return -1;
    }

    g_results = std::vector<cv::Mat>(station_count);
    g_mtx = std::vector<std::mutex>(station_count);

    // 启动 worker 线程
    std::vector<std::thread> workers;
    for (int i = 0; i < station_count; ++i)
    {
        workers.emplace_back(worker_thread, i, std::string(model_name), rtsp_urls[i]);
        usleep(200000);
    }

    // 初始化 RTSP 推流 (1280x720)
    int stream_w = 1280;
    int stream_h = 720;
    int stream_fps = 25;

    // Stream 1
    RtspStreamer streamer0(stream_w, stream_h, stream_fps, "/live");
    if (!streamer0.start(8554))
    {
        fprintf(stderr, "FATAL: Failed to start RTSP streamer 0\n");
    }

    // Stream 2
    RtspStreamer *pStreamer1 = nullptr;
    if (station_count > 1)
    {
        pStreamer1 = new RtspStreamer(stream_w, stream_h, stream_fps, "/live");
        if (!pStreamer1->start(8556))
        {
            fprintf(stderr, "FATAL: Failed to start RTSP streamer 1\n");
        }
    }

    signal(SIGINT, sigint_handler);

    printf(">>> Main Loop Started. Waiting for frames...\n");

    // =========================================================================
    while (!stop_flag.load())
    {
        bool any_frame_processed = false;

        for (int i = 0; i < station_count; ++i)
        {
            cv::Mat frame;

            //  加锁取帧
            {
                std::lock_guard<std::mutex> lk(g_mtx[i]);
                // 有数据时才取
                if (!g_results[i].empty())
                {
                    frame = g_results[i];
                    g_results[i].release(); // 显式清空全局变量
                }
            }

            if (frame.empty())
            {
                continue;
            }

            any_frame_processed = true;

            //  尺寸适配
            if (frame.cols != stream_w || frame.rows != stream_h)
            {
                cv::resize(frame, frame, cv::Size(stream_w, stream_h));
            }

            // 绘制 UI
            drawOverlay(frame, i, (i < ips.size() ? ips[i] : ""));

            // 推流
            if (i == 0)
                streamer0.pushFrame(frame);
            else if (i == 1 && pStreamer1)
                pStreamer1->pushFrame(frame);
        }

        // 处理 GStreamer 事件
        while (g_main_context_iteration(NULL, FALSE))
            ;

        // 动态休眠：
        if (any_frame_processed)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    // 清理资源
    for (auto &t : workers)
        if (t.joinable())
            t.join();
    for (int i = 0; i < station_count; ++i)
        setServoSpdMovForStation(i, 0, 0);

    streamer0.stop();
    if (pStreamer1)
    {
        pStreamer1->stop();
        delete pStreamer1;
    }

    printf("Exited cleanly\n");
    return 0;
}