#ifndef RTSP_STREAMER_H
#define RTSP_STREAMER_H

#include <string>
#include <mutex>
#include <atomic>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <opencv2/opencv.hpp>

class RtspStreamer
{
public:
    RtspStreamer(int width, int height, int fps, const std::string &mount);
    ~RtspStreamer();

    // 启动推流
    bool start(int port);
    // 停止推流
    void stop();
    // 推送一帧图片
    bool pushFrame(const cv::Mat &bgr);

private:
    // 回调函数
    static void need_data_cb(GstAppSrc *src, guint length, gpointer user_data);
    static void enough_data_cb(GstAppSrc *src, gpointer user_data);

    int width_;
    int height_;
    int fps_;
    std::string mount_point_;

    // GStreamer 核心组件
    GstElement *pipeline_ = nullptr;
    GstAppSrc *appsrc_ = nullptr;

    std::mutex appsrc_mtx_;
    std::atomic<bool> need_data_{true};
    guint64 frame_count_ = 0;
    guint64 frame_duration_ = 0;
};

#endif // RTSP_STREAMER_H