#include "rtsp_streamer.h"
#include <iostream>
#include <cstring>

RtspStreamer::RtspStreamer(int width, int height, int fps, const std::string &mount)
    : width_(width), height_(height), fps_(fps), mount_point_(mount)
{
    frame_duration_ = gst_util_uint64_scale_int(1, GST_SECOND, fps_);
    gst_init(nullptr, nullptr);
}

RtspStreamer::~RtspStreamer()
{
    stop();
}

bool RtspStreamer::start(int port)
{
    // 硬件编码码率设置 (2Mbps - 4Mbps)
    int target_bps = 2048000;

    char launch_str[2048];

    // =======================================================================
    // 终极稳定管道配置：
    // 1. videoconvert: 自动处理内存对齐和颜色转换（解决报错的关键）
    // 2. mpph264enc: RK3588 硬件编码，CPU 占用率极低
    // 3. rtspclientsink: TCP 推流给本机
    // =======================================================================
    snprintf(launch_str, sizeof(launch_str),
             "appsrc name=mysrc is-live=true block=false format=time do-timestamp=true "
             "caps=video/x-raw,format=I420,width=%d,height=%d,framerate=%d/1 "
             "! videoconvert "                         // <--- 这个会自动处理对齐，不会报错
             "! mpph264enc bps=%d gop=30 rc-mode=cbr " // 硬件编码，CBR模式更稳
             "! h264parse config-interval=1 "
             "! rtspclientsink location=rtsp://127.0.0.1:%d%s latency=0 protocols=tcp sync=false async=false",
             width_, height_, fps_, target_bps, port, mount_point_.c_str());

    std::cout << "[RtspStreamer] Pipeline: " << launch_str << std::endl;

    GError *error = nullptr;
    pipeline_ = gst_parse_launch(launch_str, &error);

    if (error)
    {
        std::cerr << "[RtspStreamer] Pipeline Error: " << error->message << std::endl;
        return false;
    }

    GstElement *appsrc_elem = gst_bin_get_by_name(GST_BIN(pipeline_), "mysrc");
    if (!appsrc_elem)
    {
        std::cerr << "[RtspStreamer] Can't find appsrc!" << std::endl;
        return false;
    }

    appsrc_ = GST_APP_SRC(appsrc_elem);

    GstAppSrcCallbacks callbacks = {0};
    callbacks.need_data = need_data_cb;
    callbacks.enough_data = enough_data_cb;
    gst_app_src_set_callbacks(appsrc_, &callbacks, this, NULL);

    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE)
    {
        std::cerr << "[RtspStreamer] Failed to play pipeline!" << std::endl;
        return false;
    }

    std::cout << "=================================================" << std::endl;
    std::cout << "硬件推流启动 (MPP+VideoConvert)" << std::endl;
    std::cout << "PC端地址: rtsp://192.168.1.50:" << port << mount_point_ << std::endl;
    std::cout << "=================================================" << std::endl;

    return true;
}

void RtspStreamer::stop()
{
    if (pipeline_)
    {
        if (appsrc_)
            gst_app_src_end_of_stream(appsrc_);
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
    }
    if (appsrc_)
    {
        gst_object_unref(appsrc_);
        appsrc_ = nullptr;
    }
}

void RtspStreamer::need_data_cb(GstAppSrc *, guint, gpointer user_data)
{
    RtspStreamer *self = static_cast<RtspStreamer *>(user_data);
    self->need_data_.store(true);
}

void RtspStreamer::enough_data_cb(GstAppSrc *, gpointer user_data)
{
    RtspStreamer *self = static_cast<RtspStreamer *>(user_data);
    self->need_data_.store(false);
}

bool RtspStreamer::pushFrame(const cv::Mat &bgr)
{
    if (!appsrc_ || !pipeline_)
        return false;
    if (bgr.empty() || bgr.cols != width_ || bgr.rows != height_)
        return false;

    // BGR -> I420
    cv::Mat yuv;
    cv::cvtColor(bgr, yuv, cv::COLOR_BGR2YUV_I420);

    guint size = width_ * height_ * 3 / 2;
    GstBuffer *buffer = gst_buffer_new_allocate(NULL, size, NULL);
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_WRITE);
    memcpy(map.data, yuv.data, size);
    gst_buffer_unmap(buffer, &map);

    GST_BUFFER_PTS(buffer) = (GstClockTime)(frame_count_ * frame_duration_);
    GST_BUFFER_DURATION(buffer) = frame_duration_;
    frame_count_++;

    GstFlowReturn ret = gst_app_src_push_buffer(appsrc_, buffer);
    if (ret != GST_FLOW_OK)
        return false;

    return true;
}