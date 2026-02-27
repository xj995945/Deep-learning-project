#ifndef COMMON_H
#define COMMON_H

#include <atomic>
#include <memory>
#include <vector>
#include <string>
#include <mutex>
#include <array>
#include <fstream>
#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// 外部声明
extern std::vector<std::unique_ptr<std::atomic<bool>>> g_cycle_flags;
extern std::atomic<bool> stop_flag;
extern std::atomic<bool> g_guidance_done;

// 全局保存检测结果与互斥量
extern std::vector<cv::Mat> g_results;
extern std::vector<std::mutex> g_mtx;
extern const std::vector<std::string> names;
extern const std::vector<cv::Scalar> colors;
extern int screen_width;
extern int screen_height;
extern int display_station_idx;
extern int control_station_idx;

extern cv::Mat safeResizeForDisplay(const cv::Mat &img, int screen_width, int screen_height);

extern std::atomic<bool> g_enable_recog; // 识别总开关
extern std::atomic<bool> g_enable_track; // 跟踪总开关

extern std::atomic<float> g_conf_threshold; // 置信度
extern std::atomic<float> g_nms_threshold;  // NMS (IoU)
extern std::atomic<float> g_track_kp;
extern std::atomic<float> g_track_kd;

extern std::atomic<bool> g_coop_mode;     // 是否开启协同
extern std::atomic<int> g_coop_speeds[2]; // 两个站的协同扫描速度

// === 全局锁定ID数组 ===
// g_locked_track_ids[0] 对应站1，[1] 对应站2
// 值 -1 表示不锁定任何目标
extern std::array<std::atomic<int>, 2> g_locked_track_ids;

void loadConfig();
void saveConfig(bool enable, int s1, int s2);

// === L形框绘制函数 ===
// img: 目标图像
// rect: 矩形区域
// color: 颜色
// thickness: 线宽
void drawCornerRect(cv::Mat &img, cv::Rect rect, cv::Scalar color, int thickness);
std::string extract_ip_from_rtsp(const std::string &rtsp);

#endif // COMMON_H
