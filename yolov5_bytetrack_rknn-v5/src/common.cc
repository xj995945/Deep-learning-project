#include "common.h"

// 屏幕分辨率
int screen_width = 1920;
int screen_height = 1080;

std::vector<std::unique_ptr<std::atomic<bool>>> g_cycle_flags;
std::atomic<bool> stop_flag(false);
std::atomic<bool> g_guidance_done(false);

std::vector<cv::Mat> g_results;
std::vector<std::mutex> g_mtx;

// === 定义全局开关 ===
std::atomic<bool> g_enable_recog(true);
std::atomic<bool> g_enable_track(false);

std::atomic<float> g_conf_threshold(0.5f);
std::atomic<float> g_nms_threshold(0.45f);
std::atomic<float> g_track_kp(35.0f);
std::atomic<float> g_track_kd(12.0f);

// 定义并初始化
std::atomic<bool> g_coop_mode(false);
std::atomic<int> g_coop_speeds[2] = {{0}, {0}}; // 默认速度0

const char *CONFIG_FILE = "config.txt";
// 初始化为 -1 (不锁定)
std::array<std::atomic<int>, 2> g_locked_track_ids;

int display_station_idx = 0;
int control_station_idx = -1;

const std::vector<std::string> names = {
    "person", "head_helmet", "head", "reflective_clothes", "smoking", "calling", "falling",
    "face_mask", "car", "bicycle", "motorcycle", "fumes", "fire", "head_hat",
    "normal_clothes", "face", "play_phone", "other", "knife"};

const std::vector<cv::Scalar> colors = {
    cv::Scalar(255, 0, 0),     // 蓝色
    cv::Scalar(0, 255, 0),     // 绿色
    cv::Scalar(0, 0, 255),     // 红色
    cv::Scalar(255, 255, 0),   // 青色
    cv::Scalar(255, 0, 255),   // 品红色
    cv::Scalar(0, 255, 255),   // 黄色
    cv::Scalar(192, 192, 192), // 浅灰色
    cv::Scalar(128, 0, 0),     // 深红色
    cv::Scalar(128, 128, 0),   // 橄榄色
    cv::Scalar(0, 128, 0),     // 深绿色
    cv::Scalar(128, 0, 128),   // 紫色
    cv::Scalar(0, 128, 128),   // 深青色
    cv::Scalar(0, 0, 128),     // 深蓝色
    cv::Scalar(255, 128, 0),   // 橙色
    cv::Scalar(255, 0, 128),   // 红紫色
    cv::Scalar(128, 255, 0),   // 黄绿色
    cv::Scalar(0, 255, 128),   // 青绿色
    cv::Scalar(128, 0, 255),   // 蓝紫色
    cv::Scalar(255, 255, 255)  // 白色
};

const float person_thres = 0.1;             // 0
const float head_helmet_thres = 0.15;       // 1
const float head_thres = 0.15;              // 2
const float reflective_clothes_thres = 0.5; // 3
const float smoking_thres = 0.15;           // 4
const float calling_thres = 0.15;           // 5
const float falling_thres = 0.5;            // 6
const float face_mask_thres = 0.5;          // 7
const float car_thres = 0.5;                // 8
const float bicycle_thres = 0.5;            // 9
const float motorcycle_thres = 0.5;         // 10
const float fumes_thres = 0.5;              // 11
const float fire_thres = 0.5;               // 12
const float head_hat_thres = 0.15;          // 13
const float normal_clothes_thres = 0.5;     // 14
const float face_thres = 0.5;               // 15
const float play_phone_thres = 0.15;        // 16
const float other_thres = 0.5;              // 17
const float knife_thres = 0.15;             // 18

// 图像调整函数
cv::Mat safeResizeForDisplay(const cv::Mat &img, int screen_width, int screen_height)
{
    if (img.empty())
    {
        printf("Warning: Input image is empty!\n");
        return cv::Mat::zeros(screen_height, screen_width, CV_8UC3);
    }

    if (img.cols <= 0 || img.rows <= 0)
    {
        printf("Warning: Input image has invalid dimensions: %dx%d\n", img.cols, img.rows);
        return cv::Mat::zeros(screen_height, screen_width, CV_8UC3);
    }

    // 如果图像尺寸已经匹配屏幕，直接返回
    if (img.cols == screen_width && img.rows == screen_height)
    {
        return img.clone();
    }

    double scale_x = (double)screen_width / img.cols;
    double scale_y = (double)screen_height / img.rows;
    double scale = std::min(scale_x, scale_y);

    // 确保缩放比例有效
    if (scale <= 0)
    {
        printf("Warning: Invalid scale factor: %f, using default scaling\n", scale);
        scale = 1.0;
    }

    int new_width = (int)(img.cols * scale);
    int new_height = (int)(img.rows * scale);

    // 确保新尺寸有效
    if (new_width <= 0)
        new_width = 1;
    if (new_height <= 0)
        new_height = 1;

    cv::Mat display_img;
    try
    {
        cv::resize(img, display_img, cv::Size(new_width, new_height));

        // 如果需要，添加黑边
        if (new_width < screen_width || new_height < screen_height)
        {
            cv::Mat bordered_img = cv::Mat::zeros(screen_height, screen_width, img.type());
            int x_offset = (screen_width - new_width) / 2;
            int y_offset = (screen_height - new_height) / 2;
            display_img.copyTo(bordered_img(cv::Rect(x_offset, y_offset, new_width, new_height)));
            return bordered_img;
        }

        return display_img;
    }
    catch (const cv::Exception &e)
    {
        printf("Error in resize: %s\n", e.what());
        return cv::Mat::zeros(screen_height, screen_width, CV_8UC3);
    }
}

void loadConfig()
{
    std::ifstream infile(CONFIG_FILE);
    if (infile.good())
    {
        int enable_int, s1, s2;
        // 文件格式: enable speed1 speed2
        if (infile >> enable_int >> s1 >> s2)
        {
            g_coop_mode.store(enable_int == 1);
            g_coop_speeds[0].store(s1);
            g_coop_speeds[1].store(s2);
            printf("[Config] Loaded: Mode=%d, S1=%d, S2=%d\n", enable_int, s1, s2);
        }
    }
    else
    {
        printf("[Config] No config file, using defaults.\n");
    }
    infile.close();
}

void saveConfig(bool enable, int s1, int s2)
{
    std::ofstream outfile(CONFIG_FILE);
    // 写入: 1 20 20 (开启, 速度20, 速度20)
    outfile << (enable ? 1 : 0) << " " << s1 << " " << s2 << std::endl;
    outfile.close();
    printf("[Config] Saved settings to file.\n");
}

// === L形框绘制函数实现 ===
void drawCornerRect(cv::Mat &img, cv::Rect rect, cv::Scalar color, int thickness)
{
    int x = rect.x;
    int y = rect.y;
    int w = rect.width;
    int h = rect.height;

    // 计算拐角长度：取宽高中较小值的 1/4
    int len = std::min(w, h) / 4;

    // 限制长度范围，防止框太小时线太短，或框太大时线太长
    if (len < 10)
        len = 10;
    if (len > w)
        len = w;
    if (len > h)
        len = h;

    // 左上角
    cv::line(img, cv::Point(x, y), cv::Point(x + len, y), color, thickness);
    cv::line(img, cv::Point(x, y), cv::Point(x, y + len), color, thickness);

    // 右上角
    cv::line(img, cv::Point(x + w, y), cv::Point(x + w - len, y), color, thickness);
    cv::line(img, cv::Point(x + w, y), cv::Point(x + w, y + len), color, thickness);

    // 右下角
    cv::line(img, cv::Point(x + w, y + h), cv::Point(x + w - len, y + h), color, thickness);
    cv::line(img, cv::Point(x + w, y + h), cv::Point(x + w, y + h - len), color, thickness);

    // 左下角
    cv::line(img, cv::Point(x, y + h), cv::Point(x + len, y + h), color, thickness);
    cv::line(img, cv::Point(x, y + h), cv::Point(x, y + h - len), color, thickness);
}

std::string extract_ip_from_rtsp(const std::string &rtsp)
{
    const std::string prefix = "rtsp://";
    if (rtsp.rfind(prefix, 0) != 0)
        return "";
    size_t pos = prefix.size();
    size_t slash = rtsp.find('/', pos);
    std::string hostport = (slash == std::string::npos) ? rtsp.substr(pos) : rtsp.substr(pos, slash - pos);
    size_t colon = hostport.find(':');
    if (colon != std::string::npos)
        return hostport.substr(0, colon);
    return hostport;
}
