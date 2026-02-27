#ifndef RKYOLOV5S_H
#define RKYOLOV5S_H

#include "rknn_api.h"

#include "opencv2/core/core.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <string>
#include <chrono>
#include <atomic>
#include <vector>
#include <mutex>
#include <set>
#include <memory>
#include <cmath>
#include "servoControl.h"

#include <fstream>
#include <chrono>
#include <string>
#include <iomanip>

static void dump_tensor_attr(rknn_tensor_attr *attr);
static unsigned char *load_data(FILE *fp, size_t ofst, size_t sz);
static unsigned char *load_model(const char *filename, int *model_size);
static int saveFloat(const char *file_name, float *output, int element_size);

void setServoSpdMovForStation(int station_idx, int nSpdH, int nSpdV);
void trackAndMoveServo_pos(int station_idx, float x_center, float y_center);
void trackAndMoveServo_spd(int station_idx, int x_center, int y_center);

extern std::vector<std::unique_ptr<std::atomic<bool>>> g_cycle_flags;

class rkYolov5s
{
public:
    struct DetectionResult
    {
        int class_id;       // 类别ID
        std::string name;   // 类别名称
        float confidence;   // 置信度
        int x1, y1, x2, y2; // 检测框坐标 (left, top, right, bottom)
        int x_center, y_center;
    };

private:
    int ret;
    std::mutex mtx;
    std::string model_path;
    unsigned char *model_data;

    rknn_context ctx;
    rknn_input_output_num io_num;
    rknn_tensor_attr *input_attrs;
    rknn_tensor_attr *output_attrs;
    rknn_input inputs[1];

    int channel, width, height;
    int img_width, img_height;

    float nms_threshold, box_conf_threshold;
    std::vector<DetectionResult> last_detections; // 存储最后一帧的检测结果
    struct sockaddr_in target_addr;

public:
    rkYolov5s(const std::string &model_path, int station_idx = 0);
    int init(rknn_context *ctx_in, bool isChild);
    rknn_context *get_pctx();
    cv::Mat infer(cv::Mat &ori_img);
    std::vector<DetectionResult> getLastDetections();
    ~rkYolov5s();

    int station_idx; // 0..g_station_count-1
};

#endif