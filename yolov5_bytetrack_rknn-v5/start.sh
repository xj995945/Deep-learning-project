# #!/bin/bash
# # 文件位置: .../install/rknn_yolov5_demo_Linux/start.sh

# # ================= 1. 基础配置 =================
# WORK_DIR="/home/cat/track/yolov5_bytetrack_rknn-v5/install/rknn_yolov5_demo_Linux"

# cd "$WORK_DIR" || exit 1
# CURRENT_DIR=$(pwd)

# # ================= 2. [核心修复] 环境变量 =================

# # 1. 动态库路径 (LD_LIBRARY_PATH)
# # 包含：
# #   $CURRENT_DIR/lib         -> 你的 librga.so, librknnrt.so
# #   /usr/lib/aarch64-linux-gnu -> 系统的基础库
# export LD_LIBRARY_PATH=$CURRENT_DIR/lib:/usr/lib/aarch64-linux-gnu:/usr/lib:/lib:$LD_LIBRARY_PATH

# # 2. GStreamer 插件路径 (GST_PLUGIN_PATH)
# # 【关键】直接指向你刚才找到的路径！
# # 这样 mppvideodec 就能被找到了
# export GST_PLUGIN_PATH=/usr/lib/aarch64-linux-gnu/gstreamer-1.0:$GST_PLUGIN_PATH

# # 3. 运行时目录 (硬件编码器必须)
# if [ -z "$XDG_RUNTIME_DIR" ]; then
#     export XDG_RUNTIME_DIR="/run/user/$(id -u)"
# fi

# # 4. 调试开关
# # 如果正常运行了，可以注释掉这行；如果还有错，保留它看日志
# export GST_DEBUG=1

# # ================= 3. 摄像头与模型配置 =================
# CAM1_IP="192.168.2.217"
# CAM2_IP="192.168.2.218"

# APP_NAME="./rknn_yolov5_demo"
# MODEL_PATH="./model/RK3588/yolov5s-640-640.rknn"

# RTSP_1="rtsp://$CAM1_IP/live"
# RTSP_2="rtsp://$CAM2_IP/live"

# # ================= 4. 网络等待逻辑 =================
# echo "[Launcher] 步骤1: 等待 MediaMTX..."
# while ! nc -z 127.0.0.1 8554; do sleep 1; done
# echo "[Launcher] MediaMTX OK."

# echo "[Launcher] 步骤2: 等待摄像头网络..."
# # 使用 ping 等待，直到摄像头网络通畅
# until ping -c 1 -W 1 $CAM1_IP >/dev/null; do echo "Wait Cam1..."; sleep 1; done
# until ping -c 1 -W 1 $CAM2_IP >/dev/null; do echo "Wait Cam2..."; sleep 1; done
# echo "[Launcher] 摄像头网络通畅."

# # ================= 5. 启动守护进程 =================
# APP_PID=0
# echo "=== RK3588 Service Started ==="

# while true; do
#     if [ $APP_PID -ne 0 ] && kill -0 $APP_PID 2>/dev/null; then
#         sleep 3
#     else
#         echo "[Launcher] Starting App..."
        
#         # 使用 stdbuf 禁用缓冲，确保日志实时输出，防止管道阻塞
#         stdbuf -o0 -e0 $APP_NAME $MODEL_PATH "$RTSP_1" "$RTSP_2" &
        
#         APP_PID=$!
#         echo "[Launcher] PID: $APP_PID"
#         sleep 5
#     fi
# done

#!/bin/bash
# 文件位置: .../install/rknn_yolov5_demo_Linux/start.sh

# ================= 1. 基础配置 =================
WORK_DIR="/home/cat/track/yolov5_bytetrack_rknn-v5/install/rknn_yolov5_demo_Linux"

cd "$WORK_DIR" || exit 1
CURRENT_DIR=$(pwd)

# 只需要基础的库路径即可 (用于 yolov5 和 rga)
# 不需要再指定 GST_PLUGIN_PATH 了，因为 avdec_h264 是系统自带的
export LD_LIBRARY_PATH=$CURRENT_DIR/lib:/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH

# 运行时目录 (GStreamer 通用需求)
if [ -z "$XDG_RUNTIME_DIR" ]; then
    export XDG_RUNTIME_DIR="/run/user/$(id -u)"
fi

# 摄像头配置
CAM1_IP="192.168.2.217"
CAM2_IP="192.168.2.218"

APP_NAME="./rknn_yolov5_demo"
MODEL_PATH="./model/RK3588/yolov5s-640-640.rknn"

RTSP_1="rtsp://$CAM1_IP/live"
RTSP_2="rtsp://$CAM2_IP/live"

# ================= 2. 等待网络 =================
echo "[Launcher] 等待 MediaMTX..."
while ! nc -z 127.0.0.1 8554; do sleep 1; done
while ! nc -z 127.0.0.1 8556; do sleep 1; done

echo "[Launcher] 等待摄像头..."
until ping -c 1 -W 1 $CAM1_IP >/dev/null; do echo "Wait Cam1..."; sleep 1; done
until ping -c 1 -W 1 $CAM2_IP >/dev/null; do echo "Wait Cam2..."; sleep 1; done

# ================= 3. 启动 =================
APP_PID=0
echo "=== RK3588 Service (Software Decode) Started ==="

while true; do
    if [ $APP_PID -ne 0 ] && kill -0 $APP_PID 2>/dev/null; then
        sleep 3
    else
        echo "[Launcher] Starting App..."
        # 依然使用 stdbuf 保证日志实时
        stdbuf -o0 -e0 $APP_NAME $MODEL_PATH "$RTSP_1" "$RTSP_2" &
        APP_PID=$!
        sleep 5
    fi
done