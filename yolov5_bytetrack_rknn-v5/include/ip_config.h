// ip_config.h
#ifndef IP_CONFIG_H
#define IP_CONFIG_H

// ================= 网络配置 (RK3588 ) =================

// PC 端 IP
// static const char *PC_IP_ADDR = "192.168.2.2";
static const char *PC_IP_ADDR = "192.168.2.255"; // 广播地址

// 接收 PC 指令的端口
static const int RK_LISTEN_PORT = 60000;

// 发送给 PC 的目标端口 (PC 端 LANCommandCenter 监听的端口)
static const int PC_LISTEN_PORT = 8080;

// 便携站 IP
static const char *STATION1_IP_ADDR = "192.168.2.217";
static const char *STATION2_IP_ADDR = "192.168.2.218";

#endif // IP_CONFIG_H