#ifndef SERVOCONTROL_H
#define SERVOCONTROL_H

// C++标准库
#include <mutex>

// C标准库
#include <cstring>
#include <cstdio>

// 系统调用
#include <pthread.h>
#include <unistd.h>
#include <arpa/inet.h>

#define MAX_PACK_LEN 256        // 数据包最大长度
#define MAX_INSTRU_MSGLEN 256   // 指令数据最大长度
#define MAX_INSTRU_PACK_NUM 100 // 一次消息最大包数量

// 一体化终端交互指令
#define TERMINAL_CMD_MODE 0x06   // 模式
#define TERMINAL_CMD_SERVO 0x15  // 云台
#define TERMINAL_CMD_WEAPON 0x04 // 武器
#define TERMINAL_CMD_SCOUT 0x03  // 侦察
#define TERMINAL_CMD_TARGET 0x08 // 目标标注
#define TERMINAL_CMD_CALIB 0xff  // 调校

// 子包结构
typedef struct InstructData_
{
    unsigned char cType;
    unsigned char dataLen;
    unsigned char cCRC;
    unsigned char szData[MAX_INSTRU_MSGLEN];
} InstructData;

// 工作模式
enum WORKMODE
{
    WORKMODE_SILENT = 0x01,        // 静默模式
    WORKMODE_SEMIAUTOMATIC = 0x00, // 半自动作战
    WORKMODE_SEMIAUTONOMUS = 0x02  // 半自主作战
};

struct sRspState
{
    sRspState()
    {
        nWorkMode = 0x00;
        nServoState = 0x00;

        nShootPermit = 0;
        nSafetyState = 0;

        fServoPosH = 0.0;
        fServoPosV = 0.0;
    }

    unsigned char nWorkMode;    // 0:半自动 1:静默 2:半自主作
    unsigned char nServoState;  // 0:disable 1:enable
    unsigned char nShootPermit; // 0:forbid  1:permit
    unsigned char nSafetyState; // 0:close   1:open

    float fServoPosH; // degree
    float fServoPosV;
};

#ifndef MAX_STATIONS
#define MAX_STATIONS 3
#endif

struct Station
{
    char ip[32];
    int port;
    struct sockaddr_in addr;
    std::mutex mtx; // protect send to this station if needed
    int spdH;       // 每站自己的水平速度
    int spdV;       // 每站自己的垂直速度
    int absPosH;
    int absPosV;
    int AbsMove;
    

};

static Station g_stations[MAX_STATIONS];
static int g_station_count = 0;
static int g_udp_sock = -1;

// 初始化站点
int InitStations(const char *ips[], int count, int port);

// 返回 true 表示成功，posH/posV 填充
bool getStationServoPos(int station_idx, float &posH, float &posV);

// 在全局区增加：每个站的回应状态（替代单一 g_sRspState）
static sRspState g_sRspStates[MAX_STATIONS];

// 设置某站伺服速度
void setServoSpdMovForStation(int station_idx, int spdH, int spdV);
void setServoAbsMovForStation(int station_idx, int nPosMilH, int nPosMilV);

// UDP 发送线程
void *sendpack_thrd_func(void *arg);

void msSleep(long ms);

/* Init params
 * szStationIP: bx station IP
 * nStationPort: bx station port
 */
// void Init(char *szStationIP = (char *)"192.168.1.217", int nStationPort = 60000);

void setWorkMode(WORKMODE nWorkMode);

/* servo enable
 * nEnable: 0:disable 1:enable
 */
void setServoEnable(int nEnable);

/* 伺服复位,注意，伺服复位使能后，应发送无动作
 * nReset: 0:无动作 1:复位
 */
void setServoReset(int nReset);

/*速度模式调转
 * nSpdH:方位向速度（-100-100）
 * nSpdV:俯仰向速度（-100-100）
 */
void setServoSpdMov(int nSpdH, int nSpdV);

/*绝对位置调转
 * nPosMilH:方位向位置（mil）
 * nPosMilV:俯仰向位置（mil）
 */
void setServoAbsMov(int nPosMilH, int nPosMilV);

// Weapon
/*预击发
 * nPreShoot: 0:关 1:开
 */
void setPreShoot(unsigned char nPreShoot);

/*保险
 * nSafety: 0:关 1:开
 */
void setSafety(unsigned char nSafety);

/*击发
 * nShoot: 0x00:无动作 0xAA:击发 0x55:击发复位
 */
void setShoot(unsigned char nShoot);

/*
 * 零位设置
 * zeroH: 0:方位向无动作 1:方位向零位设置
 * zeroV: 0:俯仰向无动作 1:俯仰向零位设置
 */
void setServoZero(unsigned char zeroH, unsigned char zeroV);

// Scout
void setCrossMov(unsigned char nCrossCmd, unsigned char nCrossDir);

float getServoPosH();
float getServoPosV();
unsigned char getSafetyState();
unsigned char getServoState();
unsigned char getShootPermit();

#endif
