#include "servoControl.h"
#include "station_guidance.h"
#include "stationManager.h"
#include "common.h"
#include "ip_config.h"
#include <string.h>
#include <pthread.h>
#include <signal.h>
#include <sys/time.h>
#include <stdio.h>
#include <sys/select.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <mutex>

WORKMODE g_nWorkMode = WORKMODE_SEMIAUTOMATIC;

unsigned char g_nServoEnable = 0x00; // 0x50 disable 0xF0 enable

int g_nServoReset = 0;

int g_bAbsMove = 0;
int g_nAbsMovH = 0;
int g_nAbsMovV = 0;

unsigned char g_nPreShoot = 0;
unsigned char g_nSafety = 0;
unsigned char g_nShoot = 0;

unsigned char g_nCrossCmd = 0;
unsigned char g_nCrossDir = 0;

bool composeInstructData(InstructData *pInstructData, unsigned char szData[], int nDataLen, unsigned char instruType)
{
    memset(pInstructData, 0, sizeof(InstructData));

    pInstructData->cType = instruType;
    pInstructData->dataLen = nDataLen + 3;

    int nInstrLen = 0;
    memcpy(pInstructData->szData, szData, nDataLen);

    unsigned char cCRC = 0;
    cCRC += instruType;
    cCRC += (nDataLen + 3);

    for (int i = 0; i < nDataLen; i++)
        cCRC += szData[i];

    pInstructData->cCRC = cCRC;

    return true;
}

int composePack(unsigned char szBufOut[], int nBufSize, InstructData *pInstruData, int nInstruNum, unsigned char packType)
{
    memset(szBufOut, 0, nBufSize);

    int nTotalLen = 0;
    szBufOut[nTotalLen++] = 0x5A;
    szBufOut[nTotalLen++] = 0xA5;
    szBufOut[nTotalLen++] = 0x00; // 帧长
    szBufOut[nTotalLen++] = 0x08; // 便携站遥测数据
    szBufOut[nTotalLen++] = 0x11; // 装备编号
    szBufOut[nTotalLen++] = packType;

    nTotalLen += 4; // 预留 机体（4）

    szBufOut[nTotalLen++] = g_nServoEnable; // 使能及自检（1）

    szBufOut[nTotalLen++] = nInstruNum;
    for (int i = 0; i < nInstruNum; i++)
    {
        szBufOut[nTotalLen++] = pInstruData[i].cType;
        szBufOut[nTotalLen++] = pInstruData[i].dataLen;
        memcpy(szBufOut + nTotalLen, pInstruData[i].szData, pInstruData[i].dataLen - 3);
        nTotalLen += (pInstruData[i].dataLen - 3);
        szBufOut[nTotalLen++] = pInstruData[i].cCRC;
    }

    nTotalLen += 3; // 2字节crc 1字节帧尾
    szBufOut[2] = nTotalLen;

    unsigned short nCRC = 0;
    for (int i = 0; i < nTotalLen - 5; i++)
        nCRC += szBufOut[i + 2];
    nCRC += 0xff;

    szBufOut[nTotalLen - 3] = nCRC & 0xff;
    szBufOut[nTotalLen - 2] = (nCRC >> 8) & 0xff;
    szBufOut[nTotalLen - 1] = 0xff;

    if (nTotalLen > MAX_PACK_LEN)
        nTotalLen = 0;
    return nTotalLen;
}

void sendS_pack1()
{
    for (int i = 0; i < g_station_count; ++i)
    {
        unsigned char szBufSend[MAX_PACK_LEN];
        InstructData sInstruData[6];
        int validPackNo = 0;

        for (int k = 0; k < 6; k++)
            memset(&sInstruData[k], 0, sizeof(InstructData));

        // ---------- 模式 ----------
        int nDataLen = 0;
        memset(szBufSend, 0, sizeof(szBufSend));
        nDataLen += 2; // reserve
        szBufSend[nDataLen++] = g_nWorkMode << 4;
        nDataLen++; // reserve
        composeInstructData(&sInstruData[validPackNo++], szBufSend, nDataLen, TERMINAL_CMD_MODE);

        // ---------- 云台 ----------
        nDataLen = 0;
        memset(szBufSend, 0, sizeof(szBufSend));
        szBufSend[nDataLen++] = 0x02; // 云台数量

        int posH = g_stations[i].spdH * 100;
        int posV = g_stations[i].spdV * 100;

        if (g_stations[i].AbsMove == 0)
        {
            // 水平方向
            if (posH < 0)
            {
                szBufSend[nDataLen + 1] = 0x01 << 6;
                posH = -posH;
            }
            szBufSend[nDataLen] = posH & 0xff;
            szBufSend[nDataLen + 1] |= posH >> 8;
            nDataLen += 2;

            // 垂直方向
            if (posV < 0)
            {
                szBufSend[nDataLen + 1] = 0x01 << 6;
                posV = -posV;
            }
            szBufSend[nDataLen] = posV & 0xff;
            szBufSend[nDataLen + 1] |= posV >> 8;
            nDataLen += 2;
        }
        else
        {
            // 绝对位置模式（保留你的原逻辑）
            short absMovMilH = (short)g_stations[i].absPosH;
            short absMovMilV = (short)g_stations[i].absPosV;
            if (absMovMilH < 0)
            {
                szBufSend[nDataLen + 1] = 0x03 << 6;
                absMovMilH = 0 - absMovMilH;
            }
            else
            {
                szBufSend[nDataLen + 1] = 0x02 << 6;
            }
            szBufSend[nDataLen] = absMovMilH & 0xff;
            szBufSend[nDataLen + 1] |= absMovMilH >> 8;
            nDataLen += 2;

            if (absMovMilV < 0)
            {
                szBufSend[nDataLen + 1] = 0x03 << 6;
                absMovMilV = 0 - absMovMilV;
            }
            else
            {
                szBufSend[nDataLen + 1] = 0x02 << 6;
            }
            szBufSend[nDataLen] = absMovMilV & 0xff;
            szBufSend[nDataLen + 1] |= absMovMilV >> 8;
            nDataLen += 2;
        }

        szBufSend[nDataLen++] = g_nServoReset; // reset
        // szBufSend[nDataLen++] = 0x01;
        szBufSend[nDataLen++] = 0x11;
        composeInstructData(&sInstruData[validPackNo++], szBufSend, nDataLen, TERMINAL_CMD_SERVO);

        // ---------- 武器快捷 ----------
        nDataLen = 0;
        memset(szBufSend, 0, sizeof(szBufSend));
        szBufSend[nDataLen++] = 0x01; // 武器
        szBufSend[nDataLen++] = 0x01; // 武器选取
        szBufSend[nDataLen++] = 0x00;

        unsigned char safetyValue = 0x00;
        if (g_nPreShoot == 0x00)
            safetyValue = 0x00;
        else if (g_nPreShoot == 0x01)
            safetyValue = 0x03 << 2;

        if (g_nSafety == 0x00)
            safetyValue |= 0x00;
        else if (g_nSafety == 0x01)
            safetyValue |= 0x03;
        szBufSend[nDataLen++] = safetyValue;

        nDataLen += 2; // reserve
        szBufSend[nDataLen++] = g_nShoot;

        composeInstructData(&sInstruData[validPackNo++], szBufSend, nDataLen, TERMINAL_CMD_WEAPON);

        // ---------- 侦察 ----------
        nDataLen = 0;
        memset(szBufSend, 0, sizeof(szBufSend));
        nDataLen += 4; // 图像/图像选取/开关/光学调整

        szBufSend[nDataLen++] = (g_nCrossCmd << 6) | (g_nCrossDir << 4);
        nDataLen++; // 目标识别

        composeInstructData(&sInstruData[validPackNo++], szBufSend, nDataLen, TERMINAL_CMD_SCOUT);

        // ---------- 打包并发送 ----------
        int nTotalBytes = composePack(szBufSend, sizeof(szBufSend),
                                      sInstruData, validPackNo, TERMINAL_CMD_SERVO);

        // {
        //     std::lock_guard<std::mutex> lk(g_stations[i].mtx);
        //     sendto(g_udp_sock, (char *)szBufSend, nTotalBytes, 0,
        //            (sockaddr *)&g_stations[i].addr, sizeof(g_stations[i].addr));
        // }
        // 1. 获取地址副本（虽然 addr 变化概率极小，但为了严谨）
        struct sockaddr_in target_addr;
        {
            std::lock_guard<std::mutex> lk(g_stations[i].mtx);
            target_addr = g_stations[i].addr;
        } // 锁在这里就释放了！

        // 2. 无锁发送 (即使网络卡顿，也不会阻塞推理线程)
        if (g_udp_sock >= 0)
        {
            sendto(g_udp_sock, (char *)szBufSend, nTotalBytes, 0,
                   (sockaddr *)&target_addr, sizeof(target_addr));
        }
    }
}

bool checkCRC(unsigned char *szMsgBuf, int nLen)
{
    unsigned short nCRC = 0;
    for (int i = 2; i < nLen - 3; i++)
    {
        nCRC += szMsgBuf[i];
    }
    nCRC += 0x00ff;

    unsigned char cLow = nCRC & 0xff;
    unsigned char cHigh = nCRC >> 8;

    if (szMsgBuf[nLen - 3] == cLow && szMsgBuf[nLen - 2] == cHigh)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool checkPackValid(unsigned char *szMsgBuf, int nLen)
{
    if (nLen < 15)
    {
        return false;
    }

    bool bCRCValid = checkCRC(szMsgBuf, nLen);
    if (szMsgBuf[0] != 0x5a || szMsgBuf[1] != 0xa5 || szMsgBuf[nLen - 1] != 0xff || nLen != szMsgBuf[2] || bCRCValid == false)
        return false;

    return true;
}

int parsePack(unsigned char *szMsgBuf, int nLen, InstructData *pInstru, int *pInstruNum, sRspState *pRspState)
{
    if (pInstru == NULL || pInstruNum == NULL || pRspState == NULL)
        return 0;

    memset(pInstru, 0, sizeof(InstructData) * (*pInstruNum));
    *pInstruNum = 1;

    int nRealInstruPackCnt = 0;
    if (checkPackValid(szMsgBuf, nLen) == false)
    {
        return 0;
    }
    // 不是便携站数据，丢弃
    if (szMsgBuf[3] != 0x08)
        return 0;

    int nIndex = 6;

    nIndex += 4;                                    // 机体预留
    unsigned char cServoState = szMsgBuf[nIndex++]; // 机体状态
    if (cServoState == 0xF0)
        pRspState->nServoState = 1;
    else
        pRspState->nServoState = 0;

    nIndex++; // 剩余电量
    nIndex += 15;

    for (int i = 0; i < (*pInstruNum); i++)
    {
        pInstru[nRealInstruPackCnt].cType = szMsgBuf[nIndex];
        pInstru[nRealInstruPackCnt].dataLen = szMsgBuf[nIndex + 1];
        int instruLen = pInstru[nRealInstruPackCnt].dataLen;
        char cCRC = 0;
        for (int j = 0; j < instruLen - 1; j++)
            cCRC += (char)szMsgBuf[nIndex + j];

        if (cCRC == (char)szMsgBuf[nIndex + instruLen - 1] && instruLen <= MAX_INSTRU_MSGLEN)
        {
            pInstru[nRealInstruPackCnt].cCRC = cCRC;
            memcpy(pInstru[nRealInstruPackCnt].szData, szMsgBuf + nIndex + 2, instruLen - 3);
            nIndex += instruLen;
            nRealInstruPackCnt++;
        }
        else // 超过最大指令包长度，直接跳过
        {
            nIndex += instruLen;
            memset(&pInstru[nRealInstruPackCnt], 0, sizeof(InstructData));
        }
    }

    *pInstruNum = nRealInstruPackCnt;
    return nRealInstruPackCnt;
}

// 根据来源地址查找 station index（精确匹配 IP）
static int findStationIndexByAddr(const struct sockaddr_in *addr)
{
    if (addr == NULL)
        return -1;
    for (int i = 0; i < g_station_count; ++i)
    {
        // 比较 IPv4 地址（按 sin_addr.s_addr）
        if (g_stations[i].addr.sin_addr.s_addr == addr->sin_addr.s_addr)
            return i;
    }
    return -1;
}

// helper: print bytes in hex
static void print_hex(const unsigned char *buf, int len)
{
    for (int i = 0; i < len; ++i)
    {
        printf("%02X ", buf[i]);
        if ((i & 0x1f) == 0x1f)
            printf("\n"); // 每行 32 字节换行，便于阅读
    }
    if (len > 0)
        printf("\n");
}

void *recvpack_thrd_func(void *arg)
{
    sockaddr_in serverResponseAddr;
    socklen_t serverResponseAddrLen = sizeof(serverResponseAddr);

    unsigned char szRecvBuf[1024];
    int nRecvLen = 0;

    // 为每个站保存历史值，防止不同站互相覆盖
    unsigned char lastServoState[MAX_STATIONS];
    unsigned char lastWorkMode[MAX_STATIONS];
    unsigned char lastShoot[MAX_STATIONS];
    unsigned char lastSafety[MAX_STATIONS];
    int lastPosH[MAX_STATIONS];
    int lastPosV[MAX_STATIONS];

    for (int i = 0; i < MAX_STATIONS; i++)
    {
        lastServoState[i] = 0xFF;
        lastWorkMode[i] = 0xFF;
        lastShoot[i] = 0xFF;
        lastSafety[i] = 0xFF;
        lastPosH[i] = 0x7FFFFFFF;
        lastPosV[i] = 0x7FFFFFFF;
    }

    // Qt 程序监听8080 端口
    struct sockaddr_in pc_addr;
    memset(&pc_addr, 0, sizeof(pc_addr));
    pc_addr.sin_family = AF_INET;
    pc_addr.sin_port = htons(PC_LISTEN_PORT);        // PC 监听端口
    pc_addr.sin_addr.s_addr = inet_addr(PC_IP_ADDR); // PC 的 IP
    // ============================

    printf("enter recv thread\n");

    while (true)
    {
        serverResponseAddrLen = sizeof(serverResponseAddr);
        nRecvLen = recvfrom(g_udp_sock, szRecvBuf, sizeof(szRecvBuf), 0, (struct sockaddr *)&serverResponseAddr, &serverResponseAddrLen);
        if (nRecvLen > 0)
        {
            // === 解析 PC 端发来的控制协议 (10 16 EF) ===
            // 协议格式: 10 16 EF [Cmd] [Data...] [CheckSum]
            if (nRecvLen >= 6 && szRecvBuf[0] == 0x10 && szRecvBuf[1] == 0x16 && szRecvBuf[2] == 0xEF)
            {
                printf("Received PC command packet, len=%d\n", nRecvLen);
                unsigned char cmd = szRecvBuf[3];
                // 速度控制指令 (0x30)
                // 协议: 10 16 EF 30 [StationID] [Speed] [CRC]
                switch (cmd)
                {
                case 0x30:
                {
                    int station_id = szRecvBuf[4];
                    int speed = (signed char)szRecvBuf[5]; // 获取速度值 (-50-50)

                    // 安全检查
                    if (station_id >= 0 && station_id < g_station_count)
                    {
                        // 参数: (站号, 水平速度, 垂直速度)
                        // 这里假设搜索只转水平，垂直给 0
                        setServoSpdMovForStation(station_id, speed, 0);

                        printf("[CMD] Station %d Speed set to %d\n", station_id, speed);
                    }
                    break;
                }
                case 0x31:
                {
                    // 系统功能开关 (Type: 1=识别 2=跟踪, State: 1=开 0=关)
                    int type = szRecvBuf[4];
                    int state = szRecvBuf[5];

                    if (type == 1)
                    {
                        g_enable_recog.store(state == 1);
                        printf("[CMD] Recognition set to %d\n", state);
                    }
                    else if (type == 2)
                    {
                        g_enable_track.store(state == 1);
                        // 如果关闭跟踪，强制让舵机停下来
                        if (state == 0)
                        {
                            setServoSpdMovForStation(0, 0, 0);
                            setServoSpdMovForStation(1, 0, 0);
                        }
                        printf("[CMD] Tracking set to %d\n", state);
                    }
                    break;
                }
                case 0x32:
                {
                    int station_id = szRecvBuf[4];
                    // 解析 ID (小端序)
                    int id = (unsigned char)szRecvBuf[5] + ((unsigned char)szRecvBuf[6] << 8);

                    if (station_id >= 0 && station_id < g_station_count)
                    {
                        // 更新全局变量，通知 worker_thread
                        g_locked_track_ids[station_id].store(id);

                        // 同时强制开启“跟踪模式”
                        g_enable_track.store(true);

                        printf("[CMD] Station %d Lock ID set to: %d\n", station_id, id);
                    }
                    break;
                }
                case 0x33:
                {
                    int conf_int = szRecvBuf[4];
                    int iou_int = szRecvBuf[5];

                    float new_conf = (float)conf_int / 100.0f;
                    float new_iou = (float)iou_int / 100.0f;

                    // 更新全局变量
                    g_conf_threshold.store(new_conf);
                    g_nms_threshold.store(new_iou);

                    printf("[CMD] Update Thresholds -> Conf: %.2f, IoU: %.2f\n", new_conf, new_iou);
                    break;
                }
                case 0x34:
                {
                    printf("[CMD] System Reset Triggered...\n");

                    // 调用 cancel_guidance 以清除 AbsMove 标志和引导状态机
                    station_guidance::cancel_guidance();

                    // 清除所有站点的锁定 ID
                    g_locked_track_ids[0].store(-1);
                    g_locked_track_ids[1].store(-1);
                    g_station_status[0].locked.store(false);
                    g_station_status[1].locked.store(false);

                    // 清除一次性引导完成标志，允许下次重新引导
                    g_guidance_done.store(false);

                    // 强制清除底层指令缓存
                    for (int i = 0; i < g_station_count; ++i)
                    {
                        setServoSpdMovForStation(i, 0, 0); // 速度归零
                        g_stations[i].AbsMove = 0;         // 强制切回速度模式
                    }

                    // 复位
                    g_nServoReset = 0x01;
                    usleep(500000); // 500ms
                    g_nServoReset = 0x00;

                    printf("[CMD] System Reset Finished (Software State Cleared).\n");
                    break;
                }
                case 0x35:
                {
                    // 协议: 10 16 EF 35 [Enable] [Spd1] [Spd2] [CRC]
                    bool enable = (szRecvBuf[4] == 0x01);
                    int spd1 = (signed char)szRecvBuf[5]; // 强转为有符号
                    int spd2 = (signed char)szRecvBuf[6];

                    saveConfig(enable, spd1, spd2);

                    // ==========================================
                    // 防止重启期间舵机乱转
                    for (int i = 0; i < g_station_count; ++i)
                    {
                        setServoSpdMovForStation(i, 0, 0);
                    }
                    // ==========================================

                    // 延迟确保指令下发和文件写入
                    usleep(200000); // 建议改大一点点，比如 200ms

                    // ==========================================
                    // 显式刷新标准输出，确保日志打印完整
                    fflush(stdout);
                    // ==========================================

                    // 自杀退出 -> start.sh 会立刻重启
                    exit(0);
                    break;
                }
                case 0x36:
                {
                    if (nRecvLen < 9)
                        break; // 长度检查

                    // 1. 解析 Kp (2字节)
                    unsigned short kp_raw = (unsigned char)szRecvBuf[4] + ((unsigned char)szRecvBuf[5] << 8);
                    // 2. 解析 Kd (2字节)
                    unsigned short kd_raw = (unsigned char)szRecvBuf[6] + ((unsigned char)szRecvBuf[7] << 8);

                    // 3. 还原为 float
                    float new_kp = (float)kp_raw / 10.0f;
                    float new_kd = (float)kd_raw / 10.0f;

                    g_track_kp.store(new_kp);
                    g_track_kd.store(new_kd);

                    printf("[CMD] Update PID -> Kp: %.1f, Kd: %.1f\n", new_kp, new_kd);
                    break;
                }
                case 0x37: // 处理协同参数设置指令
                {
                    // 检查长度: 头(4) + 数据(4) + CRC(1) = 9
                    if (nRecvLen < 9)
                        break;

                    // 解析水平距离 (2字节)
                    unsigned short h_raw = (unsigned char)szRecvBuf[4] + ((unsigned char)szRecvBuf[5] << 8);
                    // 解析垂直距离 (2字节)
                    unsigned short v_raw = (unsigned char)szRecvBuf[6] + ((unsigned char)szRecvBuf[7] << 8);

                    float h_m = (float)h_raw / 1000.0f;
                    float v_m = (float)v_raw / 1000.0f;

                    // 调用 guidance 模块更新参数
                    station_guidance::update_baseline(h_m, v_m);
                    break;
                }
                case 0x38: // 双轴摇杆指令
                {
                    if (nRecvLen < 7)
                        break;
                    int station_id = szRecvBuf[4];
                    int spdH = (signed char)szRecvBuf[5];
                    int spdV = (signed char)szRecvBuf[6];

                    // 调用底层控制
                    setServoSpdMovForStation(station_id, spdH, spdV);
                    break;
                }
                default:
                    break;
                }
                continue; // 处理完 PC 指令后直接下一次循环，不走 RK 协议解析
            }

            // get source ip:port string
            char ipbuf[64] = {0};
            inet_ntop(AF_INET, &(serverResponseAddr.sin_addr), ipbuf, sizeof(ipbuf));
            int src_port = ntohs(serverResponseAddr.sin_port);

            int station_idx = findStationIndexByAddr(&serverResponseAddr);
            if (station_idx < 0 || station_idx >= g_station_count)
            {
                printf("recv from unknown ip %s:%d, len=%d -- ignored\n", ipbuf, src_port, nRecvLen);
                // 仍然打印原始包，便于排查
                printf("RAW(%s:%d):\n", ipbuf, src_port);
                print_hex(szRecvBuf, nRecvLen);
                continue;
            }

            // 打印来源及原始数据（hex）
            // printf("RECV from station[%d] %s:%d  len=%d\n", station_idx, ipbuf, src_port, nRecvLen);
            // print_hex(szRecvBuf, nRecvLen);

            InstructData sInstruDataRecv[MAX_INSTRU_PACK_NUM];
            int nInstruPackCnt = MAX_INSTRU_PACK_NUM;
            // 解析到对应站点的状态结构
            nInstruPackCnt = parsePack(szRecvBuf, nRecvLen, sInstruDataRecv, &nInstruPackCnt, &g_sRspStates[station_idx]);

            if (nInstruPackCnt > 0)
            {
                // 获取当前站点的最新状态（线程安全读取）
                sRspState st;
                {
                    std::lock_guard<std::mutex> lk(g_stations[station_idx].mtx);
                    st = g_sRspStates[station_idx];
                }

                // === 调试打印===
                // printf("Station[%d] Parsed Angle: H=%.2f V=%.2f\n", station_idx, st.fServoPosH, st.fServoPosV);

                // === 组装发送给 PC ===
                // 协议格式: 10 16 EF 15 [StationID] [H_Low] [H_High] [V_Low] [V_High] [CheckSum]
                unsigned char sendBuf[32];
                int idx = 0;
                sendBuf[idx++] = 0x10;
                sendBuf[idx++] = 0x16;
                sendBuf[idx++] = 0xef;
                sendBuf[idx++] = 0x15;                       // 0x15 代表角度更新指令
                sendBuf[idx++] = (unsigned char)station_idx; // 站号 (0或1)

                // 将浮点数角度转为整数发送 (保留2位小数精度)
                short valH = (short)(st.fServoPosH * 100);
                short valV = (short)(st.fServoPosV * 100);

                // 填入水平角度 (低位在前)
                sendBuf[idx++] = valH & 0xFF;
                sendBuf[idx++] = (valH >> 8) & 0xFF;

                // 填入俯仰角度
                sendBuf[idx++] = valV & 0xFF;
                sendBuf[idx++] = (valV >> 8) & 0xFF;

                // 计算校验和
                unsigned char sum = 0;
                for (int k = 0; k < idx; k++)
                    sum ^= sendBuf[k];
                sendBuf[idx++] = sum;

                sendto(g_udp_sock, sendBuf, idx, 0, (struct sockaddr *)&pc_addr, sizeof(pc_addr));
            }

            // 更新 last* 缓存并按类型处理
            if (nInstruPackCnt >= 1)
            {
                if (g_sRspStates[station_idx].nServoState != lastServoState[station_idx])
                {
                    lastServoState[station_idx] = g_sRspStates[station_idx].nServoState;
                    printf("station %d cur servo state 0x%02x\n", station_idx, lastServoState[station_idx]);
                }

                if (sInstruDataRecv[0].cType == 0x06) // 模式
                {
                    unsigned char curWorkMode = sInstruDataRecv[0].szData[2];
                    if (lastWorkMode[station_idx] != curWorkMode)
                    {
                        lastWorkMode[station_idx] = curWorkMode;
                        std::lock_guard<std::mutex> lk(g_stations[station_idx].mtx);
                        g_sRspStates[station_idx].nWorkMode = curWorkMode >> 4;
                    }
                }
                else if (sInstruDataRecv[0].cType == 0x15) // 云台
                {
                    int posH = 0;
                    posH = sInstruDataRecv[0].szData[4] * 256;
                    posH += sInstruDataRecv[0].szData[3];

                    if (lastPosH[station_idx] != posH)
                    {
                        lastPosH[station_idx] = posH;
                        int calcH = posH;
                        if (posH <= 18000)
                            calcH = 0 - posH;
                        else
                            calcH = 36000 - posH;

                        std::lock_guard<std::mutex> lk(g_stations[station_idx].mtx);
                        g_sRspStates[station_idx].fServoPosH = calcH / 100.0f;
                    }

                    int posV = 0;
                    posV = sInstruDataRecv[0].szData[8] * 256;
                    posV += sInstruDataRecv[0].szData[7];

                    if (lastPosV[station_idx] != posV)
                    {
                        lastPosV[station_idx] = posV;

                        int calcV = posV;
                        if (sInstruDataRecv[0].szData[5] >> 6 == 0x01)
                            calcV = 0 - posV;

                        std::lock_guard<std::mutex> lk(g_stations[station_idx].mtx);
                        g_sRspStates[station_idx].fServoPosV = calcV / 100.0f;
                    }
                }
                else if (sInstruDataRecv[0].cType == 0x04) // 武器
                {
                    unsigned char curShoot = sInstruDataRecv[0].szData[2] & 0x0F;
                    if (lastShoot[station_idx] != curShoot)
                    {
                        lastShoot[station_idx] = curShoot;
                        std::lock_guard<std::mutex> lk(g_stations[station_idx].mtx);
                        if (curShoot == 0x05)
                            g_sRspStates[station_idx].nShootPermit = 1;
                        else
                            g_sRspStates[station_idx].nShootPermit = 0;
                    }

                    unsigned char curSafety = sInstruDataRecv[0].szData[3] & 0x03;
                    if (lastSafety[station_idx] != curSafety)
                    {
                        lastSafety[station_idx] = curSafety;
                        std::lock_guard<std::mutex> lk(g_stations[station_idx].mtx);
                        if (curSafety == 0x03)
                            g_sRspStates[station_idx].nSafetyState = 1;
                        else
                            g_sRspStates[station_idx].nSafetyState = 0;
                    }
                }
                else if (sInstruDataRecv[0].cType == 0xff) // 调校
                {
                    short shootBoundaryHL = 0;
                    short shootBoundaryHR = 0;
                    short shootBoundaryVL = 0;
                    short shootBoundaryVH = 0;

                    int nLen = 8;

                    shootBoundaryVL = (sInstruDataRecv[0].szData[nLen + 1] & 0x3F) * 256;
                    shootBoundaryVL += sInstruDataRecv[0].szData[nLen];

                    if (sInstruDataRecv[0].szData[nLen + 1] >> 6 == 0x01)
                        shootBoundaryVL = 0 - shootBoundaryVL;
                    nLen += 2;

                    shootBoundaryVH = (sInstruDataRecv[0].szData[nLen + 1] & 0x3F) * 256;
                    shootBoundaryVH += sInstruDataRecv[0].szData[nLen];

                    if (sInstruDataRecv[0].szData[nLen + 1] >> 6 == 0x01)
                        shootBoundaryVH = 0 - shootBoundaryVH;
                    nLen += 2;

                    shootBoundaryHL = (sInstruDataRecv[0].szData[nLen + 1] & 0x3F) * 256;
                    shootBoundaryHL += sInstruDataRecv[0].szData[nLen];
                    if (sInstruDataRecv[0].szData[nLen + 1] >> 6 == 0x01)
                        shootBoundaryHL = 0 - shootBoundaryHL;
                    nLen += 2;

                    shootBoundaryHR = (sInstruDataRecv[0].szData[nLen + 1] & 0x3F) * 256;
                    shootBoundaryHR += sInstruDataRecv[0].szData[nLen];
                    if (sInstruDataRecv[0].szData[nLen + 1] >> 6 == 0x01)
                        shootBoundaryHR = 0 - shootBoundaryHR;
                    nLen += 2;
                }
            }
        } // if recvlen > 0
    } // while true
}

bool getStationServoPos(int station_idx, float &posH, float &posV)
{
    if (station_idx < 0 || station_idx >= g_station_count)
        return false;
    std::lock_guard<std::mutex> lk(g_stations[station_idx].mtx);
    posH = g_sRspStates[station_idx].fServoPosH;
    posV = g_sRspStates[station_idx].fServoPosV;
    return true;
}

void msSleep(long ms)
{
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = ms * 1000;
    select(0, NULL, NULL, NULL, &tv);
}

void *sendpack_thrd_func(void *arg)
{
    while (1)
    {
        sendS_pack1();
        msSleep(20);
    }
}

// 初始化 station 列表；ips 数组长度为 count，port 相同
int InitStations(const char *ips[], int count, int port)
{
    for (int i = 0; i < 2; ++i)
    {
        g_locked_track_ids[i].store(-1);
    }
    if (count > MAX_STATIONS)
        count = MAX_STATIONS;
    g_station_count = count;

    // 如果之前有 socket，先关闭
    if (g_udp_sock >= 0)
    {
        close(g_udp_sock);
        g_udp_sock = -1;
    }

    // 创建 UDP socket
    g_udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_udp_sock < 0)
    {
        perror("socket");
        return -1;
    }

    // 允许重用地址
    int opt = 1;
    if (setsockopt(g_udp_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
    {
        perror("setsockopt SO_REUSEADDR");
        // not fatal, continue
    }

    // === 开启广播权限 ===
    int broadcast = 1;
    if (setsockopt(g_udp_sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0)
    {
        perror("setsockopt SO_BROADCAST in InitStations");
    }

    // 绑定到本地任意网卡的指定端口，这样对端回复会回到该端口并被 recvfrom 接收
    struct sockaddr_in localAddr;
    memset(&localAddr, 0, sizeof(localAddr));
    localAddr.sin_family = AF_INET;
    localAddr.sin_addr.s_addr = htonl(INADDR_ANY); // 接收任意本地网卡
    localAddr.sin_port = htons(port);

    if (bind(g_udp_sock, (struct sockaddr *)&localAddr, sizeof(localAddr)) < 0)
    {
        perror("bind");
        close(g_udp_sock);
        g_udp_sock = -1;
        return -1;
    }

    for (int i = 0; i < g_station_count; ++i)
    {
        strncpy(g_stations[i].ip, ips[i], sizeof(g_stations[i].ip) - 1);
        g_stations[i].ip[sizeof(g_stations[i].ip) - 1] = '\0';
        g_stations[i].port = port;
        memset(&g_stations[i].addr, 0, sizeof(g_stations[i].addr));
        g_stations[i].addr.sin_family = AF_INET;
        g_stations[i].addr.sin_port = htons(port);
        if (inet_pton(AF_INET, g_stations[i].ip, &g_stations[i].addr.sin_addr) != 1)
        {
            fprintf(stderr, "Invalid station IP: %s\n", ips[i]);
            // 继续，但该地址会是 0.0.0.0，findStationIndexByAddr 会匹配失败
        }
    }

    setServoEnable(1);

    pthread_t tid_recv;
    pthread_create(&tid_recv, NULL, recvpack_thrd_func, NULL);
    pthread_t tid_send;
    pthread_create(&tid_send, NULL, sendpack_thrd_func, NULL);

    return 0;
}

void setServoSpdMovForStation(int station_idx, int nSpdH, int nSpdV)
{
    g_stations[station_idx].AbsMove = 0;
    if (station_idx < 0 || station_idx >= g_station_count)
        return;
    if (g_udp_sock < 0)
        return;

    // 限幅
    if (nSpdH > 100)
        nSpdH = 100;
    else if (nSpdH < -100)
        nSpdH = -100;
    if (nSpdV > 100)
        nSpdV = 100;
    else if (nSpdV < -100)
        nSpdV = -100;

    // 存到 station 自己的字段
    g_stations[station_idx].spdH = nSpdH;
    g_stations[station_idx].spdV = nSpdV;
}

void setServoAbsMovForStation(int station_idx, int nPosMilH, int nPosMilV)
{
    if (station_idx < 0 || station_idx >= g_station_count)
        return;
    if (g_udp_sock < 0)
        return;

    g_stations[station_idx].AbsMove = 1; // 表示当前是绝对位置模式

    // 存到 station 自己的字段
    g_stations[station_idx].absPosH = nPosMilH;
    g_stations[station_idx].absPosV = nPosMilV;

    // 清零对应站的速度控制
    g_stations[station_idx].spdH = 0;
    g_stations[station_idx].spdV = 0;
}

void setServoEnable(int nEnable)
{
    if (nEnable == 0)
        g_nServoEnable = 0x50;
    else
        g_nServoEnable = 0xF0;
}
