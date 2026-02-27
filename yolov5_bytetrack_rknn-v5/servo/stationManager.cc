// station_manager.cpp
#include "stationManager.h"

extern void setServoSpdMovForStation(int station_idx, int spdH, int spdV);

// 全局数组定义
StationStatus g_station_status[MAX_STATIONS];

void setServoSpdMovForStationWrapper(int station_idx, int spdH, int spdV)
{
    if (station_idx < 0 || station_idx >= MAX_STATIONS)
        return;

    // 如果该站当前处于绝对移动模式（AbsMove == 1），则跳过速度下发，避免覆盖绝对位置
    if (g_stations[station_idx].AbsMove == 1)
    {
        printf("[setServoSpdMovForStationWrapper] station=%d AbsMove==1, skip speed set spdH=%d spdV=%d\n",
               station_idx, spdH, spdV);
        return;
    }

    // 避免重复下发同一速度
    int prevH = g_station_status[station_idx].spdH.load();
    int prevV = g_station_status[station_idx].spdV.load();
    if (prevH == spdH && prevV == spdV)
        return;

    setServoSpdMovForStation(station_idx, spdH, spdV);
    g_station_status[station_idx].spdH.store(spdH);
    g_station_status[station_idx].spdV.store(spdV);
}
