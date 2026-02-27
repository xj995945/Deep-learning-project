#ifndef STATIONMANAGER_H
#define STATIONMANAGER_H

// station_manager.h
#pragma once
#include <atomic>
#include "servoControl.h"

struct StationStatus
{
    std::atomic<bool> locked{false}; // 是否锁定目标
    std::atomic<int> spdH{0};        // 当前水平速度（-100..100 假设）
    std::atomic<int> spdV{0};        // 当前垂直速度
};

extern StationStatus g_station_status[MAX_STATIONS];

void setServoSpdMovForStationWrapper(int station_idx, int spdH, int spdV);

#endif
