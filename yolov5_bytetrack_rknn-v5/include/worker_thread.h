
#ifndef WORKER_THREAD_H
#define WORKER_THREAD_H

#include <string>

std::string extract_ip_from_rtsp(const std::string &rtsp);
void worker_thread(int station_idx, const std::string &model_path, const std::string &rtsp_url);

#endif