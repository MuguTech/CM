#ifndef __LOG_H__
#define __LOG_H__


#include "../../Main/Common.h"
#include <spdlog/spdlog.h>
//#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/details/file_helper.h>
//#include <spdlog/details/null_mutex.h>
#include <spdlog/details/synchronous_factory.h>
#include <spdlog/cfg/env.h>

#include <list>


void log_init();

unsigned long long get_cur_time_ns();
double get_cur_time_ms();

extern std::shared_ptr<spdlog::logger> log_compnt_mngr;
extern std::shared_ptr<spdlog::logger> mod_traffic_simulation;
extern std::shared_ptr<spdlog::logger> log_evaluation;



class frame_rate_tj
{
private:
    unsigned long long start_time;
    unsigned long long end_time;
    int inter_time;
    int tmp_index;

public:
    frame_rate_tj(int intervel = 60);
    ~frame_rate_tj() = default;
    void print();

};




#endif

