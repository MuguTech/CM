#include "log.h"

#include <time.h>



/*************************** 定义spdlog对象，使编译通过   start ****************************/
#ifndef V1000HZ
    std::shared_ptr<spdlog::logger> mod_road_system;
#endif
std::shared_ptr<spdlog::logger> mod_traffic_simulation;
std::shared_ptr<spdlog::logger> log_compnt_mngr;
std::shared_ptr<spdlog::logger> log_evaluation;

void log_init()
{
    /* 日志文件最大50M，最多循环生产/利用3个日志文件 */
    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("component_manager_log.txt", 1048576 * 50, 3, true);
    /* 实例化logger */
#ifndef V1000HZ
    mod_road_system = std::make_shared<spdlog::logger>("mod_road_system", file_sink);
#endif
    mod_traffic_simulation = std::make_shared<spdlog::logger>("mod_traffic_simulation", file_sink);
    log_compnt_mngr = std::make_shared<spdlog::logger>("CM", file_sink);
    log_evaluation = std::make_shared<spdlog::logger>("EVA", file_sink);

    /* 将每个logger实例注册到spdlog模块 */
#ifndef V1000HZ
    spdlog::register_logger(mod_road_system);
#endif
    spdlog::register_logger(mod_traffic_simulation);
    spdlog::register_logger(log_compnt_mngr);
    spdlog::register_logger(log_evaluation);

    /* spdlog::flush_on(spdlog::level::info); */
    spdlog::flush_every(std::chrono::seconds(5));

    spdlog::flush_on(spdlog::level::info);

    /* 读取环境变量“SPDLOG_LEVEL” */
    spdlog::cfg::load_env_levels();

    /* 每行日志的前缀内容：[时:分:秒.毫秒],[logger名],[log level缩写],[线程ID],日志内容 */
    spdlog::set_pattern("[%H:%M:%S.%e],[%n],[%^%L%$],[%t],%v");

}
/*************************** spd定义log对象，使编译通过   end ****************************/



/*************************** 定义output_log，使编译通过   start ****************************/
//此函数为空，因为该打印的CriticalLog在SimPro端已经打印过了，ComponentManager不需要重复打印
void output_log(S_OUTPUT_LOG * slog)
{
}
/*************************** 定义output_log，使编译通过   end ****************************/





unsigned long long get_cur_time_ns()
{
    struct timespec timeStamp{0, 0};
    clock_gettime(CLOCK_REALTIME, &timeStamp);
    return timeStamp.tv_sec * 1000 * 1000 * 1000 + timeStamp.tv_nsec;
}


double get_cur_time_ms()
{
    struct timespec timeStamp{0, 0};
    clock_gettime(CLOCK_REALTIME, &timeStamp);
    return timeStamp.tv_sec * 1000 + 1.0 * timeStamp.tv_nsec / 1000 / 1000;
}


frame_rate_tj::frame_rate_tj(int intervel)
{
    start_time = 0;
    end_time = 0;
    tmp_index = 0;

    if (intervel > 0)
    {
        inter_time = intervel;
    }
    else
    {
        inter_time = 60;
    }
}


void frame_rate_tj::print()
{
    if (0 == tmp_index)
    {
        start_time = get_cur_time_ns();
    }

    if (++tmp_index >= inter_time)
    {
        tmp_index = 0;
        end_time = get_cur_time_ns();
        log_compnt_mngr->debug("frame rate {}", 1000.0 * 1000 * 1000 * (inter_time - 1) / (end_time - start_time));
    }

    return;
}

