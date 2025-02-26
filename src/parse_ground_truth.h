#ifndef __PARSE_GROUND_TRUTH_H__
#define __PARSE_GROUND_TRUTH_H__

#include "common.h"
#include <string>



class parse_ground_truth
{
private:
    int frame_id;
    double sim_time;
    unsigned char sender;
    int simulate_state;
    std::string xodr;

    char *_msg;
    int _len;

    bool is_complete;

public:
    parse_ground_truth(void *msg, int len);
    ~parse_ground_truth();
    bool parse();
    int get_frame_id();
    double get_sim_time();
    unsigned char get_sender();
    int get_simulate_state();       // return: IS_START IS_STOP IS_SIMULATE
    std::string get_xodr();         // return: RoadSystem::Instance()->parseOpenDrive(filename) 使用的名字

    bool is_complete_pkg();
    bool get_len_frame_data(size_t & len); // 返回一帧数据的所有长度

private:
    void set_frame_id(int id);
    void set_sim_time(double simTime);
    void set_sender(unsigned char sender);
};





#endif  // __PARSE_GROUND_TRUTH_H__

