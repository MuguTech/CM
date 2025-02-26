#ifndef __COMMONTYPE_INCLUDED_H__
#define __COMMONTYPE_INCLUDED_H__

#include <string>
#include <mutex>
#include <map>
#include <list>


typedef char               char_t;
typedef signed char        int8_t;
typedef signed short       int16_t;
typedef signed int         int32_t;
// typedef long long          int64_t;
typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
typedef unsigned int       uint32_t;
// typedef unsigned long      uint64_t;
typedef unsigned long long uint128_t;
typedef float              float32_t;
typedef double             float64_t;
typedef long double        float128_t;


#define MSG_BUFF_SIZE     (5 * 1000 * 1000)     // 必须大于 SOCKET_BUF_SIZE


typedef struct{
    char ch[MSG_BUFF_SIZE];
    int len;
    std::mutex msgMutex;
}MSG_MUTEX;



bool is_file_exist(std::string &filepath);
bool update_v_traffic_data(void *msg, size_t len);
bool get_v_traffic_data(void *msg, size_t &len);
void resetSimproMsg();

/* for defining sensor types */
#define LIDAR_T     "lidar"
#define RADAR_T     "radar"
#define CAMERA_T    "camera"
#define ULTRASONIC_T    "ultrasonic"
#define V2X_T "v2x"

bool is_sensor_type(const std::string& plugin_type);


/* 仿真状态 */ 
#define IS_START        1
#define IS_STOP         2
#define IS_SIMULATE     3

typedef struct{
    char *msg;
    size_t msgLen;
}OSI_PB_DATA;

typedef struct{
    char *msg;
    size_t msgLen;
}COSIMU_DATA;

typedef struct {
    std::list<COSIMU_DATA> msgList; //保存消息的队列
    std::mutex msgMutex;            //互斥量
}CoSimuMsg;

CoSimuMsg &getRecvMsgFromAd();      //获取AD发来的数据
CoSimuMsg &getSendMsgToSimpro();    //获取准备发送给simpro的数据

#endif // __COMMONTYPE_INCLUDED_H__

