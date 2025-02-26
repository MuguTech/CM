#ifndef SAIMOSIM_HMIPRASEMSG_H
#define SAIMOSIM_HMIPRASEMSG_H

#include "../ConfigureMngr.h"
#include "../SensorCfg.h"
#include "../Sensor/SensorManager.h"
#include "./sensorMsgType.h"
#include "../AdadpterSocket.h"
#include "../PluginMngr.h"
#include "../FrameCtrlSocket.h"
#include "./simproSensor/simproCameraDescription.h"
#include "./simproSensor/simproLidarDescription.h"
#include "./simproSensor/simproRadarDescription.h"
#include "./simproSensor/simproSensorDescription.h"
#include "./simproSensor/simproUltrasonicDescription.h"
#include <map>  
#include <vector>  
#include <string>  
#include <sstream>  
#include <algorithm> // for std::copy  

#define RECV_BUF_SIZE 1024 * 1024 * 2

/**
 * @class HmiPraseMsg
 * @brief HMI 消息处理类，用于解析来自HMI传感器配置
 */
class HmiPraseMsg
{
public:
    HmiPraseMsg();
    ~HmiPraseMsg();
    static HmiPraseMsg *Instance();

    std::vector<PluginCfg*> getSensorList();
    std::list<std::string> getSensorDisableList();
    void clearSensorList();

    void addRecvMsg(char *msgBuff, int msgSize);
    
    std::list<LidarDescription*> getLidarDescList();
    std::list<CameraDescription*> getCameraDescList();
    std::list<RadarDescription*> getRadarDescList();
    std::list<UltrasonicDescription*> getUltrasonicDescList();
    bool getEndSensorMsg();
private:
    static HmiPraseMsg *_instance;           // 单例实例的指针
    std::vector<PluginCfg*> p_plugin_list;   //传感器列表

    char recvMsgBuf[RECV_BUF_SIZE]; //接收数据的缓冲区
    int recvMsgBufUsedSize = 0;     // recvMsgBuf已使用空间
    char swapBuf[RECV_BUF_SIZE];    //交换空间
    bool recEndSensorMsg;            //接收最后一个sensor消息
    std::list <std::string> m_sensor_type_name;//存储传传感器类型和传感器名字构成的ID 
    std::list <std::string> last_sensor_type_name;//存储上一次帧传感器类型和传感器名字构成的ID 
    
    std::list<LidarDescription*> LidarDescList;    //雷达传感器lsit
    std::list<CameraDescription*> CameraDescList;
    std::list<RadarDescription*> RadarDescList;
    std::list<UltrasonicDescription*> UltrasonicDescList;

    void checkMsgComplete(char *buffer, int bufferSize, bool &isComplete,int &msgSize);
    
    void parseMsg(char *msgBuff, const int &msgSize);
    
    void praseMSgsensor(const SensorDescription *desc, const std::string &sensorType, const int &sensorFrameRate,const int &EnableSecket,
                         const int &Type,const std::string &IP,const int &Port);

    void parseLidarMsg(LidarDescription_simpro *desc);
    void parseCameraMsg(CameraDescription_simpro *desc);
    void parseRadarMsg(RadarDescription_simpro *desc);
    void parseUltrasonicMsg(UltrasonicDescription_simpro *desc);

    void printfLidarMsg(LidarDescription *tmpLidarDesc);
    void printfCameraMsg(CameraDescription *tmpCameraDesc);
    void printfRadarMsg(RadarDescription *tmpRadarDesc);
    void printfUltrasonicMsg(UltrasonicDescription *tmpUltrasonicDesc);
};
#endif // SAIMOSIM_HMIPRASEMSG_H