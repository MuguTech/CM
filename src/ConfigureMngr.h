#ifndef __CONFIGURE_MNGR_H__
#define __CONFIGURE_MNGR_H__

#include <vector>
#include <string>
#include <map>


#include "DynamicVehicleCfg.h"
#include "PluginCfg.h"
#ifndef V1000HZ
    #include "SensorCfg.h"
#endif

#include "./HmiSensorCfgTcpClient/HmiPraseMsg.h"

#define CONFIGURE_FILE "./ComponentMngrCfg.xml"



class ConfigureMngr
{
private:

    static ConfigureMngr *_instance;
    std::vector<PluginCfg*> p_plugin_list;
    bool init;
    int simu_frame_ctl;                      // 0: 内部控制， 大于0: 外部控制
    int frame_rate;                          // 0: 数据驱动， 大于0: 定时驱动
    std::string so_dir;
    struct _socket simulator;
    struct _socket frame_ctl_srv;
    struct _socket cm_sensor_server;     //CM 传感器配置的服务端
    int sensorControllerMode;           //1:一般配置文件模式 2:HMI模式

    bool isVirtualCity; //是否开启虚拟城市

    bool isOutputOsiGroundTruthLogicalLane; //是否输出OSI数据
    int coSimuInterfaceType; //联仿接口模式 1:C-API 2:OSI 默认1:C-API
    int controlInLoopValue; //控制在环 0:关闭 1:打开 默认0:关闭
    int controlInLoopType; //控制在环类型 1:动力学挂载内部 2:动力学挂载外部 默认1:动力学挂载内部
    int frameRate; //仿真帧率 阈值[10,200] 默认50
    int coSimuSocketMode; //通信方式 1:UDP 2:TCP 默认1:UDP

    int mainVehicleId; //主车ID，从环境变量 MAIN_VEHICLE_ID 中获取

public:
    ~ConfigureMngr();
    static ConfigureMngr *getInstance();
    void update_cfg();                                // 更新配置信息
    bool is_sys_ready();
    int get_frame_ctl() const;
    int get_frame_rate() const;
    std::string get_so_dir() const;
    std::vector<PluginCfg*> get_plugin_list();
    struct _socket get_simulator_socket();
    struct _socket get_frame_ctl_socket();
    struct _socket get_cm_sensor_server();
    int getSensorControllerMode();

    //解析虚拟城市enable.xml文件
    bool parseVirtualCityConfigFile();

    //获取是否开启虚拟城市
    bool getIsVirtualCity();

    //获取是否输出OSI数据
    bool getIsOutputOsiGroundTruthLogicalLane();
    //获取联仿接口模式 1:C-API 2:OSI
    int getCoSimuInterfaceType();
    //获取控制在环开关 0:关闭 1:打开
    int getControlInLoopValue();
    //获取控制在环类型 1:动力学挂载内部 2:动力学挂载外部
    int getControlInLoopType();
    //获取仿真帧率
    int getFrameRate();
    //获取通信方式 1:UDP 2:TCP
    int getCoSimuSocketMode();

    //获取主车ID
    int getMainVehicleId();

private:
    void load_cfg();                                 // 加载配置文件
    void print_cfg();
    ConfigureMngr();
    void set_frame_ctl(int ctl);
    void set_frame_rate(int rate);
    void add_plugin_list(PluginCfg *plugin);
    void set_so_dir(const std::string &dir);
    void set_simulator_socket(const std::string ip, const std::string port, const std::string type);
    void set_frame_ctl_socket(const std::string ip, const std::string port, const std::string type);
    void set_cm_sensor_server(const std::string ip, const std::string port, const std::string type);
};





#endif // __CONFIGURE_MNGR_H__
