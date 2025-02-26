#ifndef SensorManager_h
#define SensorManager_h

#include <list>
#include <set>
#include <vector>
#include <string>
#include <CJsonObject.hpp>
#include "SensorDescription.h"
#include "LidarDescription.h"
#include "CameraDescription.h"
#include "BasicSensorDescription.h"
#include "RadarDescription.h"
#include "UltrasonicDescription.h"
#include "../../../include/Runtime/coSimu/SimProType.h"
#include "../SensorCfg.h"
#include <map>
#include <limits>

#define ASIM_SCK_API_BASICSENSOR_CFG    "ASIM_SCK_API_BASICSENSOR_CFG"
#define ASIM_SCK_API_CAMERA_CFG         "ASIM_SCK_API_CAMERA_CFG"
#define ASIM_SCK_API_DEPTHCAMERA_CFG    "ASIM_SCK_API_DEPTHCAMERA_CFG"
#define ASIM_SCK_API_GPS_CFG            "ASIM_SCK_API_GPS_CFG"
#define ASIM_SCK_API_IMU_CFG            "ASIM_SCK_API_IMU_CFG"
#define ASIM_SCK_API_LIDAR_CFG          "ASIM_SCK_API_LIDAR_CFG"
#define ASIM_SCK_API_RADAR_CFG          "ASIM_SCK_API_RADAR_CFG"
#define ASIM_SCK_API_ULTRASONIC_CFG     "ASIM_SCK_API_ULTRASONIC_CFG"
#define ASIM_SCK_API_V2XOBU_CFG         "ASIM_SCK_API_V2XOBU_CFG"
#define ASIM_SCK_API_V2XRSU_CFG         "ASIM_SCK_API_V2XRSU_CFG"

//物体识别的传感器类型
enum ObjectDetectionSensorType
{
	ObjectDetectionSensorType_IdealizedSensor,	//理想传感器
	ObjectDetectionSensorType_GeneralSensor,	//一般传感器
};

// 输出数据类型
enum GeneralOutputType
{
	GeneralOutputType_realData,	// 真值
	GeneralOutputType_rawData,	// 原始数据
};

typedef std::string         SensorId_t;
using   SensorViewConfig =  LidarDescription;


class SensorManager
{
public:
    static SensorManager *Instance();
    SensorManager();
    ~SensorManager();
    std::list<SensorDescription*> getSensorList();
    void updateSensor(SensorDescription* sensor);
    void addSensor(SensorDescription* sensor);
    void deleteSensor(const std::string &name, int type);
    void enableSensor(const std::string &name, int type);
    void disableSensor(const std::string &name, int type);

//[帧率控制数据传输架构整改]TODO: 为编译通过暂时关闭，后续再支持传感器支持导入导出json文件
#if 0 /* Sue 2023.12.26 [帧率控制数据传输架构整改] [SimPro中的Sensor移植到CM] [ADD] [START] */
    void loadSensorCfg(const std::vector <std::string>&);
    void initSensor(neb::CJsonObject&);
#endif /* Sue 2023.12.26 [帧率控制数据传输架构整改] [SimPro中的Sensor移植到CM] [ADD] [END] */
	SensorDescription* getSensor(const std::string &name, int type);

    void clearSensorList();
    void isEableDumpLidarData(const std::string &name, bool isSave);
    void isEableDumpCameraData(const std::string &name, bool isSave);
    void isEableDumpRadarData(const std::string &name, bool isSave);
    void dumpSensorDataType(const std::string &name, int type, const std::string &dumpType);
    void isVisibleSensorCloudPoint(const std::string &name, int type, bool isVisible);
	static size_t getMaxSensorDescriptionSize();
    int getSensorNum(int type); // 返回特定传感器类型的个数，比如激光雷达的个数


    void Destroy();
    // 设置传感器类型
    void setObjectDetectionSensorType(int type);

    // 设置数据类型
    void setOutputType(int type);

	// 取得传感器类型
	inline int getObjectDetectionSensorType()
	{
		return _objectDetectionSensorType;
	}

	// 取得输出数据类型
	inline int getGeneralOutputType()
	{
		return _generalOutputType;
	}

    void resetSensorInfo();// 包络线[Add]：切换理想传感器和一般传感器时，重置不同类型传感器的使能开关

    const std::map<int, std::set<int>> &getInviewIDs(); //获取所有被探测到物体的ID

    void resetSensorPkgTimer(); //重置_sensorPkgtimer

    /* manager sensors */
    void addSensor(SensorCfg *);
    void    removeSensor(const std::string &name, int type);
    int     getSensorNum() { return m_iSensorCnt;}


    /* interact with ConfigManager */
    //void    saveConfig(ConfigInfo&);
    void    loadConfig();
    //void    loadConfig(const ConfigInfo &);


    /* interact with PluginManager*/
    void init();
    bool isRunning(int ,void * sensor_info);
    bool setRunning(std::string sensor_id);
    std::map<std::string, bool> getSensorRunningMap() {
      return m_sensor_running_map;
    }
    std::map<std::string, int> getSensorFrameMap() {
      return m_sensor_frame_map;
    }

    void setIsSyncMode(bool is_sync_mode) { m_is_sync_mode = is_sync_mode; }
    bool getIsSyncMode() { return m_is_sync_mode; }
    void resetSensorRunningMap();
    bool setSensorFrameNumber(const std::string &sensor_id, int frame_number);

    // 获取SensorInfoPkg
    void getSensorInfoPkg(const SensorId_t &sensorId, char **buffer, unsigned int *size);

private:
    void doCalc(SensorId_t, int);// get_plugin_list根据各传感器频率计算调度表

    bool generateSensorInfoPkg(); // 打包SensorInfoPkg
    void releasePkgBufferMap(); //释放pkgBufferMap

    static SensorManager *__instance;
	pthread_mutex_t _mutex_sensorList;
    std::list<SensorDescription*> _sensorList;
	int _objectDetectionSensorType;
    int _generalOutputType;
    int _sensorPkgtimer;
    std::vector<S_SP_MIL_ROADMARK> road_mark_info;


    std::hash<std::string> hasher;    //创建一个哈希类型对象
    double lastSimTime;
    std::map<int, std::set<int>> InViewIDs; //保存被打开了包络线的传感器"所探测到”物体的类型与ID

    /* manager sensors */
    int m_iSensorCnt;

    /* intrinsic properties */
    int m_iMainFreq = 0;    // 跟随仿真频率，通过ConfigManager或其他模块设置
    int m_uiFrameNum;   // 跟随仿真帧号，通过isRunning调用传入
    std::map<SensorId_t, int*>        m_mapScheduleTable; // 根据各Sensor的description获取相应的传感器频率，用以计算调度表

    // sensor_running_map
    // sensor id, is running
    std::map<std::string, bool> m_sensor_running_map;
    // sensor id, need comsume frame number
    std::map<std::string, int> m_sensor_frame_map;

    bool m_is_sync_mode = false;
    // std::mutex m_sensor_running_map_mutex;

    // 记录所有传感器的SensorCfg (SensorCfg对象是由外部管理的，不需要由SensorManager释放)
    std::list<SensorCfg *> SensorCfgList;

    // 内部管理buffer
    std::map<SensorId_t, std::tuple<char *, unsigned int>> pkgBufferMap; //保存每个传感器的SensorInfoPkg Buffer <sensorId, <pkgBuffer, pkgAllocSize>>
};

#endif
