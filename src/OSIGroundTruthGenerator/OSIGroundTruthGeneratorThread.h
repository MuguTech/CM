#include <osi3/osi_groundtruth.pb.h>
#include <string>
#include <list>
#include "RoadSystem/Types.h"
#include "RoadSystem/Lane.h"
#include "RoadSystem/RoadSignal.h"
#include "Road.h"
#include <osi3/osi_hostvehicledata.pb.h>
#include <osi3/osi_sensorviewconfiguration.pb.h>
#include <osi3/osi_datarecording.pb.h>
#include <thread>
#include <semaphore.h>
#include <tuple>
#include "../common.h"
#include "octopus_osi.pb.h"
#include "../../../include/Runtime/coSimu/SimProType.h"
#include "../../../APF/RoadSystem/Types.h"
#include "sys/timeb.h"
#include "../PluginMngr.h"
#include "../Evaluation/EvaluationAPI.h"


#define D_OSI_MOVINGOBJECT_TYPE_UNKNOWN              0      /** 未知 */
#define D_OSI_MOVINGOBJECT_TYPE_OTHER                1      /** 其他 */
#define D_OSI_MOVINGOBJECT_TYPE_VEHICLE              2      /** 车辆 */
#define D_OSI_MOVINGOBJECT_TYPE_PEDESTRIAN           3      /** 行人 */
#define D_OSI_MOVINGOBJECT_TYPE_ANIMAL               4      /** 动物 */
#define D_OSI_SIZE_OBJECT_ID                         16     /** object Id 最大长度 */
#define D_OSI_SIZE_TRAFFIC_LIGHT_DATA                24     /** trafficLight数据字节大小 */
#define TIME_STEP_PB_FILE                            10.0   /** 生成pb文件时间间隔 */
#define D_EVALUATION_CMD_RUN_OFFLINE_TOOL            (1)
#define D_PARAM_PRESS_START_TIME_SIZE                (1024)
#define D_PARAM_SCENARIO_NAME_SIZE                   (1024)
#define D_PARAM_NGINFO_CSV_PATH_SIZE                 (1024)
#define D_PARAM_OSI_PB_PATH_SIZE                     (1024)

/** 车道边界(车道线) */
typedef struct LaneBoundaryType
{
    char              xodrType[16];  /** OPENDRIVE标准的type属性*/
    int16_t           id;            /** 车道线Id */
    uint16_t          type;          /** 车道线类型 */
    uint16_t          color;         /** 车道线颜色 */
} OsiLaneBoundary;

/** 主车周围的交通灯Id */
typedef struct OsiTrafficLightAroundEgoType
{
    char         currentRoadTLId[D_OSI_SIZE_OBJECT_ID];        /** 当前road的交通灯Id */
    char         leftTLId[D_OSI_SIZE_OBJECT_ID];               /** 下一个路口的左转交通灯Id */
    char         goStraightTLId[D_OSI_SIZE_OBJECT_ID];         /** 下一个路口的直行交通灯Id */
    char         rightTLId[D_OSI_SIZE_OBJECT_ID];              /** 下一个路口右转交通灯Id */
    char         nextRoadTLId[D_OSI_SIZE_OBJECT_ID];           /** 下一条非junction道路的交通灯Id */

    uint64_t     currentRoadId;                                       /** 当前road的Id */
    uint64_t     leftRoadId;                                          /** 下一个路口的左转road的Id */
    uint64_t     goStraightRoadId;                                    /** 下一个路口的直行road的Id */
    uint64_t     rightRoadId;                                         /** 下一个路口右转road的Id */

    uint64_t     nextRoadId;                                          /** 下一条非junction道路的Id */
} OsiTrafficLightAroundEgo;

/** 坐标及姿态信息 */
typedef struct OsiObjectStateBaseType
{
    double        x;                    /** 世界坐标系x，单位m */
    double        y;                    /** 世界坐标系y，单位m */
    double        z;                    /** 世界坐标系z，单位m */
    float         h;                    /** 世界坐标系，航向角，单位rad */
    float         p;                    /** 世界坐标系，俯仰角，单位rad */
    float         r;                    /** 世界坐标系，横滚角，单位rad */
    float         roadS;                /** 道路坐标系，S坐标，单位m */
    float         roadT;                /** 道路坐标系，T坐标，单位m */
    float         laneOffset;           /** 距离车道中心偏移量，单位m */
    float         hdgRel;               /** 道路坐标系，航向角，单位rad */
    float         pitchRel;             /** 道路坐标系，俯仰角，单位rad */
    float         rollRel;              /** 道路坐标系，横滚角，单位rad */
    uint64_t      roadId;               /** 道路坐标系 */
    int16_t       laneId;               /** 道路坐标系 */
    uint16_t      flags;                /** @D_MSGADAPTER_ROAD_POS_FLAG */
    bool          isOnRoad;             /** 位置是否在道路上 */
} OsiObjectStateBase;

/** 交通灯详细信息 */
typedef struct OsiTrafficLightClassificationType
{
    uint16_t      color;                /** 交通灯颜色@D_MSGADAPTER_TRAFFICLIGHT_COLOR */
    uint16_t      icon;                 /** 交通灯的图标@D_MSGADAPTER_TRAFFICLIGHT_ICON */
} OsiTrafficLightClassification;

typedef struct OsiTrafficLightType
{
    OsiObjectStateBase                   objectStateBase;               /** 坐标及姿态信息 */
    OsiTrafficLightClassification        classification;                /** 交通灯详细信息 */
    uint32_t                                id;                            /** 交通灯的Id */
} OsiTrafficLight;

/** 车辆模型数据 */
typedef struct OsiVehicleCatalogData
{
    double      mass;                       /** 质量 单位:kg */
    double      wheelBase;                  /** 轴距 单位:m */
    double      frontTrackWidth;            /** 前轴宽度 单位:m */
    double      rearTrackWidth;             /** 后轴宽度 单位:m */
    double      maxAcceleration;            /** 最大加速度 单位:m/s^2 */
    double      maxDeceleration;            /** 最大减速度 单位:m/s^2 */
} OsiVehicleCatalog;

// OSI v3.5.0
// protobuf v3.21.9
class OSIGroundTruthGeneratorThread
{
public:
    enum Color
    {
        RED,
        YELLOW,
        GREEN,
        NONE
    };

    static OSIGroundTruthGeneratorThread *Instance();
    static void Destroy();

    // OSIpb文件生成线程
    void creatGeneratorOsiPbFileThread();
    // 将信息存入消息队列
    bool sendMsgToMq(OSI_PB_DATA &data);
    //仿真停止
    void simulationStop();
    //xodr文件解析完成
    void informParseXodrComplete();
    //回收线程
    void joinThread();
    
    //将弧度角值域调整到[0, 2PI)
    double normalizeRad(double angle);
    //将弧度角值域调整到[-PI, PI)
    double normalizeRad2(double angle);

    unsigned long stoul(const std::string &input);

    int stoi(const std::string &input);

    //生成osi联仿数据
    bool generatorOsiCosimuData(OSI_PB_DATA &data);

    //获取osi数据的字符串
    const std::string &getStringOsiGroundTruth();

    //获取OSI联仿数据
    void getCoSimOSIGroundTruthPkg(char *&msgBuffer, unsigned int &msgBufferUsedSize);

protected:
    OSIGroundTruthGeneratorThread();
    ~OSIGroundTruthGeneratorThread();

private:
    // 填充pb文件数据
    void fillData();
    //保存数据到pb文件
    void savePbDataToFile();
    //读取消息队列信息
    void recvMsgFromMq();

    //填充一般数据
    void fillGeneralData();
    //填充MovingObject
    void fillMovingObject();
    //填充StationaryObject
    void fillStationaryObject();
    //填充TrafficSign
    void fillTrafficSign();
    //填充TrafficLight
    void fillTrafficLight();
    //填充roadMarking
    void fillRoadMarking();
    //填充EnvironmentalConditions
    void fillEnvironmentalConditions();
    //填充主车Object数据
    void fillEgoObjectData();

    //转换静态障碍物类型。从SimPro的静态障碍物类型转为osi标准的静态障碍物类型
    uint16_t convertStationaryObjectType(uint16_t inputType);
    //转换交通标志类型,从SimPro的交通标志类型转为osi标准的交通标志类型
    uint16_t convertTrafficSignType(const std::string &inputType);
    //转换roadmarking类型
    uint16_t convertRoadMarkingType(const std::string &inputName);
    //转换车型类型,从SimPro的车型类型转为osi标准的车型类型
    uint16_t convertVehicleType(uint8_t inputType);
    //转换车辆角色
    uint16_t convertVehicleRole(const std::string &inputName);
    //将SimPro降雨强度转换为osi标准的降雨类型
    uint16_t convertTemperature(double temperature);
    //将SimPro可见度范围转换为osi标准的雾类型
    uint16_t convertFog(double visualRange);
    //转换lane的subtype
    uint16_t convertLaneSubtype(uint16_t inputType);
    //转换laneBoundary.type
    uint16_t convertLaneBoundaryType(const RoadMark::RoadMarkType laneBoundaryType);
    //转换laneBoundary.color
    uint16_t convertLaneBoundaryColorType(const RoadMark::RoadMarkColor roadMarkColor);

    //获取centerline
    std::list<Vector3D> getCenterlinePoint(int laneId);
    //解析msg数据
    bool parseMsg(char *msgBuff, unsigned int msgSize);
    //获取lane的前后连接
    std::pair<std::list<unsigned long int>, std::list<unsigned long int>> getConnectLane(Road *road, LaneSection *currentLaneSection, int _laneId);

    //获取车道线坐标
    std::list<Vector3D> getLaneBoundaryPoint(int laneId);
    //更新主车周围交通标志(主车所在Road与下一条Road上存在的交通标志Id)
    void updateTrafficSignAroundEgo(uint32_t egoId);
    //获取主车周围的交通灯id
    OsiTrafficLightAroundEgo &getTrafficLightAroundEgo();
    //设置交通灯Map
    void setTrafficLightMap();
    //通过Road获取交通灯(signalReference)
    RoadSignal *getRefTrafficLightByRoad(const std::string &roadId) const;
    // 通过Road获取交通灯
    std::list<std::tuple<RoadSignal *, int>> getTrafficLightsByRoad(const std::string &roadId) const;
    // 在联仿数据中查找返回交通灯指针
    S_SP_TRAFFIC_LIGHT *findTrafficLight(const std::string &id);
    // 解析车辆模型catalog文件
    void parseVehicleCatalogFile();
    //转换LogicalLane类型
    uint16_t converLogicalLaneType(uint16_t type);

    bool compareDistance(Vector3D &Pos);//比较pos点距离主车是否超过120米
    double distanceForEgo(Vector3D &Pos);//计算pos点与主车距离
    void filterDataForEgo(Road *road, int dir, double &startS, double &endS);//筛选出道路需要输入坐标范围

    //创建只用于pb文件数据存储的线程
    void savePbDataFileThread();

    //LogicalLane数据填充
    void fillLogicalLane(osi3::GroundTruth &_groundTruth);

    //ReferenceLine数据填充
    void fillReferenceLine(osi3::GroundTruth &_groundTruth);

    //LogicalLaneBoundary数据填充
    void fillLogicalLaneBoundary(osi3::GroundTruth &_groundTruth);

    //筛选道路
    void filterRoad();

    //获取沿着direction方向与本条路连接的所有路
    std::map<Road *, int> getNextRoadMap(Road *road, int direction);

    //停车位数据填充
    void fillParkingSpace(Road *road);

    //填充Lane和LaneBoundary 
    void fillLaneAndLaneBoundary();

    static OSIGroundTruthGeneratorThread *_instance;
    std::thread generator_osi_pb_file_handle;
    OSI_PB_DATA pbData; //保存每一帧数据的地址和长度
    osi3::GroundTruth groundTruth;
    double timer; //时间戳
    double lastTimer; //上一次保存pb文件时间戳
    octopus::SimData simDataFrame;	// 用于一帧数据的临时变量
    int fd; //保存groundTruth数据文件描述符
    typedef struct{
        std::list<OSI_PB_DATA> msgList; //保存消息的队列
        std::mutex msgMutex; //互斥量
    }OsiMsg;

    OsiMsg osiMsg;
    sem_t sem; //信号量控制
    sem_t semParseXodr; //用于控制xodr是否解析完成

    int count; //pb文件名称序号
    std::string time_str; //pb文件目录名称(本地时间)

    double diffDisEgo;//主车100米范围内
    S_SP_MIL_EGO_STATE *egoState; //主车数据
    std::list<S_SP_MIL_OBJECT_STATE *> vehicleList; //环境车数据
    std::list<S_SP_MIL_OBJECT_STATE *> pedestrianList; //行人数据
    std::list<S_SP_MIL_OBJECT_STATE *> obstacleList; //障碍物数据
    std::list<S_SP_TRAFFIC_LIGHT *> trafficLightList; //交通灯数据
    std::list<S_SP_TRAFFIC_SIGN *> trafficSignList; //联仿交通标志数据
    std::list<S_SP_LANE_INFO *> laneList; //车道数据
    std::list<S_SP_MIL_ROADMARK *> laneBoundaryList; //车道线信息
    S_SP_ENVIRONMENT *environment; //环境数据

    std::vector<std::pair<uint64_t, int>> laneIdVec; //记录车道线信息
    int direction; //当前帧主车行驶方向
    std::list<std::string> trafficSignAroundEgo; //主车所在Road与下一条Road上存在的交通标志Id

    OsiTrafficLightAroundEgo trafficLightAroundEgo; //主车周围的交通灯Id
    std::map<std::tuple<std::string, std::string>, int> contrastLightMap; //保存符合要求的灯的type，subtype、和表示颜色数量
    std::map<std::string, std::tuple<RoadSignal *, int>> idTraffciLightMap; //<trafficLightId, trafficLight>
    std::map<std::string, RoadSignal *> roadRefTraffciLightMap; // <roadId, signalReference trafficLight> 每一条路和它的<signalReference> 对应关系
    std::map<std::string, std::list<std::tuple<RoadSignal *, int>>> roadTraffciLightsMap; // <roadId, trafficLight> 每一条路和定义在这条路上的所有<signal> 对应关系
    std::list<OsiTrafficLight> trafficLightStatusList; //交通灯状态信息数据
    std::list<RoadSignal *> trafficOverallList; //记录所有交通灯指针

    std::string xodr;
    bool simulationRunStatus; //仿真状态 (需要用osiMsg.msgMutex保护)
    bool isFirstFrame; //是否是第一帧

    S_MQ_EVALUATION_PARAM param; //通知评估模块信息 
    std::string scenarioName; //场景文件名
    std::string scenarioFilePath; //场景文件路径
    std::string xodrFilePath; //地图文件路径
    std::string evaluationDefault; //param参数默认值
    std::string currentPbFilePath; //当前正在使用的pb文件路径
    std::string csvFilePath; //生成的csv文件路径

    float egoTraveledDist; //主车行驶里程

    std::string lastPbFilePath; //上一次生成pb文件路径
    std::string lastCsvFilePath; //上一次生成pb文件时csv文件路径
    float lastEgoTraveledDist; //上一次生成pb文件时主车行驶里程

    std::map<std::string, OsiVehicleCatalog> vehicleCatalogDataMap; //车辆模型数据

    std::thread save_osi_pb_file_handle; //保存pb文件线程
    std::list<osi3::GroundTruth> osiGroundTruthList; //保存每帧osi数据List (需要用osiMsg.msgMutex保护)
    std::string osiCosimuData; //每帧osi联仿数据
    osi3::GroundTruth groundTruthOnlyWithLogicalLane;//只带有reference_line、logical_lane_boundary、logical_lane的groundTruth
    char *osiMsgBuffer; //osiPkg内存地址
    unsigned int osiMsgBufferAllocSize; //osiPkg申请内存大小
    osi3::GroundTruth osiGroundTruth; //单帧osi数据
    std::map<Road *, int> roadMap;//保存主车周围120米范围内筛选的道路 <road, 方向>
    std::map<Road *, LaneSection *> roadLaneSectionMap;//保存道路的LaneSection

    double egoRange;//主车120米范围内
    Vector3D lastUpdataDataEgoPos;//上一次更新数据时的主车位置
};
