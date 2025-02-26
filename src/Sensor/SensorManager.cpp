#include "SensorManager.h"
#include "../../../Runtime/TrafficSimulation/TrafficSimulation.h"
#include <string.h>
#include <string>
#include <iostream>
#include <fstream>
#include <memory>
#include "Common.h"
#include "../ConfigureMngr.h"
#include "../log.h"
#include "../common.h"

#define find_max(a, b, c)	(a > b ? (a > c ? a : c) : (b > c ? b : c))

SensorManager *SensorManager::__instance = NULL;

SensorManager *SensorManager::Instance()
{
    if (__instance == NULL)
    {
        __instance = new SensorManager();
    }
    return __instance;
}

void SensorManager::Destroy()
{
    if (__instance != nullptr)
    {
        delete __instance;
        __instance = nullptr;
    }
}

SensorManager::SensorManager()
{
    _mutex_sensorList = PTHREAD_MUTEX_INITIALIZER;
    _objectDetectionSensorType = ObjectDetectionSensorType_IdealizedSensor;
    _generalOutputType = GeneralOutputType_realData;
    _sensorPkgtimer = 0;
    lastSimTime = 0;
    m_iSensorCnt = 0;
    init();         // Add: duanyanling 20230510
}

SensorManager::~SensorManager()
{
    for(auto &t : m_mapScheduleTable) { // clear schedule table
        if(t.second) {
            delete [] t.second;
            t.second = nullptr;
        }
    }
    m_mapScheduleTable.clear();

    //释放pkgBufferMap
    releasePkgBufferMap();
}

std::list<SensorDescription*> SensorManager::getSensorList()
{
    return _sensorList;
}

int SensorManager::getSensorNum(int type)
{
    log_compnt_mngr->info("SensorManager::getSensorNum start.");
    log_compnt_mngr->debug("start type({})", type);

    int count = 0;

    pthread_mutex_lock(&_mutex_sensorList);
    for (std::list<SensorDescription*>::iterator it = _sensorList.begin(); it != _sensorList.end(); it++)
    {
        if((*it)->type == type)
        {
            count++;
        }
    }

    pthread_mutex_unlock(&_mutex_sensorList);

    log_compnt_mngr->debug("end count({})", count);
    log_compnt_mngr->info("SensorManager::getSensorNum end.");
    return count;
}

void SensorManager::addSensor(SensorDescription* sensor)
{
    log_compnt_mngr->info("SensorManager::addSensor start.");
    log_compnt_mngr->debug("SensorManager::addSensor Pos=({},{},{})",sensor->assemblePositionX, sensor->assemblePositionY, sensor->assemblePositionZ);
    log_compnt_mngr->debug(" sensorName  sensor size(){},{}",sensor->name,_sensorList.size() );
    pthread_mutex_lock(&_mutex_sensorList);
    std::list<SensorDescription*>::iterator it = _sensorList.begin();
    for(; it != _sensorList.end(); ++it)
    {
        //if  exists, update sensor
        if(strcmp(sensor->name,(*it)->name) == 0 && sensor->type == (*it)->type)
        {
            log_compnt_mngr->debug("SensorManager::updateSensor1 sensorName {}",sensor->name);
            *it = sensor;
            break;
        }
    }

    if(it == _sensorList.end())
    {
        _sensorList.push_back(sensor);
    }
    pthread_mutex_unlock(&_mutex_sensorList);
    log_compnt_mngr->debug(" sensorName1  sensor size1() {},{}", sensor->name,_sensorList.size() );
    log_compnt_mngr->info("SensorManager::addSensor end.");
}

void SensorManager::deleteSensor(const std::string &name, int type)
{
    log_compnt_mngr->info("SensorManager::deleteSensor start.");
    log_compnt_mngr->debug(" name sensor size() {},{}",name.c_str(), _sensorList.size() );
    pthread_mutex_lock(&_mutex_sensorList);
    for(std::list<SensorDescription*>::iterator it = _sensorList.begin(); it != _sensorList.end(); ++it)
    {
        if(strcmp(name.c_str(),(*it)->name) == 0 && type == (*it)->type)
        {
            log_compnt_mngr->debug("delete sensor name {}",name.c_str());
            delete (*it);
            _sensorList.remove((*it));
            break;
        }
    }
    pthread_mutex_unlock(&_mutex_sensorList);
    log_compnt_mngr->debug(" name sensor size1() {},{}", name.c_str(), _sensorList.size() );
    log_compnt_mngr->info("SensorManager::deleteSensor end.");
}

SensorDescription* SensorManager::getSensor(const std::string &name, int type)
{
    log_compnt_mngr->info("SensorManager::getSensor start.");
    log_compnt_mngr->debug(" name sensor size() {},{}",name.c_str(), _sensorList.size() );
    pthread_mutex_lock(&_mutex_sensorList);
    std::list<SensorDescription*>::iterator it = _sensorList.begin();
    for(; it != _sensorList.end(); ++it)
    {
        if(strcmp(name.c_str(),(*it)->name) == 0 && type == (*it)->type)
        {
            break;
        }
    }

    SensorDescription* description = (it == _sensorList.end()) ? NULL : *it;
    pthread_mutex_unlock(&_mutex_sensorList);
    log_compnt_mngr->debug(" name  sensor size1() {},{}",name.c_str(),_sensorList.size() );
    log_compnt_mngr->info("SensorManager::getSensor end.");
    return description;
}

void SensorManager::updateSensor(SensorDescription* sensor)
{
    log_compnt_mngr->info("SensorManager::updateSensor start.");
    log_compnt_mngr->debug(" name {}", sensor->name);
    pthread_mutex_lock(&_mutex_sensorList);
    for(std::list<SensorDescription*>::iterator it = _sensorList.begin(); it != _sensorList.end(); ++it)
    {
        //if  exists, update sensor
        if(strcmp(sensor->name,(*it)->name) == 0 && sensor->type == (*it)->type)
        {
            *it = sensor;
            break;
        }
    }
    pthread_mutex_unlock(&_mutex_sensorList);
    log_compnt_mngr->info("SensorManager::updateSensor end.");
}


void SensorManager::enableSensor(const std::string &name, int type)
{
    log_compnt_mngr->info("SensorManager::enableSensor start.");
    // 限制name的长度为128字节
    std::string n = name;
    if (name.size() > 127)
    {
        n = name.substr(0,127);
    }
    log_compnt_mngr->debug(" size = {}", _sensorList.size());
    pthread_mutex_lock(&_mutex_sensorList);
    for(std::list<SensorDescription*>::iterator it = _sensorList.begin(); it != _sensorList.end(); ++it)
    {
        log_compnt_mngr->debug("enableSensor 11 name =  name1 =  type = type1 = {},{},{},{}", n.c_str(),(*it)->name,type,(*it)->type );
        //if  exists, update sensor
        if(strcmp(n.c_str(),(*it)->name) == 0 && type == (*it)->type)
        {
            (*it)->enable = true;
            log_compnt_mngr->debug("enableSensor 333 = {}", (*it)->enable );
            break;
        }
    }
    pthread_mutex_unlock(&_mutex_sensorList);
    log_compnt_mngr->info("SensorManager::enableSensor end.");
}

void SensorManager::disableSensor(const std::string &name, int type)
{
    log_compnt_mngr->info("SensorManager::disableSensor start.");
    // 限制name的长度为128字节
    std::string n = name;
    if (name.size() > 127)
    {
        n = name.substr(0,127);
    }
    log_compnt_mngr->debug(" size = {}",_sensorList.size() );
    pthread_mutex_lock(&_mutex_sensorList);
    for(std::list<SensorDescription*>::iterator it = _sensorList.begin(); it != _sensorList.end(); ++it)
    {
        log_compnt_mngr->debug("disableSensor 11 name = name1 =  type =  type1 = {},{},{},{}", n.c_str(),(*it)->name,type,(*it)->type );
        //if  exists, update sensor
        if(strcmp(n.c_str(),(*it)->name) == 0 && type == (*it)->type)
        {
            (*it)->enable = false;
            log_compnt_mngr->debug("disableSensor 333 = {}",(*it)->enable );
            break;
        }
    }
    pthread_mutex_unlock(&_mutex_sensorList);
    log_compnt_mngr->info("SensorManager::disableSensor end.");
}


//[帧率控制数据传输架构整改]TODO: 为编译通过暂时关闭，后续再支持传感器支持导入导出json文件
#if 0 /* Sue 2023.12.26 [帧率控制数据传输架构整改] [SimPro中的Sensor移植到CM] [ADD] [START] */
void SensorManager::loadSensorCfg(const std::vector <std::string>& _vFileList)
{
    if (_vFileList.empty()) {
        log_compnt_mngr->warn("Sensor config file list empty.");
        return;
    }

    for (auto &fp : _vFileList) {
        log_compnt_mngr->trace("loading Sensor config file {}.", fp);
        std::ifstream       ifS(fp.data());
        std::stringstream   ssCont;
        ssCont << ifS.rdbuf();
        neb::CJsonObject    oJson(ssCont.str());
        this->initSensor(oJson);
    }
}
void SensorManager::initSensor(neb::CJsonObject& _cfg)
{
    /* json format: 1. apiname; 2. cfgfile; 3. data */
    std::string strApiName = "";
    _cfg.Get("apiname", strApiName);
    if(strApiName.empty()) {
        log_compnt_mngr->error("Invalid apiname.");
        return;
    }

    if (0 == strApiName.compare(ASIM_SCK_API_BASICSENSOR_CFG)) {
        addSensor(this->basicSensorLoad(_cfg["data"][0]));
        this->setObjectDetectionSensorType(ObjectDetectionSensorType_IdealizedSensor);
        return;
    }
    if (0 == strApiName.compare(ASIM_SCK_API_CAMERA_CFG)) {
        addSensor(this->cameraLoad(_cfg["data"][0]));
        this->setObjectDetectionSensorType(ObjectDetectionSensorType_GeneralSensor);
    }
    else if (0 == strApiName.compare(ASIM_SCK_API_LIDAR_CFG)) {
        addSensor(this->lidarLoad(_cfg["data"][0]));
        this->setObjectDetectionSensorType(ObjectDetectionSensorType_GeneralSensor);
    }
    else if (0 == strApiName.compare(ASIM_SCK_API_RADAR_CFG)) {
        addSensor(this->radarLoad(_cfg["data"][0]));
        this->setObjectDetectionSensorType(ObjectDetectionSensorType_GeneralSensor);
    }
    else if (0 == strApiName.compare(ASIM_SCK_API_ULTRASONIC_CFG)) {
        addSensor(this->ultrasonicLoad(_cfg["data"][0]));
        this->setObjectDetectionSensorType(ObjectDetectionSensorType_GeneralSensor);
    }
    else if (0 == strApiName.compare(ASIM_SCK_API_V2XOBU_CFG)) { // v2x的传感器没有指定探测类型
        addSensor(this->v2xOBULoad(_cfg["data"][0]));
    }
    else if (0 == strApiName.compare(ASIM_SCK_API_V2XRSU_CFG)) {
        addSensor(this->v2xRSULoad(_cfg["data"][0]));
    }
    else {
        /* utter ugly... */
    }

}
#endif /* Sue 2023.12.26 [帧率控制数据传输架构整改] [SimPro中的Sensor移植到CM] [ADD] [END] */

size_t SensorManager::getMaxSensorDescriptionSize()
{
    return find_max(sizeof(LidarDescription), sizeof(CameraDescription), sizeof(RadarDescription));
}

void SensorManager::clearSensorList()
{
    log_compnt_mngr->info("SensorManager::clearSensorList start.");
    pthread_mutex_lock(&_mutex_sensorList);
    for(std::list<SensorDescription*>::iterator it = _sensorList.begin(); it != _sensorList.end(); ++it)
    {
        delete (*it);
    }
    _sensorList.clear();
    pthread_mutex_unlock(&_mutex_sensorList);
    log_compnt_mngr->info("SensorManager::clearSensorList end.");
}

void SensorManager::isEableDumpLidarData(const std::string &name, bool isSave)
{
    log_compnt_mngr->info("SensorManager::isEableDumpLidarData start.");
    pthread_mutex_lock(&_mutex_sensorList);
    for(std::list<SensorDescription*>::iterator it = _sensorList.begin(); it != _sensorList.end(); ++it)
    {
        log_compnt_mngr->debug(" name = , sensor->name = {},{}", name.c_str(), (*it)->name);
        if(strcmp((*it)->name,name.c_str()) == 0)
        {
            LidarDescription *lidar = (LidarDescription*)(*it);
            log_compnt_mngr->debug(" isSave = {}", isSave );
            lidar->isSavePointCloud = isSave;
        }
    }
    pthread_mutex_unlock(&_mutex_sensorList);
    log_compnt_mngr->info("SensorManager::isEableDumpLidarData end.");
}

void SensorManager::isEableDumpCameraData(const std::string &name, bool isSave)
{
    log_compnt_mngr->info("SensorManager::isEableDumpCameraData start." );
    pthread_mutex_lock(&_mutex_sensorList);
    for(std::list<SensorDescription*>::iterator it = _sensorList.begin(); it != _sensorList.end(); ++it)
    {
        log_compnt_mngr->debug(" name = , sensor->name = {},{}", name.c_str(), (*it)->name);
        if(strcmp((*it)->name,name.c_str()) == 0)
        {
            CameraDescription *camera = (CameraDescription*)(*it);
            log_compnt_mngr->debug(" isSave = {}", isSave );
            camera->isSaveRGB = isSave;
        }
    }
    pthread_mutex_unlock(&_mutex_sensorList);
    log_compnt_mngr->info("SensorManager::isEableDumpCameraData end.");
}

void SensorManager::isEableDumpRadarData(const std::string &name, bool isSave)
{
    log_compnt_mngr->info("SensorManager::isEableDumpRadarData start." );
    pthread_mutex_lock(&_mutex_sensorList);
    for(std::list<SensorDescription*>::iterator it = _sensorList.begin(); it != _sensorList.end(); ++it)
    {
        log_compnt_mngr->debug(" name = , sensor->name = {},{}", name.c_str(), (*it)->name);
        if(strcmp((*it)->name,name.c_str()) == 0)
        {
            RadarDescription *radar = (RadarDescription*)(*it);
            log_compnt_mngr->debug(" isSave = {}", isSave );
            radar->isSavePointCloud = isSave;
        }
    }
    pthread_mutex_unlock(&_mutex_sensorList);
    log_compnt_mngr->info("SensorManager::isEableDumpRadarData end.");
}

void SensorManager::dumpSensorDataType(const std::string &name, int type, const std::string &dumpType)
{
    log_compnt_mngr->info("SensorManager::dumpSensorDataType start." );
    pthread_mutex_lock(&_mutex_sensorList);
    for(std::list<SensorDescription*>::iterator it = _sensorList.begin(); it != _sensorList.end(); ++it)
    {
        log_compnt_mngr->debug(" name = , sensor->name = {},{}", name.c_str(), (*it)->name);

        if((*it)->type == type && strcmp((*it)->name,name.c_str()) == 0)
        {
            log_compnt_mngr->debug(" dumpType = {} ", dumpType.c_str() );
            if(type == SENSOR_TYPE_LIDAR)
            {
                LidarDescription *lidar = (LidarDescription*)(*it);
                lidar->dumpRawDataType = TrafficSimulation::stoi(dumpType);
            }
            else if(type == SENSOR_TYPE_RADAR)
            {
                RadarDescription* radar = (RadarDescription*)(*it);
                radar->dumpRawDataType = TrafficSimulation::stoi(dumpType);
            }
        }
    }
    pthread_mutex_unlock(&_mutex_sensorList);
    log_compnt_mngr->info("SensorManager::dumpSensorDataType end." );
}

void SensorManager::isVisibleSensorCloudPoint(const std::string &name, int type, bool isVisible)
{
    log_compnt_mngr->info("SensorManager::isVisibleSensorCloudPoint start.");
    pthread_mutex_lock(&_mutex_sensorList);
    for(std::list<SensorDescription*>::iterator it = _sensorList.begin(); it != _sensorList.end(); ++it)
    {
        log_compnt_mngr->debug(" name = , sensor->name = {},{}",  name.c_str(), (*it)->name);
        if((*it)->type == type && strcmp((*it)->name,name.c_str()) == 0)
        {
            log_compnt_mngr->debug(" isVisible = {}", isVisible );
            LidarDescription *lidar = (LidarDescription*)(*it);
            lidar->isVisibleCloudPoint = isVisible;
        }
    }
    pthread_mutex_unlock(&_mutex_sensorList);
    log_compnt_mngr->info("SensorManager::isVisibleSensorCloudPoint end.");
}

void SensorManager::setObjectDetectionSensorType(int type)
{
    log_compnt_mngr->debug("setObjectDetectionSensorType type=, _objectDetectionSensorType={},{}",type, _objectDetectionSensorType);
    _objectDetectionSensorType = type;
}


void SensorManager::setOutputType(int type)
{
    log_compnt_mngr->debug("setOutputType type=, _generalOutputType= {},{}",type, _generalOutputType);
    _generalOutputType = type;
}

// 包络线[Add]：切换理想传感器和一般传感器时，重置不同类型传感器的使能开关
void SensorManager::resetSensorInfo()
{
    log_compnt_mngr->info("SensorManager::resetSensorInfo start.");
    pthread_mutex_lock(&_mutex_sensorList);

    //遍历所有传感器配置
    for(std::list<SensorDescription*>::iterator it = _sensorList.begin(); it != _sensorList.end(); ++it)
    {
        //物体识别的类型是理想传感器的场合
        if(_objectDetectionSensorType == ObjectDetectionSensorType_IdealizedSensor)
        {
            if((*it)->type != SENSOR_TYPE_BASIC_SENSOR)
            {
                (*it)->enable = false;
            }
        }
        //物体识别的类型是一般传感器的场合
        else if(_objectDetectionSensorType == ObjectDetectionSensorType_GeneralSensor)
        {
            if((*it)->type == SENSOR_TYPE_BASIC_SENSOR)
            {
                (*it)->enable = false;
            }
        }
        //其它
        else
        {

        }
    }
    pthread_mutex_unlock(&_mutex_sensorList);
    log_compnt_mngr->info("SensorManager::resetSensorInfo end.");
}

//增加在包络线范围内的车/人/障碍物要显示包围盒功能 
const std::map<int, std::set<int>> &SensorManager::getInviewIDs()
{
    return InViewIDs;
}

//重置_sensorPkgtimer
void SensorManager::resetSensorPkgTimer()
{
    _sensorPkgtimer = -1; //场景开始时初始化为-1，使得第一帧就能输出sensor数据
    lastSimTime = 0;      //lastSimTime置0,防止中途关闭AD会导致帧号计数不能+1
}

bool validType(std::string _s)
{
    return  !strcmp(_s.data(), LIDAR_T)
            ||  !strcmp(_s.data(), RADAR_T)
            ||  !strcmp(_s.data(), CAMERA_T)
            ||  !strcmp(_s.data(), ULTRASONIC_T)
            ;
}

void SensorManager::loadConfig()
{
    log_compnt_mngr->info("SensorManager::loadConfig start.");
    SensorViewConfig *svPtr;
    int sensorFreq = 0;
    auto what = ConfigureMngr::getInstance()->get_plugin_list();

    for (auto p : what) {
        if(!validType(p->get_type())) {
            continue;
        }
        else {
            this->addSensor((SensorCfg*)p);
        }
    }
    log_compnt_mngr->info("SensorManager::loadConfig end.");
}

void SensorManager::addSensor(SensorCfg* pSC)
{
    log_compnt_mngr->info("SensorManager::addSensor start.");
    SensorId_t sensorID =   pSC->get_id();
    int sensorFreq =        pSC->getFrmRate();
    this->doCalc(sensorID, sensorFreq);

    m_sensor_running_map[sensorID] = false;

    //记录所有传感器的SensorCfg
    SensorCfgList.push_back(pSC);
    log_compnt_mngr->info("SensorManager::addSensor end.");
}

void SensorManager::init()
{
    log_compnt_mngr->info("SensorManager::init start.");
    m_iMainFreq = ConfigureMngr::getInstance()->get_frame_rate();
    if(m_iMainFreq <= 0) {
        log_compnt_mngr->warn("WARNING: config frame rate = {} <=0.", m_iMainFreq);
    }
    this->loadConfig();

    // 打包SensorInfoPkg
    generateSensorInfoPkg();
    log_compnt_mngr->info("SensorManager::init end.");
}

bool SensorManager::setRunning(std::string sensor_id)
{
    log_compnt_mngr->info("SensorManager::setRunning start.");
    bool is_set_ok = true;
    log_compnt_mngr->debug("enter setRunning sensor_id is {}", sensor_id);

    if(m_sensor_running_map.find(sensor_id) == m_sensor_running_map.end())
    {
        log_compnt_mngr->error("sensor {} not find.", sensor_id);
        is_set_ok = false;
    }
    else
    {
        m_sensor_running_map[sensor_id] = true;
    }

    log_compnt_mngr->info("SensorManager::setRunning end.");
    return is_set_ok;
}

void SensorManager::resetSensorRunningMap()
{
for (auto &sensor_cfg : m_sensor_running_map) {
    sensor_cfg.second = false;
}
}

bool SensorManager::setSensorFrameNumber(const std::string &sensor_id,
                                        int frame_number) {
m_sensor_frame_map[sensor_id] = frame_number;
return true;
}

void SensorManager::doCalc(SensorId_t id, int _f)
{
    log_compnt_mngr->info("SensorManager::doCalc start.");
    if(0 >= m_iMainFreq) {
        log_compnt_mngr->error("m_iMainFreq is <= 0.");
        return;
    }
    if(0 == _f) {
        log_compnt_mngr->error("_f is 0.");
        return;
    }
    int* iTable = (int*)calloc(m_iMainFreq, sizeof(int));
    m_mapScheduleTable[id] = iTable;


    unsigned int iTime = _f / m_iMainFreq;
    unsigned int iRmd  = _f % m_iMainFreq;

    for(int i = 0; i<m_iMainFreq; i++) { // 除商部分
        iTable[i] = iTime;
    }

    if (iRmd) { // 余数部分，均摊到主频率中
        double dTmp = m_iMainFreq / (double)iRmd;
        for (int j = 0; j < iRmd; j++) {
            iTable [(int)(j*dTmp)] +=1;
        }
    }
    // for (int i = 0; i < m_iMainFreq; i++) {
    //     log_compnt_mngr->info("{} ",iTable[i]);
    // }
    // log_compnt_mngr->info(".");
    log_compnt_mngr->info("SensorManager::doCalc end.");
}

bool SensorManager::isRunning(int iFrmN, void * sensor_info)
{
    if(0 >= m_iMainFreq) {
        log_compnt_mngr->debug("sensor {} runs.", (char*)sensor_info);
        return true;
    }
    this->m_uiFrameNum = iFrmN;
    unsigned int uiCurrFrm = this->m_uiFrameNum % this->m_iMainFreq;

    SensorId_t p_id = *(SensorId_t*)sensor_info;
    if(m_mapScheduleTable.end() == m_mapScheduleTable.find(p_id)) {
        log_compnt_mngr->warn("sensor {} not found in SensorManager.", p_id.data());
        return false;
    }
    int * iTable = m_mapScheduleTable.at(p_id);

    if(nullptr != iTable) {
        int iRslt = iTable[uiCurrFrm];  // 查询调度表
        return 0 != iRslt;              // 如果当前帧的运行次数不为0，则表示该传感器需要运行
    }

    return false;
}

// 打包SensorInfoPkg
bool SensorManager::generateSensorInfoPkg()
{
    log_compnt_mngr->info("SensorManager::generateSensorInfoPkg start.");
    releasePkgBufferMap(); //释放pkgBufferMap

    //遍历每个传感器，每个传感器打1个SensorInfoPkg
    for (const auto &sensorCfg : SensorCfgList)
    {
        if (sensorCfg == nullptr)
        {
            log_compnt_mngr->warn("SensorManager::generateSensorInfoPkg sensorCfg == nullptr");
            continue;
        }

        //获取该传感器的description
        auto description = sensorCfg->getDesc();
        if (description == nullptr)
        {
            log_compnt_mngr->warn("SensorManager::generateSensorInfoPkg description == nullptr");
            continue;
        }

        //元素总数，即此Pkg的数据部分的元素数量
        unsigned int elementNum = 1;

        //Pkg大小
        unsigned int pkgSize = sizeof(S_SP_MSG_ENTRY_HDR) + sizeof(S_SP_SENSOR_INFO) * elementNum;

        //pkgBuffer申请内存
        char *pkgBuffer = (char *)malloc(pkgSize);//申请内存
        if (pkgBuffer == nullptr)  //失败
        {
            log_compnt_mngr->error("SensorManager::generateSensorInfoPkg malloc fail, pkgSize={}", pkgSize);
            return false;
        }

        //清空buffer
        (void)memset(pkgBuffer, 0, pkgSize);

        //开始打包数据
        S_SP_MSG_ENTRY_HDR *pkgHead = (S_SP_MSG_ENTRY_HDR *)pkgBuffer;                            //Pkg的头部指针
        S_SP_SENSOR_INFO *pkgBody = (S_SP_SENSOR_INFO *)(pkgBuffer + sizeof(S_SP_MSG_ENTRY_HDR)); //Pkg的Body指针

        //填充Pkg头部
        pkgHead->u4HeaderSize = static_cast<uint32_t>(sizeof(S_SP_MSG_ENTRY_HDR));
        pkgHead->u4DataSize = static_cast<uint32_t>(sizeof(S_SP_SENSOR_INFO)) * static_cast<uint32_t>(elementNum);
        pkgHead->u4ElementSize = static_cast<uint32_t>(sizeof(S_SP_SENSOR_INFO));
        pkgHead->u2PkgId = D_SP_PKG_ID_SENSOR_INFO;

        S_SP_SENSOR_INFO *element = pkgBody; //当前填充的元素的指针

        //填充数据部分
        if (description->type == SENSOR_TYPE::SENSOR_TYPE_LIDAR)
        {
            element->u1Type = D_SP_SENSOR_TYPE_LIDAR;
        }
        else if (description->type == SENSOR_TYPE::SENSOR_TYPE_CAMERA)
        {
            element->u1Type = D_SP_SENSOR_TYPE_CAMERA;
        }
        else if (description->type == SENSOR_TYPE::SENSOR_TYPE_IMU)
        {
            element->u1Type = D_SP_SENSOR_TYPE_IMU;
        }
        else if (description->type == SENSOR_TYPE::SENSOR_TYPE_BASIC_SENSOR)
        {
            element->u1Type = D_SP_SENSOR_TYPE_NONE;
        }
        else if (description->type == SENSOR_TYPE::SENSOR_TYPE_GPS)
        {
            element->u1Type = D_SP_SENSOR_TYPE_GPS;
        }
        else if (description->type == SENSOR_TYPE::SENSOR_TYPE_RADAR)
        {
            element->u1Type = D_SP_SENSOR_TYPE_RADAR;
        }
        else if (description->type == SENSOR_TYPE::SENSOR_TYPE_ULTRASONIC)
        {
            element->u1Type = D_SP_SENSOR_TYPE_ULTRASONIC;
        }
        else if (description->type == SENSOR_TYPE::SENSOR_TYPE_V2X_OBU)
        {
            element->u1Type = D_SP_SENSOR_TYPE_V2X_OBU;
        }
        else if (description->type == SENSOR_TYPE::SENSOR_TYPE_V2X_RSU)
        {
            element->u1Type = D_SP_SENSOR_TYPE_V2X_RSU;
        }
        else //异常情况
        {
            element->u1Type = D_SP_SENSOR_TYPE_NONE;
        }

        element->u8HostId = 1; // 主车持有传感器，主车Id为1
        (void)strncpy(element->au1Name, description->name, D_SP_SIZE_OBJECT_NAME);
        element->au4FovHV[0] = description->hFov;    // 视野（水平）
        element->au4FovHV[1] = description->vFov;    // 视野（垂直）
        element->au4ClipNF[0] = description->minimumDetectRange;
        element->au4ClipNF[1] = description->range;	 // 裁剪范围（远）

        // 传感器坐标系下传感器的坐标
        element->sPos.u8X = 0.0;
        element->sPos.u8Y = 0.0;
        element->sPos.u8Z = 0.0;
        element->sPos.u4H = 0.0;
        element->sPos.u4P = 0.0;
        element->sPos.u4R = 0.0;

        // 主车坐标系下传感器的坐标
        element->sOriginCoordSys.u8X = description->assemblePositionX;
        element->sOriginCoordSys.u8Y = description->assemblePositionY;
        element->sOriginCoordSys.u8Z = description->assemblePositionZ;

        // 主车坐标系下传感器的角度
        element->sOriginCoordSys.u4H = description->heading;
        element->sOriginCoordSys.u4P = description->pitch;
        element->sOriginCoordSys.u4R = description->roll;

        element->au4FovOffHV[0] = 0.0; // 视场偏移（水平）
        element->au4FovOffHV[1] = 0.0; // 视场偏移（垂直）

        if (ConfigureMngr::getInstance()->getSensorControllerMode() == 2)
        {
            //将字符串的传感器id转换成哈希值
            size_t hash_value = hasher(sensorCfg->get_id()); // 传感器id
            element->u8Id = hash_value % std::numeric_limits<int>::max();
            log_compnt_mngr->debug("generate pluginId is ={},hash_value is= {}",element->u8Id,hash_value);
        }
        else
        {
            element->u8Id = static_cast<uint32_t>(TrafficSimulation::stoi(sensorCfg->get_id())); // 传感器id
        }

        //存入pkgBufferMap
        pkgBufferMap[sensorCfg->get_id()] = std::make_tuple(pkgBuffer, pkgSize);
    }

    log_compnt_mngr->info("SensorManager::generateSensorInfoPkg end.");
    return true;
}

// 获取SensorInfoPkg
void SensorManager::getSensorInfoPkg(const SensorId_t &sensorId, char **buffer, unsigned int *size)
{
    log_compnt_mngr->info("SensorManager::getSensorInfoPkg start.");
    auto pkgBufferMapIt = pkgBufferMap.find(sensorId);
    if (pkgBufferMapIt != pkgBufferMap.end()) //查找成功
    {
        *buffer = std::get<0>(pkgBufferMapIt->second);
        *size = std::get<1>(pkgBufferMapIt->second);
    }
    else //查找失败
    {
        log_compnt_mngr->error("SensorManager::getSensorInfoPkg error, can not find sensorId = {}", sensorId);
    }
    log_compnt_mngr->info("SensorManager::getSensorInfoPkg end.");
}

//释放pkgBufferMap
void SensorManager::releasePkgBufferMap()
{
    log_compnt_mngr->info("SensorManager::releasePkgBufferMap start.");
    //释放pkgBufferMap
    for (auto it = pkgBufferMap.begin(); it != pkgBufferMap.end(); it++)
    {
        //取出pkgBuffer
        char *&pkgBuffer = std::get<0>(it->second); //<pkgBuffer, pkgAllocSize>
        unsigned int &pkgAllocSize = std::get<1>(it->second);

        if ((pkgBuffer != nullptr))
        {
            free(pkgBuffer);
            pkgBuffer = nullptr;
            pkgAllocSize = 0;
        }
    }
    pkgBufferMap.clear();
    log_compnt_mngr->info("SensorManager::releasePkgBufferMap end.");
}
