#include "HmiPraseMsg.h"

HmiPraseMsg *HmiPraseMsg::_instance = nullptr;
HmiPraseMsg::HmiPraseMsg()
{
    recEndSensorMsg = false;
}

HmiPraseMsg::~HmiPraseMsg()
{

}

HmiPraseMsg *HmiPraseMsg::Instance()
{
    if (_instance == nullptr)
    {
        _instance = new HmiPraseMsg();
    }
    return _instance;
}

std::vector<PluginCfg *> HmiPraseMsg::getSensorList()
{
    log_compnt_mngr->debug("HmiPraseMsg::getSensorList p_plugin_list size ={}",p_plugin_list.size());
    return p_plugin_list;
}

std::list<std::string> HmiPraseMsg::getSensorDisableList()
{
    m_sensor_type_name.sort();  
    last_sensor_type_name.sort(); 
    // 创建一个用于存储差异的list  
    std::list<std::string> diffList;
    // 使用set_difference计算差异
    std::set_difference(last_sensor_type_name.begin(), last_sensor_type_name.end(),
                         m_sensor_type_name.begin(), m_sensor_type_name.end(),
                         std::inserter(diffList, diffList.begin()));
    for (auto p : diffList)
    {
        log_compnt_mngr->debug("HmiPraseMsg::getSensorDisableList disable sensor name {}",p.c_str());
    }
    last_sensor_type_name.clear();
    last_sensor_type_name =  m_sensor_type_name;
    return diffList;
}

void HmiPraseMsg::clearSensorList()
{
    log_compnt_mngr->info("HmiPraseMsg::clearSensorList start.");
    //删除传感器列表中的传感器 
    for (auto p :p_plugin_list)
    {
        if (p != nullptr)
        {
            delete (p);
            p = nullptr;
        }
    }
    p_plugin_list.clear();

    //删除雷达传感器列表中的雷达传感器
    for (auto lidar :LidarDescList)
    {
        if (lidar != nullptr)
        {
            delete (lidar);
            lidar = nullptr;
        }
    }
    LidarDescList.clear();
    
    //删除摄像机传感器列表中的摄像机传感器
    for (auto camera :CameraDescList)
    {
        if (camera != nullptr)
        {
            delete (camera);
            camera = nullptr;
        }
    }
    CameraDescList.clear();

    //删除毫米波传感器列表中的毫米波传感器
    for (auto radar :RadarDescList)
    {
        if (radar != nullptr)
        {
            delete (radar);
            radar = nullptr;
        }
    }
    RadarDescList.clear();
    
    //删除超声波传感器列表中的超声波传感器
    for (auto ultrasonic :UltrasonicDescList)
    {
        if (ultrasonic != nullptr)
        {
            delete (ultrasonic);
            ultrasonic = nullptr;
        }
    }
    UltrasonicDescList.clear();
    
    //清空存储传感器的类型和名字构成的ID
    m_sensor_type_name.clear();
    log_compnt_mngr->info("HmiPraseMsg::clearSensorList end.");
}

void HmiPraseMsg::addRecvMsg(char *msgBuff, int msgSize)
{
    log_compnt_mngr->info("HmiPraseMsg::addRecvMsg start.");
    log_compnt_mngr->debug("HmiPraseMsg::addRecvMsg msgSize is {}, recvMsgBufUsedSize is {}", msgSize, recvMsgBufUsedSize);

    if ((msgBuff != NULL) && (msgSize > 0))
    {
       log_compnt_mngr->debug("HmiPraseMsg::addRecvMsg msgBuff is not null");

        // recvMsgBuf剩余大小
        int recvMsgBufFreeSize = RECV_BUF_SIZE - recvMsgBufUsedSize;

        // 如果该client的MsgBuffer剩余大小 足够多
        if (recvMsgBufFreeSize >= msgSize)
        {
            log_compnt_mngr->debug("HmiPraseMsg::addRecvMsg recvMsgBufFreeSize >= msgSize");

            // 将新消息 追加到 recvMsgBuf 末尾
            memcpy(recvMsgBuf + recvMsgBufUsedSize, msgBuff, msgSize);

            // 该client的MsgBuffer已使用大小
            recvMsgBufUsedSize += msgSize;

            // 是否包含完整的一帧数据
            bool isComplete = false;
            // 第一帧数据的大小
            int firstMsgSize = 0;

            // 检查 该client的MsgBuffer 是否包含完整的一帧数据
            checkMsgComplete(recvMsgBuf, recvMsgBufUsedSize, isComplete,
                             firstMsgSize);

            // 如果包含完整的一帧数据
            while (isComplete)
            {
                log_compnt_mngr->debug("HmiPraseMsg::addRecvMsg isComplete={}, firstMsgSize={}, recvMsgBufUsedSize={}",
                                        isComplete,firstMsgSize, recvMsgBufUsedSize);

                // 解析数据
                parseMsg(recvMsgBuf, static_cast<unsigned int>(firstMsgSize));
                // 将第一帧数据移除数据缓冲区
                if (recvMsgBufUsedSize ==
                    firstMsgSize) // 如果碰巧，recvMsgBuf里的数据不多不少正好一帧
                {
                    log_compnt_mngr->debug("HmiPraseMsg::addRecvMsg 2 isComplete={}, firstMsgSize={}, recvMsgBufUsedSize={}",
                                            isComplete, firstMsgSize, recvMsgBufUsedSize);

                    // 只要将clientMsgBufUsedSize重置为0
                    recvMsgBufUsedSize = 0; // 该client的MsgBuffer已使用大小
                }
                else if (recvMsgBufUsedSize >
                         firstMsgSize) // 如果 recvMsgBuf里的数据 大于 一帧的数据量
                {
                    // 借助交换空间，将第一帧数据移除数据缓冲区

                    // recvMsgBuf里剩余的数据
                    char *remainMsgBuf = recvMsgBuf + firstMsgSize;

                    // recvMsgBuf里剩余的数据
                    int remainMsgSize = recvMsgBufUsedSize - firstMsgSize;

                    log_compnt_mngr->debug("HmiPraseMsg::addRecvMsg isComplete={}, firstMsgSize={}, recvMsgBufUsedSize={}, remainMsgSize={}",
                                            isComplete, firstMsgSize, recvMsgBufUsedSize,remainMsgSize);

                    // 将第一帧数据移除数据缓冲区
                    //  remainMsgSize一定小于swapBuf的空间(RECV_BUF_SIZE)
                    memcpy(swapBuf, remainMsgBuf, remainMsgSize);
                    memcpy(recvMsgBuf, swapBuf, remainMsgSize);
                    // recvMsgBuf已使用大小
                    recvMsgBufUsedSize = remainMsgSize;
                }
                else // 如果 recvMsgBuf 小于 一帧的数据量
                {
                    // 不应该出现，说明checkMsgComplete()工作不正常
                    log_compnt_mngr->warn("HmiPraseMsg::addRecvMsg recvMsgBufUsedSize < firstMsgSize");
                }

                // 再次检查 recvMsgBuf 是否包含完整的一帧数据
                checkMsgComplete(recvMsgBuf, recvMsgBufUsedSize, isComplete,firstMsgSize);
            }
        }
        else // 如果recvMsgBuf剩余大小 不足
        {
            // 严重错误，丢弃掉新的消息
            log_compnt_mngr->warn("HmiPraseMsg::addRecvMsg recvMsgBufFreeSize not enough. recvMsgBufFreeSize={}, msgSize={}",recvMsgBufFreeSize, msgSize);
        }
    }
    log_compnt_mngr->info("HmiPraseMsg::addRecvMsg end.");
}

void HmiPraseMsg::parseMsg(char *msgBuff, const int &msgSize)
{
    log_compnt_mngr->info("HmiPraseMsg::parseMsg start.");
    //当前Pkg的身体指针
    CM_SENSOR_MSG_HDR_t *pMsgHead = (CM_SENSOR_MSG_HDR_t *)msgBuff;
    if (pMsgHead->dataSize == 0)
    {
        log_compnt_mngr->error("HmiPraseMsg::parseMsg dataSize is 0.");
        return;
    }
    log_compnt_mngr->debug("HmiPraseMsg::parseMsg msgHeadsize={}, msgDataSize={}",pMsgHead->headerSize, pMsgHead->dataSize);

    if (msgSize != pMsgHead->headerSize + pMsgHead->dataSize)
    {
        log_compnt_mngr->error("HmiPraseMsg::parseMsg pkgLen error.");
        return;
    }
    
    //HMI 传感器发送完成的标志
    recEndSensorMsg = pMsgHead->sendover;
    //当前Pkg的身体指针
    char *currentPkg = msgBuff + pMsgHead->headerSize;

    log_compnt_mngr->debug("HmiPraseMsg::parseMsg sensor type is {}.",pMsgHead->sensorType);
    //当前传感器类型激光雷达和端口没有被使用
    if (pMsgHead->sensorType == CM_SENSOR_TYPE_LIDAR && !pMsgHead->isPortUse)
    {
        LidarDescription_simpro *desc = reinterpret_cast<LidarDescription_simpro *>(currentPkg);
        //传感器开关打开，并且不是物理传感器
        if (desc->enable && (desc->modelType != SENSOR_MODEL_PHYSIC))
        {   
            parseLidarMsg(desc);
        }
    }
    //当前传感器类型摄像机和端口没有被使用
    else if(pMsgHead->sensorType == CM_SENSOR_TYPE_CAMERA && !pMsgHead->isPortUse)
    {
        CameraDescription_simpro *desc = reinterpret_cast<CameraDescription_simpro *>(currentPkg);
        //传感器开关打开，并且不是物理传感器
        if (desc->enable && (desc->modelType != SENSOR_MODEL_PHYSIC))
        {
           parseCameraMsg(desc);
        }
    }
    //当前传感器类型毫米波和端口没有被使用
    else if(pMsgHead->sensorType == CM_SENSOR_TYPE_RADAR && !pMsgHead->isPortUse)
    {
        RadarDescription_simpro *desc = reinterpret_cast<RadarDescription_simpro *>(currentPkg);
        //传感器开关打开，并且不是物理传感器
        if (desc->enable && (desc->modelType != SENSOR_MODEL_PHYSIC))
        {
            parseRadarMsg(desc);
        }
    }
    //当前传感器类型超声波和端口没有被使用
    else if(pMsgHead->sensorType == CM_SENSOR_TYPE_ULTRASONIC && !pMsgHead->isPortUse)
    {
        UltrasonicDescription_simpro *desc = reinterpret_cast<UltrasonicDescription_simpro *>(currentPkg);
        //传感器开关打开，并且不是物理传感器
        if (desc->enable && (desc->modelType != SENSOR_MODEL_PHYSIC))
        {
            parseUltrasonicMsg(desc);
        }
    }
    else
    {
        //do nothing
    }
    log_compnt_mngr->info("HmiPraseMsg::parseMsg end.");
    return;
}

void HmiPraseMsg::checkMsgComplete(char *buffer, int bufferSize, bool &isComplete, int &msgSize)
{
    log_compnt_mngr->info("HmiPraseMsg::checkMsgComplete start.");
    log_compnt_mngr->debug("HmiPraseMsg::checkMsgComplete start bufferSize={}", bufferSize);
    // 返回值：是否包含完整的一帧数据
    isComplete = false;
    // 返回值：当buffer中包含完整的一帧数据的头部时，返回第一帧数据的大小
    msgSize = 0;

    if ((buffer != nullptr) && (bufferSize > 0))
    {
        // 说明包含了完整的MsgHeader
        if (static_cast<unsigned int>(bufferSize) >= sizeof(CM_SENSOR_MSG_HDR_t))
        {
            // msg 头部指针
            CM_SENSOR_MSG_HDR_t *msgHeader = reinterpret_cast<CM_SENSOR_MSG_HDR_t *>(buffer);
            // 返回值：msg总大小
            msgSize = msgHeader->headerSize + msgHeader->dataSize;

            // 说明包含了完整的Msg
            if (bufferSize >= msgSize)
            {
                // 返回值：是否包含完整的一帧数据
                isComplete = true;
            }
        }
    }
    log_compnt_mngr->debug("HmiPraseMsg::checkMsgComplete end isComplete = {}, msgSize ={}.", isComplete,msgSize);
    log_compnt_mngr->info("HmiPraseMsg::checkMsgComplete end.");
}

void HmiPraseMsg::praseMSgsensor(const SensorDescription *desc, const std::string &sensorType, const int &sensorFrameRate,const int &EnableSecket,
                                   const int &Type,const std::string &IP,const int &Port)
{
    log_compnt_mngr->info("HmiPraseMsg::praseMSgsensor start.");
    SensorDescription *tmpDesc = nullptr;
    SensorCfg *p_sensorcfg = new SensorCfg(tmpDesc);
    std::string tmpsesnorid = std::to_string(desc->type) + "_type_" + desc->name;

    p_sensorcfg->set_name(desc->name);
    p_sensorcfg->set_type(sensorType);
    p_sensorcfg->set_id(tmpsesnorid);
    p_sensorcfg->setFrmRate(sensorFrameRate);
    log_compnt_mngr->debug("HmiPraseMsg::praseMSgsensor desc->frameRate = {}",sensorFrameRate);
    std::string lib = "libSimProSensorPlugin.so";
    p_sensorcfg->set_lib(lib);
    p_sensorcfg->set_position(desc->assemblePositionX, desc->assemblePositionY, desc->assemblePositionZ, desc->heading, desc->pitch, desc->roll);
    //根据传感器的通信端口配置开关，打开则设置true,否则为false
    if (EnableSecket)
    {
        p_sensorcfg->set_socket_enable("true");
        m_sensor_type_name.push_back(tmpsesnorid);
    }
    else
    {
        p_sensorcfg->set_socket_enable("false");
    }
    std::string CommunicationPortType = "";
    //根据传感器的通信端口配置的通信类型，'1'TCP  '2'UDP（默认）
    if (Type == 1)
    {
        CommunicationPortType = "tcp";
    }
    else
    {
        CommunicationPortType = "udp";
    }
    log_compnt_mngr->debug("desc->name = {},desc->CommunicationPortIP={},desc->CommunicationPort={},CommunicationPortType={}",
                            desc->name,IP,Port,CommunicationPortType);
    p_sensorcfg->set_socket(desc->name, IP,  std::to_string(Port), CommunicationPortType);
    p_sensorcfg->add_filter("pedestrian");
    p_sensorcfg->add_filter("vehicle");
    p_sensorcfg->add_filter("trafficSign");
    p_sensorcfg->add_filter("obstacle");
    p_sensorcfg->set_fov(desc->hFov,desc->vFov);
    p_sensorcfg->set_detect_dist(desc->minimumDetectRange, desc->range);
    //根据BoundingBox输出的开关
    p_sensorcfg->set_boundingbox_enable(desc->boundingBoxEnable ? "true" : "false");
    p_plugin_list.push_back(p_sensorcfg);

    log_compnt_mngr->info("HmiPraseMsg::praseMSgsensor end.");
}

std::list<LidarDescription*> HmiPraseMsg::getLidarDescList()
{
    return LidarDescList;
}

std::list<CameraDescription*> HmiPraseMsg::getCameraDescList()
{
    return CameraDescList;
}

std::list<RadarDescription*> HmiPraseMsg::getRadarDescList()
{
    return RadarDescList;
}

std::list<UltrasonicDescription*> HmiPraseMsg::getUltrasonicDescList()
{
    return UltrasonicDescList;
}

bool HmiPraseMsg::getEndSensorMsg()
{
    return recEndSensorMsg;
}

//将simpro sensor lidar 配置复制到CM  lidar description　
void HmiPraseMsg::parseLidarMsg(LidarDescription_simpro *desc)
{
    log_compnt_mngr->info("HmiPraseMsg::parseLidarMsg start.");
    if (desc == nullptr)
    {
        log_compnt_mngr->error("HmiPraseMsg::parseLidarMsg desc is nullptr.");
        return;
    }

    LidarDescription *tmpLidarDesc = new LidarDescription();

    (void)strncpy(tmpLidarDesc->name, desc->name, sizeof(tmpLidarDesc->name));//传感器名称

    tmpLidarDesc->type = desc->type;  //传感器类型
    tmpLidarDesc->enable = desc->enable;  //传感器开关
    tmpLidarDesc->modelType = desc->modelType;  //传感器模型类型
    tmpLidarDesc->frameRate = desc->frameRate ; //传感器频率
    tmpLidarDesc->object_detection_type = desc->object_detection_type ; //传感器识别类型
    tmpLidarDesc->enableConeRender = desc->enableConeRender; //视锥探测开关(包络线)
    tmpLidarDesc->boundingBoxEnable = desc->boundingBoxEnable;   //输出包围盒信息开关

    tmpLidarDesc->assemblePositionX = desc->assemblePositionX; //安装X轴位置
    tmpLidarDesc->assemblePositionY = desc->assemblePositionY; //安装Y轴位置
    tmpLidarDesc->assemblePositionZ = desc->assemblePositionZ; //安装Z轴位置
    tmpLidarDesc->heading = desc->heading; //航向角
    tmpLidarDesc->pitch = desc->pitch; //俯仰角
    tmpLidarDesc->roll = desc->roll; //横滚角

    tmpLidarDesc->hFov = desc->horizontalFov; // HFOV
    tmpLidarDesc->vFov = desc->verticalFov; //VFOV
    tmpLidarDesc->minimumDetectRange = desc->minimumDetectRange; //最近探测距离
    tmpLidarDesc->range = desc->range; //最远探测距离
    
    tmpLidarDesc->advSettingsEnable = desc->advSettingsEnable; //高级参数开启开关
    tmpLidarDesc->bExistProbInput = desc->bExistProbInput; // 目标物存在可能性开关
    tmpLidarDesc->fExistProb = desc->fExistProb; // 目标物存在可能性

    tmpLidarDesc->horizontalResolution = desc->horizontalResolution; //水平分辨率
    tmpLidarDesc->fWavelength = desc->fWaveLength; //波长
    tmpLidarDesc->peakOpticalPower = desc->peakOpticalPower; //激光峰值功率
    tmpLidarDesc->fTransmittingAperture = desc->fTransmittingAperture; //发射孔径
    tmpLidarDesc->fReceivingAperture = desc->fReceivingAperture; //接收孔径
    tmpLidarDesc->fTransmitTransmissivity = desc->fTransmitTransmissivity; //发射孔径透射率
    tmpLidarDesc->fReceiveTransmissivity = desc->fReceiveTransmissivity; //接收孔径透射率
    tmpLidarDesc->fVBeamDivergenceAngle = desc->fVBeamDivergenceAngle; //光束垂直发散角
    tmpLidarDesc->fHBeamDivergenceAngle = desc->fHBeamDivergenceAngle; //光束水平发散角

    tmpLidarDesc->fNEP = desc->fNEP; //噪声等效功率
    tmpLidarDesc->fNoiseBandwidth = desc->fNoiseBandwidth; //噪声带宽
    
    printfLidarMsg(tmpLidarDesc);
    LidarDescList.push_back(tmpLidarDesc);
    praseMSgsensor(tmpLidarDesc,"lidar",tmpLidarDesc->frameRate,desc->EnableCommunicationPort,desc->CommunicationPortType,
                    desc->CommunicationPortIP,desc->CommunicationPort);
    log_compnt_mngr->info("HmiPraseMsg::parseLidarMsg end.");
}

//将simpro sensor Camera 配置复制到CM  Camera description
void HmiPraseMsg::parseCameraMsg(CameraDescription_simpro *desc)
{
    log_compnt_mngr->info("HmiPraseMsg::parseCameraMsg start.");
    if (desc == nullptr)
    {
        log_compnt_mngr->error("HmiPraseMsg::parseCameraMsg desc is nullptr.");
        return;
    }

    CameraDescription *tmpCameraDesc = new CameraDescription();
    (void)strncpy(tmpCameraDesc->name, desc->name, sizeof(tmpCameraDesc->name));//传感器名称
    tmpCameraDesc->type = desc->type;  //传感器类型
    tmpCameraDesc->enable = desc->enable;  //传感器开关
    tmpCameraDesc->modelType = desc->modelType;  //传感器模型类型
    tmpCameraDesc->frameRate = desc->frameRate; //传感器频率
    tmpCameraDesc->object_detection_type = desc->object_detection_type; //传感器识别类型
    tmpCameraDesc->enableConeRender = desc->enableConeRender; //视锥探测开关(包络线)
    tmpCameraDesc->boundingBoxEnable = desc->boundingBoxEnable;   //输出包围盒信息开关

    tmpCameraDesc->assemblePositionX = desc->assemblePositionX; //安装X轴位置
    tmpCameraDesc->assemblePositionY = desc->assemblePositionY; //安装Y轴位置
    tmpCameraDesc->assemblePositionZ = desc->assemblePositionZ; //安装Z轴位置
    tmpCameraDesc->heading = desc->heading; //航向角
    tmpCameraDesc->pitch = desc->pitch; //俯仰角
    tmpCameraDesc->roll = desc->roll; //横滚角

    tmpCameraDesc->hFov = desc->horizontalFov; // HFOV
    tmpCameraDesc->vFov = desc->verticalFov; //VFOV
    tmpCameraDesc->minimumDetectRange = desc->minimumDetectRange; //最近探测距离
    tmpCameraDesc->range = desc->detectDis; //最远探测距离

    tmpCameraDesc->sensorSizeL = desc->sensorSizeL; //传感器长度
    tmpCameraDesc->sensorSizeW = desc->sensorSizeW;//传感器宽度
    tmpCameraDesc->sensorFD = desc->sensorFD;//传感器焦距
    
    tmpCameraDesc->advSettingsEnable = desc->advSettingsEnable;//高级参数开启开关
    tmpCameraDesc->resolutionL = desc->resolutionL; //水平分辨率
    tmpCameraDesc->resolutionV = desc->resolutionV; //垂直分辨率

    tmpCameraDesc->pixelSize = desc->pixelSize;   //像素尺寸
    tmpCameraDesc->gaussSigma = desc->gaussSigma;  //高斯标准差
    tmpCameraDesc->distSw = desc->distSw;           //畸变开关
    tmpCameraDesc->distCenterL = desc->distCenterL; //横向畸变中心
    tmpCameraDesc->distCenterV = desc->distCenterV; //垂直畸变中心
    tmpCameraDesc->distParamK1 = desc->distParamK1; //畸变参数K1
    tmpCameraDesc->distParamK2 = desc->distParamK2; //畸变参数K2

    printfCameraMsg(tmpCameraDesc);
    CameraDescList.push_back(tmpCameraDesc);
    praseMSgsensor(tmpCameraDesc,"camera",tmpCameraDesc->frameRate,desc->EnableCommunicationPort,desc->CommunicationPortType,
                    desc->CommunicationPortIP,desc->CommunicationPort);
    log_compnt_mngr->info("HmiPraseMsg::parseCameraMsg end.");
}

//将simpro sensor Radar 配置复制到CM  Radar description
void HmiPraseMsg::parseRadarMsg(RadarDescription_simpro *desc)
{
    log_compnt_mngr->info("HmiPraseMsg::parseRadarMsg start.");
    if (desc == nullptr)
    {
        log_compnt_mngr->error("HmiPraseMsg::parseRadarMsg desc is nullptr.");
        return;
    }

    RadarDescription *tmpRadarDesc = new RadarDescription();
    (void)strncpy(tmpRadarDesc->name, desc->name, sizeof(tmpRadarDesc->name));//传感器名称
    tmpRadarDesc->type = desc->type;  //传感器类型
    tmpRadarDesc->enable = desc->enable;  //传感器开关
    tmpRadarDesc->modelType = desc->modelType;  //传感器模型类型
    tmpRadarDesc->frameRate = desc->frameRate; //传感器频率
    tmpRadarDesc->object_detection_type = desc->object_detection_type; //传感器识别类型
    tmpRadarDesc->enableConeRender = desc->enableConeRender; //视锥探测开关(包络线)
    tmpRadarDesc->boundingBoxEnable = desc->boundingBoxEnable; //输出包围盒信息开关

    tmpRadarDesc->assemblePositionX = desc->assemblePositionX; //安装X轴位置
    tmpRadarDesc->assemblePositionY = desc->assemblePositionY; //安装Y轴位置
    tmpRadarDesc->assemblePositionZ = desc->assemblePositionZ; //安装Z轴位置
    tmpRadarDesc->heading = desc->heading; //航向角
    tmpRadarDesc->pitch = desc->pitch; //俯仰角
    tmpRadarDesc->roll = desc->roll; //横滚角

    tmpRadarDesc->hFov = desc->hFov; // HFOV
    tmpRadarDesc->vFov = desc->vFov; //VFOV
    tmpRadarDesc->minimumDetectRange = desc->minimumDetectRange; //最近探测距离
    tmpRadarDesc->range = desc->range; //最远探测距离
    
    tmpRadarDesc->advSettingsEnable = desc->advSettingsEnable; //高级参数开启开关

    tmpRadarDesc->bExistProbInput = desc->bExistProbInput; // 目标物存在可能性开关
    tmpRadarDesc->fExistProb = desc->fExistProb; // 目标物存在可能性
    
    tmpRadarDesc->horizontalResolution = desc->horizontalResolution; //水平分辨率
    tmpRadarDesc->transmitterPower = desc->transmitterPower;//发射功率
    tmpRadarDesc->effectiveArea = desc->effectiveArea;//天线有效面积
    tmpRadarDesc->transmitGain =  desc->transmitGain; //发射天线增益
    tmpRadarDesc->sensitivityThreshold = desc->sensitivityThreshold; //敏感性阈值

    tmpRadarDesc->azimuthMSENoise = desc->azimuthMSENoise; //方位角均方差噪声
    tmpRadarDesc->elevationMSENoise = desc->elevationMSENoise; //高低角均方差噪声
    tmpRadarDesc->rangeMSENoise = desc->rangeMSENoise;//距离均方差噪声 
    
    printfRadarMsg(tmpRadarDesc);
    RadarDescList.push_back(tmpRadarDesc);
    praseMSgsensor(tmpRadarDesc,"radar",tmpRadarDesc->frameRate,desc->EnableCommunicationPort,desc->CommunicationPortType,
                    desc->CommunicationPortIP,desc->CommunicationPort);
    log_compnt_mngr->info("HmiPraseMsg::parseRadarMsg end.");
}

//将simpro sensor Ultrasonic 配置复制到CM Ultrasonic description
void HmiPraseMsg::parseUltrasonicMsg(UltrasonicDescription_simpro *desc)
{
    log_compnt_mngr->info("HmiPraseMsg::parseUltrasonicMsg start.");
    if (desc == nullptr)
    {
        log_compnt_mngr->error("HmiPraseMsg::parseUltrasonicMsg desc is nullptr.");
        return;
    }

    UltrasonicDescription *tmpUltrasonicDesc = new UltrasonicDescription();
    (void)strncpy(tmpUltrasonicDesc->name, desc->name, sizeof(tmpUltrasonicDesc->name));//传感器名称
    tmpUltrasonicDesc->type = desc->type;  //传感器类型
    tmpUltrasonicDesc->enable = desc->enable;  //传感器开关
    tmpUltrasonicDesc->modelType = desc->modelType;  //传感器模型类型
    tmpUltrasonicDesc->frameRate = desc->frameRate; //传感器频率
    tmpUltrasonicDesc->object_detection_type = desc->object_detection_type; //传感器识别类型
    tmpUltrasonicDesc->enableConeRender = desc->enableConeRender; //视锥探测开关(包络线)
    tmpUltrasonicDesc->boundingBoxEnable = desc->boundingBoxEnable;   //输出包围盒信息开关

    tmpUltrasonicDesc->assemblePositionX = desc->assemblePositionX; //安装X轴位置
    tmpUltrasonicDesc->assemblePositionY = desc->assemblePositionY; //安装Y轴位置
    tmpUltrasonicDesc->assemblePositionZ = desc->assemblePositionZ; //安装Z轴位置
    tmpUltrasonicDesc->heading = desc->heading; //航向角
    tmpUltrasonicDesc->pitch = desc->pitch; //俯仰角
    tmpUltrasonicDesc->roll = desc->roll; //横滚角

    tmpUltrasonicDesc->hFov = desc->hFov; // HFOV
    tmpUltrasonicDesc->vFov = desc->vFov; //VFOV
    tmpUltrasonicDesc->minimumDetectRange = desc->minimumDetectRange; //最近探测距离
    tmpUltrasonicDesc->range = desc->range; //最远探测距离
    
    tmpUltrasonicDesc->advSettingsEnable = desc->advSettingsEnable; //高级参数开启开关

    tmpUltrasonicDesc->bExistProbInput = desc->bExistProbInput; // 目标物存在可能性开关
    tmpUltrasonicDesc->fExistProb = desc->fExistProb; // 目标物存在可能性
    
    tmpUltrasonicDesc->horizontalResolution = desc->horizontalResolution; //水平分辨率
    tmpUltrasonicDesc->azimuthGaussDistribution = desc->azimuthGaussDistribution;//方位角高斯分布
    tmpUltrasonicDesc->SPL = desc->SPL;//发射 Sound Pressure Level
    tmpUltrasonicDesc->decayTime =  desc->decayTime; //衰变时间
    
    tmpUltrasonicDesc->noiseLevel = desc->noiseLevel; //噪声水平
    tmpUltrasonicDesc->temp = desc->temp; //环境温度
    tmpUltrasonicDesc->humidity = desc->humidity; //空气湿度

    printfUltrasonicMsg(tmpUltrasonicDesc);
    UltrasonicDescList.push_back(tmpUltrasonicDesc);
	praseMSgsensor(tmpUltrasonicDesc,"ultrasonic",tmpUltrasonicDesc->frameRate,desc->EnableCommunicationPort,desc->CommunicationPortType,
                    desc->CommunicationPortIP,desc->CommunicationPort);
    log_compnt_mngr->info("HmiPraseMsg::parseUltrasonicMsg end.");
}

//输出CM 获取到simpro 雷达传感器配置到日志
void HmiPraseMsg::printfLidarMsg(LidarDescription *tmpLidarDesc)
{
    if (tmpLidarDesc == nullptr)
    {
        return;
    }

    log_compnt_mngr->trace("传感器名称: sensor->name = {}",tmpLidarDesc->name);  //传感器名称
    log_compnt_mngr->trace("传感器类型: sensor->type = {}",tmpLidarDesc->type);  //传感器类型
    log_compnt_mngr->trace("传感器开关: sensor->enable = {}",tmpLidarDesc->enable);  //传感器开关
    log_compnt_mngr->trace("传感器模型类型: sensor->modelType = {}", tmpLidarDesc->modelType);  //传感器模型类型
    log_compnt_mngr->trace("传感器频率 10: sensor->frameRate = {}", tmpLidarDesc->frameRate); //传感器频率
    log_compnt_mngr->trace("传感器识别类型: sensor->object_detection_type = {}",tmpLidarDesc->object_detection_type); //传感器识别类型
    log_compnt_mngr->trace("视锥探测开关: sensor->enableConeRender = {}",tmpLidarDesc->enableConeRender); //视锥探测开关(包络线)
    log_compnt_mngr->trace("输出包围盒信息开关: sensor->boundingBoxEnable = {}",tmpLidarDesc->boundingBoxEnable); //输出包围盒信息开关

    log_compnt_mngr->trace("安装X轴位置 0: sensor->assemblePositionX = {}",tmpLidarDesc->assemblePositionX); //安装X轴位置
    log_compnt_mngr->trace("安装Y轴位置 0: sensor->assemblePositionY = {}",tmpLidarDesc->assemblePositionY ); //安装Y轴位置
    log_compnt_mngr->trace("安装Z轴位置 1.5: sensor->assemblePositionZ = {}",tmpLidarDesc->assemblePositionZ ); //安装Z轴位置
    log_compnt_mngr->trace("航向角 0: sensor->heading = {}",tmpLidarDesc->heading); //航向角
    log_compnt_mngr->trace("俯仰角 0: sensor->pitch = {}",tmpLidarDesc->pitch ); //俯仰角
    log_compnt_mngr->trace("横滚角 0: sensor->roll = {}",tmpLidarDesc->roll ); //横滚角

    log_compnt_mngr->trace("HFOV 360: sensor->hFov = {}",tmpLidarDesc->hFov); // HFOV
    log_compnt_mngr->trace("VFOV 40: sensor->vFov = {}",tmpLidarDesc->vFov); //VFOV
    log_compnt_mngr->trace("最近探测距离 10: sensor->minimumDetectRange = {}",tmpLidarDesc->minimumDetectRange ); //最近探测距离
    log_compnt_mngr->trace("最远探测距离 250: sensor->range = {}",tmpLidarDesc->range ); //最远探测距离
    
    log_compnt_mngr->trace("高级参数开启开关: sensor->advSettingsEnable = {}" , tmpLidarDesc->advSettingsEnable);//高级参数开启开关
    log_compnt_mngr->trace("目标物存在可能性开关: sensor->bExistProbInput = {}",tmpLidarDesc->bExistProbInput); // 目标物存在可能性开关
    log_compnt_mngr->trace("目标物存在可能性 100: sensor->fExistProb = {}",tmpLidarDesc->fExistProb); // 目标物存在可能性

    log_compnt_mngr->trace("水平分辨率 1: sensor->horizontalResolution = {}",tmpLidarDesc->horizontalResolution); //水平分辨率
    log_compnt_mngr->trace("波长 1550: sensor->fWavelength = {}",tmpLidarDesc->fWavelength ); //波长
    log_compnt_mngr->trace("激光峰值功率 80: sensor->peakOpticalPower = {}",tmpLidarDesc->peakOpticalPower ); //激光峰值功率
    log_compnt_mngr->trace("发射孔径 0.15: sensor->fTransmittingAperture = {}",tmpLidarDesc->fTransmittingAperture); //发射孔径
    log_compnt_mngr->trace("接收孔径 0.15: sensor->fReceivingAperture = {}",tmpLidarDesc->fReceivingAperture ); //接收孔径
    log_compnt_mngr->trace("发射孔径透射率 0.7: sensor->fTransmitTransmissivity = {}",tmpLidarDesc->fTransmitTransmissivity ); //发射孔径透射率
    log_compnt_mngr->trace("接收孔径透射率 0.7: sensor->fReceiveTransmissivity = {}",tmpLidarDesc->fReceiveTransmissivity ); //接收孔径透射率
    log_compnt_mngr->trace("光束垂直发散角 0.6: sensor->fVBeamDivergenceAngle = {}",tmpLidarDesc->fVBeamDivergenceAngle ); //光束垂直发散角
    log_compnt_mngr->trace("光束水平发散角 0.6: sensor->fHBeamDivergenceAngle = {}",tmpLidarDesc->fHBeamDivergenceAngle ); //光束水平发散角

    log_compnt_mngr->trace("噪声等效功率 1: sensor->fNEP = {}",tmpLidarDesc->fNEP); //噪声等效功率
    log_compnt_mngr->trace("噪声带宽 1: sensor->fNoiseBandwidth = {}",tmpLidarDesc->fNoiseBandwidth); //噪声带宽
}

//输出CM 获取到simpro 摄像机配置到日志
void HmiPraseMsg::printfCameraMsg(CameraDescription *tmpCameraDesc)
{
    if (tmpCameraDesc == nullptr)
    {
       return;
    }

    log_compnt_mngr->trace("传感器名称: sensor->name = {}",tmpCameraDesc->name);  //传感器名称
    log_compnt_mngr->trace("传感器类型: sensor->type = {}",tmpCameraDesc->type);  //传感器类型
    log_compnt_mngr->trace("传感器开关: sensor->enable = {}",tmpCameraDesc->enable);  //传感器开关
    log_compnt_mngr->trace("传感器模型类型: sensor->modelType = {}",tmpCameraDesc->modelType);  //传感器模型类型
    log_compnt_mngr->trace("传感器频率10: sensor->frameRate = {}",tmpCameraDesc->frameRate); //传感器频率
    log_compnt_mngr->trace("传感器识别类型: sensor->object_detection_type = {}",tmpCameraDesc->object_detection_type); //传感器识别类型
    log_compnt_mngr->trace("视锥探测开关1: sensor->enableConeRender = {}",tmpCameraDesc->enableConeRender); //视锥探测开关(包络线)
    log_compnt_mngr->trace("输出包围盒信息开关: sensor->boundingBoxEnable = {}",tmpCameraDesc->boundingBoxEnable); //输出包围盒信息开关

    log_compnt_mngr->trace("安装X轴位置0: sensor->assemblePositionX = {}" , tmpCameraDesc->assemblePositionX); //安装X轴位置
    log_compnt_mngr->trace("安装Y轴位置0: sensor->assemblePositionY = {}" , tmpCameraDesc->assemblePositionY ); //安装Y轴位置
    log_compnt_mngr->trace("安装Z轴位置1.5: sensor->assemblePositionZ = {}" , tmpCameraDesc->assemblePositionZ ); //安装Z轴位置
    log_compnt_mngr->trace("航向角0: sensor->heading = {}" , tmpCameraDesc->heading); //航向角
    log_compnt_mngr->trace("俯仰角0: sensor->pitch = {}" , tmpCameraDesc->pitch ); //俯仰角
    log_compnt_mngr->trace("横滚角0: sensor->roll = {}" , tmpCameraDesc->roll ); //横滚角

    log_compnt_mngr->trace("HFOV50: sensor->hFov = {}" , tmpCameraDesc->hFov); // HFOV
    log_compnt_mngr->trace("VFOV28: sensor->vFov = {}" , tmpCameraDesc->vFov); //VFOV
    log_compnt_mngr->trace("最近探测距离10: sensor->minimumDetectRange = {}" , tmpCameraDesc->minimumDetectRange ); //最近探测距离
    log_compnt_mngr->trace("最远探测距离200: sensor->range = {}" , tmpCameraDesc->range ); //最远探测距离

    log_compnt_mngr->trace("传感器长度 24: sensor->sensorSizeL = {}" , tmpCameraDesc->sensorSizeL );
    log_compnt_mngr->trace("传感器宽度 16: sensor->sensorSizeW = {}" , tmpCameraDesc->sensorSizeW );
    log_compnt_mngr->trace("传感器焦距 12: sensor->sensorFD = {}" , tmpCameraDesc->sensorFD);
    
    log_compnt_mngr->trace("高级参数开启开关: sensor->advSettingsEnable = {}" , tmpCameraDesc->advSettingsEnable);
    log_compnt_mngr->trace("水平分辨率，初始化为1920: sensor->resolutionL = {}" , tmpCameraDesc->resolutionL);
    log_compnt_mngr->trace("垂直分辨率，初始化为1080: sensor->resolutionV = {}" , tmpCameraDesc->resolutionV);

    log_compnt_mngr->trace("像素尺寸0.185: sensor->pixelSize = {}" , tmpCameraDesc->pixelSize );
    log_compnt_mngr->trace("高斯标准差 0: sensor->gaussSigma = {}" , tmpCameraDesc->gaussSigma );
    log_compnt_mngr->trace("畸变开关: sensor->distSw = {}" , tmpCameraDesc->distSw );
    log_compnt_mngr->trace("横向畸变中心12: sensor->distCenterL = {}" , tmpCameraDesc->distCenterL);
    log_compnt_mngr->trace("垂直畸变中心8: sensor->distCenterV = {}" , tmpCameraDesc->distCenterV );
    log_compnt_mngr->trace("畸变参数K1 0: sensor->distParamK1 = {}" , tmpCameraDesc->distParamK1 );
    log_compnt_mngr->trace("畸变参数K2 0: sensor->distParamK2 = {}" , tmpCameraDesc->distParamK2 );
}

//输出CM 获取到simpro 毫米波配置到日志
void HmiPraseMsg::printfRadarMsg(RadarDescription *tmpRadarDesc)
{
    if (tmpRadarDesc == nullptr)
    {
       return;
    }

    log_compnt_mngr->trace("传感器名称: sensor->name = {}",tmpRadarDesc->name);  //传感器名称
    log_compnt_mngr->trace("传感器类型: sensor->type = {}",tmpRadarDesc->type);  //传感器类型
    log_compnt_mngr->trace("传感器型号: sensor->opeFreq = {}",tmpRadarDesc->opeFreq);  //传感器类型
    log_compnt_mngr->trace("传感器开关: sensor->enable = {}",tmpRadarDesc->enable);  //传感器开关
    log_compnt_mngr->trace("传感器模型类型: sensor->modelType = {}",tmpRadarDesc->modelType);  //传感器模型类型
    log_compnt_mngr->trace("传感器频率10: sensor->frameRate = {}",tmpRadarDesc->frameRate); //传感器频率
    log_compnt_mngr->trace("传感器识别类型: sensor->object_detection_type = {}",tmpRadarDesc->object_detection_type); //传感器识别类型
    log_compnt_mngr->trace("视锥探测开关1: sensor->enableConeRender = {}" , tmpRadarDesc->enableConeRender); //视锥探测开关(包络线)
    log_compnt_mngr->trace("输出BoundingBox1: sensor->boundingBoxEnable = {}" , tmpRadarDesc->boundingBoxEnable); //输出包围盒信息开关

    log_compnt_mngr->trace("安装X轴位置0: sensor->assemblePositionX = {}" , tmpRadarDesc->assemblePositionX); //安装X轴位置
    log_compnt_mngr->trace("安装Y轴位置0: sensor->assemblePositionY = {}" , tmpRadarDesc->assemblePositionY ); //安装Y轴位置
    log_compnt_mngr->trace("安装Z轴位置1.5: sensor->assemblePositionZ = {}" , tmpRadarDesc->assemblePositionZ ); //安装Z轴位置
    log_compnt_mngr->trace("航向角0: sensor->heading = {}" , tmpRadarDesc->heading); //航向角
    log_compnt_mngr->trace("俯仰角0: sensor->pitch = {}" , tmpRadarDesc->pitch ); //俯仰角
    log_compnt_mngr->trace("横滚角0: sensor->roll = {}" , tmpRadarDesc->roll ); //横滚角

    log_compnt_mngr->trace("HFOV50: sensor->hFov = {}" , tmpRadarDesc->hFov); // HFOV
    log_compnt_mngr->trace("VFOV28: sensor->vFov = {}" , tmpRadarDesc->vFov); //VFOV
    log_compnt_mngr->trace("最近探测距离 10: sensor->minimumDetectRange = {}" , tmpRadarDesc->minimumDetectRange ); //最近探测距离
    log_compnt_mngr->trace("最远探测距离 100: sensor->range = {}" , tmpRadarDesc->range ); //最远探测距离
    
    log_compnt_mngr->trace("高级参数开启开关: sensor->advSettingsEnable = {}" , tmpRadarDesc->advSettingsEnable);

    log_compnt_mngr->trace("目标物存在可能性开关: sensor->bExistProbInput = {}" , tmpRadarDesc->bExistProbInput); // 目标物存在可能性开关
    log_compnt_mngr->trace("目标物存在可能性 100: sensor->fExistProb = {}" , tmpRadarDesc->fExistProb); // 目标物存在可能性
    
    log_compnt_mngr->trace("水平分辨率 2: sensor->horizontalResolution = {}" , tmpRadarDesc->horizontalResolution );
    log_compnt_mngr->trace("发射功率 160: sensor->transmitterPower = {}" , tmpRadarDesc->transmitterPower );
    log_compnt_mngr->trace("天线有效面积 4: sensor->effectiveArea = {}" , tmpRadarDesc->effectiveArea);
    log_compnt_mngr->trace("发射天线增益 0.5: sensor->transmitGain = {}" , tmpRadarDesc->transmitGain);
    log_compnt_mngr->trace("敏感性阈值 中: sensor->sensitivityThreshold = {}",tmpRadarDesc->sensitivityThreshold);

    log_compnt_mngr->trace("方位角均方差噪声 0.3: sensor->azimuthMSENoise = {}",tmpRadarDesc->azimuthMSENoise);
    log_compnt_mngr->trace("高低角均方差噪声 0.3: sensor->elevationMSENoise = {}",tmpRadarDesc->elevationMSENoise);
    log_compnt_mngr->trace("距离均方差噪声 0.3: sensor->rangeMSENoise = {}",tmpRadarDesc->rangeMSENoise);
}

//输出CM 获取到simpro 超声波配置到日志
void HmiPraseMsg::printfUltrasonicMsg(UltrasonicDescription *tmpUltrasonicDesc)
{
    if (tmpUltrasonicDesc == nullptr)
    {
        return;
    }

    log_compnt_mngr->trace("传感器名称: sensor->name = {}",tmpUltrasonicDesc->name);  //传感器名称
    log_compnt_mngr->trace("传感器类型: sensor->type = {}",tmpUltrasonicDesc->type);  //传感器类型
    log_compnt_mngr->trace("传感器开关: sensor->enable = {}" , tmpUltrasonicDesc->enable);  //传感器开关
    log_compnt_mngr->trace("传感器模型类型: sensor->modelType = {}" , tmpUltrasonicDesc->modelType);  //传感器模型类型
    log_compnt_mngr->trace("传感器频率20: sensor->frameRate = {}" , tmpUltrasonicDesc->frameRate); //传感器频率
    log_compnt_mngr->trace("传感器识别类型: sensor->object_detection_type = {}" , tmpUltrasonicDesc->object_detection_type); //传感器识别类型
    log_compnt_mngr->trace("视锥探测开关1: sensor->enableConeRender = {}" , tmpUltrasonicDesc->enableConeRender); //视锥探测开关(包络线)
    log_compnt_mngr->trace("输出BoundingBox1: sensor->boundingBoxEnable = {}" , tmpUltrasonicDesc->boundingBoxEnable); //输出包围盒信息开关

    log_compnt_mngr->trace("安装X轴位置0: sensor->assemblePositionX = {}" , tmpUltrasonicDesc->assemblePositionX); //安装X轴位置
    log_compnt_mngr->trace("安装Y轴位置0: sensor->assemblePositionY = {}" , tmpUltrasonicDesc->assemblePositionY ); //安装Y轴位置
    log_compnt_mngr->trace("安装Z轴位置1.5: sensor->assemblePositionZ = {}" , tmpUltrasonicDesc->assemblePositionZ ); //安装Z轴位置
    log_compnt_mngr->trace("航向角0: sensor->heading = {}" , tmpUltrasonicDesc->heading); //航向角
    log_compnt_mngr->trace("俯仰角0: sensor->pitch = {}" , tmpUltrasonicDesc->pitch ); //俯仰角
    log_compnt_mngr->trace("横滚角0: sensor->roll = {}" , tmpUltrasonicDesc->roll ); //横滚角

    log_compnt_mngr->trace("HFOV50: sensor->hFov = {}" , tmpUltrasonicDesc->hFov); // HFOV
    log_compnt_mngr->trace("VFOV50: sensor->vFov = {}" , tmpUltrasonicDesc->vFov); //VFOV
    log_compnt_mngr->trace("最近探测距离 0: sensor->minimumDetectRange = {}" , tmpUltrasonicDesc->minimumDetectRange ); //最近探测距离
    log_compnt_mngr->trace("最远探测距离 100: sensor->range = {}" , tmpUltrasonicDesc->range ); //最远探测距离

    log_compnt_mngr->trace("高级参数开启开关: sensor->advSettingsEnable = {}" , tmpUltrasonicDesc->advSettingsEnable);

    log_compnt_mngr->trace("目标物存在可能性开关: sensor->bExistProbInput = {}" , tmpUltrasonicDesc->bExistProbInput); // 目标物存在可能性开关
    log_compnt_mngr->trace("目标物存在可能性 100: sensor->fExistProb = {}" , tmpUltrasonicDesc->fExistProb); // 目标物存在可能性
    
    log_compnt_mngr->trace("水平分辨率 1: sensor->horizontalResolution = {}" , tmpUltrasonicDesc->horizontalResolution );
    log_compnt_mngr->trace("方位角高斯分布 0.3: sensor->azimuthGaussDistribution = {}" , tmpUltrasonicDesc->azimuthGaussDistribution );
    log_compnt_mngr->trace("发射 Sound Pressure Level 30: sensor->SPL = {}" , tmpUltrasonicDesc->SPL);
    log_compnt_mngr->trace("衰变时间 1: sensor->decayTime = {}" , tmpUltrasonicDesc->decayTime);

    log_compnt_mngr->trace("噪声水平 0.5: sensor->noiseLevel = {}" , tmpUltrasonicDesc->noiseLevel );
    log_compnt_mngr->trace("环境温度 22: sensor->temp = {}" , tmpUltrasonicDesc->temp );
    log_compnt_mngr->trace("空气湿度 0: sensor->humidity = {}" , tmpUltrasonicDesc->humidity );
}