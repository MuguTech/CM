#include "OSIGroundTruthGeneratorThread.h"
#include <google/protobuf/text_format.h>
#include "../../../Runtime/MsgAdapter/MsgAdapterType.h"
#include "../../../APF/RoadSystem/RoadSystem.h"
#include <iostream>
#include <set>
#include "../log.h"
#include "../Evaluation/EvaluationAPI.h"
#include "../ConfigureMngr.h"

OSIGroundTruthGeneratorThread *OSIGroundTruthGeneratorThread::_instance = nullptr;

OSIGroundTruthGeneratorThread *OSIGroundTruthGeneratorThread::Instance()
{
    if (!_instance)
    {
        _instance = new OSIGroundTruthGeneratorThread();
    }
    return _instance;
}

void OSIGroundTruthGeneratorThread::Destroy()
{
    delete _instance;
    _instance = nullptr;
}

OSIGroundTruthGeneratorThread::OSIGroundTruthGeneratorThread()
{ 
    memset(&pbData, 0, sizeof(OSI_PB_DATA));
    timer = 0.0;
    lastTimer = 0.0;
    fd = -1;
    count = 1;
    diffDisEgo = 40000.0;
    direction = 0;
    egoTraveledDist = 0.0;
    lastEgoTraveledDist = 0.0;
    time_str = "";
    simulationRunStatus = true;
    environment = nullptr;
    isFirstFrame = true;
    param = {0};
    evaluationDefault = "0";
    scenarioName = "1.xosc";
    scenarioFilePath = "/opt/simpro/workspace/xosc/1.xosc";
    currentPbFilePath = "";
    csvFilePath = "";
    lastPbFilePath = "";
    lastCsvFilePath = "";
    egoRange = 120.0;

    param.cmdId = D_EVALUATION_CMD_RUN_OFFLINE_TOOL;
    param.scenarioTestTimes = 1;
    (void)strcpy(reinterpret_cast<char *>(param.pressStartTime), evaluationDefault.c_str());        /* 按下开始按钮的时间(物理时间) */
    (void)strcpy(reinterpret_cast<char *>(param.scenarioName), scenarioName.c_str());               /* 场景文件名 */
    (void)strcpy(reinterpret_cast<char *>(param.scenarioFilePath), scenarioFilePath.c_str());       /* 存储泛化前的初始场景文件路径(TBD) */
    param.physicalTime = 0;         /* 场景实际用时 */
    param.simulationTime = 0;       /* 场景仿真用时 */
    param.isFinish = false;

    //信号量初始化
    int ret = sem_init(&sem, 0, 0);
    if (ret == -1)
    {
        log_compnt_mngr->error("sem sem_init error.");
    }
    ret = sem_init(&semParseXodr, 0, 0);
    if (ret == -1)
    {
        log_compnt_mngr->error("semParseXodr sem_init error.");
    }

    osiMsgBuffer = nullptr; //osiPkg内存地址
    osiMsgBufferAllocSize = 0; //osiPkg申请内存大小
}

OSIGroundTruthGeneratorThread::~OSIGroundTruthGeneratorThread()
{
}

// 创建OSIpb文件生成线程
void OSIGroundTruthGeneratorThread::creatGeneratorOsiPbFileThread()
{
    if (ConfigureMngr::getInstance()->getCoSimuInterfaceType() == 1)
    {
        generator_osi_pb_file_handle = std::thread(&OSIGroundTruthGeneratorThread::recvMsgFromMq, this);
    }
    else
    {
        save_osi_pb_file_handle = std::thread(&OSIGroundTruthGeneratorThread::savePbDataFileThread, this);
        if (osiMsgBuffer == nullptr)
        {
            osiMsgBuffer = reinterpret_cast<char *>(malloc(OSI_BUF_SIZE));
            if (osiMsgBuffer != nullptr)
            {
                osiMsgBufferAllocSize = OSI_BUF_SIZE;
            }
        }
    }
}

void OSIGroundTruthGeneratorThread::recvMsgFromMq()
{
    sem_wait(&semParseXodr);
    xodrFilePath = PluginMngr::get_instance()->get_xodr();
    (void)strcpy(reinterpret_cast<char *>(param.xodrPath), xodrFilePath.c_str());
    parseVehicleCatalogFile();
    
    int semParseXodrValue = 0;
    sem_getvalue(&semParseXodr, &semParseXodrValue);

    while (true)
    {
        //等待解析下一帧数据
        sem_wait(&sem);

        int semValue = 0;
        sem_getvalue(&sem, &semValue);
        log_compnt_mngr->critical("OSIGroundTruthGeneratorThread recvMsgFromMq semValue = {}.", semValue);
        double osiParseMsgStart = get_cur_time_ms();
        log_compnt_mngr->critical("OSIGroundTruthGeneratorThread recvMsgFromMq osiParseMsgStart time = {}.", osiParseMsgStart);

        osiMsg.msgMutex.lock();

        if (osiMsg.msgList.size() > 0)
        {
            const auto &it = osiMsg.msgList.begin();
            pbData = *it;
            osiMsg.msgList.pop_front();            
        }

        osiMsg.msgMutex.unlock();

        //解析simpro发送给cm的数据用来填充pb文件数据
        if (parseMsg(pbData.msg, pbData.msgLen) == false)
        {
            log_compnt_mngr->error("parseMsg error");
            if (pbData.msg != nullptr)
            {
                delete [] pbData.msg;
            }
            break;
        }

        if (isFirstFrame)
        {
            // 获取当前时间点
            auto now = std::chrono::system_clock::now();

            // 转换为 time_t 类型
            std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);

            // 将 time_t 类型转换为本地时间
            std::tm* now_tm = std::localtime(&now_time_t);

            // 使用 std::strftime 将时间转换为字符串
            char buffer[80];
            std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", now_tm);
            time_str = buffer;

            //创建本次仿真pb文件目录
            std::string pbFileDir = "./output/groundTruth/" + time_str + "/1";
            const std::string cmd = "mkdir -p \"" + pbFileDir + "\"";
            (void)std::system(cmd.c_str());
            if(0 != access(pbFileDir.data(), F_OK | R_OK | W_OK)) 
            {
                log_compnt_mngr->error("{} NOT ACCESSIBLE.", pbFileDir);
            }

            csvFilePath = "/opt/simpro/workspace/output/NG_Scenario/" + time_str + "/1/" + "NG_INFO_" + std::to_string(count) + ".csv";

            currentPbFilePath = "/opt/simpro/workspace/output/groundTruth/" + time_str + "/1/" + "SimProGroundTruth_" + std::to_string(count) + ".pb";

            fd = open(currentPbFilePath.c_str(), O_RDWR | O_CREAT, S_IRWXU);
            if (fd == -1)
            {
                log_compnt_mngr->error("fd open error.");
            }

            isFirstFrame = false;
        }

        //填充pb文件数据
        fillData();

        if ((timer - lastTimer > TIME_STEP_PB_FILE) && (egoState != nullptr))
        {
            close(fd);
            fd = -1;

            if (count > 1) //延后10s发送pb文件
            {
                //10s生成pb文件后通知评估
                memset(param.OSIGroundTruthPbPath, 0, D_PARAM_OSI_PB_PATH_SIZE);
                (void)strcpy(reinterpret_cast<char *>(param.OSIGroundTruthPbPath), lastPbFilePath.c_str()); /* OSIGroundTruth的Pb文件地址 */
                param.egoDistance = lastEgoTraveledDist; /* 主车行驶里程 */

                memset(param.NGinformationCsvPath, 0, D_PARAM_NGINFO_CSV_PATH_SIZE);
                (void)strcpy(reinterpret_cast<char *>(param.NGinformationCsvPath), lastCsvFilePath.c_str()); /* csv文件保存路径 */

                EvaluationAPI::Instance()->notify(&param);
            }
            else
            {
                //do nothing
            }

            lastEgoTraveledDist = egoTraveledDist;
            lastPbFilePath = currentPbFilePath;
            lastCsvFilePath = csvFilePath;

            count++;
            currentPbFilePath = "/opt/simpro/workspace/output/groundTruth/" + time_str + "/1/" + "SimProGroundTruth_" + std::to_string(count) + ".pb";
            csvFilePath = "/opt/simpro/workspace/output/NG_Scenario/" + time_str + "/1/" + "NG_INFO_" + std::to_string(count) + ".csv";

            fd = open(currentPbFilePath.c_str(), O_RDWR | O_CREAT, S_IRWXU);
            if (fd == -1)
            {
                log_compnt_mngr->error("fd open error.");
            }

            lastTimer += TIME_STEP_PB_FILE; //更新上次保存pb文件时间
        }
        else
        {
            //do nothing
        }

        //保存数据到pb文件
        savePbDataToFile();

        //更新主车移动距离
        if (egoState != nullptr)
        {
            egoTraveledDist = egoState->sObjectState.u4TraveledDist;
        }

        //清空上一帧数据
        groundTruth.Clear();
        vehicleList.clear();
        pedestrianList.clear();
        obstacleList.clear();
        trafficLightList.clear();
        trafficSignList.clear();
        laneList.clear();
        laneBoundaryList.clear();
        trafficSignAroundEgo.clear();
        contrastLightMap.clear();
        idTraffciLightMap.clear();
        roadRefTraffciLightMap.clear();
        roadTraffciLightsMap.clear();
        trafficLightStatusList.clear();
        trafficOverallList.clear();
        laneIdVec.clear();
        direction = 0;
        egoState = nullptr;

        //释放缓存区数据
        if (pbData.msg != nullptr)
        {
            delete [] pbData.msg;
        }

        //清空pbData数据
        memset(&pbData, 0, sizeof(OSI_PB_DATA));

        double osiParseMsgEnd = get_cur_time_ms();
        log_compnt_mngr->critical("OSIGroundTruthGeneratorThread recvMsgFromMq osiParseMsgEnd time = {}.", osiParseMsgEnd);

        osiMsg.msgMutex.lock();
        if (simulationRunStatus == false)
        {
            osiMsg.msgMutex.unlock();
            break;
        }
        else
        {
            //do nothing
        }
        osiMsg.msgMutex.unlock();
    }

    //关闭文件
    close(fd);
    fd = -1;

    if (count > 1)
    {
        sleep(10);
        //倒数第二个pb文件通知评估
        memset(param.OSIGroundTruthPbPath, 0, D_PARAM_OSI_PB_PATH_SIZE);
        (void)strcpy(reinterpret_cast<char *>(param.OSIGroundTruthPbPath), lastPbFilePath.c_str()); /* OSIGroundTruth的Pb文件地址 */
        param.egoDistance = lastEgoTraveledDist; /* 主车行驶里程 */

        memset(param.NGinformationCsvPath, 0, D_PARAM_NGINFO_CSV_PATH_SIZE);
        (void)strcpy(reinterpret_cast<char *>(param.NGinformationCsvPath), lastCsvFilePath.c_str()); /* csv文件保存路径 */

        log_compnt_mngr->critical("EvaluationAPI::Instance()->notify last start ");
        EvaluationAPI::Instance()->notify(&param);
    }

    sleep(10);
    //仿真结束通知评估
    memset(param.OSIGroundTruthPbPath, 0, D_PARAM_OSI_PB_PATH_SIZE);
    (void)strcpy(reinterpret_cast<char *>(param.OSIGroundTruthPbPath), currentPbFilePath.c_str()); /* OSIGroundTruth的Pb文件地址 */
    param.egoDistance = egoTraveledDist; /* 主车行驶里程 */
    param.isFinish = true;

    memset(param.NGinformationCsvPath, 0, D_PARAM_NGINFO_CSV_PATH_SIZE);
    (void)strcpy(reinterpret_cast<char *>(param.NGinformationCsvPath), csvFilePath.c_str()); /* csv文件保存路径 */

    log_compnt_mngr->critical("EvaluationAPI::Instance()->notify finish start");
    EvaluationAPI::Instance()->notify(&param);
}

// 将信息存入消息队列
bool OSIGroundTruthGeneratorThread::sendMsgToMq(OSI_PB_DATA &data)
{
    osiMsg.msgMutex.lock();
    osiMsg.msgList.push_back(data);
    osiMsg.msgMutex.unlock();

    sem_post(&sem);

    return true;
}

void OSIGroundTruthGeneratorThread::fillData()
{
    if ((ConfigureMngr::getInstance()->getCoSimuInterfaceType() == 2) && (ConfigureMngr::getInstance()->getIsOutputOsiGroundTruthLogicalLane() == true))
    {
        //筛选主车周围200米范围内道路
        filterRoad();
    }

    //填充一般数据
    fillGeneralData();
    double time = get_cur_time_ms();
    log_compnt_mngr->critical("OSIGroundTruthGeneratorThread recvMsgFromMq fillGeneralData time = {}.", time);

    //填充MovingObject
    fillMovingObject();
    time = get_cur_time_ms();
    log_compnt_mngr->critical("OSIGroundTruthGeneratorThread recvMsgFromMq fillMovingObject time = {}.", time);

    //填充StationaryObject
    fillStationaryObject();
    time = get_cur_time_ms();
    log_compnt_mngr->critical("OSIGroundTruthGeneratorThread recvMsgFromMq fillStationaryObject time = {}.", time);

    //填充TrafficSign
    fillTrafficSign(); //TODO: crash
    time = get_cur_time_ms();
    log_compnt_mngr->critical("OSIGroundTruthGeneratorThread recvMsgFromMq fillTrafficSign time = {}.", time);

    //填充Lane和LaneBoundary
    fillLaneAndLaneBoundary();
    time = get_cur_time_ms();
    log_compnt_mngr->critical("OSIGroundTruthGeneratorThread recvMsgFromMq fillLaneAndLaneBoundary time = {}.", time);


    //填充TrafficLight
    fillTrafficLight();
    time = get_cur_time_ms();
    log_compnt_mngr->critical("OSIGroundTruthGeneratorThread recvMsgFromMq fillTrafficLight time = {}.", time);

    //填充roadMarking
    fillRoadMarking();
    time = get_cur_time_ms();
    log_compnt_mngr->critical("OSIGroundTruthGeneratorThread recvMsgFromMq fillRoadMarking time = {}.", time);

    //填充EnvironmentalConditions
    fillEnvironmentalConditions();
    time = get_cur_time_ms();
    log_compnt_mngr->critical("OSIGroundTruthGeneratorThread recvMsgFromMq fillEnvironmentalConditions time = {}.", time);
}

//填充一般数据
void OSIGroundTruthGeneratorThread::fillGeneralData()
{
    if (egoState == nullptr)
    {
        return;
    }
    
    //groundTruth.version
    osi3::InterfaceVersion *version = new osi3::InterfaceVersion();
    version->set_version_major(3);
    version->set_version_minor(5);
    version->set_version_patch(0);
    groundTruth.set_allocated_version(version);

    //填写timestamp
    long int seconds = static_cast<long int>(timer); //秒数
    unsigned int nanoseconds = static_cast<long int>((timer - seconds) * 1e9); //纳秒数
    osi3::Timestamp *timestamp = new osi3::Timestamp();
    timestamp->set_seconds(seconds);
    timestamp->set_nanos(nanoseconds);
    groundTruth.set_allocated_timestamp(timestamp);

    //填写host_vehicle_id
    osi3::Identifier *identifier = new osi3::Identifier();
    identifier->set_value(egoState->sObjectState.u4Id); //主车Id
    groundTruth.set_allocated_host_vehicle_id(identifier);

    log_compnt_mngr->info("OSIGroundTruthGeneratorThread fillGeneralData vehicleId = {}.", egoState->sObjectState.u4Id);
}

void OSIGroundTruthGeneratorThread::fillMovingObject()
{
    if (egoState == nullptr)
    {
        return;
    }

    //填充主车数据
    (void)fillEgoObjectData();

    //填充每个车元素
    for(std::list<S_SP_MIL_OBJECT_STATE *>::const_iterator it = vehicleList.begin(); it != vehicleList.end(); it++)
    {
        const S_SP_MIL_OBJECT_STATE * const &movingObject = *it;

        //只提供主车100米内的物体数据
        double distence = pow((egoState->sObjectState.sPos.u8X - movingObject->sObjectState.sPos.u8X), 2) + pow((egoState->sObjectState.sPos.u8Y - movingObject->sObjectState.sPos.u8Y), 2) + pow((egoState->sObjectState.sPos.u8Z - movingObject->sObjectState.sPos.u8Z), 2);

        if (distence > diffDisEgo)
        {
            continue;
        }

        osi3::MovingObject *movingObjectOSI = groundTruth.add_moving_object();

        //movingObject.id
        osi3::Identifier *identifier = new osi3::Identifier();
        identifier->set_value(movingObject->sObjectState.u4Id);
        movingObjectOSI->set_allocated_id(identifier);

        log_compnt_mngr->info("OSIGroundTruthGeneratorThread fillMovingObject objectId = {}.", movingObject->sObjectState.u4Id);

        //movingObject.type;
        movingObjectOSI->set_type(osi3::MovingObject::Type(D_OSI_MOVINGOBJECT_TYPE_VEHICLE));

        if (movingObject->sObjectState.u1IsOnRoad)
        {
            //movingObject.assigned_lane_id //根据华为的要求，该字段值为(roadId + 1) * 1000 + laneId
            unsigned long int laneId = (movingObject->sObjectState.u8RoadId + 1) * 1000 + movingObject->sObjectState.u1LaneId;
            osi3::Identifier *laneIdOSI = movingObjectOSI->add_assigned_lane_id();
            laneIdOSI->set_value(laneId);
        }

        //movingObject.objectStateBase
        osi3::BaseMoving *baseMoving = new osi3::BaseMoving();
        movingObjectOSI->set_allocated_base(baseMoving);

        //movingObject.objectStateBase.position
        osi3::Vector3d *position = new osi3::Vector3d();
        baseMoving->set_allocated_position(position);
        position->set_x(movingObject->sObjectState.sPos.u8X);
        position->set_y(movingObject->sObjectState.sPos.u8Y);
        position->set_z(movingObject->sObjectState.sPos.u8Z);

        //movingObject.objectStateBase.orientation
        osi3::Orientation3d *orientation = new osi3::Orientation3d();
        baseMoving->set_allocated_orientation(orientation);
        orientation->set_yaw(movingObject->sObjectState.sPos.u4H);
        orientation->set_pitch(movingObject->sObjectState.sPos.u4P);
        orientation->set_roll(movingObject->sObjectState.sPos.u4R);

        //movingObject.objectStateBase.velocity
        osi3::Vector3d *velocity = new osi3::Vector3d();
        baseMoving->set_allocated_velocity(velocity);
        velocity->set_x(movingObject->sObjectState.sSpeed.u8X);
        velocity->set_y(movingObject->sObjectState.sSpeed.u8Y);
        velocity->set_z(movingObject->sObjectState.sSpeed.u8Z);

        //movingObject.objectStateBase.orientation_rate
        osi3::Orientation3d *orientationRate = new osi3::Orientation3d();
        baseMoving->set_allocated_orientation_rate(orientationRate);
        orientationRate->set_yaw(movingObject->sObjectState.sSpeed.u4H);
        orientationRate->set_pitch(movingObject->sObjectState.sSpeed.u4P);
        orientationRate->set_roll(movingObject->sObjectState.sSpeed.u4R);

        //movingObject.objectStateBase.acceleration
        osi3::Vector3d *acceleration = new osi3::Vector3d();
        baseMoving->set_allocated_acceleration(acceleration);
        acceleration->set_x(movingObject->sObjectState.sAccel.u8X);
        acceleration->set_y(movingObject->sObjectState.sAccel.u8Y);
        acceleration->set_z(movingObject->sObjectState.sAccel.u8Z);

        //movingObject.objectStateBase.orientation_acceleration
        osi3::Orientation3d *orientationAcceleration = new osi3::Orientation3d();
        baseMoving->set_allocated_orientation_acceleration(orientationAcceleration);
        orientationAcceleration->set_yaw(movingObject->sObjectState.sAccel.u4H);
        orientationAcceleration->set_pitch(movingObject->sObjectState.sAccel.u4P);
        orientationAcceleration->set_roll(movingObject->sObjectState.sAccel.u4R);

        //movingObject.objectStateBase.dimension
        osi3::Dimension3d *dimension = new osi3::Dimension3d();
        baseMoving->set_allocated_dimension(dimension);
        dimension->set_length(movingObject->sObjectState.sGeo.u4DimX);
        dimension->set_width(movingObject->sObjectState.sGeo.u4DimY);
        dimension->set_height(movingObject->sObjectState.sGeo.u4DimZ);

        //movingObject.VehicleAttributes
        osi3::MovingObject::VehicleAttributes *vehicleAttributes = new osi3::MovingObject::VehicleAttributes();
        movingObjectOSI->set_allocated_vehicle_attributes(vehicleAttributes);
        std::string movingObjectName(movingObject->sObjectState.au1Name);

        //movingObject.source_reference
        osi3::ExternalReference *sourceReference = movingObjectOSI->add_source_reference();
        std::string objectTypeName = "Vehicle";
        //设置movingObjectOSI的类型与名称
        sourceReference->add_identifier(objectTypeName);
        sourceReference->add_identifier(movingObjectName);

        auto catalogData = vehicleCatalogDataMap.find(std::string(movingObject->au1ModelName));
        if (catalogData != vehicleCatalogDataMap.end())
        {
            //movingObject.source_reference 质量
            osi3::ExternalReference *sourceReference_2 = movingObjectOSI->add_source_reference();
            std::string *type_2 = new std::string("mass");
            std::string *mass = new std::string(std::to_string(catalogData->second.mass));
            sourceReference_2->set_allocated_type(type_2);
            sourceReference_2->set_allocated_reference(mass);

            //movingObject.source_reference 轴距
            osi3::ExternalReference *sourceReference_3 = movingObjectOSI->add_source_reference();
            std::string *type_3 = new std::string("wheelBase");
            std::string *wheelBase = new std::string(std::to_string(catalogData->second.wheelBase));
            sourceReference_3->set_allocated_type(type_3);
            sourceReference_3->set_allocated_reference(wheelBase);

            //movingObject.source_reference 前轴宽度
            osi3::ExternalReference *sourceReference_4 = movingObjectOSI->add_source_reference();
            std::string *type_4 = new std::string("frontTrackWidth");
            std::string *frontTrackWidth = new std::string(std::to_string(catalogData->second.frontTrackWidth));
            sourceReference_4->set_allocated_type(type_4);
            sourceReference_4->set_allocated_reference(frontTrackWidth);

            //movingObject.source_reference 后轴宽度
            osi3::ExternalReference *sourceReference_5 = movingObjectOSI->add_source_reference();
            std::string *type_5 = new std::string("rearTrackWidth");
            std::string *rearTrackWidth = new std::string(std::to_string(catalogData->second.rearTrackWidth));
            sourceReference_5->set_allocated_type(type_5);
            sourceReference_5->set_allocated_reference(rearTrackWidth);
        }

        //movingObject.VehicleAttributes.bbcenter_to_rear
        //需求为包围盒中心到后轴中心的向量，此处centerX/Y/Z为参考点到几何中心的偏移量
        osi3::Vector3d *bbcenter_to_rear = new osi3::Vector3d();
        vehicleAttributes->set_allocated_bbcenter_to_rear(bbcenter_to_rear);
        bbcenter_to_rear->set_x(-movingObject->sObjectState.sGeo.u4OffX);
        bbcenter_to_rear->set_y(0.0);//通常横向偏移量为0
        bbcenter_to_rear->set_z(0.0);
        //bbcenter_to_rear->set_z(-movingObject.geo.centerZ); 需要包围盒中心到后轴中心的向量，而此处的z不属于此向量

        //movingObject.vehicle_classification
        osi3::MovingObject::VehicleClassification *vehicleClassification = new osi3::MovingObject::VehicleClassification();
        movingObjectOSI->set_allocated_vehicle_classification(vehicleClassification);

        uint16_t soType = convertVehicleType(movingObject->sObjectState.u1Type);  //转换车型类型 符合osi标准
        vehicleClassification->set_type(osi3::MovingObject::VehicleClassification::Type(soType));
        osi3::MovingObject::VehicleClassification::LightState *lightState = new osi3::MovingObject::VehicleClassification::LightState();
        vehicleClassification->set_allocated_light_state(lightState);

        osi3::MovingObject::VehicleClassification::LightState::IndicatorState indicatorState = osi3::MovingObject::VehicleClassification::LightState::INDICATOR_STATE_OTHER;
        if (movingObject->sMilBase.u4LightMask == D_SP_VEHICLE_LIGHT_REAR_BRAKE)
        {
            indicatorState = osi3::MovingObject::VehicleClassification::LightState::INDICATOR_STATE_WARNING;
        }
        else if (movingObject->sMilBase.u4LightMask & D_SP_VEHICLE_LIGHT_INDICATOR_L)
        {
            indicatorState = osi3::MovingObject::VehicleClassification::LightState::INDICATOR_STATE_LEFT;
        }
        else if (movingObject->sMilBase.u4LightMask & D_SP_VEHICLE_LIGHT_INDICATOR_R)
        {
            indicatorState = osi3::MovingObject::VehicleClassification::LightState::INDICATOR_STATE_RIGHT;
        }
        else
        {
            // do nothing
        }
        lightState->set_indicator_state(indicatorState);

        //movingObject.vehicle_classification.role
        vehicleClassification->set_role(osi3::MovingObject::VehicleClassification::Role(convertVehicleRole(movingObject->au1ModelName)));

        //movingObject.moving_object_classification.logical_lane_assignment
        osi3::MovingObject::MovingObjectClassification *movingObjectClassification = new osi3::MovingObject::MovingObjectClassification();
        movingObjectOSI->set_allocated_moving_object_classification(movingObjectClassification);

        osi3::LogicalLaneAssignment *logicalLaneAssignment =  movingObjectClassification->add_logical_lane_assignment();
        logicalLaneAssignment->set_s_position(movingObject->sObjectState.u4RoadS);
        logicalLaneAssignment->set_t_position(movingObject->sObjectState.u4RoadT);
        logicalLaneAssignment->set_angle_to_lane(movingObject->sObjectState.u4HdgRel);

        //movingObject.model_reference
        std::string modelReference = "VehicleCatalog/" + std::string(movingObject->au1ModelName);
        movingObjectOSI->set_allocated_model_reference(new std::string(modelReference));
    }

    //填充每个人/动物元素
    for(std::list<S_SP_MIL_OBJECT_STATE *>::const_iterator it = pedestrianList.begin(); it != pedestrianList.end(); it++)
    {
        const S_SP_MIL_OBJECT_STATE * const &movingObject = *it;

        //只提供主车100米内的物体数据
        double distence = pow((egoState->sObjectState.sPos.u8X - movingObject->sObjectState.sPos.u8X), 2) + pow((egoState->sObjectState.sPos.u8Y - movingObject->sObjectState.sPos.u8Y), 2) + pow((egoState->sObjectState.sPos.u8Z - movingObject->sObjectState.sPos.u8Z), 2);
        if (distence > diffDisEgo)
        {
            continue;
        }

        osi3::MovingObject *movingObjectOSI = groundTruth.add_moving_object();

        //movingObject.id
        osi3::Identifier *identifier = new osi3::Identifier();
        identifier->set_value(movingObject->sObjectState.u4Id);
        movingObjectOSI->set_allocated_id(identifier);

        log_compnt_mngr->info("OSIGroundTruthGeneratorThread fillMovingObject objectId = {}.", movingObject->sObjectState.u4Id);

        //movingObject.type;
        if (movingObject->sObjectState.u1Type == D_SP_OBJECT_TYPE_PEDESTRIAN)
        {
            movingObjectOSI->set_type(osi3::MovingObject::Type(D_OSI_MOVINGOBJECT_TYPE_PEDESTRIAN));
        }
        else
        {
            movingObjectOSI->set_type(osi3::MovingObject::Type(D_OSI_MOVINGOBJECT_TYPE_ANIMAL));
        }

        if (movingObject->sObjectState.u1IsOnRoad)
        {
            //movingObject.assigned_lane_id //根据华为的要求，该字段值为(roadId + 1) * 1000 + laneId
            unsigned long int laneId = (movingObject->sObjectState.u8RoadId + 1) * 1000 + movingObject->sObjectState.u1LaneId;
            osi3::Identifier *laneIdOSI = movingObjectOSI->add_assigned_lane_id();
            laneIdOSI->set_value(laneId);
        }

        //movingObject.objectStateBase
        osi3::BaseMoving *baseMoving = new osi3::BaseMoving();
        movingObjectOSI->set_allocated_base(baseMoving);

        //movingObject.objectStateBase.position
        osi3::Vector3d *position = new osi3::Vector3d();
        baseMoving->set_allocated_position(position);
        position->set_x(movingObject->sObjectState.sPos.u8X);
        position->set_y(movingObject->sObjectState.sPos.u8Y);
        position->set_z(movingObject->sObjectState.sPos.u8Z);

        //movingObject.objectStateBase.orientation
        osi3::Orientation3d *orientation = new osi3::Orientation3d();
        baseMoving->set_allocated_orientation(orientation);
        orientation->set_yaw(movingObject->sObjectState.sPos.u4H);
        orientation->set_pitch(movingObject->sObjectState.sPos.u4P);
        orientation->set_roll(movingObject->sObjectState.sPos.u4R);

        //movingObject.objectStateBase.velocity
        osi3::Vector3d *velocity = new osi3::Vector3d();
        baseMoving->set_allocated_velocity(velocity);
        velocity->set_x(movingObject->sObjectState.sSpeed.u8X);
        velocity->set_y(movingObject->sObjectState.sSpeed.u8Y);
        velocity->set_z(movingObject->sObjectState.sSpeed.u8Z);

        //movingObject.objectStateBase.orientation_rate
        osi3::Orientation3d *orientationRate = new osi3::Orientation3d();
        baseMoving->set_allocated_orientation_rate(orientationRate);
        orientationRate->set_yaw(movingObject->sObjectState.sSpeed.u4H);
        orientationRate->set_pitch(movingObject->sObjectState.sSpeed.u4P);
        orientationRate->set_roll(movingObject->sObjectState.sSpeed.u4R);

        //movingObject.objectStateBase.acceleration
        osi3::Vector3d *acceleration = new osi3::Vector3d();
        baseMoving->set_allocated_acceleration(acceleration);
        acceleration->set_x(movingObject->sObjectState.sAccel.u8X);
        acceleration->set_y(movingObject->sObjectState.sAccel.u8Y);
        acceleration->set_z(movingObject->sObjectState.sAccel.u8Z);

        //movingObject.objectStateBase.orientation_acceleration
        osi3::Orientation3d *orientationAcceleration = new osi3::Orientation3d();
        baseMoving->set_allocated_orientation_acceleration(orientationAcceleration);
        orientationAcceleration->set_yaw(movingObject->sObjectState.sAccel.u4H);
        orientationAcceleration->set_pitch(movingObject->sObjectState.sAccel.u4P);
        orientationAcceleration->set_roll(movingObject->sObjectState.sAccel.u4R);

        //movingObject.objectStateBase.dimension
        osi3::Dimension3d *dimension = new osi3::Dimension3d();
        baseMoving->set_allocated_dimension(dimension);
        dimension->set_length(movingObject->sObjectState.sGeo.u4DimX);
        dimension->set_width(movingObject->sObjectState.sGeo.u4DimY);
        dimension->set_height(movingObject->sObjectState.sGeo.u4DimZ);

        //movingObject.VehicleAttributes
        osi3::MovingObject::VehicleAttributes *vehicleAttributes = new osi3::MovingObject::VehicleAttributes();
        movingObjectOSI->set_allocated_vehicle_attributes(vehicleAttributes);
        std::string movingObjectName(movingObject->sObjectState.au1Name);

        //movingObject.source_reference
        osi3::ExternalReference *sourceReference = movingObjectOSI->add_source_reference();
        std::string objectTypeName = "Pedestrian";
        //设置movingObjectOSI的类型与名称
        sourceReference->add_identifier(objectTypeName);
        sourceReference->add_identifier(movingObjectName);

        //movingObject.VehicleAttributes.bbcenter_to_rear
        //需求为包围盒中心到后轴中心的向量，此处centerX/Y/Z为参考点到几何中心的偏移量
        osi3::Vector3d *bbcenter_to_rear = new osi3::Vector3d();
        vehicleAttributes->set_allocated_bbcenter_to_rear(bbcenter_to_rear);
        bbcenter_to_rear->set_x(-movingObject->sObjectState.sGeo.u4OffX);
        bbcenter_to_rear->set_y(0.0);//通常横向偏移量为0
        bbcenter_to_rear->set_z(0.0);
        //bbcenter_to_rear->set_z(-movingObject.geo.centerZ); 需要包围盒中心到后轴中心的向量，而此处的z不属于此向量

        //movingObject.vehicle_classification
        osi3::MovingObject::VehicleClassification *vehicleClassification = new osi3::MovingObject::VehicleClassification();
        movingObjectOSI->set_allocated_vehicle_classification(vehicleClassification);
        vehicleClassification->set_type(osi3::MovingObject::VehicleClassification::Type(osi3::MovingObject_VehicleClassification::TYPE_UNKNOWN));

        osi3::MovingObject::VehicleClassification::LightState *lightState = new osi3::MovingObject::VehicleClassification::LightState();
        vehicleClassification->set_allocated_light_state(lightState);
        bool indicatorLeft = false;
        bool indicatorRight = false;
        if (movingObject->sMilBase.u4LightMask & D_MSGADAPTER_VEHICLE_LIGHT_INDICATOR_LEFT)
        {
            indicatorLeft = true;
        }
        if (movingObject->sMilBase.u4LightMask & D_MSGADAPTER_VEHICLE_LIGHT_INDICATOR_RIGHT)
        {
            indicatorRight = true;
        }
        osi3::MovingObject::VehicleClassification::LightState::IndicatorState indicatorState = osi3::MovingObject::VehicleClassification::LightState::INDICATOR_STATE_OTHER;
        if (indicatorLeft && (!indicatorRight))
        {
            indicatorState = osi3::MovingObject::VehicleClassification::LightState::INDICATOR_STATE_LEFT;
        }
        else if ((!indicatorLeft) && indicatorRight)
        {
            indicatorState = osi3::MovingObject::VehicleClassification::LightState::INDICATOR_STATE_RIGHT;
        }
        else if (indicatorLeft && indicatorRight)
        {
            indicatorState = osi3::MovingObject::VehicleClassification::LightState::INDICATOR_STATE_WARNING;
        }
        lightState->set_indicator_state(indicatorState);

        //movingObject.vehicle_classification.role
        vehicleClassification->set_role(osi3::MovingObject::VehicleClassification::Role(convertVehicleRole(movingObject->au1ModelName)));

        //movingObject.moving_object_classification.logical_lane_assignment
        osi3::MovingObject::MovingObjectClassification *movingObjectClassification = new osi3::MovingObject::MovingObjectClassification();
        movingObjectOSI->set_allocated_moving_object_classification(movingObjectClassification);

        osi3::LogicalLaneAssignment *logicalLaneAssignment =  movingObjectClassification->add_logical_lane_assignment();
        logicalLaneAssignment->set_s_position(movingObject->sObjectState.u4RoadS);
        logicalLaneAssignment->set_t_position(movingObject->sObjectState.u4RoadT);
        logicalLaneAssignment->set_angle_to_lane(movingObject->sObjectState.u4HdgRel);

        //movingObject.model_reference
        std::string modelReference = "PedestrianCatalog/" + std::string(movingObject->au1ModelName);
        movingObjectOSI->set_allocated_model_reference(new std::string(modelReference));
    }
}

//填充StationaryObject
void OSIGroundTruthGeneratorThread::fillStationaryObject()
{
    if (egoState == nullptr)
    {
        return;
    }

    //填充障碍物元素
    for(std::list<S_SP_MIL_OBJECT_STATE *>::const_iterator it = obstacleList.begin(); it != obstacleList.end(); it++)
    {
        const S_SP_MIL_OBJECT_STATE * const &stationaryObject = *it;

        if (stationaryObject->sObjectState.u1Type == D_SP_OBJECT_TYPE_GRATICULE)//跳过标线
        {
            continue;
        }

        //只提供主车100米内的物体数据
        double distence = pow((egoState->sObjectState.sPos.u8X - stationaryObject->sObjectState.sPos.u8X), 2) + pow((egoState->sObjectState.sPos.u8Y - stationaryObject->sObjectState.sPos.u8Y), 2) + pow((egoState->sObjectState.sPos.u8Z - stationaryObject->sObjectState.sPos.u8Z), 2);
        if (distence > diffDisEgo)
        {
            continue;
        }

        osi3::StationaryObject *stationaryObjectOSI = groundTruth.add_stationary_object();

        //StationaryObject.id
        osi3::Identifier *identifier = new osi3::Identifier();
        identifier->set_value(stationaryObject->sObjectState.u4Id);
        stationaryObjectOSI->set_allocated_id(identifier);

        log_compnt_mngr->info("OSIGroundTruthGeneratorThread fillStationaryObject stationaryObjectId = {}.", stationaryObject->sObjectState.u4Id);

        //StationaryObject.model_reference
        std::string modelReference = "/" + std::string(stationaryObject->au1ModelName);
        stationaryObjectOSI->set_allocated_model_reference(new std::string(modelReference));

        //StationaryObject.objectStateBase
        osi3::BaseStationary *baseStationary = new osi3::BaseStationary();
        stationaryObjectOSI->set_allocated_base(baseStationary);

        //set its type in classification
        //StationaryObject.classification
        osi3::StationaryObject::Classification *soClassfication = new osi3::StationaryObject::Classification();
        stationaryObjectOSI->set_allocated_classification(soClassfication);

        //StationaryObject.classification.type
        uint16_t soType = convertStationaryObjectType(stationaryObject->sObjectState.u1Type);  //转换静态障碍物类型 符合osi标准
        soClassfication->set_type(osi3::StationaryObject::Classification::Type(soType));

        if (stationaryObject->sObjectState.u1IsOnRoad) //如果在道路上
        {
            //StationaryObject.classification.assigned_lane_id //根据华为的要求，该字段值为(roadId + 1) * 1000 + laneId
            unsigned long int laneId = (stationaryObject->sObjectState.u8RoadId + 1) * 1000 + stationaryObject->sObjectState.u1LaneId;
            osi3::Identifier *laneIdOSI = soClassfication->add_assigned_lane_id();
            laneIdOSI->set_value(laneId);
        }

        //StationaryObject.objectStateBase.dimension
        osi3::Dimension3d *dimension = new osi3::Dimension3d();
        baseStationary->set_allocated_dimension(dimension);
        dimension->set_length(stationaryObject->sObjectState.sGeo.u4DimX);
        dimension->set_width(stationaryObject->sObjectState.sGeo.u4DimY);
        dimension->set_height(stationaryObject->sObjectState.sGeo.u4DimZ);

        //objectStateBase.position
        osi3::Vector3d *position = new osi3::Vector3d();
        baseStationary->set_allocated_position(position);
        position->set_x(stationaryObject->sObjectState.sPos.u8X);
        position->set_y(stationaryObject->sObjectState.sPos.u8Y);
        position->set_z(stationaryObject->sObjectState.sPos.u8Z);

        //objectStateBase.orientation
        osi3::Orientation3d *orientation = new osi3::Orientation3d();
        baseStationary->set_allocated_orientation(orientation);
        orientation->set_yaw(stationaryObject->sObjectState.sPos.u4H);
        orientation->set_pitch(stationaryObject->sObjectState.sPos.u4P);
        orientation->set_roll(stationaryObject->sObjectState.sPos.u4R);

        //StationaryObject.source_reference
        osi3::ExternalReference *sourceReference = stationaryObjectOSI->add_source_reference();
        sourceReference->set_type("net.asam.opendrive");

        //设置StationaryObject的类型与名称
        sourceReference->add_identifier("MiscObject");
        sourceReference->add_identifier(stationaryObject->sObjectState.au1Name);

        Road* road = RoadSystem::Instance()->getRoad(std::to_string(stationaryObject->sObjectState.u8RoadId));
        if (road != nullptr)
        {
            std::vector<RoadObject *> obstacleVector;
            road->getObstacleVector(obstacleVector);
            for (auto object : obstacleVector)
            {
                if (object != nullptr)
                {
                    if (object->getMiscObjectID() == stationaryObject->sObjectState.u4Id)
                    {
                        char name[32] = {0};
                        char xodrType[16] = {0};

                        log_compnt_mngr->info("fillStationaryObject object->getMiscObjectID() = {}.", object->getMiscObjectID());

                        strncpy(name, object->getName().c_str(), sizeof(name) - 1);
                        //stationaryObjectOSI.sourceReference.openDriveName
                        osi3::ExternalReference *sourceReference1 = stationaryObjectOSI->add_source_reference();
                        if (sourceReference1 != nullptr)
                        {
                            sourceReference1->set_type("openDriveName");
                            sourceReference1->set_reference(name);
                        }

                        strncpy(xodrType, object->getType().c_str(), sizeof(xodrType) - 1);
                        //stationaryObjectOSI.sourceReference.openDriveType
                        osi3::ExternalReference *sourceReference2 = stationaryObjectOSI->add_source_reference();
                        if (sourceReference2 != nullptr)
                        {
                            sourceReference2->set_type("openDriveType");
                            sourceReference2->set_reference(xodrType);
                        } 
                    }
                }
            }
        }
    }
}

//填充TrafficSign
void OSIGroundTruthGeneratorThread::fillTrafficSign()
{
    if (egoState == nullptr)
    {
        return;
    }

    //更新主车周围的交通标志 (主车所在Road与下一条Road上存在的交通标志Id)
    (void)updateTrafficSignAroundEgo(egoState->sObjectState.u8RoadId);
    std::set<std::string> trafficSignAroundEgoSet(trafficSignAroundEgo.begin(), trafficSignAroundEgo.end());
    log_compnt_mngr->info("fillTrafficSign trafficSignAroundEgoSet size = {}.", trafficSignAroundEgoSet.size());

    //填充每个元素
    for(std::list<S_SP_TRAFFIC_SIGN *>::const_iterator it = trafficSignList.begin(); it != trafficSignList.end(); it++)
    {
        log_compnt_mngr->info("fillTrafficSign trafficSignList size = {}.", trafficSignList.size());

        const S_SP_TRAFFIC_SIGN * const &trafficSign = *it;
        RoadSignal* roadSignal = nullptr;

        std::vector<RoadSignal *> trafficSignVector = RoadSystem::Instance()->getTrafficSignVector();
        for (auto ele : trafficSignVector)
        {
            if (ele != nullptr)
            {
                if (std::stoi(ele->getId()) == trafficSign->u4TrafficSignId)
                {
                    roadSignal = ele;
                }
            }
        }

        //只提供主车100米内的物体数据
        double distence = pow((egoState->sObjectState.sPos.u8X - trafficSign->sPos.u8X), 2) + pow((egoState->sObjectState.sPos.u8Y - trafficSign->sPos.u8Y), 2) + pow((egoState->sObjectState.sPos.u8Z - trafficSign->sPos.u8Z), 2);
        if (distence > diffDisEgo)
        {
            if (roadSignal != nullptr)
            {
                if (std::stoi(roadSignal->getRoad()->getId()) != egoState->sObjectState.u8RoadId) //与主车在同一条路上的交通标志，不需要100m范围过滤
                {
                    continue;
                }
                else //与主车在同一条路上的交通标志，不需要100m范围过滤
                {
                    // do nothing
                }
            }
        }

        //排除远处的交通标志
        std::string idString = std::to_string(trafficSign->u4TrafficSignId);
        if (trafficSignAroundEgoSet.find(idString) == trafficSignAroundEgoSet.end()) //如果不存在于trafficSignAroundEgoSet
        {
            continue;
        }

        osi3::TrafficSign *trafficSignOSI = groundTruth.add_traffic_sign();

        //TrafficSign.id
        osi3::Identifier *identifier = new osi3::Identifier();
        identifier->set_value(trafficSign->u4TrafficSignId);
        trafficSignOSI->set_allocated_id(identifier);

        log_compnt_mngr->info("OSIGroundTruthGeneratorThread fillTrafficSign trafficSignId = {}.", trafficSign->u4TrafficSignId);

        //TrafficSign.main_sign
        osi3::TrafficSign::MainSign *mainSign = new osi3::TrafficSign::MainSign();
        trafficSignOSI->set_allocated_main_sign(mainSign);

        //TrafficSign.main_sign.classification
        osi3::TrafficSign::MainSign::Classification *classification = new osi3::TrafficSign::MainSign::Classification();
        mainSign->set_allocated_classification(classification);

        //TrafficSign.main_sign.classification.type
        uint16_t typeOfOSI = convertTrafficSignType(trafficSign->au1Type); //转换交通标志类型 符合osi标准
        classification->set_type(osi3::TrafficSign::MainSign::Classification::Type(typeOfOSI));

        //TrafficSign.main_sign.classification.value
        osi3::TrafficSignValue *value = new osi3::TrafficSignValue();
        classification->set_allocated_value(value);

        //当输出单位小于D_MSGADAPTER_TRAFFICSIGN_UNIT_METER_PER_SECOND时输出，否则输出空
        RoadSignal * signal = RoadSystem::Instance()->getSignal(std::to_string(trafficSign->u4TrafficSignId));
        if (signal != nullptr)
        {
            const std::string &unit = signal->getUnit(); //获取单位 D_MSGADAPTER_TRAFFICSIGN_UNIT

            if (unit == "m/s")
            {
                //TrafficSign.main_sign.classification.value.value
                value->set_value(static_cast<int>(trafficSign->u4Value * 3.6));

                // TrafficSign.main_sign.classification.value.value_unit
                value->set_value_unit(osi3::TrafficSignValue::Unit::TrafficSignValue_Unit_UNIT_KILOMETER_PER_HOUR);
            }
            else if (unit == "km/h")
            {
                //TrafficSign.main_sign.classification.value.value
                value->set_value(static_cast<int>(trafficSign->u4Value));

                // TrafficSign.main_sign.classification.value.value_unit
                value->set_value_unit(osi3::TrafficSignValue::Unit::TrafficSignValue_Unit_UNIT_KILOMETER_PER_HOUR); 
            }
            else if (unit.empty())
            {
                //TrafficSign.main_sign.classification.value.value
                value->set_value(static_cast<int>(trafficSign->u4Value));

                // TrafficSign.main_sign.classification.value.value_unit
                value->set_value_unit(osi3::TrafficSignValue::Unit::TrafficSignValue_Unit_UNIT_NO_UNIT);                   
            }
            else
            {
                //TrafficSign.main_sign.classification.value.value
                value->set_value(static_cast<int>(trafficSign->u4Value));

                // TrafficSign.main_sign.classification.value.value_unit
                value->set_value_unit(osi3::TrafficSignValue::Unit::TrafficSignValue_Unit_UNIT_UNKNOWN);              
            }                

        }
        else
        {
            /* do nothing */
        }

        //TrafficSign.main_sign.base
        osi3::BaseStationary *baseStationary = new osi3::BaseStationary();
        mainSign->set_allocated_base(baseStationary);

        //objectStateBase.position
        osi3::Vector3d *position = new osi3::Vector3d();
        baseStationary->set_allocated_position(position);
        position->set_x(trafficSign->sPos.u8X);
        position->set_y(trafficSign->sPos.u8Y);
        position->set_z(trafficSign->sPos.u8Z);

        //objectStateBase.orientation
        osi3::Orientation3d *orientation = new osi3::Orientation3d();
        baseStationary->set_allocated_orientation(orientation);
        orientation->set_yaw(trafficSign->sPos.u4H);
        orientation->set_pitch(trafficSign->sPos.u4P);
        orientation->set_roll(trafficSign->sPos.u4R);

        //TrafficSign.main_sign.classification.assigned_lane_id
        if (roadSignal != nullptr)
        {
            Road *road = roadSignal->getRoad();
            if (road != nullptr)
            {
                LaneSection *laneSection = road->getLaneSection(roadSignal->getS());

                if (laneSection != nullptr)
                {
                    std::map<int, Lane *> laneMap = laneSection->getLaneMap();
                    std::set<int> validitySet = roadSignal->getValiditySet();

                    if (validitySet.size() > 0) //具有有效车道信息
                    {
                        for (auto tmpId : validitySet)
                        {
                            if (laneMap.find(tmpId) != laneMap.end())
                            {
                                osi3::Identifier *laneId = classification->add_assigned_lane_id();
                                if (laneId != nullptr)
                                {
                                    laneId->set_value((std::stoi(road->getId()) + 1) * 1000 + tmpId);//根据华为的要求，laneId字段值为(roadId + 1) * 1000 + laneId   
                                }  
                            }  
                        }
                    }
                    else
                    {
                        for (std::map<int, Lane *>::iterator laneMapIt = laneMap.begin(); laneMapIt != laneMap.end(); laneMapIt++)
                        {
                            //只填充同向车道
                            if (((roadSignal->getLaneId() > 0) && (laneMapIt->first > 0)) || ((roadSignal->getLaneId() < 0) && (laneMapIt->first < 0)) || ((roadSignal->getLaneId() == 0) && (laneMapIt->first == 0)))
                            {
                                osi3::Identifier *laneId = classification->add_assigned_lane_id();
                                if (laneId != nullptr)
                                {
                                    laneId->set_value((std::stoi(road->getId()) + 1) * 1000 + laneMapIt->first);//根据华为的要求，laneId字段值为(roadId + 1) * 1000 + laneId
                                }
                            }
                        }
                    }
                }
            }
        }

        //trafficSignOSI.sourceReference.openDriveType
        osi3::ExternalReference *sourceReference1 = trafficSignOSI->add_source_reference();
        if (sourceReference1 != nullptr)
        {
            sourceReference1->set_type("openDriveType");
            sourceReference1->set_reference(trafficSign->au1Type);
        }

        //trafficSignOSI.sourceReference.openDriveSubtype
        osi3::ExternalReference *sourceReference2 = trafficSignOSI->add_source_reference();
        if (sourceReference2 != nullptr)
        {
            sourceReference2->set_type("openDriveSubtype");
            sourceReference2->set_reference(trafficSign->au1SubType);
        }
    }
}


//填充TrafficLight
void OSIGroundTruthGeneratorThread::fillTrafficLight()
{
    if (egoState == nullptr)
    {
        return;
    }

    // 获取主车周围交通灯id
    getTrafficLightAroundEgo();
    // 设置交通灯Map
    setTrafficLightMap();

    std::map<unsigned long int, uint64_t> trafficLightAroundEgoMap;       // 四个路口的 <交通灯 ID, road ID> (每个路口交通灯个数的最小值不是1)

    /* 循环遍历路的 ID, 找出所有的交通灯, 包括 signalReference 和 signal */
    /* 1. 当前路的交通灯 */
    RoadSignal *trafficLight_1 = getRefTrafficLightByRoad(std::to_string(trafficLightAroundEgo.currentRoadId));
    if (NULL == trafficLight_1) {
        std::list<std::tuple<RoadSignal *, int>> trafficLightList_1 = getTrafficLightsByRoad(std::to_string(trafficLightAroundEgo.currentRoadId));

        for (std::list<std::tuple<RoadSignal *, int>>::iterator it = trafficLightList_1.begin(); it != trafficLightList_1.end(); it++) {
            trafficLightAroundEgoMap.insert(std::pair<unsigned long int, uint64_t>(stoul(std::get<0>(*it)->getId()), trafficLightAroundEgo.currentRoadId));
        }
    } else {
        trafficLightAroundEgoMap.insert(std::pair<unsigned long int, uint64_t>(stoul(trafficLight_1->getId()), trafficLightAroundEgo.currentRoadId));
    }

    /* 2. 左转向路口的交通灯 */
    RoadSignal *trafficLight_2 = getRefTrafficLightByRoad(std::to_string(trafficLightAroundEgo.leftRoadId));
    if (NULL == trafficLight_2) {
        std::list<std::tuple<RoadSignal *, int>> trafficLightList_2 = getTrafficLightsByRoad(std::to_string(trafficLightAroundEgo.leftRoadId));

        for (std::list<std::tuple<RoadSignal *, int>>::iterator it = trafficLightList_2.begin(); it != trafficLightList_2.end(); it++) {
            trafficLightAroundEgoMap.insert(std::pair<unsigned long int, uint64_t>(stoul(std::get<0>(*it)->getId()), trafficLightAroundEgo.leftRoadId));
        }
    } else {
        trafficLightAroundEgoMap.insert(std::pair<unsigned long int, uint64_t>(stoul(trafficLight_2->getId()), trafficLightAroundEgo.leftRoadId));
    }

    /* 3. 直行路口的交通灯 */
    RoadSignal *trafficLight_3 = getRefTrafficLightByRoad(std::to_string(trafficLightAroundEgo.goStraightRoadId));
    if (NULL == trafficLight_3) {
        std::list<std::tuple<RoadSignal *, int>> trafficLightList_3 = getTrafficLightsByRoad(std::to_string(trafficLightAroundEgo.goStraightRoadId));

        for (std::list<std::tuple<RoadSignal *, int>>::iterator it = trafficLightList_3.begin(); it != trafficLightList_3.end(); it++) {
            trafficLightAroundEgoMap.insert(std::pair<unsigned long int, uint64_t>(stoul(std::get<0>(*it)->getId()), trafficLightAroundEgo.goStraightRoadId));
        }
    } else {
        trafficLightAroundEgoMap.insert(std::pair<unsigned long int, uint64_t>(stoul(trafficLight_3->getId()), trafficLightAroundEgo.goStraightRoadId));
    }

    /* 4. 右转向路口的交通灯 */
    RoadSignal *trafficLight_4 = getRefTrafficLightByRoad(std::to_string(trafficLightAroundEgo.rightRoadId));
    if (NULL == trafficLight_4) {
        std::list<std::tuple<RoadSignal *, int>> trafficLightList_4 = getTrafficLightsByRoad(std::to_string(trafficLightAroundEgo.rightRoadId));

        for (std::list<std::tuple<RoadSignal *, int>>::iterator it = trafficLightList_4.begin(); it != trafficLightList_4.end(); it++) {
            trafficLightAroundEgoMap.insert(std::pair<unsigned long int, uint64_t>(stoul(std::get<0>(*it)->getId()), trafficLightAroundEgo.rightRoadId));
        }
    } else {
        trafficLightAroundEgoMap.insert(std::pair<unsigned long int, uint64_t>(stoul(trafficLight_4->getId()), trafficLightAroundEgo.rightRoadId));
    }

    /* 5. 下一条路的交通灯 */
    RoadSignal *trafficLight_5 = getRefTrafficLightByRoad(std::to_string(trafficLightAroundEgo.nextRoadId));
    if (NULL == trafficLight_5) {
        std::list<std::tuple<RoadSignal *, int>> trafficLightList_5 = getTrafficLightsByRoad(std::to_string(trafficLightAroundEgo.nextRoadId));

        for (std::list<std::tuple<RoadSignal *, int>>::iterator it = trafficLightList_5.begin(); it != trafficLightList_5.end(); it++) {
            trafficLightAroundEgoMap.insert(std::pair<unsigned long int, uint64_t>(stoul(std::get<0>(*it)->getId()), trafficLightAroundEgo.nextRoadId));
        }
    } else {
        trafficLightAroundEgoMap.insert(std::pair<unsigned long int, uint64_t>(stoul(trafficLight_5->getId()), trafficLightAroundEgo.nextRoadId));
    }

    std::set<uint32_t> lightSet;//需要填充的灯集合

    //获取主车周围需要输出的交通灯
    for(std::list<OsiTrafficLight>::const_iterator it = trafficLightStatusList.begin(); it != trafficLightStatusList.end(); it++)
    {
        const OsiTrafficLight &trafficLight = *it;

        //排除远处的交通灯
        if (trafficLightAroundEgoMap.find(trafficLight.id) == trafficLightAroundEgoMap.end()) //如果不存在于trafficLightAroundEgoSet
        {
            continue;
        }

        //只提供主车100米内的物体数据
        double distence = pow((egoState->sObjectState.sPos.u8X - trafficLight.objectStateBase.x), 2) + pow((egoState->sObjectState.sPos.u8Z - trafficLight.objectStateBase.y), 2) + pow((egoState->sObjectState.sPos.u8Z - trafficLight.objectStateBase.z), 2);
        if (distence > diffDisEgo)
        {
            continue;
        }

        lightSet.insert(trafficLight.id);
    }

    //获取环境车需要输出的交通灯
    for (std::list<S_SP_MIL_OBJECT_STATE *>::iterator vehIt = vehicleList.begin(); vehIt != vehicleList.end(); vehIt++)
    {
        S_SP_MIL_OBJECT_STATE *pAgentVehicle = *vehIt;

        if (pAgentVehicle == nullptr)
        {
            continue;
        }

        if (std::string(pAgentVehicle->sObjectState.au1Name) == egoState->sObjectState.au1Name)//只读取环境车
        {
            continue;
        }

        //只提供主车100米内的环境车
        double distence = pow((egoState->sObjectState.sPos.u8X - pAgentVehicle->sObjectState.sPos.u8X), 2) + pow((egoState->sObjectState.sPos.u8Y - pAgentVehicle->sObjectState.sPos.u8Y), 2) + pow((egoState->sObjectState.sPos.u8Z - pAgentVehicle->sObjectState.sPos.u8Z), 2);
        if (distence > diffDisEgo)
        {
            continue;
        }

        Road *road = RoadSystem::Instance()->getRoad(std::to_string(pAgentVehicle->sObjectState.u8RoadId));//获取环境车所在道路

        if (road != nullptr)
        {
            RoadSignal *trafficLight = getRefTrafficLightByRoad(road->getId());

            if (trafficLight != nullptr)
            {
                lightSet.insert(static_cast<uint32_t>(stoul(trafficLight->getId())));
            }
            else//如果不存在signalReference标签则遍历signal标签
            {
                std::list<std::tuple<RoadSignal *, int>> pTrafficLightList = getTrafficLightsByRoad(road->getId());

                for (std::list<std::tuple<RoadSignal *, int>>::iterator it = pTrafficLightList.begin(); it != pTrafficLightList.end(); it++)
                {
                    lightSet.insert(static_cast<uint32_t>(stoul(std::get<0>(*it)->getId())));
                }
            }
        }
    }

    //填充每个元素
    for(std::list<OsiTrafficLight>::const_iterator it = trafficLightStatusList.begin(); it != trafficLightStatusList.end(); it++)
    {
        const OsiTrafficLight &trafficLight = *it;

        if (lightSet.find(trafficLight.id) == lightSet.end())//不符合填充要求
        {
            continue;
        }

        osi3::TrafficLight *trafficLightOSI = groundTruth.add_traffic_light();

        //TrafficLight.id
        osi3::Identifier *identifier = new osi3::Identifier();
        identifier->set_value(trafficLight.id);
        trafficLightOSI->set_allocated_id(identifier);

        log_compnt_mngr->info("OSIGroundTruthGeneratorThread fillTrafficLight trafficLightId = {}.", trafficLight.id);

        //TrafficLight.base
        osi3::BaseStationary *baseStationary = new osi3::BaseStationary();
        trafficLightOSI->set_allocated_base(baseStationary);

        //TrafficLight.base.position
        osi3::Vector3d *position = new osi3::Vector3d();
        baseStationary->set_allocated_position(position);
        position->set_x(trafficLight.objectStateBase.x);
        position->set_y(trafficLight.objectStateBase.y);
        position->set_z(trafficLight.objectStateBase.z);

        //TrafficLight.base.orientation
        osi3::Orientation3d *orientation = new osi3::Orientation3d();
        baseStationary->set_allocated_orientation(orientation);
        orientation->set_yaw(trafficLight.objectStateBase.h);
        orientation->set_pitch(trafficLight.objectStateBase.p);
        orientation->set_roll(trafficLight.objectStateBase.r);

        //TrafficLight.classification
        osi3::TrafficLight::Classification *classification = new osi3::TrafficLight::Classification();
        trafficLightOSI->set_allocated_classification(classification);

        //TrafficLight.classification.color
        classification->set_color(osi3::TrafficLight::Classification::Color(trafficLight.classification.color));

        //TrafficLight.classification.icon
        classification->set_icon(osi3::TrafficLight::Classification::Icon(trafficLight.classification.icon));

        //TrafficLight.classification.assigned_lane_id
        Road *road = RoadSystem::Instance()->getRoad(std::to_string(trafficLight.objectStateBase.roadId));

        if (road != nullptr)
        {
            LaneSection *laneSection = road->getLaneSection(trafficLight.objectStateBase.roadS);

            if (laneSection != nullptr)
            {
                std::map<int, Lane *> laneMap = laneSection->getLaneMap();

                for (std::map<int, Lane *>::iterator laneMapIt = laneMap.begin(); laneMapIt != laneMap.end(); laneMapIt++)
                {
                    if (laneMapIt->first == 0)
                    {
                        continue;
                    }

                    osi3::Identifier *laneId = classification->add_assigned_lane_id();
                    laneId->set_value((trafficLight.objectStateBase.roadId + 1) * 1000 + laneMapIt->first);//根据华为的要求，laneId字段值为(roadId + 1) * 1000 + laneId
                }
            }
        }

    }
}

//设置交通灯Map
void OSIGroundTruthGeneratorThread::setTrafficLightMap()
{
    //填充用来进行对比的map
    contrastLightMap[std::make_tuple("1000001", "-1")] = 3;
    contrastLightMap[std::make_tuple("1000011", "30")] = 3;
    contrastLightMap[std::make_tuple("1000011", "10")] = 3;
    contrastLightMap[std::make_tuple("1000011", "20")] = 3;
    contrastLightMap[std::make_tuple("1000011", "60")] = 3;
    contrastLightMap[std::make_tuple("1000013", "-1")] = 2;
    contrastLightMap[std::make_tuple("1000002", "-1")] = 2;
    contrastLightMap[std::make_tuple("1000009", "-1")] = 2;

    //获取所有signal
    const std::vector<RoadSignal *> &signalVector = RoadSystem::Instance()->getSignalVector();    

    //遍历每个signal
    for (const auto &roadSignal : signalVector)
    {
        if (roadSignal != nullptr)
        {
            std::tuple<std::string, std::string> lightTup = std::make_tuple(roadSignal->getType(), (roadSignal->getSubtype()));//需要查找的灯
            auto itar = contrastLightMap.find(lightTup);

            if ((itar != contrastLightMap.end()) && (roadSignal->getCountry() == "OpenDRIVE")) //查找成功并且灯的country为OpenDRIVE
            {
                Road *road = roadSignal->getRoad();
                if (road != nullptr)
                {
                    //添加到交通灯Map
                    if (NULL == roadSignal) {

                        return;
                    }
                    idTraffciLightMap[roadSignal->getId()] = std::make_tuple(roadSignal, itar->second);

                    /* 填充 roadTraffciLightsMap */
                    Road *road = roadSignal->getRoad();
                    if (road != nullptr)
                    {
                        std::string roadId = road->getId();
                        if (roadTraffciLightsMap.find(roadId) == roadTraffciLightsMap.end()) {
                            std::list<std::tuple<RoadSignal *, int>> lightList;
                            lightList.push_back(std::make_tuple(roadSignal, itar->second));
                            roadTraffciLightsMap[roadId] = lightList;
                        }
                        else {
                            roadTraffciLightsMap.find(roadId)->second.push_back(std::make_tuple(roadSignal, itar->second));
                        }
                    }

                    // 添加交通灯到trafficOverallList
                    trafficOverallList.push_back(roadSignal);
                }
            }
        }
    }

    //填充roadTraffciLightMap
    std::vector<Road *> roadVector = RoadSystem::Instance()->getRoadVector();
    for (unsigned int roadIt = 0; roadIt < roadVector.size(); ++roadIt) //遍历每条路
    {
        Road *searchRoad = roadVector[roadIt];
        if(searchRoad != nullptr)
        {
            std::map<std::string, std::tuple<RoadSignal *, int>>::iterator itor = idTraffciLightMap.find(searchRoad->getRefSignalId());
            if (itor != idTraffciLightMap.end())
            {
                roadRefTraffciLightMap[searchRoad->getId()] = std::get<0>((*itor).second);
            }
            else
            {
                continue;
            }
        }
    }

    //填充trafficLightStatusList
    for (auto it : trafficOverallList)
    {
        OsiTrafficLight trafficLightData;

        trafficLightData.id = stoul(it->getId());

        trafficLightData.objectStateBase.x = it->getTransform().v().x();	//世界坐标系x，单位m
        trafficLightData.objectStateBase.y = it->getTransform().v().y();	//世界坐标系y，单位m
        trafficLightData.objectStateBase.z = it->getTransform().v().z();	//世界坐标系z，单位m
        trafficLightData.objectStateBase.h = 0.0;				//世界坐标系，航向角，单位rad
        trafficLightData.objectStateBase.p = 0.0;				//世界坐标系，俯仰角，单位rad
        trafficLightData.objectStateBase.r = 0.0;				//世界坐标系，横滚角，单位rad

        trafficLightData.objectStateBase.isOnRoad = true;                               //位置是否在道路上
        trafficLightData.objectStateBase.roadId = stoul(it->getRoad()->getId());      //道路坐标系
        trafficLightData.objectStateBase.laneId = static_cast<int16_t>(it->getLaneId());	//道路坐标系
        trafficLightData.objectStateBase.flags = D_MSGADAPTER_ROAD_POS_FLAG_NONE;		//@D_MSGADAPTER_ROAD_POS_FLAG
        trafficLightData.objectStateBase.roadS = static_cast<float>(it->getS());   //道路坐标系，S坐标，单位m
        trafficLightData.objectStateBase.roadT = static_cast<float>(it->getT());   //道路坐标系，T坐标，单位m
        trafficLightData.objectStateBase.laneOffset = static_cast<float>(it->getLaneOffset());         //距离车道中心偏移量，单位m
        trafficLightData.objectStateBase.hdgRel = static_cast<float>(it->getHdg());	                //道路坐标系，航向角，单位rad
        trafficLightData.objectStateBase.pitchRel = static_cast<float>(it->getPitch());//道路坐标系，俯仰角，单位rad
        trafficLightData.objectStateBase.rollRel = static_cast<float>(it->getRoll());	//道路坐标系，横滚角，单位rad

        trafficLightData.classification.color = osi3::TrafficLight::Classification::Color::TrafficLight_Classification_Color_COLOR_UNKNOWN;
        int color = -1;

        auto result = findTrafficLight(it->getId());
        if (result != nullptr)
        {
            float status = result->u4State;

            float timeSum = 0.0;
            float lastTimeSum = 0.0;
            for (int i = 0; i < result->u4NoPhases; i++)
            {
                S_SP_TRAFFIC_LIGHT_PHASE *phase = (S_SP_TRAFFIC_LIGHT_PHASE *)((char *)result + D_OSI_SIZE_TRAFFIC_LIGHT_DATA * i);
                
                timeSum += phase->u4Duration;
                if (result->u4State >= lastTimeSum && result->u4State < timeSum)
                {
                    color = phase->u1Status;
                    break;
                }
                lastTimeSum = timeSum;
            }
            if (color == D_SP_TRLIGHT_STATUS_STOP)
            {
                trafficLightData.classification.color = osi3::TrafficLight::Classification::Color::TrafficLight_Classification_Color_COLOR_RED;
            }
            else if (color == D_SP_TRLIGHT_STATUS_ATTN)
            {
                trafficLightData.classification.color = osi3::TrafficLight::Classification::Color::TrafficLight_Classification_Color_COLOR_YELLOW;
            }
            else if (color == D_SP_TRLIGHT_STATUS_GO)
            {
                trafficLightData.classification.color = osi3::TrafficLight::Classification::Color::TrafficLight_Classification_Color_COLOR_GREEN;
            }
            else
            {
                trafficLightData.classification.color = osi3::TrafficLight::Classification::Color::TrafficLight_Classification_Color_COLOR_OTHER;
            }
        }

        uint16_t _icon = osi3::TrafficLight::Classification::Icon::TrafficLight_Classification_Icon_ICON_OTHER;
        std::string type = it->getType();
        std::string subtype = it->getSubtype();

        if ((type == "1000011") && ((subtype == "60") || (subtype == "30")))//竖排直行圆灯,竖排直行灯
        {
            _icon = osi3::TrafficLight::Classification::Icon::TrafficLight_Classification_Icon_ICON_ARROW_STRAIGHT_AHEAD;
        }
        else if ((type == "1000011") && (subtype == "10"))//竖排左转灯
        {
            _icon = osi3::TrafficLight::Classification::Icon::TrafficLight_Classification_Icon_ICON_ARROW_LEFT;
        }
        else if ((type == "1000011") && (subtype == "20"))//竖排右转灯
        {
            _icon = osi3::TrafficLight::Classification::Icon::TrafficLight_Classification_Icon_ICON_ARROW_RIGHT;
        }
        else if ((type == "1000002") && (subtype == "-1"))//竖排人行灯
        {
            _icon = osi3::TrafficLight::Classification::Icon::TrafficLight_Classification_Icon_ICON_PEDESTRIAN;
        }
        else if ((type == "1000013") && (subtype == "-1"))//单车指示灯
        {
            _icon = osi3::TrafficLight::Classification::Icon::TrafficLight_Classification_Icon_ICON_BICYCLE;
        }

        trafficLightData.classification.icon = _icon;

        trafficLightStatusList.push_back(trafficLightData);//添加到trafficLightList中     
    }
}

// 在联仿数据中查找返回交通灯指针
S_SP_TRAFFIC_LIGHT *OSIGroundTruthGeneratorThread::findTrafficLight(const std::string &id)
{
    S_SP_TRAFFIC_LIGHT *result = nullptr;

    for (auto ele : trafficLightList)
    {
        if (ele->u4Id == std::stoi(id))
        {
            result = ele;
        }
    }

    return result;
}

//通过Road获取交通灯(signalReference)
RoadSignal *OSIGroundTruthGeneratorThread::getRefTrafficLightByRoad(const std::string &roadId) const
{
    RoadSignal *traffficLig = NULL;

    auto roadTraffciLightMapIt = roadRefTraffciLightMap.find(roadId);
    if (roadTraffciLightMapIt != roadRefTraffciLightMap.end())
    {
        traffficLig = roadTraffciLightMapIt->second;
    }
    return traffficLig;
}

// 通过Road获取交通灯
std::list<std::tuple<RoadSignal *, int>> OSIGroundTruthGeneratorThread::getTrafficLightsByRoad(const std::string &roadId) const
{
    std::list<std::tuple<RoadSignal *, int>> TrafficLightsList;
    auto roadTraffciLightsMapIt = roadTraffciLightsMap.find(roadId);

    if (roadTraffciLightsMapIt != roadTraffciLightsMap.end()){
        TrafficLightsList = roadTraffciLightsMapIt->second;
    }

    return TrafficLightsList;
}

//填充roadMarking
void OSIGroundTruthGeneratorThread::fillRoadMarking()
{
    if (egoState == nullptr)
    {
        return;
    }

    std::vector<RoadObject *> miscObjectOverallVector;

    //获取所有Road
    std::vector<Road *> roadVector = RoadSystem::Instance()->getRoadVector();

    //遍历每个Road
    for (const auto &road : roadVector)
    {
        if (road != nullptr)
        {
            //获取这条路上的所有object
            std::vector<RoadObject *> obstacleVector;
            road->getObstacleVector(obstacleVector);
            
            //将每个object加入到miscObjectOverallVector
            for (const auto &roadObject : obstacleVector)
            {
                if (roadObject != nullptr)
                {
                    miscObjectOverallVector.push_back(roadObject);
                }
            }
        }
    }

    log_compnt_mngr->info("fillRoadMarking miscObjectOverallVector size = {}.", miscObjectOverallVector.size());

    //填充roadMarking
    for (std::vector<RoadObject *>::const_iterator miscIt = miscObjectOverallVector.begin(); miscIt < miscObjectOverallVector.end(); miscIt++)
    {
        RoadObject *pMiscObject = *miscIt;

        if (pMiscObject == nullptr)
        {
            continue;
        }

        if (pMiscObject->getObstacleType() != RoadObject::RoadObjectType::TYPE_GRATICULE)//只填充标线
        {
            continue;
        }

        Transform miscObjectTransform = pMiscObject->getRoadObjectTransform();//获取世界坐标

        //只提供主车100米内的物体数据
        double distence = pow((egoState->sObjectState.sPos.u8X - miscObjectTransform.v().x()), 2) + pow((egoState->sObjectState.sPos.u8Y - miscObjectTransform.v().y()), 2) + pow((egoState->sObjectState.sPos.u8Z - miscObjectTransform.v().z()), 2);
        if (distence > diffDisEgo)
        {
            continue;
        }

        osi3::RoadMarking *roadMarking = groundTruth.add_road_marking();

        //traffic_main_sign_type
        osi3::RoadMarking::Classification *classfication = new osi3::RoadMarking::Classification();
        roadMarking->set_allocated_classification(classfication);

        std::string name = pMiscObject->getName();
        uint16_t type = convertRoadMarkingType(name);

        classfication->set_traffic_main_sign_type(osi3::TrafficSign::MainSign::Classification::Type(type));

        //BaseStationary.position
        osi3::BaseStationary *baseStationary = new osi3::BaseStationary();
        roadMarking->set_allocated_base(baseStationary);

        osi3::Vector3d *position = new osi3::Vector3d();
        baseStationary->set_allocated_position(position);
        position->set_x(miscObjectTransform.v().x());
        position->set_y(miscObjectTransform.v().y());
        position->set_z(miscObjectTransform.v().z());

        //Dimension
        osi3::Dimension3d *dimension = new osi3::Dimension3d();
        baseStationary->set_allocated_dimension(dimension);
        dimension->set_length(pMiscObject->getDimensionX());
        dimension->set_width(pMiscObject->getDimensionY());
        dimension->set_height(pMiscObject->getDimensionZ());

        //orientation
        osi3::Orientation3d *orientation = new osi3::Orientation3d();
        baseStationary->set_allocated_orientation(orientation);
        orientation->set_yaw(normalizeRad2(pMiscObject->getYawWorld()));
        orientation->set_pitch(normalizeRad2(pMiscObject->getPitchWorld()));
        orientation->set_roll(normalizeRad2(pMiscObject->getRollWorld()));

        //RoadMarking.sourceReference
        Road *road = pMiscObject->getRoad();
        if (road != nullptr)
        {
            osi3::ExternalReference *sourceReference = roadMarking->add_source_reference();
            sourceReference->set_type("roadId");
            sourceReference->set_reference(road->getId());

            osi3::ExternalReference *sourceReference2 = roadMarking->add_source_reference();
            sourceReference2->set_type("laneId");
            sourceReference2->set_reference(std::to_string(pMiscObject->getLaneId()));
        }

        //RoadMarking.sourceReference.openDriveName
        osi3::ExternalReference *sourceReference1 = roadMarking->add_source_reference();
        if (sourceReference1 != nullptr)
        {
            sourceReference1->set_type("openDriveName");
            sourceReference1->set_reference(pMiscObject->getName());
        }

        //RoadMarking.sourceReference.openDriveType
        osi3::ExternalReference *sourceReference2 = roadMarking->add_source_reference();
        if (sourceReference2 != nullptr)
        {
            sourceReference2->set_type("openDriveType");
            sourceReference2->set_reference(pMiscObject->getType());
        }
    }

    //填充停车位
    Road *egoRoad = RoadSystem::Instance()->getRoad(std::to_string(egoState->sObjectState.u8RoadId));
    if (egoRoad != nullptr)
    {
        fillParkingSpace(egoRoad);

        for (auto it : roadMap)
        {
            Road *road = it.first;
            if (road == egoRoad)
            {
                continue;
            }

            fillParkingSpace(road);
        }        
    }
}

//填充EnvironmentalConditions
void OSIGroundTruthGeneratorThread::fillEnvironmentalConditions()
{
    if (egoState == nullptr)
    {
        return;
    }

    osi3::EnvironmentalConditions *environmentalConditionsOSI = new osi3::EnvironmentalConditions();
    groundTruth.set_allocated_environmental_conditions(environmentalConditionsOSI);

    //EnvironmentalConditions.time_of_day
    osi3::EnvironmentalConditions::TimeOfDay *timeOfDay = new osi3::EnvironmentalConditions::TimeOfDay();
    environmentalConditionsOSI->set_allocated_time_of_day(timeOfDay);
    if (environment == nullptr)
    {
        log_compnt_mngr->error("environment == nullptr");
    }
    timeOfDay->set_seconds_since_midnight(43200);

    struct timeb sysTime{};
    (void)ftime(&sysTime);
    //sysTime.time是从UTC时间1970年1月1日午夜(00:00:00)起累计的秒数；sysTime.millitm是一秒内的毫秒数
    time_t sysTimeMs = static_cast<time_t>(sysTime.millitm) + sysTime.time * static_cast<time_t>(1000);

    //EnvironmentalConditions.unix_timestamp
    environmentalConditionsOSI->set_unix_timestamp(sysTimeMs);

    //EnvironmentalConditions.temperature
    environmentalConditionsOSI->set_temperature(20.0);

    //EnvironmentalConditions.relative_humidity
    environmentalConditionsOSI->set_relative_humidity(0.0);

    //EnvironmentalConditions.precipitation
    uint16_t temperature = convertTemperature(environment->u8RainIntensity);
    environmentalConditionsOSI->set_precipitation(osi3::EnvironmentalConditions::Precipitation(temperature));

    //EnvironmentalConditions.fog
    uint16_t fog = convertFog(environment->u8FogVisualRange);

    environmentalConditionsOSI->set_fog(osi3::EnvironmentalConditions::Fog(fog));    
}

//填充主车Object数据
void OSIGroundTruthGeneratorThread::fillEgoObjectData()
{
    osi3::MovingObject *movingObjectOSI = groundTruth.add_moving_object();

    //movingObject.id
    osi3::Identifier *identifier = new osi3::Identifier();
    identifier->set_value(egoState->sObjectState.u4Id);
    movingObjectOSI->set_allocated_id(identifier);

    //movingObject.type;
    movingObjectOSI->set_type(osi3::MovingObject::Type(D_OSI_MOVINGOBJECT_TYPE_VEHICLE));

    if (egoState->sObjectState.u1IsOnRoad)
    {
        //movingObject.assigned_lane_id //根据华为的要求，该字段值为(roadId + 1) * 1000 + laneId
        unsigned long int laneId = (egoState->sObjectState.u8RoadId + 1) * 1000 + egoState->sObjectState.u1LaneId;
        osi3::Identifier *laneIdOSI = movingObjectOSI->add_assigned_lane_id();
        laneIdOSI->set_value(laneId);
    }

    //movingObject.objectStateBase
    osi3::BaseMoving *baseMoving = new osi3::BaseMoving();
    movingObjectOSI->set_allocated_base(baseMoving);

    //movingObject.objectStateBase.position
    osi3::Vector3d *position = new osi3::Vector3d();
    baseMoving->set_allocated_position(position);
    position->set_x(egoState->sObjectState.sPos.u8X);
    position->set_y(egoState->sObjectState.sPos.u8Y);
    position->set_z(egoState->sObjectState.sPos.u8Z);

    //movingObject.objectStateBase.orientation
    osi3::Orientation3d *orientation = new osi3::Orientation3d();
    baseMoving->set_allocated_orientation(orientation);
    orientation->set_yaw(egoState->sObjectState.sPos.u4H);
    orientation->set_pitch(egoState->sObjectState.sPos.u4P);
    orientation->set_roll(egoState->sObjectState.sPos.u4R);

    //movingObject.objectStateBase.velocity
    osi3::Vector3d *velocity = new osi3::Vector3d();
    baseMoving->set_allocated_velocity(velocity);
    velocity->set_x(egoState->sObjectState.sSpeed.u8X);
    velocity->set_y(egoState->sObjectState.sSpeed.u8Y);
    velocity->set_z(egoState->sObjectState.sSpeed.u8Z);

    //movingObject.objectStateBase.orientation_rate
    osi3::Orientation3d *orientationRate = new osi3::Orientation3d();
    baseMoving->set_allocated_orientation_rate(orientationRate);
    orientationRate->set_yaw(egoState->sObjectState.sSpeed.u4H);
    orientationRate->set_pitch(egoState->sObjectState.sSpeed.u4P);
    orientationRate->set_roll(egoState->sObjectState.sSpeed.u4R);

    //movingObject.objectStateBase.acceleration
    osi3::Vector3d *acceleration = new osi3::Vector3d();
    baseMoving->set_allocated_acceleration(acceleration);
    acceleration->set_x(egoState->sObjectState.sAccel.u8X);
    acceleration->set_y(egoState->sObjectState.sAccel.u8Y);
    acceleration->set_z(egoState->sObjectState.sAccel.u8Z);

    //movingObject.objectStateBase.orientation_acceleration
    osi3::Orientation3d *orientationAcceleration = new osi3::Orientation3d();
    baseMoving->set_allocated_orientation_acceleration(orientationAcceleration);
    orientationAcceleration->set_yaw(egoState->sObjectState.sAccel.u4H);
    orientationAcceleration->set_pitch(egoState->sObjectState.sAccel.u4P);
    orientationAcceleration->set_roll(egoState->sObjectState.sAccel.u4R);

    //movingObject.objectStateBase.dimension
    osi3::Dimension3d *dimension = new osi3::Dimension3d();
    baseMoving->set_allocated_dimension(dimension);
    dimension->set_length(egoState->sObjectState.sGeo.u4DimX);
    dimension->set_width(egoState->sObjectState.sGeo.u4DimY);
    dimension->set_height(egoState->sObjectState.sGeo.u4DimZ);

    //movingObject.VehicleAttributes
    osi3::MovingObject::VehicleAttributes *vehicleAttributes = new osi3::MovingObject::VehicleAttributes();
    movingObjectOSI->set_allocated_vehicle_attributes(vehicleAttributes);
    std::string movingObjectName(egoState->sObjectState.au1Name);

    //movingObject.source_reference
    osi3::ExternalReference *sourceReference = movingObjectOSI->add_source_reference();
    std::string objectTypeName = "Vehicle";
    //设置movingObjectOSI的类型与名称
    sourceReference->add_identifier(objectTypeName);
    sourceReference->add_identifier(movingObjectName);

    auto catalogData = vehicleCatalogDataMap.find(std::string(egoState->au1ModelName));
    if (catalogData != vehicleCatalogDataMap.end())
    {
        //movingObject.source_reference 质量
        osi3::ExternalReference *sourceReference_2 = movingObjectOSI->add_source_reference();
        std::string *type_2 = new std::string("mass");
        std::string *mass = new std::string(std::to_string(catalogData->second.mass));
        sourceReference_2->set_allocated_type(type_2);
        sourceReference_2->set_allocated_reference(mass);

        //movingObject.source_reference 轴距
        osi3::ExternalReference *sourceReference_3 = movingObjectOSI->add_source_reference();
        std::string *type_3 = new std::string("wheelBase");
        std::string *wheelBase = new std::string(std::to_string(catalogData->second.wheelBase));
        sourceReference_3->set_allocated_type(type_3);
        sourceReference_3->set_allocated_reference(wheelBase);

        //movingObject.source_reference 前轴宽度
        osi3::ExternalReference *sourceReference_4 = movingObjectOSI->add_source_reference();
        std::string *type_4 = new std::string("frontTrackWidth");
        std::string *frontTrackWidth = new std::string(std::to_string(catalogData->second.frontTrackWidth));
        sourceReference_4->set_allocated_type(type_4);
        sourceReference_4->set_allocated_reference(frontTrackWidth);

        //movingObject.source_reference 后轴宽度
        osi3::ExternalReference *sourceReference_5 = movingObjectOSI->add_source_reference();
        std::string *type_5 = new std::string("rearTrackWidth");
        std::string *rearTrackWidth = new std::string(std::to_string(catalogData->second.rearTrackWidth));
        sourceReference_5->set_allocated_type(type_5);
        sourceReference_5->set_allocated_reference(rearTrackWidth);

        //movingObject.source_reference 最大加速度
        osi3::ExternalReference *sourceReference_6 = movingObjectOSI->add_source_reference();
        std::string *type = new std::string("maxAcceleration");
        std::string *maxAcc = new std::string(std::to_string(catalogData->second.maxAcceleration));
        sourceReference_6->set_allocated_type(type);
        sourceReference_6->set_allocated_reference(maxAcc);

        //movingObject.source_reference 最大减速度
        osi3::ExternalReference *sourceReference_7 = movingObjectOSI->add_source_reference();
        std::string *type2 = new std::string("maxDeceleration");
        std::string *maxDece = new std::string(std::to_string(catalogData->second.maxDeceleration));
        sourceReference_7->set_allocated_type(type2);
        sourceReference_7->set_allocated_reference(maxDece);
    }

    //movingObject.source_reference 摩擦系数
    if (egoState->sObjectState.u1IsOnRoad)
    {
        Road* road = RoadSystem::Instance()->getRoad(std::to_string(egoState->sObjectState.u8RoadId));
        osi3::ExternalReference *sourceReference_8 = movingObjectOSI->add_source_reference();
        std::string *type3 = new std::string("friction");
        std::string *pavement = nullptr;

        if (road != nullptr)
        {
            LaneSection* laneSection = road->getLaneSection(egoState->sObjectState.u4RoadS);
            if (laneSection != nullptr)
            {
                Lane* lane = laneSection->getLane(egoState->sObjectState.u1LaneId);
                if ((lane != nullptr) && (environment != nullptr))
                {
                    const std::map<double, double> &pavementMap = lane->getPavementMap();

                    if (pavementMap.empty() == false)//map非空
                    {
                        auto itar = (--pavementMap.upper_bound(egoState->sObjectState.u4RoadS));

                        if (itar != pavementMap.end())
                        {
                            double roadPavement = itar->second;
                            pavement = new std::string(std::to_string(0.8 * roadPavement * (1 - environment->u8RainIntensity) * (1 - environment->u8SnowIntensity)));                                
                        }
                        else
                        {
                            pavement = new std::string(std::to_string(0.8 * 1.0 * (1 - environment->u8RainIntensity) * (1 - environment->u8SnowIntensity)));
                        }
                    }
                    else
                    {
                        pavement = new std::string(std::to_string(0.8 * 1.0 * (1 - environment->u8RainIntensity) * (1 - environment->u8SnowIntensity)));
                    }

                    sourceReference_8->set_allocated_type(type3);
                    sourceReference_8->set_allocated_reference(pavement); 
                }
            }
        }
    }

    //movingObject.VehicleAttributes.bbcenter_to_rear
    //需求为包围盒中心到后轴中心的向量，此处centerX/Y/Z为参考点到几何中心的偏移量
    osi3::Vector3d *bbcenter_to_rear = new osi3::Vector3d();
    vehicleAttributes->set_allocated_bbcenter_to_rear(bbcenter_to_rear);
    bbcenter_to_rear->set_x(-egoState->sObjectState.sGeo.u4OffX);
    bbcenter_to_rear->set_y(0.0);//通常横向偏移量为0
    bbcenter_to_rear->set_z(0.0);
    //bbcenter_to_rear->set_z(-movingObject.geo.centerZ); 需要包围盒中心到后轴中心的向量，而此处的z不属于此向量

    //movingObject.vehicle_classification
    osi3::MovingObject::VehicleClassification *vehicleClassification = new osi3::MovingObject::VehicleClassification();
    movingObjectOSI->set_allocated_vehicle_classification(vehicleClassification);

    uint16_t soType = convertVehicleType(egoState->sObjectState.u1Type);  //转换车型类型 符合osi标准
    vehicleClassification->set_type(osi3::MovingObject::VehicleClassification::Type(soType));
    osi3::MovingObject::VehicleClassification::LightState *lightState = new osi3::MovingObject::VehicleClassification::LightState();
    vehicleClassification->set_allocated_light_state(lightState);
    bool indicatorLeft = false;
    bool indicatorRight = false;
    if (egoState->u4LightMask & D_SP_VEHICLE_LIGHT_INDICATOR_L)
    {
        indicatorLeft = true;
    }
    if (egoState->u4LightMask & D_SP_VEHICLE_LIGHT_INDICATOR_R)
    {
        indicatorRight = true;
    }

    osi3::MovingObject::VehicleClassification::LightState::IndicatorState indicatorState = osi3::MovingObject::VehicleClassification::LightState::INDICATOR_STATE_OTHER;
    if (indicatorLeft && (!indicatorRight))
    {
        indicatorState = osi3::MovingObject::VehicleClassification::LightState::INDICATOR_STATE_LEFT;
    }
    else if ((!indicatorLeft) && indicatorRight)
    {
        indicatorState = osi3::MovingObject::VehicleClassification::LightState::INDICATOR_STATE_RIGHT;
    }
    else if (indicatorLeft && indicatorRight)
    {
        indicatorState = osi3::MovingObject::VehicleClassification::LightState::INDICATOR_STATE_WARNING;
    }
    lightState->set_indicator_state(indicatorState);

    //movingObject.vehicle_classification.role
    vehicleClassification->set_role(osi3::MovingObject::VehicleClassification::Role(convertVehicleRole(egoState->au1ModelName)));

    //movingObject.moving_object_classification.logical_lane_assignment
    osi3::MovingObject::MovingObjectClassification *movingObjectClassification = new osi3::MovingObject::MovingObjectClassification();
    movingObjectOSI->set_allocated_moving_object_classification(movingObjectClassification);

    osi3::LogicalLaneAssignment *logicalLaneAssignment =  movingObjectClassification->add_logical_lane_assignment();
    logicalLaneAssignment->set_s_position(egoState->sObjectState.u4RoadS);
    logicalLaneAssignment->set_t_position(egoState->sObjectState.u4RoadT);
    logicalLaneAssignment->set_angle_to_lane(egoState->sObjectState.u4HdgRel);

    //movingObject.model_reference
    std::string modelReference = "VehicleCatalog/" + std::string(egoState->au1ModelName);
    movingObjectOSI->set_allocated_model_reference(new std::string(modelReference));    
}

//转换roadmarking类型
uint16_t OSIGroundTruthGeneratorThread::convertRoadMarkingType(const std::string &inputName)
{
    uint16_t outputType = osi3::TrafficSign::MainSign::Classification::TYPE_OTHER;

    if (inputName == "Crosswalk_Line")//人行横道线
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_ZEBRA_CROSSING;
    }
    else if (inputName == "Arrow_Forward")//直行
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_PRESCRIBED_STRAIGHT;
    }
    else if (inputName == "Arrow_Left_And_Forward")//指示前方可直行或左转
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_PRESCRIBED_LEFT_TURN_AND_STRAIGHT;
    }
    else if (inputName == "Arrow_Right_And_Forward")//指示前方可直行或右转
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_PRESCRIBED_RIGHT_TURN_AND_STRAIGHT;
    }
    else if (inputName == "Arrow_Right")//向右转弯
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_PRESCRIBED_RIGHT_TURN;
    }
    else if (inputName == "Arrow_Left")//向左转弯
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_PRESCRIBED_LEFT_TURN;
    }
    else if (inputName == "Arrow_Left_And_U_Turns")//指示前方可左转或掉头
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_PRESCRIBED_U_TURN_LEFT;
    }
    else if (inputName == "Arrow_Left_And_Right")//向左和向右转弯
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_PRESCRIBED_LEFT_TURN_AND_RIGHT_TURN;
    }
    else if (inputName == "Non_Motor_Vehicle")//非机动车道路面标记
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_MOTOR_VEHICLES_PROHIBITED;
    }
    else if (inputName == "Stop_Line")//停止线
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_STOP;
    }
    else if (inputName == "Stop_To_Give_Way")//停车让行线
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_GIVE_WAY;
    }
    else if (inputName == "Bus_Only_Lane_Line")//公交专用车道线
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_BUS_LANE;
    }
    else if (inputName == "Parking_Space_Mark")//停车位
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_CAR_PARKING;
    }

    return outputType;
}

//转换交通标志类型,从SimPro的交通标志类型转为osi标准的交通标志类型
uint16_t OSIGroundTruthGeneratorThread::convertTrafficSignType(const std::string &inputType)
{
    //用于输出的交通标志类型
    uint16_t outputType = osi3::TrafficSign::MainSign::Classification::TYPE_OTHER; //初值为1(TYPE_OTHER)

    if (inputType == "1010103400001111") //注意危险标志
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_DANGER_SPOT;
    }
    else if (inputType == "1010300100002413")//直行
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_PRESCRIBED_STRAIGHT;
    }
    else if (inputType == "1010301500002413")//最低限速标志
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_MINIMUM_SPEED_BEGIN;
    }
    else if (inputType == "1010300700002413")//靠右侧道路行驶
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_PASS_RIGHT;
    }
    else if (inputType == "1010300800002413")//靠左侧道路行驶
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_PASS_LEFT;
    }
    else if (inputType == "1010301800002616")//人行横道
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_ZEBRA_CROSSING;
    }
    else if (inputType == "1010300300002413")//向右转弯
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_PRESCRIBED_RIGHT_TURN;
    }
    else if (inputType == "1010300200002413")//向左转弯
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_PRESCRIBED_LEFT_TURN;
    }
    else if (inputType == "1010301100002416")//环岛行驶
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_ROUNDABOUT;
    }
    else if (inputType == "1010300500002413")//直行和向右转弯
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_PRESCRIBED_RIGHT_TURN_AND_STRAIGHT;
    }
    else if (inputType == "1010300400002413")//直行和向左转弯
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_PRESCRIBED_LEFT_TURN_AND_STRAIGHT;
    }
    else if (inputType == "1010302014002413")//非机动车行驶
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_BICYCLES_ONLY;
    }
    else if (inputType == "1010300600002413")//向左和向右转弯
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_PRESCRIBED_LEFT_TURN_AND_RIGHT_TURN;
    }
    else if (inputType == "1010302111002416")//停车位
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_CAR_PARKING;
    }
    else if (inputType == "1010200200002012")//减速让行
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_GIVE_WAY;
    }
    else if (inputType == "1010101100001111")//注意儿童标志
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_CHILDREN_CROSSING;
    }
    else if (inputType == "1010100511001111")//上坡路
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_HILL_UPWARDS;
    }
    else if (inputType == "1010103500001111")//施工
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_ROAD_WORKS;
    }
    else if (inputType == "1010100711001111")//左侧变窄
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_ROAD_NARROWING_LEFT;
    }
    else if (inputType == "1010100713001111")//两侧变窄
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_ROAD_NARROWING;
    }
    else if (inputType == "1010102812001111")//无人看守铁路道口
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_RAILWAY_TRAFFIC_PRIORITY;
    }
    else if (inputType == "1010102400001111")//路面不平
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_UNEVEN_ROAD;
    }
    else if (inputType == "1010100600001111")//下坡路
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_HILL_DOWNWARDS;
    }
    else if (inputType == "1010102100001111")//隧道标志
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_TUNNEL;
    }
    else if (inputType == "1010100712001111")//右侧变窄
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_ROAD_NARROWING_RIGHT;
    }
    else if (inputType == "1010102900001111")//注意非机动车
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_CYCLE_ROUTE;
    }
    else if (inputType == "1010101400001111")//注意信号灯
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_ATTENTION_TRAFFIC_LIGHT;
    }
    else if (inputType == "1010100900001111")//双向交通标志
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_TWO_WAY_TRAFFIC;
    }
    else if (inputType == "1010101000001111")//注意行人
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_PEDESTRIANS;
    }
    else if (inputType == "1010200600001413")//禁止机动车驶入
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_MOTORIZED_MULTITRACK_PROHIBITED;
    }
    else if (inputType == "1010203800001413")//限速
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_SPEED_LIMIT_BEGIN;
    }
    else if (inputType == "1010200100001914")//停车让行
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_STOP;
    }
    else if (inputType == "1010200500001513")//禁止驶入
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_DO_NOT_ENTER;
    }
    else if (inputType == "1010201400001413")//禁止二轮摩托车驶入
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_MOTORCYCLES_PROHIBITED;
    }
    else if (inputType == "1010203600001413")//限重
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_OVER_WEIGHT_VEHICLES_PROHIBITED;
    }
    else if (inputType == "1010203111001713")//禁止车辆临时或长时停放
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_NO_STOPPING;
    }
    else if (inputType == "1010203200001713") //禁止车辆长时停放
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_NO_PARKING;
    }
    else if (inputType == "1010301300002413") //步行 / 行人通行标志
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_PEDESTRIANS_ONLY;
    }
    else if (inputType == "1010201200001413")//禁止拖拉机驶入
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_TRACTORS_PROHIBITED;
    }
    else if (inputType == "1010200400001213")//禁止通行
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_ALL_PROHIBITED;
    }
    else if (inputType == "1010200300002113")//会车让行
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_PRIORITY_TO_OPPOSITE_DIRECTION;
    }
    else if (inputType == "1010200900001413")//禁止大型客车驶入
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_CARS_PROHIBITED;
    }
    else if (inputType == "1010201600001413")//禁止非机动车进入
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_BICYCLES_PROHIBITED;
    }
    else if (inputType == "1010202100001413")//禁止行人进入
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_PEDESTRIANS_PROHIBITED;
    }
    else if (inputType == "1010202900001413")//禁止超车
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_OVERTAKING_BAN_BEGIN;
    }
    else if (inputType == "1010202800001413")//禁止掉头
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_NO_U_TURN_LEFT;
    }
    else if (inputType == "1010100400001111")//连续转弯（首先向左）标志
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_DOUBLE_TURN_LEFT;
    }
    else if (inputType == "1010203500001413")//限制高度
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_VEHICLES_EXCESS_HEIGHT_PROHIBITED;
    }
    else if (inputType == "1010203400001413")//限制宽度
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_VEHICLES_EXCESS_WIDTH_PROHIBITED;
    }
    else if (inputType == "1010203000001613")//解除禁止超车
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_OVERTAKING_BAN_END;
    }
    else if (inputType == "1010103313001111")//右侧绕行
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_DETOUR_RIGHT;
    }
    else if (inputType == "1010103312001111")//左侧绕行
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_DETOUR_LEFT;
    }
    else if (inputType == "1010203900001613")//解除限速
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_SPEED_LIMIT_END; 
    }
    else if (inputType == "1010203500001413")//限高
    {
        outputType = osi3::TrafficSign::MainSign::Classification::TYPE_VEHICLES_EXCESS_HEIGHT_PROHIBITED; 
    }
    else
    {
        //do nothing
    }

    return outputType;
}

//转换车辆角色
uint16_t OSIGroundTruthGeneratorThread::convertVehicleRole(const std::string &inputName)
{
    uint16_t outputRole = osi3::MovingObject::VehicleClassification::ROLE_OTHER;

    if (inputName.find("救护车") != std::string::npos)
    {
        outputRole = osi3::MovingObject::VehicleClassification::ROLE_AMBULANCE;
    }
    else if (inputName.find("消防车") != std::string::npos)
    {
        outputRole = osi3::MovingObject::VehicleClassification::ROLE_FIRE;
    }
    else if (inputName.find("警车") != std::string::npos)
    {
        outputRole = osi3::MovingObject::VehicleClassification::ROLE_POLICE; 
    }
    else if (inputName.find("公交车") != std::string::npos)
    {
        outputRole = osi3::MovingObject::VehicleClassification::ROLE_PUBLIC_TRANSPORT; 
    }

    return outputRole;
}

//转换车型类型,从SimPro的车型类型转为osi标准的车型类型
uint16_t OSIGroundTruthGeneratorThread::convertVehicleType(uint8_t inputType)
{
    //用于输出的车型类型
    uint16_t outputType = osi3::MovingObject_VehicleClassification::TYPE_OTHER; //初值为 TYPE_OTHER

    if (inputType == D_SP_OBJECT_TYPE_CAR)
    {
        outputType = osi3::MovingObject_VehicleClassification::TYPE_SMALL_CAR;
    }
    else if (inputType == D_SP_OBJECT_TYPE_MOTORBIKE)
    {
        outputType = osi3::MovingObject_VehicleClassification::TYPE_MOTORBIKE;
    }
    else if (inputType == D_SP_OBJECT_TYPE_BICYCLE)
    {
        outputType = osi3::MovingObject_VehicleClassification::TYPE_BICYCLE;
    }
    else if (inputType == D_SP_OBJECT_TYPE_BUS)
    {
        outputType = osi3::MovingObject_VehicleClassification::TYPE_BUS;
    }
    else if (inputType == D_SP_OBJECT_TYPE_TRUCK)
    {
        outputType = osi3::MovingObject_VehicleClassification::TYPE_HEAVY_TRUCK;
    }

    return outputType;    
}

//转换静态障碍物类型。从SimPro的静态障碍物类型转为osi标准的静态障碍物类型
uint16_t OSIGroundTruthGeneratorThread::convertStationaryObjectType(uint16_t inputType)
{
    //用于输出的静态障碍物类型
    uint16_t outputType = osi3::StationaryObject::Classification::TYPE_OTHER; //初值为 TYPE_OTHER

    if (inputType == D_SP_OBJECT_TYPE_BUILDING)
    {
        outputType = osi3::StationaryObject::Classification::TYPE_BUILDING;
    }
    else if (inputType == D_SP_OBJECT_TYPE_POLE)
    {
        outputType = osi3::StationaryObject::Classification::TYPE_POLE;
    }
    else if (inputType == D_SP_OBJECT_TYPE_TREE)
    {
        outputType = osi3::StationaryObject::Classification::TYPE_TREE;
    }
    else if (inputType == D_SP_OBJECT_TYPE_BARRIER)
    {
        outputType = osi3::StationaryObject::Classification::TYPE_BARRIER;
    }
    else if ((inputType == D_SP_OBJECT_TYPE_GRASS) || (inputType == D_SP_OBJECT_TYPE_SHRUB))
    {
        outputType = osi3::StationaryObject::Classification::TYPE_VEGETATION;
    }
    else if (inputType == D_SP_OBJECT_TYPE_ROADCURB_BARRIER)
    {
        outputType = osi3::StationaryObject::Classification::TYPE_CURBSTONE;
    }
    else if (inputType == D_SP_OBJECT_TYPE_STREET_LAMP)
    {
        outputType = osi3::StationaryObject::Classification::TYPE_EMITTING_STRUCTURE;
    }
    else if (inputType == D_SP_OBJECT_TYPE_DECELERATION_ZONE_BARRIER)
    {
        outputType = osi3::StationaryObject::Classification::TYPE_SPEED_BUMP;
    }
    else if (inputType == D_SP_OBJECT_TYPE_BRIDGE)
    {
        outputType = osi3::StationaryObject::Classification::TYPE_BRIDGE;
    }
    else
    {
        outputType = osi3::StationaryObject::Classification::TYPE_OTHER;
    }

    return outputType;
}

//将SimPro降雨强度转换为osi标准的降雨类型
uint16_t OSIGroundTruthGeneratorThread::convertTemperature(double temperature)
{
    //用于输出的降雨类型
    uint16_t outputType = osi3::EnvironmentalConditions::Precipitation::EnvironmentalConditions_Precipitation_PRECIPITATION_UNKNOWN; //初值为 EnvironmentalConditions_Precipitation_PRECIPITATION_UNKNOWN
    const double rainIntensity = temperature * 255.0;

    if (rainIntensity < 0.1)
    {
        outputType = osi3::EnvironmentalConditions::Precipitation::EnvironmentalConditions_Precipitation_PRECIPITATION_NONE;			//没有降水,[0,0.1)mm/h
    }
    else if ((rainIntensity >= 0.1) && (rainIntensity < 0.5))
    {
        outputType = osi3::EnvironmentalConditions::Precipitation::EnvironmentalConditions_Precipitation_PRECIPITATION_VERY_LIGHT;	//非常轻的降水强度,[0.1,0.5)mm/h
    }
    else if ((rainIntensity >= 0.5) && (rainIntensity < 1.9))
    {
        outputType = osi3::EnvironmentalConditions::Precipitation::EnvironmentalConditions_Precipitation_PRECIPITATION_LIGHT;			//低强度降水,[0.5,1.9)mm/h
    }
    else if ((rainIntensity >= 1.9) && (rainIntensity < 8.1))
    {
        outputType = osi3::EnvironmentalConditions::Precipitation::EnvironmentalConditions_Precipitation_PRECIPITATION_MODERATE;		//中强度降水,[1.9,8.1)mm/h
    }
    else if ((rainIntensity >= 8.1) && (rainIntensity < 34.0))
    {
        outputType = osi3::EnvironmentalConditions::Precipitation::EnvironmentalConditions_Precipitation_PRECIPITATION_HEAVY;			//强降水,[8.1,34)mm/h
    }
    else if ((rainIntensity >= 34.0) && (rainIntensity < 149.0))
    {
        outputType = osi3::EnvironmentalConditions::Precipitation::EnvironmentalConditions_Precipitation_PRECIPITATION_VERY_HEAVY;	//强降水,[34,149)mm/h
    }
    else if (rainIntensity >= 149.0)
    {
        outputType = osi3::EnvironmentalConditions::Precipitation::EnvironmentalConditions_Precipitation_PRECIPITATION_EXTREME;		//极端降水强度,[149,inf)mm/h
    }
    else
    {
        outputType = osi3::EnvironmentalConditions::Precipitation::EnvironmentalConditions_Precipitation_PRECIPITATION_OTHER;			//其他
    }

    return outputType;
}

//将SimPro可见度范围转换为osi标准的雾类型
uint16_t OSIGroundTruthGeneratorThread::convertFog(double visualRange)
{
    //用于输出的降雨类型
    uint16_t outputType = osi3::EnvironmentalConditions::Fog::EnvironmentalConditions_Fog_FOG_UNKNOWN; //初值为 EnvironmentalConditions_Fog_FOG_UNKNOWN

    if (visualRange >= 40000.0 )
    {
        outputType = osi3::EnvironmentalConditions::Fog::EnvironmentalConditions_Fog_FOG_EXCELLENT_VISIBILITY;				//能见度极好,[40000,inf)m
    }
    else if ((visualRange >= 10000.0) && (visualRange < 40000.0))
    {
        outputType = osi3::EnvironmentalConditions::Fog::EnvironmentalConditions_Fog_FOG_GOOD_VISIBILITY;					//能见度好,[10000,40000)m
    }
    else if ((visualRange >= 4000.0) && (visualRange < 10000.0))
    {
        outputType = osi3::EnvironmentalConditions::Fog::EnvironmentalConditions_Fog_FOG_MODERATE_VISIBILITY;				//能见度中等,[4000,10000)m
    }
    else if ((visualRange >= 2000.0) && (visualRange < 4000.0))
    {
        outputType = osi3::EnvironmentalConditions::Fog::EnvironmentalConditions_Fog_FOG_POOR_VISIBILITY;					//能见度低,[2000,4000)m
    }
    else if ((visualRange >= 1000.0) && (visualRange < 2000.0))
    {
        outputType = osi3::EnvironmentalConditions::Fog::EnvironmentalConditions_Fog_FOG_MIST;								//薄雾,[1000,2000)m
    }
    else if ((visualRange >= 200.0) && (visualRange < 1000.0))
    {
        outputType = osi3::EnvironmentalConditions::Fog::EnvironmentalConditions_Fog_FOG_LIGHT;								//雾,[200,1000)m
    }
    else if ((visualRange >= 50.0) && (visualRange < 200.0))
    {
        outputType = osi3::EnvironmentalConditions::Fog::EnvironmentalConditions_Fog_FOG_THICK;								//浓雾,[50,200)m
    }
    else if ((visualRange >= 0.0) && (visualRange < 50.0))
    {
        outputType = osi3::EnvironmentalConditions::Fog::EnvironmentalConditions_Fog_FOG_DENSE;								//浓雾,[0,50)m
    }
    else
    {
        // Comment: Nothing to do.
    }

    return outputType;    
}

//转换lane的subtype
uint16_t OSIGroundTruthGeneratorThread::convertLaneSubtype(uint16_t inputType)
{
    uint16_t laneSubtype = osi3::Lane::Classification::Subtype::Lane_Classification_Subtype_SUBTYPE_OTHER;

    if (inputType == Lane::DRIVING)
    {
        laneSubtype = osi3::Lane::Classification::Subtype::Lane_Classification_Subtype_SUBTYPE_NORMAL;
    }
    else if (inputType == Lane::BIKING)
    {
        laneSubtype = osi3::Lane::Classification::Subtype::Lane_Classification_Subtype_SUBTYPE_BIKING;
    }
    else if (inputType == Lane::SIDEWALK)
    {
        laneSubtype = osi3::Lane::Classification::Subtype::Lane_Classification_Subtype_SUBTYPE_SIDEWALK;
    }
    else if (inputType == Lane::PARKING)
    {
        laneSubtype = osi3::Lane::Classification::Subtype::Lane_Classification_Subtype_SUBTYPE_PARKING;
    }
    else if (inputType == Lane::STOP)
    {
        laneSubtype = osi3::Lane::Classification::Subtype::Lane_Classification_Subtype_SUBTYPE_STOP;
    }
    else if (inputType == Lane::RESTRICTED)
    {
        laneSubtype = osi3::Lane::Classification::Subtype::Lane_Classification_Subtype_SUBTYPE_RESTRICTED;
    }
    else if (inputType == Lane::BORDER)
    {
        laneSubtype = osi3::Lane::Classification::Subtype::Lane_Classification_Subtype_SUBTYPE_BORDER;
    }
    else if (inputType == Lane::SHOULDER)
    {
        laneSubtype = osi3::Lane::Classification::Subtype::Lane_Classification_Subtype_SUBTYPE_SHOULDER;
    }
    else if (inputType == Lane::MWYEXIT)
    {
        laneSubtype = osi3::Lane::Classification::Subtype::Lane_Classification_Subtype_SUBTYPE_EXIT;
    }
    else if (inputType == Lane::MWYENTRY)
    {
        laneSubtype = osi3::Lane::Classification::Subtype::Lane_Classification_Subtype_SUBTYPE_ENTRY;
    }
    else if (inputType == Lane::ONRAMP)
    {
        laneSubtype = osi3::Lane::Classification::Subtype::Lane_Classification_Subtype_SUBTYPE_ONRAMP;
    }
    else if (inputType == Lane::OFFRAMP)
    {
        laneSubtype = osi3::Lane::Classification::Subtype::Lane_Classification_Subtype_SUBTYPE_OFFRAMP;
    }
    else if (inputType == Lane::CONNECTINGRAMP)
    {
        laneSubtype = osi3::Lane::Classification::Subtype::Lane_Classification_Subtype_SUBTYPE_CONNECTINGRAMP;
    }
    else
    {
        // do nothing
    }

    return laneSubtype;    
}

//转换laneBoundary.type
uint16_t OSIGroundTruthGeneratorThread::convertLaneBoundaryType(const RoadMark::RoadMarkType laneBoundaryType)
{
    uint16_t type = osi3::LaneBoundary::Classification::Type::LaneBoundary_Classification_Type_TYPE_UNKNOWN;

    if(laneBoundaryType == RoadMark::TYPE_NONE)
    {
        type = osi3::LaneBoundary::Classification::Type::LaneBoundary_Classification_Type_TYPE_UNKNOWN;        //未知
    }
    else if(laneBoundaryType == RoadMark::TYPE_SOLID)
    {
        type = osi3::LaneBoundary::Classification::Type::LaneBoundary_Classification_Type_TYPE_SOLID_LINE;     //实线
    }
    else if(laneBoundaryType == RoadMark::TYPE_BROKEN)
    {
        type = osi3::LaneBoundary::Classification::Type::LaneBoundary_Classification_Type_TYPE_DASHED_LINE;    //虚线
    }
    else if(laneBoundaryType == RoadMark::TYPE_SOLIDSOLID)
    {
        type = osi3::LaneBoundary::Classification::Type::LaneBoundary_Classification_Type_TYPE_SOLID_LINE;     //双实线对应OSI的实线
    }
    else if(laneBoundaryType == RoadMark::TYPE_BROKENBROKEN)
    {
        type = osi3::LaneBoundary::Classification::Type::LaneBoundary_Classification_Type_TYPE_DASHED_LINE;    //双虚线对应OSI的虚线
    }
    else if(laneBoundaryType == RoadMark::TYPE_GRASS)
    {
        type = osi3::LaneBoundary::Classification::Type::LaneBoundary_Classification_Type_TYPE_GRASS_EDGE;     //草
    }
    else if(laneBoundaryType == RoadMark::TYPE_CURB)
    {
        type = osi3::LaneBoundary::Classification::Type::LaneBoundary_Classification_Type_TYPE_CURB ;          //curb
    }
    else
    {
        type = osi3::LaneBoundary::Classification::Type::LaneBoundary_Classification_Type_TYPE_OTHER;          //其它
    }

    return type;    
}

//转换laneBoundary.color
uint16_t OSIGroundTruthGeneratorThread::convertLaneBoundaryColorType(const RoadMark::RoadMarkColor roadMarkColor)
{
    uint16_t type = osi3::LaneBoundary::Classification::Color::LaneBoundary_Classification_Color_COLOR_UNKNOWN;

    if (roadMarkColor == RoadMark::COLOR_STANDARD)
    {
        type = osi3::LaneBoundary::Classification::Color::LaneBoundary_Classification_Color_COLOR_WHITE;        //白色
    }
    else if (roadMarkColor == RoadMark::COLOR_YELLOW)
    {
        type = osi3::LaneBoundary::Classification::Color::LaneBoundary_Classification_Color_COLOR_YELLOW;       //黄色
    }
    else if (roadMarkColor == RoadMark::COLOR_RED)
    {
        type = osi3::LaneBoundary::Classification::Color::LaneBoundary_Classification_Color_COLOR_RED;          //红色
    }
    else if (roadMarkColor == RoadMark::COLOR_BLUE)
    {
        type = osi3::LaneBoundary::Classification::Color::LaneBoundary_Classification_Color_COLOR_BLUE;         //蓝色
    }
    else if (roadMarkColor == RoadMark::COLOR_GREEN)
    {
        type = osi3::LaneBoundary::Classification::Color::LaneBoundary_Classification_Color_COLOR_GREEN;        //绿色
    }
    else if (roadMarkColor == RoadMark::COLOR_ORANGE)
    {
        type = osi3::LaneBoundary::Classification::Color::LaneBoundary_Classification_Color_COLOR_ORANGE;       //橙色
    }
    else
    {
        // for MISRA
        type = osi3::LaneBoundary::Classification::Color::LaneBoundary_Classification_Color_COLOR_UNKNOWN;  //
    }

    return type;
}

//获取centerline
std::list<Vector3D> OSIGroundTruthGeneratorThread::getCenterlinePoint(int laneId)
{
    std::list<Vector3D> centerlinePointList; // 存储centerline的坐标点
    if (egoState != nullptr)
    {
        Road *road = RoadSystem::Instance()->getRoad(std::to_string(egoState->sObjectState.u8RoadId));
        if (road != nullptr)
        {
            LaneSection *laneSection = road->getLaneSection(egoState->sObjectState.u4RoadS);
            if (laneSection != nullptr)
            {
                double u = egoState->sObjectState.u4RoadS;
                double roadLength = road->getLength();
                double sampleS = 0.0; //采样点s坐标 从车后方5米开始

                if (direction > 0)
                {
                    sampleS = u - 5.0;
                }
                else
                {
                    sampleS = u + 5.0;
                }

                Vector2D laneCenter = laneSection->getLaneCenter(laneId, u);

                for (int i = 0; i < 50; i++)
                {
                    sampleS += 1 * direction;
                    if ((sampleS < 0.0) || (sampleS > roadLength))
                    {
                        continue;
                    }
                    Transform vehicleTransform = road->getRoadTransform(sampleS, laneCenter[0]);
                    centerlinePointList.push_back(vehicleTransform.v());

                }

            }
        }
    }

    return centerlinePointList;
}

//更新主车周围交通标志(主车所在Road与下一条Road上存在的交通标志Id)
void OSIGroundTruthGeneratorThread::updateTrafficSignAroundEgo(uint32_t egoId)
{
    /*************** 更新主车周围的交通标志Id，保存到trafficSignAroundEgo *****************/
    trafficSignAroundEgo.clear(); //先清空list

    if (egoState != nullptr)
    {
        Road *currentRoad = RoadSystem::Instance()->getRoad(std::to_string(egoState->sObjectState.u8RoadId));

        if (currentRoad != nullptr)
        {
            //获取当前Road的TrafficSignVector
            std::vector<RoadSignal *> trafficSignVector = currentRoad->getTrafficSignVector();

            log_compnt_mngr->info("road id = {}, updateTrafficSignAroundEgo size = {}", currentRoad->getId(), trafficSignVector.size());

            //获取下一条Road的TrafficSignVector
            int nextRoadType = 0; //1非junction，2是junction
            Road* newRoad = NULL;
            std::list<std::pair<double, Road *>> junctionRoadList; //1左转 0直行 -1右转

            (void)currentRoad->getNextRoad(nextRoadType, newRoad, junctionRoadList, direction);
            
            if (nextRoadType == 1) // 下一条路非junction
            {
                if (newRoad != nullptr)
                {
                    std::vector<RoadSignal *> nextRoadTrafficSignVector = newRoad->getTrafficSignVector();
                    (void)trafficSignVector.insert(trafficSignVector.end(), nextRoadTrafficSignVector.begin(), nextRoadTrafficSignVector.end());
                }
            }

            //更新trafficSignAroundEgo
            for (std::vector<RoadSignal *>::iterator It = trafficSignVector.begin(); It != trafficSignVector.end(); It++)
            {
                trafficSignAroundEgo.push_back((*It)->getId());
            }
        }
        else
        {
            log_compnt_mngr->error("updateTrafficSignAroundEgo road = nullptr.");
        }
    }
    else
    {
        //donothing
    }
}

//保存数据到pb文件
void OSIGroundTruthGeneratorThread::savePbDataToFile()
{
    simDataFrame.Clear();

    if (egoState != nullptr)
    {
        simDataFrame.Clear();
        osi3::GroundTruth *newGroundTruth = simDataFrame.add_frames();
        
        *newGroundTruth = groundTruth;

        if (fd > 0)
        {
            simDataFrame.SerializePartialToFileDescriptor(fd);
        }
        else
        {
            log_compnt_mngr->error("fd error.");
        }        
    }
}

//仿真停止
void OSIGroundTruthGeneratorThread::simulationStop()
{
    osiMsg.msgMutex.lock();

    simulationRunStatus = false;

    osiMsg.msgMutex.unlock();
}

bool OSIGroundTruthGeneratorThread::parseMsg(char *msgBuff, unsigned int msgSize)
{
    bool result = true; //是否成功

    S_SP_MSG_HDR *msgHead = (S_SP_MSG_HDR *)msgBuff; //Msg的头部指针

    //考虑msg长度错误的情况
    if (((msgBuff == nullptr) || (msgHead->u4HeaderSize + msgHead->u4DataSize) != msgSize))
    {
        result = false;
        return result;
    }

    timer = msgHead->u8SimTime;

    char *currentPkg = msgBuff + msgHead->u4HeaderSize; //当前Pkg的头部指针

    // 解析每个pkg
    while (true)
    {
        S_SP_MSG_ENTRY_HDR *pkgHead = (S_SP_MSG_ENTRY_HDR *)currentPkg; //Pkg的头部指针

        if (pkgHead->u2PkgId == D_SP_MIL_PKG_ID_EGO_DATA) //解析D_SP_MIL_PKG_ID_EGO_DATA
        {
            S_SP_MIL_EGO_STATE *pkgData = (S_SP_MIL_EGO_STATE *)(currentPkg + pkgHead->u4HeaderSize); //数据部分指针
            egoState = pkgData;
        }
        else if (pkgHead->u2PkgId == D_SP_MIL_PKG_ID_OBJECT_DATA) //解析S_SP_MIL_OBJECT_STATE
        {
            S_SP_MIL_OBJECT_STATE *pkgData = (S_SP_MIL_OBJECT_STATE *)(currentPkg + pkgHead->u4HeaderSize);  //数据部分指针

            int elementNum = pkgHead->u4DataSize / pkgHead->u4ElementSize; //元素数量

            for (int i = 0; i < elementNum; i++) //解析每个元素
            {
                uint8_t type = pkgData->sObjectState.u1Type;
                //车辆
                if ((type == D_SP_OBJECT_TYPE_CAR) || (type == D_SP_OBJECT_TYPE_MOTORBIKE) || (type == D_SP_OBJECT_TYPE_BICYCLE)
                    || (type == D_SP_OBJECT_TYPE_BUS) || (type == D_SP_OBJECT_TYPE_TRUCK))
                {
                    vehicleList.push_back(pkgData);
                }
                //行人
                else if ((type == D_SP_OBJECT_TYPE_PEDESTRIAN) || (type == D_SP_OBJECT_TYPE_ANIMAL))
                {
                    pedestrianList.push_back(pkgData);
                }
                //障碍物
                else
                {
                    obstacleList.push_back(pkgData);
                }

                pkgData = (S_SP_MIL_OBJECT_STATE *)(((char *)pkgData) + pkgHead->u4ElementSize);
            }
        }
        else if (pkgHead->u2PkgId == D_SP_PKG_ID_TRAFFIC_LIGHT) //解析D_SP_PKG_ID_TRAFFIC_LIGHT
        {
            S_SP_TRAFFIC_LIGHT *pkgData = (S_SP_TRAFFIC_LIGHT *)(currentPkg + pkgHead->u4HeaderSize); //数据部分指针
            trafficLightList.push_back(pkgData);
        }
        else if (pkgHead->u2PkgId == D_SP_PKG_ID_TRAFFIC_SIGN) //解析D_SP_PKG_ID_TRAFFIC_SIGN
        {
            S_SP_TRAFFIC_SIGN *pkgData = (S_SP_TRAFFIC_SIGN *)(currentPkg + pkgHead->u4HeaderSize);  //数据部分指针

            int elementNum = pkgHead->u4DataSize / pkgHead->u4ElementSize; //元素数量

            for (int i = 0; i < elementNum; i++) //解析每个元素
            {
                trafficSignList.push_back(pkgData);

                pkgData = (S_SP_TRAFFIC_SIGN *)(((char *)pkgData) + pkgHead->u4ElementSize);
            }
        }
        else if (pkgHead->u2PkgId == D_SP_PKG_ID_LANE_INFO) //解析D_SP_PKG_ID_LANE_INFO
        {
            S_SP_LANE_INFO *pkgData = (S_SP_LANE_INFO *)(currentPkg + pkgHead->u4HeaderSize); //数据部分指针

            int elementNum = pkgHead->u4DataSize / pkgHead->u4ElementSize; //元素数量

            for (int i = 0; i < elementNum; i++) //解析每个元素
            {
                laneList.push_back(pkgData);

                laneIdVec.push_back(std::make_pair(pkgData->u8RoadId, (int)pkgData->u1Id));

                pkgData = (S_SP_LANE_INFO *)((char *)pkgData + pkgHead->u4ElementSize);
            }
        }
        else if (pkgHead->u2PkgId == D_SP_MIL_PKG_ID_ROADMARK) //解析D_SP_MIL_PKG_ID_ROADMARK
        {
            S_SP_MIL_ROADMARK *pkgData = (S_SP_MIL_ROADMARK *)(currentPkg + pkgHead->u4HeaderSize); //数据部分指针

            int elementNum = pkgHead->u4DataSize / pkgHead->u4ElementSize; //元素数量

            for (int i = 0; i < elementNum; i++) //解析每个元素
            {
                laneBoundaryList.push_back(pkgData);
            }

            pkgData = (S_SP_MIL_ROADMARK *)((char *)pkgData + pkgHead->u4ElementSize);     
        }
        else if (pkgHead->u2PkgId == D_SP_PKG_ID_ENVIRONMENT) //解析D_SP_PKG_ID_ENVIRONMENT
        {
            S_SP_ENVIRONMENT *pkgData = (S_SP_ENVIRONMENT *)(currentPkg + pkgHead->u4HeaderSize); //数据部分指针
            environment = pkgData;
        }
        // if (pkgHead->u2PkgId == D_SP_PKG_ID_INIT_PARAM)
        // {
        //     S_SP_INIT_PARAM *pkgData = (S_SP_INIT_PARAM *)(currentPkg + pkgHead->u4HeaderSize); //数据部分指针
        //     xodr = pkgData->au1OpenDrive;
        //     log_compnt_mngr->debug("parse IS_START xodr={}.", xodr.c_str());

        //     if (lastTimer == 0.0)
        //     {
        //         RoadSystem::Instance()->parseOpenDrive(xodr);               
        //     }
        // }
        // else if (pkgHead->u2PkgId == D_SP_PKG_ID_SENSOR_DETECTION_INFO) //解析D_SP_SENSOR_INFO
        // {
        //     S_SP_SENSOR_INFO *pkgData = (S_SP_SENSOR_INFO *)(currentPkg + pkgHead->u4HeaderSize); //数据部分指针

        //     int elementNum = pkgHead->u4DataSize / pkgHead->u4ElementSize; //元素数量

        //     for (int i = 0; i < elementNum; i++) //解析每个元素
        //     {
        //         sensorList.push_back(pkgData);
        //     }

        //     pkgData = (S_SP_SENSOR_INFO *)((char *)pkgData + pkgHead->u4ElementSize);    
        // }
        // else if (pkgHead->u2PkgId == D_SP_PKG_ID_SENSOR_DETECTION_INFO) //解析D_SP_PKG_ID_SENSOR_DETECTION_INFO
        // {
        //     S_SP_SENSOR_DETECTION_INFO *pkgData = (S_SP_SENSOR_DETECTION_INFO *)(currentPkg + pkgHead->u4HeaderSize); //数据部分指针

        //     int elementNum = pkgHead->u4DataSize / pkgHead->u4ElementSize; //元素数量

        //     for (int i = 0; i < elementNum; i++) //解析每个元素
        //     {
        //         sensorDetectionList.push_back(pkgData);
        //     }

        //     pkgData = (S_SP_SENSOR_DETECTION_INFO *)((char *)pkgData + pkgHead->u4ElementSize);                
        // }
        // else if (pkgHead->u2PkgId == D_SP_PKG_ID_MODEL_OUTLINE) //解析D_SP_PKG_ID_MODEL_OUTLINE
        // {
        //     S_SP_MODEL_OUTLINE *pkgData = (S_SP_MODEL_OUTLINE *)(currentPkg + pkgHead->u4HeaderSize);  //数据部分指针

        //     int elementNum = pkgHead->u4DataSize / pkgHead->u4ElementSize; //元素数量

        //     m_modelOutline.clear();
        //     for (int i = 0; i < elementNum; i++) //解析每个元素
        //     {
        //         std::vector<Vector3D> pointVec;

        //         //遍历每个点
        //         for (unsigned int j = 0; j < 16; j++)
        //         {
        //             Vector3D xyz(pkgData->au8OutlinePoints[j][0], pkgData->au8OutlinePoints[j][1], pkgData->au8OutlinePoints[j][2]);
        //             pointVec.push_back(xyz);
        //         }

        //         m_modelOutline[pkgData->au1ModelName] = pointVec;

        //         pkgData = (S_SP_MODEL_OUTLINE *)(((char *)pkgData) + pkgHead->u4ElementSize);
        //     }
        // }
        else if (pkgHead->u2PkgId == D_SP_PKG_ID_END_FRAME) //如果是最后一个Pkg
        {
            break;
        }

        currentPkg += pkgHead->u4HeaderSize + pkgHead->u4DataSize; //指向下一个pkg
    }

    return result;
}

//获取lane的前后连接
std::pair<std::list<unsigned long int>, std::list<unsigned long int>> OSIGroundTruthGeneratorThread::getConnectLane(Road *road, LaneSection *currentLaneSection, int _laneId)
{
    std::pair<std::list<unsigned long int>, std::list<unsigned long int>> laneConnectPair; // 用于保存前后连接的lane的id
    std::list<unsigned long int> antecessorLaneIdList; // 用于保存上一条连接的lane的id
    std::list<unsigned long int> successorLaneIdList; // 用于保存下一条连接的lane的id

    if (egoState != nullptr)
    {
        if ((road != nullptr) && (currentLaneSection != nullptr))
        {
            bool isLastLaneSection = false; // 当前laneSection是否是最后一段laneSection
            bool isFirstLaneSection = false; // 当前laneSection是第一段laneSection
            if (road->getLaneSectionNext(currentLaneSection->getStart()) == nullptr)
            {
                isLastLaneSection = true;
            }

            if (currentLaneSection->getStart() == 0)
            {
                isFirstLaneSection = true;
            }

            int tempDirection = 0;

            if (road == RoadSystem::Instance()->getRoad(std::to_string(egoState->sObjectState.u8RoadId)))
            {
                tempDirection = this->direction;
            }
            else
            {
                if (_laneId > 0)
                {
                    tempDirection = -1;
                }
                else
                {
                    tempDirection = 1;
                }
            }

            if (isFirstLaneSection) // road的开始
            {
                //获取下一条路
                TarmacConnection *tarmacConnection = nullptr;
                if (tempDirection == 1)
                {
                    tarmacConnection = road->getPredecessorConnection();
                }
                else
                {
                    tarmacConnection = road->getSuccessorConnection();
                }

                Junction *nextJunction = nullptr;
                if (tarmacConnection != nullptr)
                {
                    nextJunction = dynamic_cast<Junction *>(tarmacConnection->getConnectingTarmac()); //如果下一条路是路口则返回NULL

                    Road *nexConnectRoad = dynamic_cast<Road *>(tarmacConnection->getConnectingTarmac());

                    if (nextJunction != nullptr) // 下一条路是junction
                    {
                        PathConnectionSet connectSet = nextJunction->getPathConnectionSet(road, _laneId); // 与当前road当前lane的所有的连接关系
                        std::map<double, PathConnection *> pathConnect = connectSet.getFrequencySumMap();
                        for (const auto &it :  pathConnect)
                        {
                            Road *nextRoad = nullptr;
                            nextRoad = it.second->getConnectingPath(); // 获取相连接的下一条路
                            if (nextRoad != nullptr)
                            {
                                int connectLaneId = nextJunction->getConnectingLane(road, nextRoad, _laneId);
                                if (connectLaneId != Lane::NOLANE)
                                {
                                    antecessorLaneIdList.push_back((stoul(nextRoad->getId()) + 1) * 1000 + connectLaneId);
                                }
                            }
                        }
                    }
                    else if (nexConnectRoad != nullptr)
                    {
                        int predecessorLaneId = currentLaneSection->getLanePredecessor(_laneId);
                        if (predecessorLaneId != Lane::NOLANE)
                        {
                            antecessorLaneIdList.push_back((stoul(nexConnectRoad->getId()) + 1 ) * 1000 + predecessorLaneId);
                        }
                    }
                }
            }
            if (isLastLaneSection) // road的尽头
            {
                //获取下一条路
                TarmacConnection *tarmacConnection = nullptr;
                if (tempDirection == 1)
                {
                    tarmacConnection = road->getSuccessorConnection();
                }
                else
                {
                    tarmacConnection = road->getPredecessorConnection();
                }

                Junction *nextJunction = nullptr;
                if (tarmacConnection != nullptr)
                {
                    nextJunction = dynamic_cast<Junction *>(tarmacConnection->getConnectingTarmac()); //如果下一条路是路口则返回NULL

                    Road *nexConnectRoad = dynamic_cast<Road *>(tarmacConnection->getConnectingTarmac());

                    if (nextJunction != nullptr) // 下一条路是junction
                    {
                        PathConnectionSet connectSet = nextJunction->getPathConnectionSet(road, _laneId); // 与当前road当前lane的所有的连接关系
                        std::map<double, PathConnection *> pathConnect = connectSet.getFrequencySumMap();
                        for (const auto &it :  pathConnect)
                        {
                            Road *nextRoad = nullptr;
                            nextRoad = it.second->getConnectingPath(); // 获取相连接的下一条路

                            if (nextRoad != nullptr)
                            {
                                int connectLaneId = nextJunction->getConnectingLane(road, nextRoad, _laneId);
                                if (connectLaneId != Lane::NOLANE)
                                {
                                    successorLaneIdList.push_back((stoul(nextRoad->getId()) + 1) * 1000 + connectLaneId);
                                }
                            }
                        }
                    }
                    else if (nexConnectRoad != nullptr)
                    {
                        int successorLaneId = currentLaneSection->getLaneSuccessor(_laneId);
                        if (successorLaneId != Lane::NOLANE)
                        {
                            successorLaneIdList.push_back((stoul(nexConnectRoad->getId()) + 1 ) * 1000 + successorLaneId);
                        }
                    }
                }
            }
            if (!isLastLaneSection) // 非road尽头
            {
                int successorLaneId = currentLaneSection->getLaneSuccessor(_laneId);
                if (successorLaneId != Lane::NOLANE)
                {
                    successorLaneIdList.push_back((stoul(road->getId()) + 1 ) * 1000 + successorLaneId);
                }
            }
            if (!isFirstLaneSection) // 非road开始
            {
                int predecessorLaneId = currentLaneSection->getLanePredecessor(_laneId);
                if (predecessorLaneId != Lane::NOLANE)
                {
                    antecessorLaneIdList.push_back((stoul(road->getId()) + 1 ) * 1000 + predecessorLaneId);
                }
            }
        }
    }
    laneConnectPair.first = antecessorLaneIdList;
    laneConnectPair.second = successorLaneIdList;
    return laneConnectPair;
}

unsigned long OSIGroundTruthGeneratorThread::stoul(const std::string &input)
{
    unsigned long output = 0.0;
    try
    {
        output = std::stoul(input);
    }
    catch (const std::exception &e)
    {
        perror("stoul, catch exception.");
    }
    return output;
}

//获取车道线坐标
std::list<Vector3D> OSIGroundTruthGeneratorThread::getLaneBoundaryPoint(int laneId)
{
    std::list<Vector3D> pointList; //车道线的点
    if (egoState != nullptr)
    {
        Road *road = RoadSystem::Instance()->getRoad(std::to_string(egoState->sObjectState.u8RoadId));
        if (road != nullptr)
        {
            float s = egoState->sObjectState.u4RoadS;

            double roadLength = road->getLength();
            double sampleS = 0.0; //采样点s坐标 从车后方5米开始
            if (direction > 0)
            {
                sampleS = s - 5.0;
            }
            else
            {
                sampleS = s + 5.0;
            }

            //获取左侧右侧车道线的一系列坐标点（世界坐标）
            for (int i = 0; i < 50; i++) //取50个点
            {
                //更新sampleS，即下一个采样点s坐标
                sampleS += 2.0 * direction; //间隔2米
                if (sampleS < 0 || sampleS > roadLength)
                {
                    continue;
                }

                RoadPoint pointIn(0, 0, 0, 0, 0, 0); //内侧车道线的世界坐标
                RoadPoint pointOut(0, 0, 0, 0, 0, 0); //外侧车道线的世界坐标
                double disIn = 0;
                double disOut = 0;
                //获取内侧与外侧车道线的世界坐标和t坐标
                road->getLaneRoadPoints(sampleS, laneId, pointIn, pointOut, disIn, disOut);
                Vector3D pointOutVector3D(pointOut.x(), pointOut.y(), pointOut.z()); //只用外侧车道线上的点
                pointList.push_back(pointOutVector3D);
            }
        }
    }

    return pointList;
}

//将弧度角值域调整到[0, 2PI)
double OSIGroundTruthGeneratorThread::normalizeRad(double angle)
{
    //值域调整到(-2PI, 2PI)
    double output = fmod(angle, 2 * M_PI);

    //值域调整到[0, 2PI)
    if (output < 0)
    {
        output += 2 * M_PI;
    }

    return output;
}

//将弧度角值域调整到[-PI, PI)
double OSIGroundTruthGeneratorThread::normalizeRad2(double angle)
{
    //值域调整到[0, 2PI)
    double output = normalizeRad(angle);

    //值域调整到[-PI, PI)
    if (output >= M_PI) //范围[M_PI, 2PI)
    {
        output -= 2 * M_PI;
    }

    return output;
}

//xodr文件解析完成
void OSIGroundTruthGeneratorThread::informParseXodrComplete()
{
    sem_post(&semParseXodr);
}

//回收线程
void OSIGroundTruthGeneratorThread::joinThread()
{
    if (ConfigureMngr::getInstance()->getCoSimuInterfaceType() == 1)
    {
        if (generator_osi_pb_file_handle.joinable())
        {
            generator_osi_pb_file_handle.join();
        }
    }
    else
    {
        if (save_osi_pb_file_handle.joinable())
        {
            save_osi_pb_file_handle.join();
        }
    }
}

//获取主车周围的交通灯id
OsiTrafficLightAroundEgo &OSIGroundTruthGeneratorThread::getTrafficLightAroundEgo()
{
    /*************** 更新主车周围的交通灯Id，保存到trafficLightAroundEgo *****************/
    //赋初值
    (void)strcpy(trafficLightAroundEgo.currentRoadTLId, "");                         /** 当前road的交通灯Id */
    (void)strcpy(trafficLightAroundEgo.leftTLId, "");                                /** 下一个路口的左转交通灯Id */
    (void)strcpy(trafficLightAroundEgo.goStraightTLId, "");                          /** 下一个路口的直行交通灯Id */
    (void)strcpy(trafficLightAroundEgo.rightTLId, "");                               /** 下一个路口右转交通灯Id */

    trafficLightAroundEgo.currentRoadId = 0;                                         /** 当前road的交通灯Id */
    trafficLightAroundEgo.leftRoadId = 0;                                            /** 下一个路口的左转交通灯Id */
    trafficLightAroundEgo.goStraightRoadId = 0;                                      /** 下一个路口的直行交通灯Id */
    trafficLightAroundEgo.rightRoadId = 0;                                           /** 下一个路口右转交通灯Id */

    (void)strcpy(trafficLightAroundEgo.nextRoadTLId, "");                  /** 下一条非junction道路的交通灯Id */
    trafficLightAroundEgo.nextRoadId = 0;                                            /** 下一条非junction道路的Id */

    //获取交通灯id
    if (egoState != nullptr)
    {
        Road *currentRoad = RoadSystem::Instance()->getRoad(std::to_string(egoState->sObjectState.u8RoadId));

        if (currentRoad != nullptr)
        {
            std::string refSignalId = currentRoad->getRefSignalId();
            const auto refSignalIdSize = refSignalId.length();

            if (refSignalIdSize > D_OSI_SIZE_OBJECT_ID - 2)
            {
                refSignalId = refSignalId.substr(0, D_OSI_SIZE_OBJECT_ID - 2);
            }

            (void)strcpy(trafficLightAroundEgo.currentRoadTLId, refSignalId.c_str());                       /** 当前road的交通灯Id */

            trafficLightAroundEgo.currentRoadId = stoul(currentRoad->getId());

            if (currentRoad->isJunctionPath()) //如果主车在路口内， 只需填充当前road的交通灯Id
            {
                // DO NOTHING
            }
            else
            {
                //获取下一条路
                int nextRoadType = 0; //1非junction，2是junction
                Road* newRoad = NULL;
                std::list<std::pair<double, Road *>> junctionRoadList; //1左转 0直行 -1右转

                (void)currentRoad->getNextRoad(nextRoadType, newRoad, junctionRoadList, direction);
                if (nextRoadType == 1) // 下一条路非junction, 有交通灯
                {
                    if (newRoad != nullptr)
                    {
                        (void)strncpy(trafficLightAroundEgo.nextRoadTLId, newRoad->getRefSignalId().c_str(), D_OSI_SIZE_OBJECT_ID);
                        trafficLightAroundEgo.nextRoadId = stoul(newRoad->getId());
                    }
                }
                else if (nextRoadType == 2) // 下一条路是junction，填充相应方向的交通灯Id
                {
                    for (std::list<std::pair<double , Road *> >::iterator roadIt = junctionRoadList.begin(); roadIt != junctionRoadList.end(); roadIt++)
                    {
                        if (roadIt->second != nullptr)
                        {
                            Road *road = roadIt->second;

                            const std::string &refSignalId = road->getRefSignalId();

                            //获取这条路的Id
                            unsigned long int nextRoadId = stoul(road->getId());

                            if( abs(roadIt->first - 1.0) < 1e-5 ) //左转
                            {
                                log_compnt_mngr->info("updateAroundEgo, updateAroundEgo 1");
                                (void)strcpy(trafficLightAroundEgo.leftTLId, refSignalId.c_str());        /** 下一个路口的左转交通灯Id */
                                trafficLightAroundEgo.leftRoadId = nextRoadId;                            /** 下一个路口的左转road的Id */
                            }
                            else if( fabs(roadIt->first) < 1e-5 ) //直行
                            {
                                log_compnt_mngr->info("updateAroundEgo, updateAroundEgo 2");
                                (void)strcpy(trafficLightAroundEgo.goStraightTLId, refSignalId.c_str());    /** 下一个路口的直行交通灯Id */
                                trafficLightAroundEgo.goStraightRoadId = nextRoadId;                        /** 下一个路口的直行road的Id */
                            }
                            else if( abs(roadIt->first + 1.0) < 1e-5 ) //右转
                            {
                                log_compnt_mngr->info("updateAroundEgo, updateAroundEgo 3");
                                (void)strcpy(trafficLightAroundEgo.rightTLId, refSignalId.c_str());        /** 下一个路口右转交通灯Id */
                                trafficLightAroundEgo.rightRoadId = nextRoadId;                            /** 下一个路口右转road的Id */
                            }
                            else
                            {
                                // Comment: Nothing to do.
                            }
                        }
                    }
                }
            }
        }
        else
        {
            //do nothing 
        }
    }
    else
    {
        // do nothing
    }
}

// 解析车辆模型catalog文件
void OSIGroundTruthGeneratorThread::parseVehicleCatalogFile()
{
    vehicleCatalogDataMap.clear();
    simproxml::XMLDocument doc;
    std::string vehicleCatalogFile = "./Catalogs/VehicleCatalogs/Vehicle_0_9.xosc";

    //加载文件
    if (doc.LoadFile(vehicleCatalogFile.c_str()) == simproxml::XML_SUCCESS)
    {
        log_compnt_mngr->info("OSIGroundTruthGeneratorThread vehicleCatalogFile load success");
        simproxml::XMLElement* root = doc.RootElement();
        if (root != nullptr)
        {
            log_compnt_mngr->info("OSIGroundTruthGeneratorThread vehicleCatalogFile getRootElement success");
            //获取Catalog标签
            simproxml::XMLElement* CatalogNode = root->FirstChildElement("Catalog");
            {
                if (CatalogNode != nullptr)
                {
                    log_compnt_mngr->critical("OSIGroundTruthGeneratorThread vehicleCatalogFile getCatalogNode success");
                    //遍历每个vehicle节点
                    for (simproxml::XMLElement *vehicleNode = CatalogNode->FirstChildElement(); vehicleNode != NULL; vehicleNode = vehicleNode->NextSiblingElement())
                    {
                        std::string vehicleName = "";
                        OsiVehicleCatalog vehicleData = {0};
                        const char *vehicleNameAttribute = vehicleNode->Attribute("name");
                        if (vehicleNameAttribute != nullptr)
                        {
                            vehicleName = std::string(vehicleNameAttribute);

                            //解析最大减速度、质量
                            simproxml::XMLElement* performanceNode = vehicleNode->FirstChildElement("Performance");
                            if (performanceNode != nullptr)
                            {
                                vehicleData.maxDeceleration = performanceNode->DoubleAttribute("maxDeceleration");
                                vehicleData.mass = performanceNode->DoubleAttribute("mass");
                            }

                            //解析前轴宽度、后轴宽度
                            simproxml::XMLElement* axlesNode = vehicleNode->FirstChildElement("Axles");
                            if (axlesNode != nullptr)
                            {
                                simproxml::XMLElement* frontNode = axlesNode->FirstChildElement("Front");
                                if (frontNode != nullptr)
                                {
                                    vehicleData.frontTrackWidth = frontNode->DoubleAttribute("trackWidth");
                                }

                                simproxml::XMLElement* rearNode = axlesNode->FirstChildElement("Rear");
                                if (rearNode != nullptr)
                                {
                                    vehicleData.rearTrackWidth = rearNode->DoubleAttribute("trackWidth");
                                }
                            }

                            //解析最大加速度、轴距
                            simproxml::XMLElement* propertiesNode = vehicleNode->FirstChildElement("Properties");
                            if (propertiesNode != nullptr)
                            {
                                for (simproxml::XMLElement* propertyNode = propertiesNode->FirstChildElement("Property"); propertyNode != nullptr; propertyNode = propertyNode->NextSiblingElement())
                                {
                                    const char* propertyNameAttribute = propertyNode->Attribute("name");
                                    if (propertyNameAttribute != nullptr)
                                    {
                                        if (strncmp(propertyNameAttribute, "maxAcceleration", 16) == 0)
                                        {
                                            vehicleData.maxAcceleration = propertyNode->DoubleAttribute("value");
                                        }
                                        else if (strncmp(propertyNameAttribute, "wheelBase", 10) == 0)
                                        {
                                            vehicleData.wheelBase = propertyNode->DoubleAttribute("value");
                                        }
                                        else
                                        {
                                            // do nothing
                                        }
                                    }
                                }
                            }

                            vehicleCatalogDataMap.insert(std::make_pair(vehicleName, vehicleData));
                            log_compnt_mngr->info("parseVehicleCatalogFile insert vehicleName = {}, mass = {}, wheelBase = {}.", vehicleName.c_str(), vehicleData.mass, vehicleData.wheelBase);
                        }
                    }
                }
            }
        }
    }
}

//创建只用于pb文件数据存储的线程
void OSIGroundTruthGeneratorThread::savePbDataFileThread()
{
    while (true)
    {
        sem_wait(&sem);
        int semValue = 0;
        sem_getvalue(&sem, &semValue);
        log_compnt_mngr->critical("savePbDataFileThread sem_wait value = {}", semValue);
        simDataFrame.Clear();

        osiMsg.msgMutex.lock();

        if (osiGroundTruthList.size() > 0)
        {
            osiGroundTruth = osiGroundTruthList.front();
            osiGroundTruthList.pop_front();
        }

        osiMsg.msgMutex.unlock();

        osi3::GroundTruth *newGroundTruth = simDataFrame.add_frames();
        *newGroundTruth = osiGroundTruth;
        if (fd > 0)
        {
            simDataFrame.SerializePartialToFileDescriptor(fd);
        }
        else
        {
            log_compnt_mngr->error("fd error.");
        }

        osiMsg.msgMutex.lock();

        log_compnt_mngr->info("simulationRunStatus = {}, osiGroundTruthList.size = {}. ", simulationRunStatus, osiGroundTruthList.size());
        if ((simulationRunStatus == false) && (osiGroundTruthList.size() == 0))
        {
            osiMsg.msgMutex.unlock();
            break;
        }
        else
        {
            //do nothing
        }

        osiMsg.msgMutex.unlock();
    }

    //关闭文件
    close(fd);
    fd = -1;

    if (count > 1)
    {
        //仿真结束延迟通知评估
        sleep(10);

        //倒数第二个pb文件通知评估
        memset(param.OSIGroundTruthPbPath, 0, D_PARAM_OSI_PB_PATH_SIZE);
        (void)strcpy(reinterpret_cast<char *>(param.OSIGroundTruthPbPath), lastPbFilePath.c_str()); /* OSIGroundTruth的Pb文件地址 */
        param.egoDistance = lastEgoTraveledDist; /* 主车行驶里程 */

        memset(param.NGinformationCsvPath, 0, D_PARAM_NGINFO_CSV_PATH_SIZE);
        (void)strcpy(reinterpret_cast<char *>(param.NGinformationCsvPath), lastCsvFilePath.c_str()); /* csv文件保存路径 */

        log_compnt_mngr->critical("EvaluationAPI::Instance()->notify last start ");
        EvaluationAPI::Instance()->notify(&param);
    }

    //仿真结束延迟通知评估
    sleep(10);

    //仿真结束通知评估
    memset(param.OSIGroundTruthPbPath, 0, D_PARAM_OSI_PB_PATH_SIZE);
    (void)strcpy(reinterpret_cast<char *>(param.OSIGroundTruthPbPath), currentPbFilePath.c_str()); /* OSIGroundTruth的Pb文件地址 */
    param.egoDistance = egoTraveledDist; /* 主车行驶里程 */
    param.isFinish = true;

    memset(param.NGinformationCsvPath, 0, D_PARAM_NGINFO_CSV_PATH_SIZE);
    (void)strcpy(reinterpret_cast<char *>(param.NGinformationCsvPath), csvFilePath.c_str()); /* csv文件保存路径 */

    log_compnt_mngr->critical("EvaluationAPI::Instance()->notify finish start");
    EvaluationAPI::Instance()->notify(&param);

    //释放OSI数据内存
    if (osiMsgBuffer != nullptr)
    {
        free(osiMsgBuffer);
        osiMsgBuffer = nullptr;
    }
}

//生成osi联仿数据
bool OSIGroundTruthGeneratorThread::generatorOsiCosimuData(OSI_PB_DATA &data)
{
    //解析simpro发送给cm的数据用来填充pb文件数据
    if (parseMsg(data.msg, data.msgLen) == false)
    {
        log_compnt_mngr->error("OSIGroundTruthGeneratorThread generatorOsiCosimuData parseMsg error");
        return false;
    }

    if (isFirstFrame)
    {
        parseVehicleCatalogFile();

        // 获取当前时间点
        auto now = std::chrono::system_clock::now();

        // 转换为 time_t 类型
        std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);

        // 将 time_t 类型转换为本地时间
        std::tm* now_tm = std::localtime(&now_time_t);

        // 使用 std::strftime 将时间转换为字符串
        char buffer[80];
        std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", now_tm);
        time_str = buffer;

        //创建本次仿真pb文件目录
        std::string pbFileDir = "./output/groundTruth/" + time_str + "/1";
        const std::string cmd = "mkdir -p \"" + pbFileDir + "\"";
        (void)std::system(cmd.c_str());
        if(0 != access(pbFileDir.data(), F_OK | R_OK | W_OK)) 
        {
            log_compnt_mngr->error("{} NOT ACCESSIBLE.", pbFileDir);
        }

        csvFilePath = "/opt/simpro/workspace/output/NG_Scenario/" + time_str + "/1/" + "NG_INFO_" + std::to_string(count) + ".csv";

        currentPbFilePath = "/opt/simpro/workspace/output/groundTruth/" + time_str + "/1/" + "SimProGroundTruth_" + std::to_string(count) + ".pb";

        fd = open(currentPbFilePath.c_str(), O_RDWR | O_CREAT, S_IRWXU);
        if (fd == -1)
        {
            log_compnt_mngr->error("fd open error.");
        }

        isFirstFrame = false;
    }

    //填充pb文件数据
    fillData();

    if ((timer - lastTimer > TIME_STEP_PB_FILE) && (egoState != nullptr))
    {
        close(fd);
        fd = -1;

        if (count > 1) //延后10s发送pb文件
        {
            //10s生成pb文件后通知评估
            memset(param.OSIGroundTruthPbPath, 0, D_PARAM_OSI_PB_PATH_SIZE);
            (void)strcpy(reinterpret_cast<char *>(param.OSIGroundTruthPbPath), lastPbFilePath.c_str()); /* OSIGroundTruth的Pb文件地址 */
            param.egoDistance = lastEgoTraveledDist; /* 主车行驶里程 */

            memset(param.NGinformationCsvPath, 0, D_PARAM_NGINFO_CSV_PATH_SIZE);
            (void)strcpy(reinterpret_cast<char *>(param.NGinformationCsvPath), lastCsvFilePath.c_str()); /* csv文件保存路径 */

            EvaluationAPI::Instance()->notify(&param);
        }
        else
        {
            //do nothing
        }

        lastEgoTraveledDist = egoTraveledDist;
        lastPbFilePath = currentPbFilePath;
        lastCsvFilePath = csvFilePath;

        count++;
        currentPbFilePath = "/opt/simpro/workspace/output/groundTruth/" + time_str + "/1/" + "SimProGroundTruth_" + std::to_string(count) + ".pb";
        csvFilePath = "/opt/simpro/workspace/output/NG_Scenario/" + time_str + "/1/" + "NG_INFO_" + std::to_string(count) + ".csv";

        fd = open(currentPbFilePath.c_str(), O_RDWR | O_CREAT, S_IRWXU);
        if (fd == -1)
        {
            log_compnt_mngr->error("fd open error.");
        }

        lastTimer += TIME_STEP_PB_FILE; //更新上次保存pb文件时间
    }
    else
    {
        //do nothing
    }

    //保存数据到pb文件
    osiMsg.msgMutex.lock();
    osiGroundTruthList.push_back(groundTruth);
    osiMsg.msgMutex.unlock();
    sem_post(&sem);

    //填充一汽OSI导航算法所需数据
    if (ConfigureMngr::getInstance()->getIsOutputOsiGroundTruthLogicalLane())
    {
        if (isFirstFrame || (distanceForEgo(lastUpdataDataEgoPos) >= 5.0))//场景运行第一帧或主车位置与上一次更新数据时的位置距离超过5米
        {
            groundTruthOnlyWithLogicalLane.Clear();

            fillLogicalLane(groundTruthOnlyWithLogicalLane);
            fillReferenceLine(groundTruthOnlyWithLogicalLane);
            fillLogicalLaneBoundary(groundTruthOnlyWithLogicalLane);
            if (egoState != nullptr)
            {
                lastUpdataDataEgoPos = Vector3D(egoState->sObjectState.sPos.u8X, egoState->sObjectState.sPos.u8Y, egoState->sObjectState.sPos.u8Z);
            }
        }

        // std::cout << "logical_lane_size size = " << groundTruthOnlyWithLogicalLane.logical_lane_size() << " reference_line_size size = " << groundTruthOnlyWithLogicalLane.reference_line_size() << " logical_lane_boundary_size size = " << groundTruthOnlyWithLogicalLane.logical_lane_boundary_size() << std::endl;

        for (int i = 0; i < groundTruthOnlyWithLogicalLane.logical_lane_size(); i++)
        {
            const osi3::LogicalLane &logicalLane = groundTruthOnlyWithLogicalLane.logical_lane(i);
            osi3::LogicalLane *newLogicalLane = groundTruth.add_logical_lane();
            *newLogicalLane = logicalLane;
        }

        for (int i = 0; i < groundTruthOnlyWithLogicalLane.reference_line_size(); i++)
        {
            const osi3::ReferenceLine &referenceLine = groundTruthOnlyWithLogicalLane.reference_line(i);
            osi3::ReferenceLine *newReferenceLine = groundTruth.add_reference_line();
            *newReferenceLine = referenceLine;
        }

        for (int i = 0; i < groundTruthOnlyWithLogicalLane.logical_lane_boundary_size(); i++)
        {
            const osi3::LogicalLaneBoundary &logicalLaneBoundary = groundTruthOnlyWithLogicalLane.logical_lane_boundary(i);
            osi3::LogicalLaneBoundary *newLogicalLaneBoundary = groundTruth.add_logical_lane_boundary();
            *newLogicalLaneBoundary = logicalLaneBoundary;
        }
    }

    //osi联仿数据序列化为字符串
    groundTruth.SerializeToString(&osiCosimuData);

    //更新主车移动距离
    if (egoState != nullptr)
    {
        egoTraveledDist = egoState->sObjectState.u4TraveledDist;
    }

    //清空上一帧数据
    groundTruth.Clear();
    vehicleList.clear();
    pedestrianList.clear();
    obstacleList.clear();
    trafficLightList.clear();
    trafficSignList.clear();
    laneList.clear();
    laneBoundaryList.clear();
    trafficSignAroundEgo.clear();
    contrastLightMap.clear();
    idTraffciLightMap.clear();
    roadRefTraffciLightMap.clear();
    roadTraffciLightsMap.clear();
    trafficLightStatusList.clear();
    trafficOverallList.clear();
    laneIdVec.clear();
    direction = 0;
    egoState = nullptr;

    //释放缓存区数据
    delete [] data.msg;

    //清空data数据
    memset(&data, 0, sizeof(OSI_PB_DATA));

    return true;
}

//获取osi数据的字符串
const std::string &OSIGroundTruthGeneratorThread::getStringOsiGroundTruth()
{
    return osiCosimuData;
}

//打包OSI联仿数据
void OSIGroundTruthGeneratorThread::getCoSimOSIGroundTruthPkg(char *&msgBuffer, unsigned int &msgBufferUsedSize)
{
    //Pkg大小
    int pkgSize = static_cast<int>(sizeof(S_SP_MSG_ENTRY_HDR)) + osiCosimuData.length();

    //nullptr check
    if (pkgSize > osiMsgBufferAllocSize)
    {
        osiMsgBuffer = reinterpret_cast<char *>(realloc(osiMsgBuffer, pkgSize));
        if (osiMsgBuffer == nullptr) //realloc失败
        {
            log_compnt_mngr->error("getPkgData,realloc error");
            return;
        }
        osiMsgBufferAllocSize = pkgSize;
    }

    msgBuffer = osiMsgBuffer;
    msgBufferUsedSize = pkgSize;
    char *currentPkg = msgBuffer;
    S_SP_MSG_ENTRY_HDR *pkgHead = reinterpret_cast<S_SP_MSG_ENTRY_HDR *>(currentPkg); //Pkg的头部指针
    char *pkgBody = currentPkg + sizeof(S_SP_MSG_ENTRY_HDR); //Pkg的Body指针

    //填充Pkg头部
    pkgHead->u4HeaderSize = static_cast<uint32_t>(sizeof(S_SP_MSG_ENTRY_HDR));
    pkgHead->u4DataSize = osiCosimuData.length();
    pkgHead->u4ElementSize = osiCosimuData.length();
    pkgHead->u2PkgId = D_SP_PKG_ID_OSI_GROUNDTRUTH;

    //填充Body
    memcpy(pkgBody, osiCosimuData.data(), osiCosimuData.length());
}

//LogicalLane数据填充
void OSIGroundTruthGeneratorThread::fillLogicalLane(osi3::GroundTruth &_groundTruth)
{
    for (auto it : roadMap)
    {
        Road *road = it.first;
        if (road != nullptr)
        {
            unsigned long long roadId = stoul(road->getId());
            roadId = (roadId + 1) * 1000;

            std::map<double, LaneSection *> laneSectionMap = road->getLaneSectionMap();
            int i = 1;//laneSection序号

            for (auto it = laneSectionMap.begin(); it != laneSectionMap.end(); it++)
            {
                if (it->second != nullptr)
                {
                    std::map<int, Lane *> laneMap = it->second->getLaneMap();

                    for (auto itar : laneMap)
                    {
                        osi3::LogicalLane *logicalLane = _groundTruth.add_logical_lane();

                        //LogicalLane.id
                        osi3::Identifier *identifier = new osi3::Identifier();
                        identifier->set_value((roadId + itar.first) * 1000 + i);//(((roadId +1) * 1000) + laneId) * 1000 + laneSection的序号
                        logicalLane->set_allocated_id(identifier);

                        //LogicalLane.type
                        logicalLane->set_type(osi3::LogicalLane::Type(converLogicalLaneType(itar.second->getLaneType())));
                        
                        //LogicalLane.physical_lane_reference.id
                        osi3::LogicalLane_PhysicalLaneReference *physicalLaneReference = logicalLane->add_physical_lane_reference();
                        osi3::Identifier *identifier2 = new osi3::Identifier();
                        identifier2->set_value(roadId + itar.first);//(roadId +1) * 1000 + laneId
                        physicalLaneReference->set_allocated_physical_lane_id(identifier2);

                        //LogicalLane.start_s
                        logicalLane->set_start_s(it->first);

                        //LogicalLane.end_s
                        if (laneSectionMap.upper_bound(it->first) != laneSectionMap.end())
                        {
                            logicalLane->set_end_s(laneSectionMap.upper_bound(it->first)->first);
                        }
                        else
                        {
                            logicalLane->set_end_s(road->getLength());
                        }

                        //LogicalLane.move_direction
                        if (itar.first < 0)
                        {
                            logicalLane->set_move_direction(osi3::LogicalLane::MoveDirection::LogicalLane_MoveDirection_MOVE_DIRECTION_INCREASING_S);
                        }
                        else if (itar.first > 0)
                        {
                            logicalLane->set_move_direction(osi3::LogicalLane::MoveDirection::LogicalLane_MoveDirection_MOVE_DIRECTION_DECREASING_S);
                        }
                        else
                        {
                            logicalLane->set_move_direction(osi3::LogicalLane::MoveDirection::LogicalLane_MoveDirection_MOVE_DIRECTION_OTHER);
                        }

                        int leftLaneId = itar.first + 1; 											//左车道Id
                        int rightLaneId = itar.first - 1;											//右车道Id

                        //LogicalLane.right_adjacent_lane.id
                        if (laneMap.find(rightLaneId) != laneMap.end())
                        {
                            osi3::LogicalLane_LaneRelation *laneRight = logicalLane->add_right_adjacent_lane();
                            osi3::Identifier *identifier3 = new osi3::Identifier();
                            identifier3->set_value((roadId + rightLaneId) * 1000 + i);//(((roadId +1) * 1000) + laneId) * 1000 + laneSection的序号
                            laneRight->set_allocated_other_lane_id(identifier3);
                        }

                        //LogicalLane.left_adjacent_lane.id
                        if (laneMap.find(leftLaneId) != laneMap.end())
                        {
                            osi3::LogicalLane_LaneRelation *laneLeft = logicalLane->add_left_adjacent_lane();
                            osi3::Identifier *identifier4 = new osi3::Identifier();
                            identifier4->set_value((roadId + leftLaneId) * 1000 + i);//(((roadId +1) * 1000) + laneId) * 1000 + laneSection的序号
                            laneLeft->set_allocated_other_lane_id(identifier4);
                        }

                        int leftLaneBoundaryId = 0;//左侧车道线id
                        int rightLaneBoundaryId = 0;//右侧车道线id

                        if (itar.first < 0)
                        {
                            rightLaneBoundaryId = itar.first;
                            leftLaneBoundaryId = itar.first + 1;
                        }
                        else if (itar.first > 0)
                        {
                            leftLaneBoundaryId = itar.first;
                            rightLaneBoundaryId = itar.first - 1;
                        }

                        //LogicalLane.right_boundary_id
                        osi3::Identifier *identifier5 = logicalLane->add_right_boundary_id();
                        identifier5->set_value((roadId + rightLaneBoundaryId) * 1000 + i);//(((roadId +1) * 1000) + laneId) * 1000 + laneSection的序号

                        //LogicalLane.left_boundary_id
                        osi3::Identifier *identifier6 = logicalLane->add_left_boundary_id();
                        identifier6->set_value((roadId + leftLaneBoundaryId) * 1000 + i);//(((roadId +1) * 1000) + laneId) * 1000 + laneSection的序号
                        
                        if (laneSectionMap.size() > 1)
                        {
                            if (it == laneSectionMap.begin())//第一个元素 只有后继车道
                            {
                                int succesId = itar.second->getSuccessor();

                                if (succesId != Lane::NOLANE)
                                {
                                    //LogicalLane.successor_lane.id
                                    osi3::LogicalLane_LaneConnection *successorLane = logicalLane->add_successor_lane();
                                    osi3::Identifier *identifier7 = new osi3::Identifier(); 
                                    identifier7->set_value((succesId + roadId) * 1000 + i + 1);//(((roadId +1) * 1000) + laneId) * 1000 + laneSection的序号
                                    successorLane->set_allocated_other_lane_id(identifier7);

                                    //LogicalLane.successor_lane.at_begin_of_other_lane
                                    successorLane->set_at_begin_of_other_lane(true);
                                }
                            }
                            else if (std::next(it) == laneSectionMap.end())//最后一个元素 只有前继车道
                            {
                                int predecesId = itar.second->getPredecessor();

                                if (predecesId != Lane::NOLANE)
                                {
                                    //LogicalLane.predecessor_lane.id
                                    osi3::LogicalLane_LaneConnection *predecessorLane = logicalLane->add_predecessor_lane();
                                    osi3::Identifier *identifier8 = new osi3::Identifier(); 
                                    identifier8->set_value((predecesId + roadId) * 1000 + i - 1);//(((roadId +1) * 1000) + laneId) * 1000 + laneSection的序号
                                    predecessorLane->set_allocated_other_lane_id(identifier8);

                                    //LogicalLane.predecessor_lane.at_begin_of_other_lane
                                    predecessorLane->set_at_begin_of_other_lane(false);
                                }                   
                            }   
                            else
                            {
                                int succesId = itar.second->getSuccessor(), predecesId = itar.second->getPredecessor();
                                
                                if (succesId != Lane::NOLANE)
                                {
                                    //LogicalLane.successor_lane.id
                                    osi3::LogicalLane_LaneConnection *successorLane = logicalLane->add_successor_lane();
                                    osi3::Identifier *identifier9 = new osi3::Identifier(); 
                                    identifier9->set_value((succesId + roadId) * 1000 + i + 1);//(((roadId +1) * 1000) + laneId) * 1000 + laneSection的序号
                                    successorLane->set_allocated_other_lane_id(identifier9);

                                    //LogicalLane.successor_lane.at_begin_of_other_lane
                                    successorLane->set_at_begin_of_other_lane(true);
                                }

                                if (predecesId != Lane::NOLANE)
                                {
                                    //LogicalLane.predecessor_lane.id
                                    osi3::LogicalLane_LaneConnection *predecessorLane = logicalLane->add_predecessor_lane();
                                    osi3::Identifier *identifier10 = new osi3::Identifier(); 
                                    identifier10->set_value((predecesId + roadId) * 1000 + i - 1);//(((roadId +1) * 1000) + laneId) * 1000 + laneSection的序号
                                    predecessorLane->set_allocated_other_lane_id(identifier10);

                                    //LogicalLane.predecessor_lane.at_begin_of_other_lane
                                    predecessorLane->set_at_begin_of_other_lane(false);
                                }
                            }
                        }
                    }
                }

                i++;
            }
        }
    }
}

//ReferenceLine数据填充
void OSIGroundTruthGeneratorThread::fillReferenceLine(osi3::GroundTruth &_groundTruth)
{
    for (auto it : roadMap)
    {
        Road *road = it.first;

        if (road != nullptr)
        {
            double startS = 0.0, endS = 0.0;
            filterDataForEgo(road, std::get<1>(it), startS, endS);//筛选出道路坐标点范围

            osi3::ReferenceLine *referenceLine = _groundTruth.add_reference_line();

            //ReferenceLine.id
            osi3::Identifier *identifier = new osi3::Identifier();
            identifier->set_value(stoi(road->getId()));
            referenceLine->set_allocated_id(identifier);//roadid

            //ReferenceLine.poly_line
            const std::map<double, std::tuple<double, Vector3D>> &points = road->getReferenceLinePoints();
            auto startIt = points.lower_bound(startS);
            auto endIt = points.upper_bound(endS);

            if ((startIt != points.begin()) && (startIt != points.end()))
            {
                startIt--;
            }

            for (auto point = startIt; point != endIt; point++)
            {
                osi3::ReferenceLine_ReferenceLinePoint *referenceLinePoint = referenceLine->add_poly_line();

                //ReferenceLine.poly_line.wordld_position
                osi3::Vector3d *vector3d = new osi3::Vector3d();
                vector3d->set_x(std::get<1>(point->second)[0]);
                vector3d->set_y(std::get<1>(point->second)[1]);
                vector3d->set_z(std::get<1>(point->second)[2]);
                referenceLinePoint->set_allocated_world_position(vector3d);

                //ReferenceLine.poly_line.s_position
                referenceLinePoint->set_s_position(point->first);
            }

            if (endIt != points.end())
            {
                osi3::ReferenceLine_ReferenceLinePoint *referenceLinePoint = referenceLine->add_poly_line();

                //ReferenceLine.poly_line.wordld_position
                osi3::Vector3d *vector3d = new osi3::Vector3d();
                vector3d->set_x(std::get<1>(endIt->second)[0]);
                vector3d->set_y(std::get<1>(endIt->second)[1]);
                vector3d->set_z(std::get<1>(endIt->second)[2]);
                referenceLinePoint->set_allocated_world_position(vector3d);

                //ReferenceLine.poly_line.s_position
                referenceLinePoint->set_s_position(endIt->first);
            }
        }
    }
}

//LogicalLaneBoundary数据填充
void OSIGroundTruthGeneratorThread::fillLogicalLaneBoundary(osi3::GroundTruth &_groundTruth)
{
    for (auto it : roadMap)
    {
        Road *road = it.first;

        if (road != nullptr)
        {
            double startS = 0.0, endS = 0.0;
            filterDataForEgo(road, std::get<1>(it), startS, endS);//筛选出道路坐标点范围

            std::map<double, LaneSection *> laneSectionMap = road->getLaneSectionMap();
            unsigned long long roadId = stoul(road->getId());
            roadId = (roadId + 1) * 1000;
            int i = 1;//LaneSection序号

            for (auto it : laneSectionMap)
            {
                if (it.second != nullptr)
                {
                    std::map<int, Lane *> laneMap = it.second->getLaneMap();

                    for (auto itar : laneMap)
                    {
                        if (itar.second != nullptr)
                        {
                            const std::map<double, std::tuple<double, Vector3D>> &logicalLaneBoundaryPointsMap = itar.second->getLogicalLaneBoundaryPointsMap();

                            if (!logicalLaneBoundaryPointsMap.empty())
                            {
                                auto firstIt = logicalLaneBoundaryPointsMap.begin();
                                auto lastIt = std::prev(logicalLaneBoundaryPointsMap.end());

                                if ((firstIt != logicalLaneBoundaryPointsMap.end()) && (lastIt != logicalLaneBoundaryPointsMap.end()))
                                {
                                    if (((firstIt->first < startS) && (lastIt->first < startS)) || ((firstIt->first > endS) && (lastIt->first > endS)))
                                    {
                                        continue;
                                    }
                                }
                            }
                            else
                            {
                                continue;
                            }

                            auto startIt = logicalLaneBoundaryPointsMap.lower_bound(startS);
                            auto endIt = logicalLaneBoundaryPointsMap.upper_bound(endS);

                            if ((startIt != logicalLaneBoundaryPointsMap.begin()) && (startIt != logicalLaneBoundaryPointsMap.end()))
                            {
                                startIt--;
                            }

                            osi3::LogicalLaneBoundary *laneBoundary = _groundTruth.add_logical_lane_boundary();

                            //LogicalLaneBoundary.id
                            osi3::Identifier *identifier = new osi3::Identifier();
                            identifier->set_value((roadId + itar.first) * 1000 + i);//(((roadId +1) * 1000) + laneId) * 1000 + laneSection的序号
                            laneBoundary->set_allocated_id(identifier);

                            //LogicalLaneBoundary.boundary_line
                            for (auto point = startIt; point != endIt; point++)
                            {
                                osi3::LogicalLaneBoundary_LogicalBoundaryPoint *boundary_line = laneBoundary->add_boundary_line();

                                //LogicalLaneBoundary.boundary_line.position
                                osi3::Vector3d *vector3d = new osi3::Vector3d();
                                vector3d->set_x(std::get<1>(point->second)[0]);
                                vector3d->set_y(std::get<1>(point->second)[1]);
                                vector3d->set_z(std::get<1>(point->second)[2]);

                                boundary_line->set_allocated_position(vector3d);

                                //LogicalLaneBoundary.boundary_line.s_position
                                boundary_line->set_s_position(point->first);

                                //LogicalLaneBoundary.boundary_line.t_position
                                boundary_line->set_t_position(std::get<0>(point->second));
                            }

                            if (endIt != logicalLaneBoundaryPointsMap.end())
                            {
                                osi3::LogicalLaneBoundary_LogicalBoundaryPoint *boundary_line = laneBoundary->add_boundary_line();

                                //LogicalLaneBoundary.boundary_line.position
                                osi3::Vector3d *vector3d = new osi3::Vector3d();
                                vector3d->set_x(std::get<1>(endIt->second)[0]);
                                vector3d->set_y(std::get<1>(endIt->second)[1]);
                                vector3d->set_z(std::get<1>(endIt->second)[2]);

                                boundary_line->set_allocated_position(vector3d);

                                //LogicalLaneBoundary.boundary_line.s_position
                                boundary_line->set_s_position(endIt->first);

                                //LogicalLaneBoundary.boundary_line.t_position
                                boundary_line->set_t_position(std::get<0>(endIt->second));
                            }

                            //LogicalLaneBoundary.reference_line_id
                            osi3::Identifier *identifier2 = new osi3::Identifier();
                            identifier2->set_value(stoi(road->getId()));
                            laneBoundary->set_allocated_reference_line_id(identifier2);//roadId

                            //LogicalLaneBoundary.physical_boundary_id
                            osi3::Identifier *identifier3 = laneBoundary->add_physical_boundary_id();
                            identifier3->set_value(roadId + itar.first);//(roadId +1) * 1000 + laneId
                        }
                    }
                }

                i++;
            }
        }
    }
}

//筛选道路
void OSIGroundTruthGeneratorThread::filterRoad()
{
    if (egoState == nullptr)
    {
        return;
    }

    if (egoState->sObjectState.u1Dir == D_SP_ROAD_DIR_FLAG_DIR_FWD)
    {
        direction = 1;
    }
    else if (egoState->sObjectState.u1Dir == D_SP_ROAD_DIR_FLAG_DIR_REAR)
    {
        direction = -1;
    }
    else
    {
        //do nothing
    }

    roadMap.clear();
    roadLaneSectionMap.clear();
    Road *road = RoadSystem::Instance()->getRoad(std::to_string(egoState->sObjectState.u8RoadId));

    if (road != nullptr)
    {
        std::list<Road *> needFilterRoadList;//记录需要筛选的路
        std::set<Road *> usedRoadSet;//记录已经筛选过的路
        double distance = pow(egoRange, 2);//筛选范围为120米
        roadMap[road] = direction;
        needFilterRoadList.push_back(road);
        roadLaneSectionMap[road] = road->getLaneSection(egoState->sObjectState.u4RoadS);

        while (!needFilterRoadList.empty())
        {
            Road *_road = needFilterRoadList.front();

            if (_road != nullptr)
            {
                needFilterRoadList.pop_front(); //删除第一个元素
                usedRoadSet.insert(_road);

                Transform transform = _road->getRoadTransform(_road->getLength(), 0.0);
                
                if ((pow(transform.v().x() - egoState->sObjectState.sPos.u8X, 2) + pow(transform.v().y() - egoState->sObjectState.sPos.u8Y, 2) + pow(transform.v().z() - egoState->sObjectState.sPos.u8Z, 2)) < distance)//满足范围内
                {
                    std::map<Road *, int> nextRoadMap = getNextRoadMap(_road, 1);

                    for (auto it : nextRoadMap)
                    {
                        Road *nextRoad = it.first;

                        if (nextRoad != nullptr)
                        {
                            if (usedRoadSet.find(nextRoad) == usedRoadSet.end())
                            {
                                if (roadMap.find(nextRoad) == roadMap.end())
                                {
                                    roadMap[nextRoad] = it.second;
                                    needFilterRoadList.insert(needFilterRoadList.begin(), nextRoad);

                                    if (it.second > 0)
                                    {
                                        roadLaneSectionMap[nextRoad] = nextRoad->getLaneSection(0.0);
                                    }
                                    else
                                    {
                                        roadLaneSectionMap[nextRoad] = nextRoad->getLaneSection(nextRoad->getLength());
                                    }
                                }
                            }
                        }
                    }
                }

                Transform transform2 = _road->getRoadTransform(0.0, 0.0);
                
                if ((pow(transform2.v().x() - egoState->sObjectState.sPos.u8X, 2) + pow(transform2.v().y() - egoState->sObjectState.sPos.u8Y, 2) + pow(transform2.v().z() - egoState->sObjectState.sPos.u8Z, 2)) < distance)//满足范围内
                {
                    std::map<Road *, int> preRoadMap = getNextRoadMap(_road, -1);

                    for (auto it : preRoadMap)
                    {
                        Road *preRoad = it.first;

                        if (preRoad != nullptr)
                        {
                            if (usedRoadSet.find(preRoad) == usedRoadSet.end())
                            {
                                if (roadMap.find(preRoad) == roadMap.end())
                                {
                                    roadMap[preRoad] = it.second;
                                    needFilterRoadList.insert(needFilterRoadList.begin(), preRoad);

                                    if (it.second > 0)
                                    {
                                        roadLaneSectionMap[preRoad] = preRoad->getLaneSection(0.0);
                                    }
                                    else
                                    {
                                        roadLaneSectionMap[preRoad] = preRoad->getLaneSection(preRoad->getLength());
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

//获取沿着direction方向与本条路连接的所有路
std::map<Road *, int> OSIGroundTruthGeneratorThread::getNextRoadMap(Road *road, int direction)
{
    std::map<Road *, int> nextRoadMap; //返回值

    if (road != nullptr) //如果这条路有提供合法的点
    {
        //获取direction方向上与本条路连接的所有路
        RoadTransition roadTransition(road, direction);
        std::list<RoadTransition> roadTransitionList = road->getConnectingRoadTransitionSet(roadTransition);

        //遍历roadTransitionList
        for (const auto &roadTransitionIt : roadTransitionList)
        {
            if ((roadTransitionIt.road != nullptr) && (abs(roadTransitionIt.direction) == 1)) //异常值check
            {
                nextRoadMap[roadTransitionIt.road] =  roadTransitionIt.direction;
            }
        }
    }

    return nextRoadMap;
}

//停车位数据填充
void OSIGroundTruthGeneratorThread::fillParkingSpace(Road *road)
{
    if (egoState == nullptr)
    {
        return;
    }

    if (road != nullptr)
    {
        std::vector<RoadObject *> obstacleVector;
        road->getObstacleVector(obstacleVector);

        //将停车位添加到RoadMarking中
        for (auto it = obstacleVector.begin(); it != obstacleVector.end(); it++)
        {
            RoadObject *roadObject = *it;

            if (roadObject == nullptr)
            {
                continue;
            }

            if (roadObject->getObstacleType() != RoadObject::RoadObjectType::TYPE_PARKINGSPACE)//只保留停车位
            {
                continue;
            }

            double objectHdg = roadObject->getHdg();
            osi3::RoadMarking *roadMarking = groundTruth.add_road_marking();

            //roadmark.Identifier.id
            osi3::Identifier *id = new osi3::Identifier();
            id->set_value(roadObject->getMiscObjectID());
            roadMarking->set_allocated_id(id);

            osi3::BaseStationary *baseStationary = new osi3::BaseStationary();
            roadMarking->set_allocated_base(baseStationary);

            //roadmark.BaseStationary.dimension 停车位的长宽
            osi3::Dimension3d *dimension3d = new osi3::Dimension3d();
            dimension3d->set_length(roadObject->getLength());
            dimension3d->set_width(roadObject->getWidth());
            baseStationary->set_allocated_dimension(dimension3d);

            //roadmark.BaseStationary.position 停车位的中心
            Transform roadObjectTransform = roadObject->getRoadObjectTransform();
            osi3::Vector3d *vector3d = new osi3::Vector3d();
            vector3d->set_x(roadObjectTransform.v().x());
            vector3d->set_y(roadObjectTransform.v().y());
            vector3d->set_z(roadObjectTransform.v().z());
            baseStationary->set_allocated_position(vector3d);

            //roadmark.BaseStationary.orientation 停车位的朝向
            osi3::Orientation3d *orientation3d = new osi3::Orientation3d();
            orientation3d->set_pitch(normalizeRad2(roadObject->getPitchWorld()));
            orientation3d->set_roll(normalizeRad2(roadObject->getRollWorld()));
            orientation3d->set_yaw(normalizeRad2(roadObject->getYawWorld()));
            baseStationary->set_allocated_orientation(orientation3d);

            //roadmark.BaseStationary.base_polygon 停车位的四个角点 左上、左下、右下、右上
            const std::vector<RoadOutline *> &outlinesVtr = roadObject->getOutlinesVtr();

            for (auto it = outlinesVtr.begin(); it != outlinesVtr.end(); it++)
            {
                for (auto cornerRoad = (*it)->std::vector<CornerRoad>::begin(); cornerRoad != (*it)->std::vector<CornerRoad>::end(); cornerRoad++)
                {
                    Transform vecTransform = road->getRoadTransform(cornerRoad->getS(), cornerRoad->getT());//角点道路坐标转世界坐标
                    Vector3D relativeVec = Vector3D(0.0, 0.0, 0.0);
                    roadObjectTransform.relativeCoordinate(roadObject->getYawWorld(), vecTransform.v(), relativeVec);//计算相对坐标

                    osi3::Vector2d *vector2d = baseStationary->add_base_polygon();
                    vector2d->set_x(relativeVec.x());
                    vector2d->set_y(relativeVec.y());
                }
            }

            //roadmark.Classification.type 固定为other
            osi3::RoadMarking_Classification *classification = new osi3::RoadMarking_Classification();
            roadMarking->set_allocated_classification(classification);
            classification->set_type(osi3::RoadMarking_Classification_Type::RoadMarking_Classification_Type_TYPE_OTHER);

            //ExternalReference.source_reference.type 补充信息
            osi3::ExternalReference *source_reference = roadMarking->add_source_reference();
            std::string *type = new std::string("ParkingSpace");
            source_reference->set_allocated_type(type);

            //ExternalReference.source_reference.identifier 补充信息
            std::string *identifier = source_reference->add_identifier();//ParkingSLotType type (UNKNOWN_TYPE, VERTICAL, PARALLEL, ANGLED)
            std::string *identifier2 = source_reference->add_identifier();//is_target_slot(true,false)是否是选择的目标车位 由AD Adapter决定，SimPro输出固定值false
            std::string *identifier3 = source_reference->add_identifier();//stopper_distance 限位器位置，沿车位框相反方向、从中心开始算距离，SimPro输出固定值-1，代表无效值
            *identifier2 = "false";
            *identifier3 = "-1";

            std::string name = roadObject->getName();
            double cosHdg = fabs(cos(objectHdg));
            double sinHdg = fabs(sin(objectHdg));

            if ((name == "Parking_6m") || (name == "Parking_5m"))//矩形
            {
                if (fabs(cosHdg - 1.0) < 0.000001)//与道路平行
                {
                    *identifier = "PARALLEL";
                }
                else//与道路不平行
                {
                    if (fabs(sinHdg - 1.0) < 0.000001)//与道路垂直
                    {
                        *identifier = "VERTICAL";
                    }
                    else
                    {
                        *identifier = "ANGLED";
                    }
                }
            }
            else//非矩形
            {
                if (fabs(cosHdg - 1.0) < 0.000001)//与道路平行
                {
                    *identifier = "ANGLED";
                }
                else//与道路不平行
                {
                    *identifier = "UNKNOWN_TYPE";
                }
            }
        }     
    }
}

//填充Lane和LaneBoundary
void OSIGroundTruthGeneratorThread::fillLaneAndLaneBoundary()
{
    if (egoState == nullptr)
    {
        return;
    }

    Road *egoRoad = RoadSystem::Instance()->getRoad(std::to_string(egoState->sObjectState.u8RoadId));
    int egoLaneId = egoState->sObjectState.u1LaneId;
    bool outputLogicalLane = ConfigureMngr::getInstance()->getIsOutputOsiGroundTruthLogicalLane();

    if (egoRoad != nullptr)
    {
        for (auto it : roadLaneSectionMap)
        {
            Road *road = it.first;

            if (road == nullptr)
            {
                continue;
            }

            if (!outputLogicalLane)//LogicalLane开关关闭
            {
                if (road != egoRoad)
                {
                    continue;
                }
            }

            LaneSection *laneSection = it.second;

            if (laneSection != nullptr)
            {
                std::map<int, Lane *> laneMap = laneSection->getLaneMap();
                unsigned long long roadId = stoul(road->getId());
                roadId = (roadId + 1) * 1000;

                for (auto laneIt : laneMap)
                {
                    osi3::Lane *laneOSI = groundTruth.add_lane();
                    Lane *lane = laneIt.second;

                    if (lane != nullptr)
                    {
                        //Lane.id
                        osi3::Identifier *identifier = new osi3::Identifier();
                        identifier->set_value(roadId + laneIt.first);//根据华为的要求，laneId字段值为(roadId + 1) * 1000 + laneId
                        laneOSI->set_allocated_id(identifier);

                        //Lane.classification
                        osi3::Lane::Classification *classification = new osi3::Lane::Classification();
                        laneOSI->set_allocated_classification(classification);
                        
                        uint16_t type = 0, subType = convertLaneSubtype(lane->getLaneType());
                        if (road->isJunctionPath())//路口内车道
                        {
                            type = osi3::Lane::Classification::Type::Lane_Classification_Type_TYPE_INTERSECTION;;                            //车道类型@D_MSGADAPTER_LANE_TYPE
                        }
                        else
                        {
                            if (lane->isDriving())
                            {
                                type = osi3::Lane::Classification::Type::Lane_Classification_Type_TYPE_DRIVING;
                            }
                            else
                            {
                                type = osi3::Lane::Classification::Type::Lane_Classification_Type_TYPE_NONDRIVING;
                            }
                        }

                        //Lane.classification.type
                        classification->set_type(osi3::Lane::Classification::Type(type));

                        //Lane.classification.subtype
                        classification->set_subtype(osi3::Lane::Classification::Subtype(subType));

                        int leftLaneId = 0;//左车道Id
                        int rightLaneId = 0;//右车道Id
                        int leftLaneBoundaryId = 0;//左侧车道线id
                        int rightLaneBoundaryId = 0;//右侧车道线id

                        if (road == egoRoad)
                        {
                            bool isHostVehicleLane = false;//是否是主车所在车道

                            if (egoLaneId == laneIt.first)
                            {
                                isHostVehicleLane = true;

                                //Lane.classification.centerLine
                                std::list<Vector3D> pointList = getCenterlinePoint(laneIt.first);
                                for (const auto &it : pointList)
                                {
                                    osi3::Vector3d *centerlinePoint = classification->add_centerline();
                                    centerlinePoint->set_x(it.x());
                                    centerlinePoint->set_y(it.y());
                                    centerlinePoint->set_z(it.z());
                                }

                                //Lane.classification.centerline_is_driving_direction
                                if (direction == 1)
                                {
                                    classification->set_centerline_is_driving_direction(true);
                                }
                                else if (direction == -1)
                                {
                                    classification->set_centerline_is_driving_direction(false);
                                }
                                else
                                {
                                    // do nothing;
                                }
                            }
                            
                            //Lane.classification.is_host_vehicle_lane
                            classification->set_is_host_vehicle_lane(isHostVehicleLane);

                            leftLaneId = laneIt.first + direction;
                            rightLaneId = laneIt.first - direction;

                            if (laneIt.first > 0)//正车道
                            {
                                if (direction > 0)
                                {
                                    leftLaneBoundaryId = laneIt.first - 1;
                                    rightLaneBoundaryId = laneIt.first;
                                }
                                else
                                {
                                    leftLaneBoundaryId = laneIt.first;
                                    rightLaneBoundaryId = laneIt.first - 1;
                                }
                            }
                            else if (laneIt.first < 0)//负车道
                            {
                                if (direction > 0)
                                {
                                    leftLaneBoundaryId = laneIt.first + 1;
                                    rightLaneBoundaryId = laneIt.first;
                                }
                                else
                                {
                                    leftLaneBoundaryId = laneIt.first;
                                    rightLaneBoundaryId = laneIt.first + 1;
                                }
                            }
                            else if (laneIt.first == 0)//0车道
                            {
                                leftLaneBoundaryId = 0;
                                rightLaneBoundaryId = 0;
                            }
                        }
                        else
                        {
                            leftLaneId = laneIt.first + 1;
                            rightLaneId = laneIt.first - 1;

                            if (laneIt.first < 0)
                            {
                                rightLaneBoundaryId = laneIt.first;
                                leftLaneBoundaryId = laneIt.first + 1;
                            }
                            else if (laneIt.first > 0)
                            {
                                leftLaneBoundaryId = laneIt.first;
                                rightLaneBoundaryId = laneIt.first - 1;
                            }
                        }

                        //检查左右车道是否存在
                        if (laneMap.find(leftLaneId) == laneMap.end())
                        {
                            leftLaneId = Lane::NOLANE;
                        }

                        if (laneMap.find(rightLaneId) == laneMap.end())
                        {
                            rightLaneId = Lane::NOLANE;
                        }

                        //Lane.classification.left_adjacent_lane_id
                        if (leftLaneId != Lane::NOLANE)
                        {
                            osi3::Identifier *identifier = classification->add_left_adjacent_lane_id();
                            identifier->set_value(roadId + static_cast<int16_t>(leftLaneId));//根据华为的要求，laneId字段值为(roadId + 1) * 1000 + laneId
                        }

                        //Lane.classification.right_adjacent_lane_id
                        if (rightLaneId != Lane::NOLANE)
                        {
                            osi3::Identifier *identifier = classification->add_right_adjacent_lane_id();
                            identifier->set_value(roadId + static_cast<int16_t>(rightLaneId));//根据华为的要求，laneId字段值为(roadId + 1) * 1000 + laneId
                        }

                        std::pair<std::list<unsigned long int>, std::list<unsigned long int>> connectLanePair = getConnectLane(road, laneSection, laneIt.first);

                        //Lane.classification.antecessor_lane_id
                        for (const auto &it : connectLanePair.first)
                        {
                            osi3::Lane_Classification_LanePairing *lanePairing = classification->add_lane_pairing();

                            osi3::Identifier *identifier = new osi3::Identifier();
                            identifier->set_value(it);
                            lanePairing->set_allocated_antecessor_lane_id(identifier);
                        }

                        //Lane.classification.successor_lane_id
                        for(const auto &it : connectLanePair.second)
                        {
                            osi3::Lane_Classification_LanePairing *lanePairing = classification->add_lane_pairing();

                            osi3::Identifier *identifier = new osi3::Identifier();
                            identifier->set_value(it);
                            lanePairing->set_allocated_successor_lane_id(identifier);
                        }

                        //Lane.classification.left_lane_boundary_id
                        osi3::Identifier *leftLaneBoundary = classification->add_left_lane_boundary_id();
                        leftLaneBoundary->set_value(roadId + leftLaneBoundaryId);

                        //Lane.classification.right_lane_boundary_id
                        osi3::Identifier *rightLaneBoundary = classification->add_right_lane_boundary_id();
                        rightLaneBoundary->set_value(roadId + rightLaneBoundaryId);

                        //Lane.source_reference
                        osi3::ExternalReference *externalReference = laneOSI->add_source_reference();
                        externalReference->set_type("length");
                        externalReference->set_reference(std::to_string(road->getLength()));

                        std::map<double, RoadMark *> roadMarkMap = lane->getRoadMarkMap();

                        for (auto roadMarkIt : roadMarkMap)
                        {
                            RoadMark *roadMark = roadMarkIt.second;

                            if (roadMark != nullptr)
                            {
                                osi3::LaneBoundary *laneBoundaryOSI = groundTruth.add_lane_boundary();

                                //LaneBoundary.id
                                osi3::Identifier *identifier = new osi3::Identifier();
                                identifier->set_value(roadId + laneIt.first);//根据华为的要求，laneId字段值为(roadId + 1) * 1000 + laneId
                                laneBoundaryOSI->set_allocated_id(identifier);

                                //LaneBoundary.classification
                                osi3::LaneBoundary::Classification *classification = new osi3::LaneBoundary::Classification();
                                laneBoundaryOSI->set_allocated_classification(classification);

                                uint16_t type2 = 0, color = 0;
                                const RoadMark::RoadMarkType laneBoundaryType = roadMark->getType();
                                const RoadMark::RoadMarkColor roadMarkColor = roadMark->getColor();

                                if(laneBoundaryType == RoadMark::TYPE_NONE)
                                {
                                    type2 = D_MSGADAPTER_LANE_BOUNDARY_TYPE_UNKNOWN;        //未知
                                }
                                else if(laneBoundaryType == RoadMark::TYPE_SOLID)
                                {
                                    type2 = D_MSGADAPTER_LANE_BOUNDARY_TYPE_SOLID_LINE;     //实线
                                }
                                else if(laneBoundaryType == RoadMark::TYPE_BROKEN)
                                {
                                    type2 = D_MSGADAPTER_LANE_BOUNDARY_TYPE_DASHED_LINE;    //虚线
                                }
                                else if(laneBoundaryType == RoadMark::TYPE_SOLIDSOLID)
                                {
                                    type = D_MSGADAPTER_LANE_BOUNDARY_TYPE_SOLID_LINE;     //双实线对应OSI的实线
                                }
                                else if(laneBoundaryType == RoadMark::TYPE_BROKENBROKEN)
                                {
                                    type2 = D_MSGADAPTER_LANE_BOUNDARY_TYPE_DASHED_LINE;    //双虚线对应OSI的虚线
                                }
                                else if(laneBoundaryType == RoadMark::TYPE_GRASS)
                                {
                                    type2 = D_MSGADAPTER_LANE_BOUNDARY_TYPE_GRASS_EDGE;     //草
                                }
                                else if(laneBoundaryType == RoadMark::TYPE_CURB)
                                {
                                    type2 = D_MSGADAPTER_LANE_BOUNDARY_TYPE_CURB ;          //curb
                                }
                                else
                                {
                                    type2 = D_MSGADAPTER_LANE_BOUNDARY_TYPE_OTHER;          //其它
                                }

                                if (roadMarkColor == RoadMark::COLOR_STANDARD)
                                {
                                    color = D_MSGADAPTER_LANE_BOUNDARY_COLOR_WHITE;        //白色
                                }
                                else if (roadMarkColor == RoadMark::COLOR_YELLOW)
                                {
                                    color = D_MSGADAPTER_LANE_BOUNDARY_COLOR_YELLOW;       //黄色
                                }
                                else if (roadMarkColor == RoadMark::COLOR_RED)
                                {
                                    color = D_MSGADAPTER_LANE_BOUNDARY_COLOR_RED;          //红色
                                }
                                else if (roadMarkColor == RoadMark::COLOR_BLUE)
                                {
                                    color = D_MSGADAPTER_LANE_BOUNDARY_COLOR_BLUE;         //蓝色
                                }
                                else if (roadMarkColor == RoadMark::COLOR_GREEN)
                                {
                                    color = D_MSGADAPTER_LANE_BOUNDARY_COLOR_GREEN;        //绿色
                                }
                                else if (roadMarkColor == RoadMark::COLOR_ORANGE)
                                {
                                    color = D_MSGADAPTER_LANE_BOUNDARY_COLOR_ORANGE;       //橙色
                                }
                                else
                                {
                                    // for MISRA
                                    color = D_MSGADAPTER_LANE_BOUNDARY_COLOR_UNKNOWN;
                                }

                                //LaneBoundary.classification.type
                                classification->set_type(osi3::LaneBoundary::Classification::Type(type2));

                                //LaneBoundary.classification.color
                                classification->set_color(osi3::LaneBoundary::Classification::Color(color));

                                if (road == egoRoad)
                                {
                                    //仅提供主车所在车道两侧的车道线
                                    bool isEgoLane = false;
                                    if (laneIt.first == egoLaneId)
                                    {
                                        isEgoLane = true;
                                    }
                                    else if (egoLaneId > 0 && ((egoLaneId - 1) == laneIt.first))//主车在正车道
                                    {
                                        isEgoLane = true;
                                    }
                                    else if (egoLaneId < 0 && ((egoLaneId + 1) == laneIt.first))//主车在负车道
                                    {
                                        isEgoLane = true;
                                    }

                                    if (isEgoLane)
                                    {
                                        //LaneBoundary.BoundaryPoint
                                        std::list<Vector3D> boundaryPointList = getLaneBoundaryPoint(laneIt.first);
                                        for (const auto &it : boundaryPointList)
                                        {
                                            osi3::LaneBoundary_BoundaryPoint *boundaryPoint = laneBoundaryOSI->add_boundary_line();
                                            osi3::Vector3d *point = new osi3::Vector3d();
                                            point->set_x(it.x());
                                            point->set_y(it.y());
                                            point->set_z(it.z());
                                            boundaryPoint->set_allocated_position(point);
                                        }
                                    }
                                }

                                //LaneBoundary.source_reference.openDriveType
                                osi3::ExternalReference *sourceReference = laneBoundaryOSI->add_source_reference();
                                sourceReference->set_type("openDriveType");
                                sourceReference->set_reference(roadMark->getTypeString());
                            }
                        }
                    }
                }
            }

            if (!outputLogicalLane)//LogicalLane开关关闭
            {
                //主车所在道路id
                unsigned long int roadId = egoState->sObjectState.u8RoadId;
                roadId = (roadId + 1) * 1000; //根据华为的要求，laneId字段值为(roadId + 1) * 1000 + laneId
                
                for (std::list<S_SP_LANE_INFO *>::const_iterator it = laneList.begin(); it != laneList.end(); it++)
                {
                    const S_SP_LANE_INFO * const &lane = *it;
                
                    //车道线的roadid等于主车所在道路id，直接continue
                    if (((lane->u8RoadId + 1) * 1000) == roadId)
                    {
                        continue;
                    }

                    osi3::Lane *laneOSI = groundTruth.add_lane();

                    //Lane.id
                    osi3::Identifier *identifier = new osi3::Identifier();
                    identifier->set_value((lane->u8RoadId + 1) * 1000 + lane->u1Id);//根据华为的要求，laneId字段值为(roadId + 1) * 1000 + laneId
                    laneOSI->set_allocated_id(identifier);

                    //Lane.classification
                    osi3::Lane::Classification *classification = new osi3::Lane::Classification();
                    laneOSI->set_allocated_classification(classification);

                    //Lane.classification.type
                    uint16_t typeOfOSI = -1;
                    Road *road = RoadSystem::Instance()->getRoad(std::to_string(lane->u8RoadId));
                    Road *egoRoad = RoadSystem::Instance()->getRoad(std::to_string(egoState->sObjectState.u8RoadId));

                    if (road != nullptr)
                    {
                        Lane *pLane = road->getLaneSection(egoState->sObjectState.u4RoadS)->getLane(lane->u1Id);
                        if (road->isJunctionPath())//路口内车道
                        {
                            typeOfOSI = osi3::Lane::Classification::Type::Lane_Classification_Type_TYPE_INTERSECTION;
                        }
                        else//非路口内车道
                        {
                            if ((pLane != NULL) && (pLane->isDriving()))
                            {
                                typeOfOSI = osi3::Lane::Classification::Type::Lane_Classification_Type_TYPE_DRIVING;
                            }
                            else
                            {
                                typeOfOSI = osi3::Lane::Classification::Type::Lane_Classification_Type_TYPE_NONDRIVING;
                            }
                        }
                    }
                    classification->set_type(osi3::Lane::Classification::Type(typeOfOSI));

                    //Lane.classification.centerline_is_driving_direction
                    if (road != nullptr && egoRoad != nullptr)
                    {
                        //获取当前Road的“所有下一条路”（所有与之连接的路）
                        int nextRoadType = 0; //1非junction，2是junction
                        RoadTransition nextRoadTransition;
                        std::list<std::pair<double, RoadTransition>> junctionRoadTransitionList;
                        egoRoad->getNextRoadTransition(nextRoadType, nextRoadTransition, junctionRoadTransitionList, direction);
                        if (nextRoadType == 1)
                        {
                            if (nextRoadTransition.direction == 1)
                            {
                                classification->set_centerline_is_driving_direction(true);
                            }
                            else if (nextRoadTransition.direction == -1)
                            {
                                classification->set_centerline_is_driving_direction(false);
                            }
                            else
                            {
                                //do nothing
                            }
                        }
                        else if (nextRoadType == 2)
                        {
                            for (auto transition : junctionRoadTransitionList)
                            {
                                if (transition.second.road->getId() == road->getId())
                                {
                                    if (transition.second.direction == 1)
                                    {
                                        classification->set_centerline_is_driving_direction(true);
                                    }
                                    else if (transition.second.direction == -1)
                                    {
                                        classification->set_centerline_is_driving_direction(false);
                                    }
                                    else
                                    {
                                        //do nothing
                                    }

                                    break;
                                }
                            }
                        }
                        else
                        {
                            // do nothing
                        }
                    }

                    //Lane.source_reference
                    if (road != nullptr)
                    {
                        osi3::ExternalReference *externalReference = laneOSI->add_source_reference();
                        externalReference->set_type("length");
                        externalReference->set_reference(std::to_string(road->getLength()));
                    }
                }
            }
        }
    }
}

//转换LogicalLane类型
uint16_t OSIGroundTruthGeneratorThread::converLogicalLaneType(uint16_t type)
{
    uint16_t outputType = osi3::LogicalLane::Type::LogicalLane_Type_TYPE_OTHER;

    if (type == Lane::LaneType::DRIVING)
    {
        outputType = osi3::LogicalLane::Type::LogicalLane_Type_TYPE_NORMAL; 
    }
    else if (type == Lane::LaneType::STOP)
    {
        outputType = osi3::LogicalLane::Type::LogicalLane_Type_TYPE_STOP; 
    }
    else if (type == Lane::LaneType::SHOULDER)
    {
        outputType = osi3::LogicalLane::Type::LogicalLane_Type_TYPE_SHOULDER; 
    }
    else if (type == Lane::LaneType::BIKING)
    {
        outputType = osi3::LogicalLane::Type::LogicalLane_Type_TYPE_BIKING; 
    }
    else if (type == Lane::LaneType::SIDEWALK)
    {
        outputType = osi3::LogicalLane::Type::LogicalLane_Type_TYPE_SIDEWALK; 
    }
    else if (type == Lane::LaneType::BORDER)
    {
        outputType = osi3::LogicalLane::Type::LogicalLane_Type_TYPE_BORDER; 
    }
    else if (type == Lane::LaneType::RESTRICTED)
    {
        outputType = osi3::LogicalLane::Type::LogicalLane_Type_TYPE_RESTRICTED; 
    }
    else if (type == Lane::LaneType::PARKING)
    {
        outputType = osi3::LogicalLane::Type::LogicalLane_Type_TYPE_PARKING; 
    }
    else if (type == Lane::LaneType::MEDIAN)
    {
        outputType = osi3::LogicalLane::Type::LogicalLane_Type_TYPE_MEDIAN; 
    }
    else if (type == Lane::LaneType::TRAM)
    {
        outputType = osi3::LogicalLane::Type::LogicalLane_Type_TYPE_TRAM; 
    }
    else if (type == Lane::LaneType::RAIL)
    {
        outputType = osi3::LogicalLane::Type::LogicalLane_Type_TYPE_RAIL; 
    }
    else if (type == Lane::LaneType::OFFRAMP)
    {
        outputType = osi3::LogicalLane::Type::LogicalLane_Type_TYPE_OFFRAMP; 
    }
    else if (type == Lane::LaneType::ONRAMP)
    {
        outputType = osi3::LogicalLane::Type::LogicalLane_Type_TYPE_ONRAMP; 
    }
    else if (type == Lane::LaneType::CONNECTINGRAMP)
    {
        outputType = osi3::LogicalLane::Type::LogicalLane_Type_TYPE_CONNECTINGRAMP; 
    }
    else if (type == Lane::LaneType::MWYEXIT)
    {
        outputType = osi3::LogicalLane::Type::LogicalLane_Type_TYPE_EXIT; 
    }
    else if (type == Lane::LaneType::MWYENTRY)
    {
        outputType = osi3::LogicalLane::Type::LogicalLane_Type_TYPE_ENTRY; 
    }

    return outputType;
}

int OSIGroundTruthGeneratorThread::stoi(const std::string &input)
{
    int output = 0;
    try
    {
        output = std::stoi(input);
    }
    //catch (const std::exception &e)
    catch (...)
    {
        log_compnt_mngr->error("stoi, catch exception.");
    }
    return output;
}

//比较二点距离是否超过120米
bool OSIGroundTruthGeneratorThread::compareDistance(Vector3D &Pos)
{
    if (egoState != nullptr)
    {
        Vector3D ego3D = Vector3D(egoState->sObjectState.sPos.u8X, egoState->sObjectState.sPos.u8Y, egoState->sObjectState.sPos.u8Z);

        if ((pow(ego3D.x() - Pos.x(), 2) + pow(ego3D.y() - Pos.y(), 2) + pow(ego3D.z() - Pos.z(), 2)) < pow(egoRange, 2))
        {
            return true;
        }
        else
        {
            return false;
        }        
    }
    else
    {
        return false;
    }
}

//计算pos点与主车距离
double OSIGroundTruthGeneratorThread::distanceForEgo(Vector3D &Pos)
{
    double distance = 0.0;

    if (egoState != nullptr)
    {
        Vector3D ego3D = Vector3D(egoState->sObjectState.sPos.u8X, egoState->sObjectState.sPos.u8Y, egoState->sObjectState.sPos.u8Z);

        distance = std::sqrt((pow(ego3D.x() - Pos.x(), 2) + pow(ego3D.y() - Pos.y(), 2) + pow(ego3D.z() - Pos.z(), 2)));        
    }
    else
    {
        // do nothing
    }

    return distance;
}

//筛选出道路需要输入坐标范围
void OSIGroundTruthGeneratorThread::filterDataForEgo(Road *road, int dir, double &startS, double &endS)
{
    if (egoState == nullptr)
    {
        return;
    }

    Road *egoRoad = RoadSystem::Instance()->getRoad(std::to_string(egoState->sObjectState.u8RoadId));

    if (egoRoad != nullptr)
    {
        double roadLength = road->getLength();
        startS = 0.0;
        endS = roadLength;

        if (egoRoad == road)//主车所在道路
        {
            double distance = 150.0;//筛选范围为150米

            if (roadLength >= distance)
            {
                startS = egoState->sObjectState.u4RoadS - distance;
                endS = egoState->sObjectState.u4RoadS + distance;
            }
        }
        else
        {
            //取道路s轴起点、中间点、终点判断是否处于主车120范围内
            bool isInRange = compareDistance(road->getRoadPoint(0.0, 0.0).pos());
            bool isInRange2 = compareDistance(road->getRoadPoint(roadLength / 2.0, 0.0).pos());
            bool isInRange3 = compareDistance(road->getRoadPoint(roadLength, 0.0).pos());

            if (isInRange || isInRange2 || isInRange3)//若有一个点在主车120范围内
            {
                if (dir > 0)
                {
                    if (isInRange && !isInRange2 && !isInRange3)
                    {
                        startS = 0.0;
                        endS = egoRange - distanceForEgo(road->getRoadPoint(0.0, 0.0).pos());
                    }
                    else if (isInRange && isInRange2 && !isInRange3)
                    {
                        startS = 0.0;
                        endS = egoRange - distanceForEgo(road->getRoadPoint(roadLength / 2.0, 0.0).pos()) + roadLength / 2.0;
                    }
                    else if (isInRange && !isInRange2 && isInRange3)
                    {
                        startS = 0.0;
                        endS = egoRange - distanceForEgo(road->getRoadPoint(0.0, 0.0).pos());
                    }
                }
                else
                {
                    if (isInRange && !isInRange2 && isInRange3)
                    {
                        startS = roadLength - (egoRange - distanceForEgo(road->getRoadPoint(roadLength, 0.0).pos()));
                        endS = roadLength;
                    }
                    else if (!isInRange && !isInRange2 && isInRange3)
                    {
                        startS = roadLength - (egoRange - distanceForEgo(road->getRoadPoint(roadLength, 0.0).pos()));
                        endS = roadLength;
                    }
                    else if (!isInRange && isInRange2 && isInRange3)
                    {
                        startS = roadLength - (roadLength / 2.0) - (egoRange - distanceForEgo(road->getRoadPoint(roadLength / 2.0, 0.0).pos()));
                        endS = roadLength;
                    }
                }
            }
        }

        if (endS > roadLength)
        {
            endS = roadLength;
        }

        if (startS < 0.0)
        {
            startS = 0.0;
        }
    }
}