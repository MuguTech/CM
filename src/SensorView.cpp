#include "SensorView.h"
#include "log.h"
#include "google/protobuf/text_format.h"

#include<vector>

void log(std::string key,std::string value)
{
    if(value == "\n")
    {
        log_compnt_mngr->debug("SensorView::log key = {} .",key.c_str());
    }
    else
    {
        log_compnt_mngr->debug("SensorView::log key = {} , value = {}.",key.c_str(), value.c_str());
    }
    
}

SensorView::SensorView()
{
    m_pEgoMsg = (S_SP_MIL_EGO_STATE*)malloc(sizeof(S_SP_MIL_EGO_STATE));
}

SensorView::~SensorView()
{
    if(m_pEgoMsg != NULL)
    {
        free(m_pEgoMsg);
        m_pEgoMsg = NULL;
    }
}

bool SensorView::coverDataFrame(void *input, size_t inlen, void *output, size_t &outlen)
{
    log_compnt_mngr->info("SensorView::coverDataFrame start.");
    osi3::GroundTruth* groundTruth = m_OSISensorView.mutable_global_ground_truth();
    char* pkgBuff = (char*)input;
    if(nullptr == pkgBuff)
    {
        outlen = 0;
        log_compnt_mngr->error("SensorView::coverDataFrame nullptr == pkgBuff");
        return false;
    }

    if(nullptr == groundTruth )
    {
        outlen = 0;
        log_compnt_mngr->error("SensorView::coverDataFrame nullptr == groundTruth");
        return false;
    }

    //if(startPkgHead->u4DataSize < inlen) return false;
    //std::cout << "parsePackage  start" << std::dec << __LINE__ << std::endl;

    //解析数据
    S_SP_MSG_HDR *msgHead    = (S_SP_MSG_HDR *)pkgBuff;                                                     //Msg的头部指针
    //std::cout << "headerSize=" << msgHead->headerSize << " dataSize=" << msgHead->dataSize << std::endl;
    char         *currentPkg = pkgBuff + msgHead->u4HeaderSize;                                             //当前Pkg的头部指针

    //第一帧为D_SP_PKG_ID_START_FRAME
    S_SP_MSG_ENTRY_HDR *startPkgHead = (S_SP_MSG_ENTRY_HDR *)currentPkg;
    if (startPkgHead->u2PkgId != D_SP_PKG_ID_START_FRAME)
    {
        outlen = 0;
        log_compnt_mngr->debug("startPkgHead->u2PkgId:{}", startPkgHead->u2PkgId);
        log_compnt_mngr->error("SensorView::coverDataFrame startPkgHead->u2PkgId != D_SP_PKG_ID_START_FRAME");
        
        return false;
    }

    //std::cout << "parsePackage  " << std::dec << __LINE__ << std::endl;


    log_compnt_mngr->debug("D_SP_PKG_ID_START_FRAME.");
    currentPkg += startPkgHead->u4HeaderSize + startPkgHead->u4DataSize;                                    //指向下一个pkg


    while (1)
    {
        S_SP_MSG_ENTRY_HDR *pkgHead = (S_SP_MSG_ENTRY_HDR *)currentPkg;                                      //Pkg的头部指针
        if (pkgHead->u2PkgId == D_SP_MIL_PKG_ID_EGO_DATA)                                                    //S_SP_MIL_EGO_STATE
        {
            log_compnt_mngr->trace("receive D_SP_MIL_PKG_ID_EGO_DATA");
            S_SP_MIL_EGO_STATE *pkgData = (S_SP_MIL_EGO_STATE *)(currentPkg + pkgHead->u4HeaderSize);        //数据部分指针
            memcpy(m_pEgoMsg,pkgData,sizeof(S_SP_MIL_EGO_STATE));
            
            //1.解析数据
            int elementNum = pkgHead->u4DataSize / pkgHead->u4ElementSize;                                   //元素数量

            for (int i = 0; i < elementNum; i++) 
            {
                osi3::MovingObject *movingObjectOSI = groundTruth->add_moving_object();
                movingObjectOSI->set_type( osi3::MovingObject::Type::MovingObject_Type_TYPE_VEHICLE);
                osi3::Identifier *identifier = movingObjectOSI->mutable_id();
                identifier->set_value(pkgData->sObjectState.u4Id);
                osi3::ExternalReference* source_reference = movingObjectOSI->add_source_reference();
                source_reference->add_identifier(pkgData->sObjectState.au1Name);

                osi3::BaseMoving* base = movingObjectOSI->mutable_base();
                //Ego车(为了获取主车位置方向信息)
                movingObjectOSI->mutable_vehicle_attributes();
                movingObjectOSI->mutable_vehicle_classification();

                //主车所在车道
                osi3::Identifier *lane_id = movingObjectOSI->add_assigned_lane_id();
                lane_id->set_value(pkgData->sObjectState.u1LaneId);

                //Ego车姿态角、Ego车世界坐标
                osi3::Vector3d* position = base->mutable_position();
                position->set_x(pkgData->sObjectState.sPos.u8X);
                position->set_y(pkgData->sObjectState.sPos.u8Y);
                position->set_z(pkgData->sObjectState.sPos.u8Z);
                osi3::Orientation3d* orientation = base->mutable_orientation();
                orientation->set_yaw(pkgData->sObjectState.sPos.u4H);
                orientation->set_roll(pkgData->sObjectState.sPos.u4R);
                orientation->set_pitch(pkgData->sObjectState.sPos.u4P);

                //环境车相对速度
                osi3::Vector3d* volocity = base->mutable_velocity();
                volocity->set_x(pkgData->sObjectState.sSpeed.u8X);
                volocity->set_y(pkgData->sObjectState.sSpeed.u8Y);
                volocity->set_z(pkgData->sObjectState.sSpeed.u8Z);
                osi3::Orientation3d* orientation_rate = base->mutable_orientation_rate();
                orientation_rate->set_yaw(pkgData->sObjectState.sSpeed.u4H);
                orientation_rate->set_roll(pkgData->sObjectState.sSpeed.u4R);
                orientation_rate->set_pitch(pkgData->sObjectState.sSpeed.u4P);

                //相对加速度
                osi3::Vector3d* acceleration = base->mutable_acceleration();
                acceleration->set_x(pkgData->sObjectState.sAccel.u8X);
                acceleration->set_y(pkgData->sObjectState.sAccel.u8Y);
                acceleration->set_z(pkgData->sObjectState.sAccel.u8Z);
                osi3::Orientation3d* orientation_acceleration = base->mutable_orientation_acceleration();
                orientation_acceleration->set_yaw(pkgData->sObjectState.sSpeed.u4H);
                orientation_acceleration->set_roll(pkgData->sObjectState.sSpeed.u4R);
                orientation_acceleration->set_pitch(pkgData->sObjectState.sSpeed.u4P);

                //主车ID 或 名字
                osi3::Identifier *host_vehicle_id = groundTruth->mutable_host_vehicle_id();
                host_vehicle_id->set_value(pkgData->sObjectState.u4Id);

                
                m_OSIhost = movingObjectOSI;
                pkgData += 1;
            }

        }
        else if (pkgHead->u2PkgId == D_SP_MIL_PKG_ID_OBJECT_DATA)                                            //S_SP_MIL_OBJECT_STATE
        {
            log_compnt_mngr->trace("receive D_SP_MIL_PKG_ID_OBJECT_DATA");
            S_SP_MIL_OBJECT_STATE *pkgData = (S_SP_MIL_OBJECT_STATE *)(currentPkg + pkgHead->u4HeaderSize);  //数据部分指针
            int elementNum = pkgHead->u4DataSize / pkgHead->u4ElementSize;
            for (int i = 0; i < elementNum; i++) 
            {
                int type = pkgData->sObjectState.u1Type;
                if(type == D_SP_OBJECT_TYPE_CAR ||
                    type == D_SP_OBJECT_TYPE_PEDESTRIAN
                    )
                {
                    osi3::MovingObject *movingObjectOSI = groundTruth->add_moving_object();

                    //环境车类型
                    if(type == D_SP_OBJECT_TYPE_CAR)
                    {
                        movingObjectOSI->set_type( osi3::MovingObject::Type::MovingObject_Type_TYPE_VEHICLE);
                        movingObjectOSI->mutable_vehicle_attributes();
                        movingObjectOSI->mutable_vehicle_classification();
                    }
                    else if(type == D_SP_OBJECT_TYPE_PEDESTRIAN)
                    {
                        movingObjectOSI->set_type( osi3::MovingObject::Type::MovingObject_Type_TYPE_PEDESTRIAN);
                    }                
                    
                    osi3::Identifier *identifier = movingObjectOSI->mutable_id();
                    identifier->set_value(pkgData->sObjectState.u4Id);
                    
                    osi3::ExternalReference* source_reference = movingObjectOSI->add_source_reference();
                    source_reference->add_identifier(pkgData->sObjectState.au1Name);

                    osi3::BaseMoving* base = movingObjectOSI->mutable_base();
                    //环境车世界坐标
                    osi3::Vector3d* position = base->mutable_position();
                    position->set_x(pkgData->sObjectState.sPos.u8X);
                    position->set_y(pkgData->sObjectState.sPos.u8Y);
                    position->set_z(pkgData->sObjectState.sPos.u8Z);
                    osi3::Orientation3d* orientation = base->mutable_orientation();
                    orientation->set_yaw(pkgData->sObjectState.sPos.u4H);
                    orientation->set_roll(pkgData->sObjectState.sPos.u4R);
                    orientation->set_pitch(pkgData->sObjectState.sPos.u4P);

                    //环境车相对速度
                    osi3::Vector3d* volocity = base->mutable_velocity();
                    volocity->set_x(pkgData->sObjectState.sSpeed.u8X);
                    volocity->set_y(pkgData->sObjectState.sSpeed.u8Y);
                    volocity->set_z(pkgData->sObjectState.sSpeed.u8Z);
                    osi3::Orientation3d* orientation_rate = base->mutable_orientation_rate();
                    orientation_rate->set_yaw(pkgData->sObjectState.sSpeed.u4H);
                    orientation_rate->set_roll(pkgData->sObjectState.sSpeed.u4R);
                    orientation_rate->set_pitch(pkgData->sObjectState.sSpeed.u4P);

                    //环境车相对加速度
                    osi3::Vector3d* acceleration = base->mutable_acceleration();
                    acceleration->set_x(pkgData->sObjectState.sAccel.u8X);
                    acceleration->set_y(pkgData->sObjectState.sAccel.u8Y);
                    acceleration->set_z(pkgData->sObjectState.sAccel.u8Z);
                    osi3::Orientation3d* orientation_acceleration = base->mutable_orientation_acceleration();
                    orientation_acceleration->set_yaw(pkgData->sObjectState.sAccel.u4H);
                    orientation_acceleration->set_roll(pkgData->sObjectState.sAccel.u4R);
                    orientation_acceleration->set_pitch(pkgData->sObjectState.sAccel.u4P);

                    //获取包围盒四个点的世界坐标
                    osi3::Dimension3d* dimension = base->mutable_dimension();
                    dimension->set_height(pkgData->sObjectState.sGeo.u4DimZ);
                    dimension->set_length(pkgData->sObjectState.sGeo.u4DimY);
                    dimension->set_width(pkgData->sObjectState.sGeo.u4DimX);

                    //车辆中心点
                    osi3::MovingObject_VehicleAttributes* vehicle_attributes = movingObjectOSI->mutable_vehicle_attributes();
                    osi3::Vector3d* bbcenter_to_front  = vehicle_attributes->mutable_bbcenter_to_front();
                    bbcenter_to_front->set_x(pkgData->sObjectState.sGeo.u4OffX);
                    bbcenter_to_front->set_y(pkgData->sObjectState.sGeo.u4OffY);
                    bbcenter_to_front->set_z(pkgData->sObjectState.sGeo.u4OffZ);

                }
                else if(((type == D_SP_OBJECT_TYPE_BARRIER) || (type == D_SP_OBJECT_TYPE_BUILDING) || (type == D_SP_OBJECT_TYPE_TREE) || (type == D_SP_OBJECT_TYPE_SHRUB) ||
                (type == D_SP_OBJECT_TYPE_GRASS) || (type == D_SP_OBJECT_TYPE_GARBAGE_CAN) || (type == D_SP_OBJECT_TYPE_POLE) || (type == D_SP_OBJECT_TYPE_PARKING_SPACE) || 
                (type == D_SP_OBJECT_TYPE_BRIDGE) || (type == D_SP_OBJECT_TYPE_TUNNEL)))
                {
                    osi3::StationaryObject *stationary_object = groundTruth->add_stationary_object();
                    osi3::StationaryObject_Classification* classification = stationary_object->mutable_classification();
                    if(type == D_SP_OBJECT_TYPE_BARRIER)
                    {
                        classification->set_type(osi3::StationaryObject_Classification::TYPE_BARRIER);
                    }
                    else if(type == D_SP_OBJECT_TYPE_BUILDING)
                    {
                        classification->set_type(osi3::StationaryObject_Classification::TYPE_BUILDING);
                    }
                    else if(type == D_SP_OBJECT_TYPE_TREE)
                    {
                        classification->set_type(osi3::StationaryObject_Classification::TYPE_TREE );
                    }
                    else if(type == D_SP_OBJECT_TYPE_POLE)
                    {
                        classification->set_type(osi3::StationaryObject_Classification::TYPE_POLE  );
                    }
                    else if(type == D_SP_OBJECT_TYPE_BRIDGE)
                    {
                        classification->set_type(osi3::StationaryObject_Classification::TYPE_BRIDGE  );
                    }
                    else if(type == D_SP_OBJECT_TYPE_TUNNEL)
                    {
                        classification->set_type(osi3::StationaryObject_Classification::TYPE_UNKNOWN );
                    }


                    osi3::Identifier *identifier = stationary_object->mutable_id();
                    identifier->set_value(pkgData->sObjectState.u4Id);
                    
                    osi3::ExternalReference* source_reference = stationary_object->add_source_reference();
                    source_reference->add_identifier(pkgData->sObjectState.au1Name);

                    osi3::BaseStationary* base = stationary_object->mutable_base();
                    //环境车世界坐标
                    osi3::Vector3d* position = base->mutable_position();
                    position->set_x(pkgData->sObjectState.sPos.u8X);
                    position->set_y(pkgData->sObjectState.sPos.u8Y);
                    position->set_z(pkgData->sObjectState.sPos.u8Z);
                    osi3::Orientation3d* orientation = base->mutable_orientation();
                    orientation->set_yaw(pkgData->sObjectState.sPos.u4H);
                    orientation->set_roll(pkgData->sObjectState.sPos.u4R);
                    orientation->set_pitch(pkgData->sObjectState.sPos.u4P);

                    //获取包围盒四个点的世界坐标
                    osi3::Dimension3d* dimension = base->mutable_dimension();
                    dimension->set_height(pkgData->sObjectState.sGeo.u4DimZ);
                    dimension->set_length(pkgData->sObjectState.sGeo.u4DimY);
                    dimension->set_width(pkgData->sObjectState.sGeo.u4DimX); 
                }
                pkgData += 1;
            }
  
            
        }
        else if(pkgHead->u2PkgId == D_SP_PKG_ID_TRAFFIC_LIGHT)
        {
            S_SP_TRAFFIC_LIGHT *pkgData = (S_SP_TRAFFIC_LIGHT *)(currentPkg + pkgHead->u4HeaderSize);  //数据部分指针
            int elementNum = pkgHead->u4DataSize / pkgHead->u4ElementSize;
            for (int i = 0; i < elementNum; i++)
            {
                osi3::TrafficLight *traffic_light = groundTruth->add_traffic_light();
                osi3::Identifier *identifier = traffic_light->mutable_id();
                identifier->set_value(pkgData->u4Id);

            #if OPEN_ROADSYSTEM
                std::string signalId = std::to_string(pkgData->u4Id);
                auto p = RoadSystem::Instance()->getSignal(signalId);
                if (nullptr != p)
                {
                    Transform transform = p->getTransform();
                    osi3::BaseStationary* base = traffic_light->mutable_base();
                    osi3::Vector3d* position = base->mutable_position();
                    position->set_x(transform.v().x());
                    position->set_y(transform.v().y());
                    position->set_z(transform.v().z());
                }
            #endif
                // osi3::Orientation3d* orientation = base->mutable_orientation();
                // orientation->set_yaw(pkgData->sObjectState.sPos.u4H);
                // orientation->set_roll(pkgData->sObjectState.sPos.u4R);
                // orientation->set_pitch(pkgData->sObjectState.sPos.u4P);
                pkgData++;
            }

        }
        else if(pkgHead->u2PkgId == D_SP_PKG_ID_TRAFFIC_SIGN)
        {
            S_SP_TRAFFIC_SIGN *pkgData = (S_SP_TRAFFIC_SIGN *)(currentPkg + pkgHead->u4HeaderSize);  //数据部分指针
            int elementNum = pkgHead->u4DataSize / pkgHead->u4ElementSize;
            for (int i = 0; i < elementNum; i++)
            {
                osi3::TrafficSign *traffic_sign = groundTruth->add_traffic_sign();
                osi3::Identifier *identifier = traffic_sign->mutable_id();
                identifier->set_value(pkgData->u4TrafficSignId);

                osi3::TrafficSign_MainSign* main_sign = traffic_sign->mutable_main_sign();
                osi3::BaseStationary* base = main_sign->mutable_base();
                osi3::Vector3d* position = base->mutable_position();
                position->set_x(pkgData->sPos.u8X);
                position->set_y(pkgData->sPos.u8Y);
                position->set_z(pkgData->sPos.u8Z);
                osi3::Orientation3d* orientation = base->mutable_orientation();
                orientation->set_yaw(pkgData->sPos.u4H);
                orientation->set_roll(pkgData->sPos.u4R);
                orientation->set_pitch(pkgData->sPos.u4P);

                pkgData++;
            }

        }
        else if(pkgHead->u2PkgId == D_SP_MIL_PKG_ID_ROADMARK)
        {

        }
        else if(pkgHead->u2PkgId == D_SP_PKG_ID_LANE_INFO)
        {
            S_SP_LANE_INFO *pkgData = (S_SP_LANE_INFO *)(currentPkg + pkgHead->u4HeaderSize);  //数据部分指针
            int elementNum = pkgHead->u4DataSize / pkgHead->u4ElementSize;
            for (int i = 0; i < elementNum; i++) 
            {
                osi3::Lane* lane = groundTruth->add_lane();
                osi3::Identifier *identifier = lane->mutable_id();
                identifier->set_value(pkgData->u1Id);

     
                osi3::Lane_Classification* classification =  lane->mutable_classification();
                int u1LeftLaneId = pkgData->u1LeftLaneId != 127 ? 0:pkgData->u1LeftLaneId;
                int u1RightLaneId = pkgData->u1RightLaneId != 127 ? 0:pkgData->u1RightLaneId;
                osi3::Identifier *left_identifier = classification->add_left_lane_boundary_id();
                left_identifier->set_value(u1LeftLaneId);
                osi3::Identifier *right_identifier = classification->add_right_lane_boundary_id();
                right_identifier->set_value(u1RightLaneId);

                if(m_pEgoMsg && m_pEgoMsg->sObjectState.u1LaneId == pkgData->u1Id)//主车道
                {
                    classification->set_is_host_vehicle_lane(true);
                }   

                osi3::ExternalReference* source_reference = lane->add_source_reference();
                   

                pkgData++;
            }

            
            osi3::LaneBoundary* lane_boundary = groundTruth->add_lane_boundary();
            osi3::Identifier *identifier = lane_boundary->mutable_id();
            identifier->set_value(pkgData->u1Id);
            osi3::LaneBoundary_BoundaryPoint* boundary_line = lane_boundary->add_boundary_line();
            
        }
        else if (pkgHead->u2PkgId == D_SP_PKG_ID_END_FRAME)                                                  //如果是最后一个Pkg
        {
            log_compnt_mngr->debug("D_SP_PKG_ID_END_FRAME.");
            break;
        }
        currentPkg += pkgHead->u4HeaderSize + pkgHead->u4DataSize;                                           //指向下一个pkg
    }
#if OPEN_ROADSYSTEM
    afterParseMessage(groundTruth);
#endif

    uint32_t totalSize = m_OSISensorView.ByteSizeLong();
    if(totalSize > outlen)
    {
        log_compnt_mngr->debug("osi sensor view totalSize:{},outlen:{}",totalSize,outlen);
        outlen = 0;
        log_compnt_mngr->debug("cover err");
    }
    else
    {
        if(m_OSISensorView.SerializePartialToArray(output,totalSize))
        {
            outlen = totalSize;
        }
        else
        {
            outlen = 0;
        }
        //m_OSISensorView.ParsePartialFromArray(output,totalSize);
       
        log_compnt_mngr->debug("osi sensor view totalSize:{} ", outlen);
    }

    std::string strlog;
    google::protobuf::TextFormat::PrintToString(m_OSISensorView, &strlog);
    log_compnt_mngr->debug("SensorView::m_OSISensorView:{}",strlog.c_str());
    log_compnt_mngr->info("SensorView::coverDataFrame end.");
    return true;
}

void SensorView::printLog()
{
    log_compnt_mngr->info("SensorView::printLog start.");
    if(!m_OSISensorView.has_global_ground_truth())
    {
        log_compnt_mngr->error("SensorView::m_OSISensorView.global_ground_truth is false.");
        return;
    }

    const osi3::GroundTruth groundTruth = m_OSISensorView.global_ground_truth();
    
    osi3::Identifier host_vehicle_id;
    if(groundTruth.has_host_vehicle_id())
    {
        host_vehicle_id = groundTruth.host_vehicle_id();
        log("host_vehicle_id",std::to_string(host_vehicle_id.value()));
    }

    int moving_object_size = groundTruth.moving_object_size();
    for(int i = 0;i < moving_object_size; i++)
    {
        log("===" + std::to_string(i) + "===","\n");
        osi3::MovingObject movingObjectOSI = groundTruth.moving_object(i);
        if(movingObjectOSI.type() == osi3::MovingObject::Type::MovingObject_Type_TYPE_VEHICLE)
        {
            if(movingObjectOSI.id().value() == host_vehicle_id.value())
            {
             
            }
        }
        else if(movingObjectOSI.type() == osi3::MovingObject::Type::MovingObject_Type_TYPE_PEDESTRIAN)
        {

        }
        else if( movingObjectOSI.type() == osi3::MovingObject::Type::MovingObject_Type_TYPE_ANIMAL)
        {

        }

       
       if(movingObjectOSI.has_id())
       {
            uint id = movingObjectOSI.id().value();
            log("id",std::to_string(id));
       }


        if(!movingObjectOSI.has_base())
        {
            log("not own base","\n");
            continue;
        }

        osi3::BaseMoving base = movingObjectOSI.base();


        if(base.has_velocity())
        {
            osi3::Vector3d vol = base.velocity();
            double velocity_x = vol.x();
            double velocity_y = vol.y();
            double velocity_z = vol.z();
            log("velocity_x",std::to_string(velocity_x));
            log("velocity_y",std::to_string(velocity_y));
            log("velocity_z",std::to_string(velocity_z));
        }

  
        if(base.has_acceleration())
        {
            osi3::Vector3d acc = base.acceleration();
            double acc_x = acc.x();
            double acc_y = acc.y();
            double acc_z = acc.z();
            log("acc_x",std::to_string(acc_x));
            log("acc_y",std::to_string(acc_y));
            log("acc_z",std::to_string(acc_z));
        }


        if(base.has_position())
        {
            osi3::Vector3d pos = base.position();
            double pos_x = pos.x();
            double pos_y = pos.y();
            double pos_z = pos.z();
            log("pos_x",std::to_string(pos_x));
            log("pos_y",std::to_string(pos_y));
            log("pos_z",std::to_string(pos_z));
        }

        if(movingObjectOSI.source_reference_size() > 0)
        {
            osi3::ExternalReference source_reference = movingObjectOSI.source_reference(0);
            if(source_reference.identifier_size() > 0)
            {
                std::string name = source_reference.identifier(0);
                log("name",name);
            }

        }

        if(base.has_dimension())
        {
            osi3::Dimension3d dimension = base.dimension();
            double width = dimension.width();
            double height = dimension.height();
            double length = dimension.length();
            log("width",std::to_string(width));
            log("height",std::to_string(height));
            log("length",std::to_string(length));
        }


    }

    log_compnt_mngr->info("SensorView::afterParseMessage end.");
}

#if OPEN_ROADSYSTEM

void SensorView::afterParseMessage(osi3::GroundTruth* groundTruth)
{
    
    log_compnt_mngr->info("SensorView::afterParseMessage start.");
    if(m_pEgoMsg == NULL) 
    {
        log_compnt_mngr->error("SensorView::afterParseMessage m_pEgoMsg is NULL.");
        return;
    }
    // 获取主车当前所在车道ID
    auto currLane = m_pEgoMsg->sObjectState.u1LaneId;
    int vehicleDir = 0; //被添加车辆方向
    if (m_pEgoMsg->sObjectState.u1Dir == D_SP_ROAD_DIR_FLAG_DIR_FWD)
    {
        vehicleDir = 1;
    }
    else if(m_pEgoMsg->sObjectState.u1Dir == D_SP_ROAD_DIR_FLAG_DIR_REAR)
    {
        vehicleDir = -1;
    }

    // 获取主车当前所在道路
    Road *currRoad = RoadSystem::Instance()->getRoad(std::to_string(m_pEgoMsg->sObjectState.u8RoadId));
    if(currRoad == NULL)
    {
        log_compnt_mngr->error("SensorView::afterParseMessage currRoad == NULL");
        return;
    }

	osi3::Orientation3d orientation = m_OSIhost->base().orientation();		// 获取主车的航向角
	osi3::Vector3d position = m_OSIhost->base().position();			// 获取主车在当前道路上的U坐标
    double roadLength = currRoad->getLength();

    // 获取主车在当前道路上的U坐标
    float egoU = m_pEgoMsg->sObjectState.u4RoadS;
    RoadTransition(currRoad, vehicleDir);
	int  egoDir = vehicleDir;	// 主车相对于道路S轴的朝向，1和S轴同向，-1为反向
	Vector3D camLookTo = Vector3D(1,0,0);	// 存储相机朝向的向量

	int camDir = camLookTo.x()>=0 ? 1 : -1;		// 摄像机的朝向，1为和主车车头朝向同向，-1为反向
    int sampleDir = egoDir;
	const double sampleIntv = 5.0;
	int sampleNumMax = 0;
    log_compnt_mngr->debug("LaneBoundary start ");
    //车子前方
    sampleNumMax = 40;
	std::vector<Vector3D> laneLinePoisLFVec,laneLinePoisRFVec;	// 用于存储左右侧的车道线采样点坐标
    getLaneLinePois(sampleIntv, sampleNumMax, currRoad, currLane, egoU, sampleDir, laneLinePoisLFVec, laneLinePoisRFVec);

#if DEBUG_SENSER_VIEW
    log_compnt_mngr->debug("laneLinePoisLFVec size :{}",laneLinePoisLFVec.size());
    log_compnt_mngr->debug("laneLinePoisRFVec size :{}",laneLinePoisRFVec.size());

    std::stringstream ss;
    for(int i = 0;i<laneLinePoisLFVec.size();i++)
    {
        ss <<"x:"<< laneLinePoisLFVec[i].x() <<"\t\t";
        ss <<"y:"<< laneLinePoisLFVec[i].y() <<"\t\t";
        ss <<"z:"<< laneLinePoisLFVec[i].z() <<"\t\t";
    }

    for(int i = 0;i<laneLinePoisRFVec.size();i++)
    {
        ss <<"x:"<< laneLinePoisRFVec[i].x() <<"\t\t";
        ss <<"y:"<< laneLinePoisRFVec[i].y() <<"\t\t";
        ss <<"z:"<< laneLinePoisRFVec[i].z() <<"\t\t";
    }

#endif
    //车子后方
    sampleNumMax = 10;
    std::vector<Vector3D> laneLinePoisLTVec,laneLinePoisRTVec;	// 用于存储左右侧的车道线采样点坐标
    getLaneLinePois(sampleIntv, sampleNumMax, currRoad, currLane, egoU, -sampleDir, laneLinePoisLTVec, laneLinePoisRTVec);

#if DEBUG_SENSER_VIEW
    log_compnt_mngr->debug("laneLinePoisLTVec size :{}",laneLinePoisLTVec.size());
    log_compnt_mngr->debug("laneLinePoisRTVec size :{}",laneLinePoisRTVec.size());

    for(int i = 0;i<laneLinePoisLTVec.size();i++)
    {
        ss <<"x:"<< laneLinePoisLTVec[i].x() <<"\t\t";
        ss <<"y:"<< laneLinePoisLTVec[i].y() <<"\t\t";
        ss <<"z:"<< laneLinePoisLTVec[i].z() <<"\t\t";
    }

    for(int i = 0;i<laneLinePoisRTVec.size();i++)
    {
        ss <<"x:"<< laneLinePoisRTVec[i].x() <<"\t\t";
        ss <<"y:"<< laneLinePoisRTVec[i].y() <<"\t\t";
        ss <<"z:"<< laneLinePoisRTVec[i].z() <<"\t\t";
    }

    log_compnt_mngr->debug("LaneBoundary:{}",ss.str().c_str());
#endif

    for(int i = laneLinePoisLTVec.size() - 1; i >= 0; i--)
    {
        laneLinePoisLFVec.insert(laneLinePoisLFVec.begin(),laneLinePoisLTVec[i]);
    }

    for(int i = laneLinePoisRTVec.size() - 1; i >= 0; i--)
    {
        laneLinePoisRFVec.insert(laneLinePoisRFVec.begin(),laneLinePoisRTVec[i]);
    }

    osi3::LaneBoundary* lane_boundary = groundTruth->add_lane_boundary();
    osi3::Identifier *identifier = lane_boundary->mutable_id();
    identifier->set_value(m_pEgoMsg->sObjectState.u1LaneId);

    log_compnt_mngr->debug("afterParseMessage sObjectState.u1LaneId:{}",m_pEgoMsg->sObjectState.u1LaneId);

    for(int i = 0;i<laneLinePoisLFVec.size();i++)
    {
        osi3::LaneBoundary_BoundaryPoint* boundary_line = lane_boundary->add_boundary_line();
        osi3::Vector3d* position = boundary_line->mutable_position();
        position->set_x(laneLinePoisLFVec[i].x());
        position->set_y(laneLinePoisLFVec[i].y());
        position->set_z(laneLinePoisLFVec[i].z());
        boundary_line->set_dash(osi3::LaneBoundary_BoundaryPoint::DASH_START);
    }


    for(int i = 0;i<laneLinePoisRFVec.size();i++)
    {
        osi3::LaneBoundary_BoundaryPoint* boundary_line = lane_boundary->add_boundary_line();
        osi3::Vector3d* position = boundary_line->mutable_position();
        position->set_x(laneLinePoisRFVec[i].x());
        position->set_y(laneLinePoisRFVec[i].y());
        position->set_z(laneLinePoisRFVec[i].z());

        boundary_line->set_dash(osi3::LaneBoundary_BoundaryPoint::DASH_END);
    }

    log_compnt_mngr->debug("LaneBoundary end ");

    int host_roadId = m_pEgoMsg->sObjectState.u8RoadId;
    int host_landId = m_pEgoMsg->sObjectState.u1LaneId;

    LaneSection *laneSection =  currRoad->getLaneSection(egoU);
    std::map<int, Lane *> laneMap = laneSection->getLaneMap();
    for (std::map<int, Lane *>::iterator laneMapIt = laneMap.begin(); laneMapIt != laneMap.end(); laneMapIt++)
    {
        int curLandId = laneMapIt->first;
        osi3::Lane* osi_lane = groundTruth->add_lane();
        osi3::Identifier *identifier = osi_lane->mutable_id();
        uint64_t laneId = (host_roadId + 1) * 1000 + curLandId;
        log_compnt_mngr->debug("\tlaneMapIt laneId:{}",laneId);
        identifier->set_value(laneId);
        osi3::Lane_Classification* classification =  osi_lane->mutable_classification();
        if(curLandId == host_landId)
        {
            classification->set_is_host_vehicle_lane(true);
            if(m_pEgoMsg->sObjectState.u1Dir == D_SP_ROAD_DIR_FLAG_DIR_FWD)
            {
                classification->set_centerline_is_driving_direction(true);
            }
            else if(m_pEgoMsg->sObjectState.u1Dir == D_SP_ROAD_DIR_FLAG_DIR_REAR)
            {
                classification->set_centerline_is_driving_direction(false);
            }

            if(D_SP_OBJECT_TYPE_NONE == m_pEgoMsg->sObjectState.u1Type)
            {
                classification->set_type(osi3::Lane_Classification_Type_TYPE_UNKNOWN );
                classification->set_subtype(osi3::Lane_Classification_Subtype_SUBTYPE_UNKNOWN);
            }
            else if(D_SP_OBJECT_TYPE_CAR == m_pEgoMsg->sObjectState.u1Type)
            {
                classification->set_type(osi3::Lane_Classification_Type_TYPE_DRIVING  );
                classification->set_subtype(osi3::Lane_Classification_Subtype_SUBTYPE_NORMAL);
            }
            else if(D_SP_OBJECT_TYPE_PEDESTRIAN == m_pEgoMsg->sObjectState.u1Type)
            {
                classification->set_type(osi3::Lane_Classification_Type_TYPE_NONDRIVING  );
                classification->set_subtype(osi3::Lane_Classification_Subtype_SUBTYPE_SIDEWALK);
            }
            else if(D_SP_OBJECT_TYPE_BARRIER == m_pEgoMsg->sObjectState.u1Type)
            {
                classification->set_type(osi3::Lane_Classification_Type_TYPE_NONDRIVING  );
                classification->set_subtype(osi3::Lane_Classification_Subtype_SUBTYPE_STOP);
            }
            else if(D_SP_OBJECT_TYPE_STREET_LAMP == m_pEgoMsg->sObjectState.u1Type)
            {
                classification->set_type(osi3::Lane_Classification_Type_TYPE_NONDRIVING  );
                classification->set_subtype(osi3::Lane_Classification_Subtype_SUBTYPE_OTHER);
            }
            else if(D_SP_OBJECT_TYPE_TRAFFIC_SIGN == m_pEgoMsg->sObjectState.u1Type)
            {
                classification->set_type(osi3::Lane_Classification_Type_TYPE_NONDRIVING  );
                classification->set_subtype(osi3::Lane_Classification_Subtype_SUBTYPE_OTHER);
            }
            else if(D_SP_OBJECT_TYPE_LANE == m_pEgoMsg->sObjectState.u1Type)
            {
                classification->set_type(osi3::Lane_Classification_Type_TYPE_DRIVING  );
                classification->set_subtype(osi3::Lane_Classification_Subtype_SUBTYPE_NORMAL);
            }
            else if(D_SP_OBJECT_TYPE_PARKING_SPACE == m_pEgoMsg->sObjectState.u1Type)
            {
                classification->set_type(osi3::Lane_Classification_Type_TYPE_NONDRIVING  );
                classification->set_subtype(osi3::Lane_Classification_Subtype_SUBTYPE_PARKING);
            }
            else
            {
                classification->set_type(osi3::Lane_Classification_Type_TYPE_NONDRIVING  );
                classification->set_subtype(osi3::Lane_Classification_Subtype_SUBTYPE_OTHER);
            }

        }

        if(laneMap.find(curLandId - 1) != laneMap.end())
        {
            osi3::Identifier *left_adjacent_lane_id = classification->add_left_adjacent_lane_id();
            uint64_t left_laneId = (host_roadId + 1) * 1000 + (host_landId - 1);
            left_adjacent_lane_id->set_value(left_laneId);
        }

        if(laneMap.find(curLandId + 1) != laneMap.end())
        {
            osi3::Identifier *right_adjacent_lane_id = classification->add_right_adjacent_lane_id();
            uint64_t right_laneId = (host_roadId + 1) * 1000 + (host_landId + 1);
            right_adjacent_lane_id->set_value(right_laneId);
        }


        Lane *lane = laneMapIt->second;
        std::map<double, RoadMark *> roadMarkMap = lane->getRoadMarkMap();
        for (std::map<double, RoadMark *>::iterator roadMarkMapIt = roadMarkMap.begin(); roadMarkMapIt != roadMarkMap.end(); roadMarkMapIt++)
        {
            RoadMark *roadMark = roadMarkMapIt->second;
            double roadMarkStart = roadMark->getStart();
            log_compnt_mngr->debug("roadMark roadMarkStart:",roadMarkStart);
            RoadMark::RoadMarkType const roadMarkType = roadMark->getType();
            RoadMark::RoadMarkColor const roadMarkColor = roadMark->getColor();
            osi3::ColorDescription* color_description =  lane_boundary->mutable_color_description();
            osi3::ColorRGB* rgb = color_description->mutable_rgb();
            if(roadMarkColor ==  RoadMark::RoadMarkColor::COLOR_RED)
            {
                rgb->set_red(1.0);
            }
            else if(roadMarkColor ==  RoadMark::RoadMarkColor::COLOR_GREEN)
            {
                rgb->set_green(1.0);
            }
            else if(roadMarkColor ==  RoadMark::RoadMarkColor::COLOR_BLUE)
            {
                rgb->set_blue(1.0);
            }
            else if(roadMarkColor == RoadMark::RoadMarkColor::COLOR_STANDARD)
            {
                rgb->set_red(1.0);
                rgb->set_green(1.0);
                rgb->set_blue(1.0);
            }

            osi3::LaneBoundary_Classification* mutable_classification = lane_boundary->mutable_classification();
            if(roadMarkType == RoadMark::RoadMarkType::TYPE_NONE)
            {
                mutable_classification->set_type(osi3::LaneBoundary_Classification_Type_TYPE_UNKNOWN );
            }
            else if(roadMarkType == RoadMark::RoadMarkType::TYPE_SOLID)
            {
                mutable_classification->set_type(osi3::LaneBoundary_Classification_Type_TYPE_SOLID_LINE );
            }
            else if(roadMarkType == RoadMark::RoadMarkType::TYPE_BROKEN)
            {
                mutable_classification->set_type(osi3::LaneBoundary_Classification_Type_TYPE_NO_LINE );
            }
            else if(roadMarkType == RoadMark::RoadMarkType::TYPE_SOLIDSOLID)
            {
                mutable_classification->set_type(osi3::LaneBoundary_Classification_Type_TYPE_SOLID_LINE );
            }
            else if(roadMarkType == RoadMark::RoadMarkType::TYPE_SOLIDBROKEN)
            {
                mutable_classification->set_type(osi3::LaneBoundary_Classification_Type_TYPE_DASHED_LINE );
            }
            else if(roadMarkType == RoadMark::RoadMarkType::TYPE_BROKENSOLID)
            {
                mutable_classification->set_type(osi3::LaneBoundary_Classification_Type_TYPE_DASHED_LINE );
            }
            else if(roadMarkType == RoadMark::RoadMarkType::TYPE_BROKENBROKEN)
            {
                mutable_classification->set_type(osi3::LaneBoundary_Classification_Type_TYPE_DASHED_LINE );
            }
            else if(roadMarkType == RoadMark::RoadMarkType::TYPE_BOTTSDOTS)
            {
                mutable_classification->set_type(osi3::LaneBoundary_Classification_Type_TYPE_BOTTS_DOTS );
            }
            else if(roadMarkType == RoadMark::RoadMarkType::TYPE_GRASS)
            {
                mutable_classification->set_type(osi3::LaneBoundary_Classification_Type_TYPE_GRASS_EDGE );
            }
            else if(roadMarkType == RoadMark::RoadMarkType::TYPE_CURB)
            {
                mutable_classification->set_type(osi3::LaneBoundary_Classification_Type_TYPE_CURB );
            }
        }

    }
    //lane width
    //m_pEgoMsg->sObjectState.u4RoadS

    //egoOffset
    //m_pEgoMsg->sObjectState.u4LaneOffset


    log_compnt_mngr->info("SensorView::afterParseMessage end.");
    
}

/**
 * @Date: 2023-04-18 11:34:15
 * @Author: Zale
 * @LastEditors: Zale
 * @decription: 依据制定的采样间隔、采样距离、采样方向，在指定道路上采样指定数量的车道线坐标点
 * @param {double} _dis 采样间隔
 * @param {int} _num 采样点数
 * @param {Road*} _road 采样起始点所在的道路
 * @param {int} _laneId 采样起始点所在的车道id
 * @param {double} _startu 采样起始点的u坐标
 * @param {int} _dir 采样方向，1为S轴正方向，-1为反向
 * @param {vector<Vector3D>} _lPois 左侧车道线点集合
 * @param {vector<Vector3D>} _rPois 右侧车道线点集合
 * @return {int} 最终采样到的车道线数量
 */
int SensorView::getLaneLinePois(double _intv, int _num, Road* _road, int _laneId, double _startu, int _dir, std::vector<Vector3D> &_lPois, std::vector<Vector3D> &_rPois)
{
    log_compnt_mngr->info("SensorView::getLaneLinePois start.");
	int poi = 0;
	auto currRoad = _road;
	auto currLane = _laneId;
	auto sampleDir = _dir;
	double sampleS = _startu;
	double currRoadLen = currRoad->getLength();
	for(; poi < _num; ++poi)
	{
		int currDir = 1;
		TarmacConnection *nextTmp;
		int sampleFlag = 0;	// 储存采样状态，-1为向前越界，1为向后越界，0为正常
		if(sampleS < 0 )	// 采样越界，越过了道路的起点，需要从当前道路的predecessor继续采样
		{
			sampleFlag = -1;
            log_compnt_mngr->debug("SensorView::getLaneLinePois reach begin find predecessor");
			nextTmp = currRoad->getPredecessorConnection();
		}
		else if(sampleS > currRoadLen)	// 采样越界，越过了道路的终点，需要从当前道路的successor继续采样
		{
			sampleFlag = 1;
            log_compnt_mngr->debug("SensorView::getLaneLinePois reach end, find successor");
			nextTmp = currRoad->getSuccessorConnection();
		}
		else	// 正常采样
		{
			log_compnt_mngr->trace("Sensor::getLaneLinePois in the curr road");
			sampleFlag = 0;
		}
		if(sampleFlag != 0)		// 针对采样越界时的处理
		{
			if(nextTmp == nullptr)	// 后继为空，直接break
			{
				log_compnt_mngr->warn("SensorView::getLaneLinePois no nextRoad, break");
				break;
			}
			Road *nextRoadTmp = dynamic_cast<Road *>(nextTmp->getConnectingTarmac());
			Junction *nextJuncTmp = dynamic_cast<Junction *>(nextTmp->getConnectingTarmac());
			int nextSampleLane = 0;		// 下一条采样车道的ID
			if(nextRoadTmp != nullptr)	// 后续采样对象为Road
			{
				// 根据越界方式的不同，决定后续采样方向
				nextSampleLane = sampleFlag > 0 ? currRoad->traceLane(currLane, currRoadLen - 1.0, currRoadLen + 1.0): currRoad->traceLane(currLane, 0, -_intv);
				
				// 判断当前车道和下一条车道的符号是否一致，以此判断两条道路的S轴方向是否一致
				if(nextSampleLane * currLane > 0)	// 同号，一致
				{
					currDir = 1;
					sampleS = sampleFlag > 0 ? (0 + _intv) : (nextRoadTmp->getLength() - _intv);
				}
				else if(nextSampleLane * currLane < 0)			// 异号，不一致
				{
					currDir = -1;
					sampleS = sampleFlag > 0 ? (nextRoadTmp->getLength() - _intv) : (0 + _intv);
				}
				else		// 其中有一条车道的id为0，这是不符合opendrive标准的
				{
					log_compnt_mngr->warn("SensorView::getLaneLinePois lane connect to 0, break");
					break;
				}
				currRoad = nextRoadTmp;
				currLane = nextSampleLane;
			}
			else if (nextJuncTmp != nullptr)		// 后续采样对象为Junction
			{
				PathConnectionSet connSetTmp = nextJuncTmp->getPathConnectionSet(currRoad, currLane);	// 获取后继Junction上所有和当前采样车道具有连接关系的车道
				if(connSetTmp.getFrequencySumMap().size() <1)	// map为空，跳出
				{
					log_compnt_mngr->warn("SensorView::getLaneLinePois connSetTmp map size = 0 break");
					break;
				}
				PathConnection *sampleCon = connSetTmp.getFrequencySumMap().begin()->second;	// 从具备连接关系的车道中选取第一条车道，作为下一个采样车道
				if(sampleCon != nullptr)	// 车道不为空
				{
					log_compnt_mngr->trace("SensorView::getLaneLinePois sampleCon != nullptr");
					// 更新当前道路信息
					nextSampleLane = sampleCon->getConnectingLane(currLane, false);
					log_compnt_mngr->trace("SensorView::getLaneLinePois getConnectingPath");
					currRoad = sampleCon->getConnectingPath();
					log_compnt_mngr->trace("SensorView::getLaneLinePois getLength");
					currRoadLen = currRoad->getLength();
					log_compnt_mngr->trace("SensorView::getLaneLinePois currDur=[1] currRoadLen=[{}], currSampleS=[{}]", currRoadLen, sampleS);
					currLane = nextSampleLane;
					if(nextSampleLane * currLane > 0)	// 同号，一致
					{
						currDir = 1;
						sampleS = sampleFlag > 0 ? (0 + _intv):(currRoadLen - _intv);
						log_compnt_mngr->trace("SensorView::getLaneLinePois currDur=[1] currRoadLen=[{}], currSampleS=[{}]", currRoadLen, sampleS);
					}
					else if(nextSampleLane * currLane < 0)			// 异号，不一致
					{
						currDir = -1;
						sampleS = sampleFlag > 0 ? (currRoadLen - _intv):(0 + _intv);
						log_compnt_mngr->trace("SensorView::getLaneLinePois currDur=[-1] currRoadLen=[{}], currSampleS=[{}]", currRoadLen, sampleS);
					}
					else		// 其中有一条车道的id为0，这是不符合opendrive标准的
					{
						log_compnt_mngr->warn("SensorView::getLaneLinePois laneID = 0, break");
						break;
					}
				}
				else
				{
					log_compnt_mngr->warn("SensorView::getLaneLinePois connSetTmp begin = null break");
					break;
				}
				
			}
			sampleDir = sampleDir * currDir;	// 更新采样方向
			// 如果laneID异常，终止采样
			if((abs(nextSampleLane) > 10000) or (abs(currLane) > 10000)) break;
		}
		else	// 正常采样
		{
			log_compnt_mngr->trace("Sensor::getLaneLinePois currRoad=[{}] currLane=[{}] sampleDir=[{}], sampleS=[{}]",currRoad->getId(), currLane, sampleDir, sampleS);
			RoadPoint poiOut(0,0,0,0,0,0);		// 外侧车道线的坐标信息
			RoadPoint poiIn(0,0,0,0,0,0);		// 内侧车道线的坐标信息
			double disOut = 0;					// 外侧车道线距离道路参考线的距离，仅作为下面函数的参数用
			double disIn  = 0;					// 内侧车道线距离道路参考线的距离，仅作为下面函数的参数用
			// 获取内测与外侧车道线的世界坐标
			currRoad->getLaneRoadPoints(sampleS, currLane, poiIn, poiOut, disIn, disOut);
			_rPois.push_back(Vector3D(poiOut.x(), poiOut.y(), poiOut.z()));	// 存储外侧车道线的采样点坐标
			_lPois.push_back(Vector3D(poiIn.x(), poiIn.y(), poiIn.z()));		// 存储内侧车道线的采样点坐标
			log_compnt_mngr->trace("Sensor::getLaneLinePois poi[{}] PoiL=[{}, {}, {}], PoiR=[{}, {}, {}]", poi, poiIn.x(), poiIn.y(), poiIn.z(), poiOut.x(), poiOut.y(), poiOut.z());
			sampleS += _intv * sampleDir;	// 前往下一个点采样	
		}
		
	}

    log_compnt_mngr->info("SensorView::getLaneLinePois end.");
}
#endif

