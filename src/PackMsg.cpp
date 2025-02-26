#include "PackMsg.h"
#include "../../include/Runtime/coSimu/SimProType.h"
#include "PluginMngr.h"
#include "log.h"
#include "ConfigureMngr.h"

#include "common.h"
#include "FmiDynamicAdapter/VirtualCityFmiDynamicAdapter.h"

PackMsg::PackMsg()
{
}

PackMsg::~PackMsg()
{
}

/**
 * @brief                           把所有pkg打包成msg
 * @param groundTruthMsg            SimPro输出的groundTruthMsg
 * @param groundTruthMsgLen         SimPro输出的groundTruthMsg长度
 * @param inputBufferList           所有新增的的pkg buffer作为输入 <buffer, size>
 * @param output                    输出的buffer
 * @param outlen                    输入时代表输出的buffer大小，输出时代表打包出来的msg大小
 */
void PackMsg::getPkgData(void *groundTruthMsg, size_t groundTruthMsgLen, const std::list<std::tuple<char *, unsigned int>> &inputBufferList, void *output, size_t &outlen)
{
    log_compnt_mngr->info("PackMsg::getPkgData start.");
    //异常检查
    if ((output == nullptr) || (0 > outlen))
    {
        log_compnt_mngr->error("PackMsg::getPkgData error1");
        return;
    }

    std::set<uint32_t> vehiclePedObsIdSet; //SENSOR_DETECTION_INFO中的车、人、障碍物id集合
    std::set<uint32_t> trafficLightIdSet; //SENSOR_DETECTION_INFO中的交通灯id集合
    std::set<uint32_t> trafficSignIdSet; //SENSOR_DETECTION_INFO中的交通标志id集合

    std::list<S_SP_MIL_OBJECT_STATE *> vehiclePedObsList;   //groundTruthMsg中要保留的车、人、障碍物指针
    std::list<char *> trafficLightPkgList;                  //groundTruthMsg中要保留的交通灯pkg指针
    std::list<S_SP_TRAFFIC_SIGN *> trafficSignList;         //groundTruthMsg中要保留的交通标志指针
    unsigned int reserveSize = 0;                           //groundTruthMsg中要保留的size

    if (ConfigureMngr::getInstance()->getIsVirtualCity() && (ConfigureMngr::getInstance()->getCoSimuInterfaceType() == 2))
    {
        // do nothing
    }
    else
    {
        //解析SensorDetectionInfoPkg
        parseSensorDetectionInfoPkg(inputBufferList, vehiclePedObsIdSet, trafficLightIdSet, trafficSignIdSet);

        //解析groundTruthMsg
        parseGroundTruthMsg(groundTruthMsg, groundTruthMsgLen, vehiclePedObsIdSet, trafficLightIdSet, trafficSignIdSet
                            , vehiclePedObsList, trafficLightPkgList, trafficSignList, reserveSize);
    }

    unsigned int inputBufferSizeAll = 0; //输入的buffer总大小
    for (auto it = inputBufferList.begin(); it != inputBufferList.end(); it++)
    {
        if (std::get<0>(*it) == nullptr)
        {
            log_compnt_mngr->error("PackMsg::getPkgData error2");
            return;
        }

        inputBufferSizeAll += std::get<1>(*it);
    }
    inputBufferSizeAll += reserveSize;

    //size check
    if (outlen < (sizeof(S_SP_MSG_ENTRY_HDR) + inputBufferSizeAll))
    {
        log_compnt_mngr->error("PackMsg::getPkgData error3");
        return;
    }

    //清空msgBuffer
    memset(output, 0, outlen);
    int msgBufferUsedSize   = 0;                                                               //对应的已使用空间

    //生成Msg头部
    S_SP_MSG_HDR *msgHead = (S_SP_MSG_HDR *)output;

    //填充Msg头部
    msgHead->u8SimTime    = PluginMngr::get_instance()->get_sim_time();                        //仿真时间
    msgHead->u4HeaderSize = sizeof(S_SP_MSG_HDR);                                              //Msg头部大小
    msgHead->u4FrameNo    = PluginMngr::get_instance()->get_frame_id();                        //帧号
    log_compnt_mngr->debug("getPkgData CM send msg to AD frame no is {}",msgHead->u4FrameNo);
    msgHead->u1Sender    = D_SP_SENDER_COMPONENT_MANAGER;
    msgBufferUsedSize    += sizeof(S_SP_MSG_HDR);                                              //更新msgBuffer的已使用空间

    //生成D_SP_PKG_ID_START_FRAME
    S_SP_MSG_ENTRY_HDR *pkgStart = (S_SP_MSG_ENTRY_HDR *)((char *)output + msgBufferUsedSize);

    //填充PKG头部
    pkgStart->u4HeaderSize       = sizeof(S_SP_MSG_ENTRY_HDR);
    pkgStart->u4DataSize         = 0;
    pkgStart->u4ElementSize      = 0;
    pkgStart->u2PkgId            = D_SP_PKG_ID_START_FRAME;

    msgBufferUsedSize          += pkgStart->u4HeaderSize + pkgStart->u4DataSize;               //更新msgBuffer的已使用空间

    if (ConfigureMngr::getInstance()->getIsVirtualCity() && (ConfigureMngr::getInstance()->getCoSimuInterfaceType() == 2))
    {
        // do nothing
    }
    else
    {
        //打包groundTruthMsg中要保留的pkg
        generateGroundTruthPkg(groundTruthMsg, groundTruthMsgLen, vehiclePedObsList, trafficLightPkgList, trafficSignList, output, msgBufferUsedSize);
    }

    //将新增的pkg打包起来
    for (auto it = inputBufferList.begin(); it != inputBufferList.end(); it++)
    {
        char *buffer = std::get<0>(*it);
        unsigned int size = std::get<1>(*it);

        char *currentPtr = (char *)output + msgBufferUsedSize;
        memcpy(currentPtr, buffer, size);
        msgBufferUsedSize += size;                                                             //更新msgBuffer的已使用空间
    }

    //生成D_SP_PKG_ID_END_FRAME
    S_SP_MSG_ENTRY_HDR* pkgEnd = (S_SP_MSG_ENTRY_HDR *)((char *)output + msgBufferUsedSize);

    //填充PKG头部
    pkgEnd->u4HeaderSize       = sizeof(S_SP_MSG_ENTRY_HDR);
    pkgEnd->u4DataSize         = 0;
    pkgEnd->u4ElementSize      = 0;
    pkgEnd->u2PkgId            = D_SP_PKG_ID_END_FRAME;

    msgBufferUsedSize         += pkgEnd->u4HeaderSize + pkgEnd->u4DataSize;                     //更新msgBuffer的已使用空间

    msgHead->u4DataSize        = msgBufferUsedSize - msgHead->u4HeaderSize;                     //Msg数据部分大小

    outlen = msgBufferUsedSize;

    log_compnt_mngr->info("PackMsg::getPkgData end.");
    return;
}


void PackMsg::getStopPkgData(void *input, size_t inlen, void *output, size_t &outlen)
{
    log_compnt_mngr->info("PackMsg::getStopPkgData start.");
    if (NULL == input || NULL == output || 0 == inlen || 0 == outlen)
    {
        log_compnt_mngr->error("get stop pkg err, input err.");
        outlen = 0;
        return;
    }

    if (inlen > outlen)
    {
        log_compnt_mngr->error("get stop pkg err, inlen {}, outlen {}.", inlen, outlen);
        outlen = 0;
        return;
    }

    // 清空内存
    memset(output, 0, outlen);

    // 全部拷贝然后更新 msg 头
    memcpy(output, input, inlen);
    outlen = inlen;

    S_SP_MSG_HDR *msgHead = (S_SP_MSG_HDR *)output;
    msgHead->u1Sender    = D_SP_SENDER_COMPONENT_MANAGER;

    log_compnt_mngr->info("PackMsg::getStopPkgData end.");
    return;
}

/**
 * @brief                           解析SensorDetectionInfoPkg
 * @param inputBufferList           存放要解析的SensorDetectionInfoPkg
 * @param vehiclePedObsIdSet        输出：车、人、障碍物id集合
 * @param trafficLightIdSet         输出：交通灯id集合
 * @param trafficSignIdSet          输出：交通标志id集合
 */
void PackMsg::parseSensorDetectionInfoPkg(const std::list<std::tuple<char *, unsigned int>> &inputBufferList, std::set<uint32_t> &vehiclePedObsIdSet, std::set<uint32_t> &trafficLightIdSet, std::set<uint32_t> &trafficSignIdSet)
{
    log_compnt_mngr->info("PackMsg::parseSensorDetectionInfoPkg start.");
    //遍历inputBufferList
    for (auto it = inputBufferList.begin(); it != inputBufferList.end(); it++)
    {
        char *buffer = std::get<0>(*it);
        unsigned int size = std::get<1>(*it);

        S_SP_MSG_ENTRY_HDR *pkgHead = (S_SP_MSG_ENTRY_HDR *)buffer; //Pkg的头部指针
        if (pkgHead->u2PkgId == D_SP_PKG_ID_SENSOR_DETECTION_INFO) //解析D_SP_PKG_ID_SENSOR_DETECTION_INFO
        {
            S_SP_SENSOR_DETECTION_INFO *pkgData = (S_SP_SENSOR_DETECTION_INFO *)(buffer + pkgHead->u4HeaderSize);  //数据部分指针

            int elementNum = pkgHead->u4DataSize / pkgHead->u4ElementSize; //元素数量

            for (int i = 0; i < elementNum; i++) //解析每个元素
            {
                uint8_t type = pkgData->u1Type;

                if (type == D_SP_OBJECT_TYPE_TRAFFIC_LIGHT) //交通灯
                {
                    trafficLightIdSet.insert(pkgData->u4Id);
                }
                else if (type == D_SP_OBJECT_TYPE_TRAFFIC_SIGN) //交通标志
                {
                    trafficSignIdSet.insert(pkgData->u4Id);
                }
                else //车、人、障碍物
                {
                    vehiclePedObsIdSet.insert(pkgData->u4Id);
                }

                pkgData = (S_SP_SENSOR_DETECTION_INFO *)(((char *)pkgData) + pkgHead->u4ElementSize);
            }
            log_compnt_mngr->debug("parseSensorDetectionInfoPkg pkgData u8x={}, u8y={}, u8z={}", pkgData->sExtraInfo.sDynamicObj.sBoundingBox[0].u8x, pkgData->sExtraInfo.sDynamicObj.sBoundingBox[0].u8y,pkgData->sExtraInfo.sDynamicObj.sBoundingBox[0].u8z );
            break; //因为inputBufferList只有1个SensorDetectionInfoPkg
        }
    }
    log_compnt_mngr->info("PackMsg::parseSensorDetectionInfoPkg end.");
}

/**
 * @brief                               解析groundTruthMsg
 * @param groundTruthMsg                SimPro输出的groundTruthMsg
 * @param groundTruthMsgLen             SimPro输出的groundTruthMsg长度
 * @param vehiclePedObsIdSet            要保留的车、人、障碍物id集合
 * @param trafficLightIdSet             要保留的交通灯id集合
 * @param trafficSignIdSet              要保留的交通标志id集合
 * @param vehiclePedObsList             输出：要保留的车、人、障碍物指针
 * @param trafficLightPkgList           输出：要保留的交通灯pkg指针
 * @param trafficSignList               输出：要保留的交通标志指针
 * @param reserveSize                   输出：groundTruthMsg中要保留的size
 */
void PackMsg::parseGroundTruthMsg(void *groundTruthMsg, size_t groundTruthMsgLen, const std::set<uint32_t> &vehiclePedObsIdSet, const std::set<uint32_t> &trafficLightIdSet, const std::set<uint32_t> &trafficSignIdSet
                                , std::list<S_SP_MIL_OBJECT_STATE *> &vehiclePedObsList, std::list<char *> &trafficLightPkgList, std::list<S_SP_TRAFFIC_SIGN *> &trafficSignList, unsigned int &reserveSize)
{
    log_compnt_mngr->info("PackMsg::parseGroundTruthMsg start.");
    reserveSize = 0;
    S_SP_MSG_HDR *msgHead = (S_SP_MSG_HDR *)groundTruthMsg; //Msg的头部指针
    char *currentPkg = (char *)groundTruthMsg + msgHead->u4HeaderSize; //当前Pkg的头部指针

    // 解析每个pkg
    while (true)
    {
        S_SP_MSG_ENTRY_HDR *pkgHead = (S_SP_MSG_ENTRY_HDR *)currentPkg; //Pkg的头部指针

        if (pkgHead->u2PkgId == D_SP_MIL_PKG_ID_OBJECT_DATA) //解析S_SP_MIL_OBJECT_STATE
        {
            S_SP_MIL_OBJECT_STATE *pkgData = (S_SP_MIL_OBJECT_STATE *)(currentPkg + pkgHead->u4HeaderSize);  //数据部分指针

            int elementNum = pkgHead->u4DataSize / pkgHead->u4ElementSize; //元素数量

            for (int i = 0; i < elementNum; i++) //解析每个元素
            {
                uint32_t id = pkgData->sObjectState.u4Id;
                if (vehiclePedObsIdSet.find(id) != vehiclePedObsIdSet.end())
                {
                    vehiclePedObsList.push_back(pkgData);
                }

                pkgData = (S_SP_MIL_OBJECT_STATE *)(((char *)pkgData) + pkgHead->u4ElementSize);
            }
        }
        else if (pkgHead->u2PkgId == D_SP_PKG_ID_TRAFFIC_LIGHT) //解析D_SP_PKG_ID_TRAFFIC_LIGHT
        {
            S_SP_TRAFFIC_LIGHT *pkgData = (S_SP_TRAFFIC_LIGHT *)(currentPkg + pkgHead->u4HeaderSize);  //数据部分指针
            uint32_t id = static_cast<uint32_t>(pkgData->u4Id);
            if (trafficLightIdSet.find(id) != trafficLightIdSet.end())
            {
                trafficLightPkgList.push_back(currentPkg);
                reserveSize += pkgHead->u4HeaderSize + pkgHead->u4DataSize;
            }
        }
        else if (pkgHead->u2PkgId == D_SP_PKG_ID_TRAFFIC_SIGN) //解析D_SP_PKG_ID_TRAFFIC_SIGN
        {
            S_SP_TRAFFIC_SIGN *pkgData = (S_SP_TRAFFIC_SIGN *)(currentPkg + pkgHead->u4HeaderSize);  //数据部分指针

            int elementNum = pkgHead->u4DataSize / pkgHead->u4ElementSize; //元素数量

            for (int i = 0; i < elementNum; i++) //解析每个元素
            {
                uint32_t id = pkgData->u4TrafficSignId;
                if (trafficSignIdSet.find(id) != trafficSignIdSet.end())
                {
                    trafficSignList.push_back(pkgData);
                }

                pkgData = (S_SP_TRAFFIC_SIGN *)(((char *)pkgData) + pkgHead->u4ElementSize);
            }
        }
        //这些pkg不需要统计size
        else if ((pkgHead->u2PkgId == D_SP_PKG_ID_START_FRAME) || (pkgHead->u2PkgId == D_SP_PKG_ID_MODEL_OUTLINE)
                || (pkgHead->u2PkgId == D_SP_PKG_ID_SENSOR_INFO) || (pkgHead->u2PkgId == D_SP_PKG_ID_SENSOR_DETECTION_INFO)
                || (pkgHead->u2PkgId == D_SP_V2X_PKG_ID_BASICSAFETYMESSAGE) || (pkgHead->u2PkgId == D_SP_V2X_PKG_ID_ROADSIDESAFETYMESSAGE)
                || (pkgHead->u2PkgId == D_SP_V2X_PKG_ID_ROADSIDEINFORMATION) || (pkgHead->u2PkgId == D_SP_V2X_PKG_ID_SPAT)
                || (pkgHead->u2PkgId == D_SP_V2X_PKG_ID_MAP) || (pkgHead->u2PkgId == D_SP_PKG_ID_V2X_WARNINGINFORMATION)
                || (pkgHead->u2PkgId == D_SP_PKG_ID_OBU_VNFP_INFO) || (pkgHead->u2PkgId == D_SP_PKG_ID_RSU_VNFP_INFO)
                || (pkgHead->u2PkgId == D_SP_PKG_ID_GNSS_DATA) || (pkgHead->u2PkgId == D_SP_PKG_ID_IMU_DATA)
                || (pkgHead->u2PkgId == D_SP_PKG_ID_UWB_DATA))
        {
            // do nothing
        }
        else if (pkgHead->u2PkgId == D_SP_PKG_ID_END_FRAME) //如果是最后一个Pkg
        {
            log_compnt_mngr->trace("parseGroundTruthMsg: End frame pkg, break");
            break;
        }
        else //其他pkg
        {
            //整个pkg都保留
            reserveSize += pkgHead->u4HeaderSize + pkgHead->u4DataSize;
        }

        currentPkg += pkgHead->u4HeaderSize + pkgHead->u4DataSize; //指向下一个pkg
    }

    //统计D_SP_MIL_PKG_ID_OBJECT_DATA和D_SP_PKG_ID_TRAFFIC_SIGN的size
    reserveSize += sizeof(S_SP_MSG_ENTRY_HDR) * 2 + sizeof(S_SP_MIL_OBJECT_STATE) * vehiclePedObsList.size() + sizeof(S_SP_TRAFFIC_SIGN) * trafficSignList.size();
    log_compnt_mngr->info("PackMsg::parseGroundTruthMsg end.");
}

/**
 * @brief                               打包groundTruthMsg中要保留的pkg
 * @param groundTruthMsg                SimPro输出的groundTruthMsg
 * @param groundTruthMsgLen             SimPro输出的groundTruthMsg长度
 * @param vehiclePedObsList             要保留的车、人、障碍物指针
 * @param trafficLightPkgList           要保留的交通灯pkg指针
 * @param trafficSignList               要保留的交通标志指针
 * @param msgBuffer                     输出：打包pkg用的buffer
 * @param msgBufferUsedSize             输出：msgBuffer已使用空间
 */
void PackMsg::generateGroundTruthPkg(void *groundTruthMsg, size_t groundTruthMsgLen, const std::list<S_SP_MIL_OBJECT_STATE *> &vehiclePedObsList, const std::list<char *> &trafficLightPkgList
                                    , const std::list<S_SP_TRAFFIC_SIGN *> &trafficSignList, void *msgBuffer, int &msgBufferUsedSize)
{
    log_compnt_mngr->info("PackMsg::generateGroundTruthPkg start.");
    S_SP_MSG_HDR *msgHead = (S_SP_MSG_HDR *)groundTruthMsg; //Msg的头部指针
    char *currentPkg = (char *)groundTruthMsg + msgHead->u4HeaderSize; //当前Pkg的头部指针
    bool processTrafficLight = false; //是否已处理交通灯pkg

    // 解析每个pkg
    while (true)
    {
        S_SP_MSG_ENTRY_HDR *pkgHead = (S_SP_MSG_ENTRY_HDR *)currentPkg; //Pkg的头部指针

        if (pkgHead->u2PkgId == D_SP_MIL_PKG_ID_OBJECT_DATA)
        {
            char *currentPtr = (char *)msgBuffer + msgBufferUsedSize;

            //拷贝pkgHead
            memcpy(currentPtr, pkgHead, sizeof(S_SP_MSG_ENTRY_HDR));
            msgBufferUsedSize += sizeof(S_SP_MSG_ENTRY_HDR); //更新msgBufferUsedSize

            S_SP_MSG_ENTRY_HDR *destPkgHead = (S_SP_MSG_ENTRY_HDR *)currentPtr;
            destPkgHead->u4DataSize = sizeof(S_SP_MIL_OBJECT_STATE) * vehiclePedObsList.size();
            currentPtr += sizeof(S_SP_MSG_ENTRY_HDR);

            //拷贝pkg data
            for (const auto &it : vehiclePedObsList)
            {
                memcpy(currentPtr, it, sizeof(S_SP_MIL_OBJECT_STATE));
                msgBufferUsedSize += sizeof(S_SP_MIL_OBJECT_STATE); //更新msgBufferUsedSize
                currentPtr += sizeof(S_SP_MIL_OBJECT_STATE);
            }
        }
        else if (pkgHead->u2PkgId == D_SP_PKG_ID_TRAFFIC_LIGHT)
        {
            if (!processTrafficLight)
            {
                //拷贝所有交通灯pkg
                for (const auto &it: trafficLightPkgList)
                {
                    char *currentPtr = (char *)msgBuffer + msgBufferUsedSize;
                    S_SP_MSG_ENTRY_HDR *trafficLightPkgHead = (S_SP_MSG_ENTRY_HDR *)it; //Pkg的头部指针
                    unsigned int pkgSize = trafficLightPkgHead->u4HeaderSize + trafficLightPkgHead->u4DataSize;

                    //拷贝整个pkg
                    memcpy(currentPtr, it, pkgSize);
                    msgBufferUsedSize += pkgSize; //更新msgBufferUsedSize
                }

                processTrafficLight = true; //标记 已处理交通灯pkg
            }
        }
        else if (pkgHead->u2PkgId == D_SP_PKG_ID_TRAFFIC_SIGN)
        {
            char *currentPtr = (char *)msgBuffer + msgBufferUsedSize;

            //拷贝pkgHead
            memcpy(currentPtr, pkgHead, sizeof(S_SP_MSG_ENTRY_HDR));
            msgBufferUsedSize += sizeof(S_SP_MSG_ENTRY_HDR); //更新msgBufferUsedSize

            S_SP_MSG_ENTRY_HDR *destPkgHead = (S_SP_MSG_ENTRY_HDR *)currentPtr;
            destPkgHead->u4DataSize = sizeof(S_SP_TRAFFIC_SIGN) * trafficSignList.size();
            currentPtr += sizeof(S_SP_MSG_ENTRY_HDR);

            //拷贝pkg data
            for (const auto &it : trafficSignList)
            {
                memcpy(currentPtr, it, sizeof(S_SP_TRAFFIC_SIGN));
                msgBufferUsedSize += sizeof(S_SP_TRAFFIC_SIGN); //更新msgBufferUsedSize
                currentPtr += sizeof(S_SP_TRAFFIC_SIGN);
            }
        }
        //这些pkg不需要处理
        else if ((pkgHead->u2PkgId == D_SP_PKG_ID_START_FRAME) || (pkgHead->u2PkgId == D_SP_PKG_ID_MODEL_OUTLINE)
                || (pkgHead->u2PkgId == D_SP_PKG_ID_SENSOR_INFO) || (pkgHead->u2PkgId == D_SP_PKG_ID_SENSOR_DETECTION_INFO)
                || (pkgHead->u2PkgId == D_SP_V2X_PKG_ID_BASICSAFETYMESSAGE) || (pkgHead->u2PkgId == D_SP_V2X_PKG_ID_ROADSIDESAFETYMESSAGE)
                || (pkgHead->u2PkgId == D_SP_V2X_PKG_ID_ROADSIDEINFORMATION) || (pkgHead->u2PkgId == D_SP_V2X_PKG_ID_SPAT)
                || (pkgHead->u2PkgId == D_SP_V2X_PKG_ID_MAP) || (pkgHead->u2PkgId == D_SP_PKG_ID_V2X_WARNINGINFORMATION)
                || (pkgHead->u2PkgId == D_SP_PKG_ID_OBU_VNFP_INFO) || (pkgHead->u2PkgId == D_SP_PKG_ID_RSU_VNFP_INFO)
                || (pkgHead->u2PkgId == D_SP_PKG_ID_GNSS_DATA) || (pkgHead->u2PkgId == D_SP_PKG_ID_IMU_DATA)
                || (pkgHead->u2PkgId == D_SP_PKG_ID_UWB_DATA))
        {
            // do nothing
        }
        else if (pkgHead->u2PkgId == D_SP_PKG_ID_END_FRAME) //如果是最后一个Pkg
        {
            log_compnt_mngr->trace("generateGroundTruthPkg: End Frame,break");
            break;
        }
        else //其他pkg
        {
            char *currentPtr = (char *)msgBuffer + msgBufferUsedSize;
            unsigned int pkgSize = pkgHead->u4HeaderSize + pkgHead->u4DataSize;

            //拷贝整个pkg
            memcpy(currentPtr, currentPkg, pkgSize);
            msgBufferUsedSize += pkgSize; //更新msgBufferUsedSize
        }

        currentPkg += pkgHead->u4HeaderSize + pkgHead->u4DataSize; //指向下一个pkg
    }
    log_compnt_mngr->info("PackMsg::generateGroundTruthPkg end.");
}

void PackMsg::generateMsg(char *input, size_t inlen)
{
    log_compnt_mngr->info("PackMsg::generateMsg start.");
    if (NULL == input || 0 == inlen)
    {
        log_compnt_mngr->error("PackMsg::generateMsg() msg is null.");
        return;
    }

    //解析数据
    S_SP_MSG_HDR *msgHead = (S_SP_MSG_HDR *)input;      //Msg的头部指针
    msgHead->u1Sender = D_SP_SENDER_COMPONENT_MANAGER;

    char *currentPkg = input + msgHead->u4HeaderSize;   //当前Pkg的头部指针
    //第一帧为D_SP_PKG_ID_START_FRAME
    S_SP_MSG_ENTRY_HDR *startPkgHead = (S_SP_MSG_ENTRY_HDR *)currentPkg;
    if (startPkgHead->u2PkgId != D_SP_PKG_ID_START_FRAME)
    {
        log_compnt_mngr->error("PackMsg::generateMsg start package error.");
        return;
    }
    currentPkg += startPkgHead->u4HeaderSize + startPkgHead->u4DataSize;    //指向下一个pkg

    if ((ConfigureMngr::getInstance()->getControlInLoopValue() == 1) && (ConfigureMngr::getInstance()->getControlInLoopType() == 1)) //控制在环打开，动力学挂载内部
    {
        COSIMU_AD_DATA_MANAGER_t cosimuData = {0};

        while (true)
        {
            S_SP_MSG_ENTRY_HDR *pkgHead = (S_SP_MSG_ENTRY_HDR *)currentPkg;                                 //Pkg的头部指针
            if (pkgHead->u2PkgId == D_SP_PKG_ID_DRIVER_CTRL)                                                //S_SP_DRIVER_CTRL
            {
                S_SP_DRIVER_CTRL *pkgData = (S_SP_DRIVER_CTRL *)(currentPkg + pkgHead->u4HeaderSize);       //数据部分指针

                //给动力学输入控制数据进行计算
                cosimuData.driver_ctrl.MasterCylinderPressure = pkgData->u8MasterCylinderPressure;          //制动主缸压力
                cosimuData.driver_ctrl.steeringWheel = pkgData->u8SteeringWheel;                            //方向盘转角
                cosimuData.driver_ctrl.throttlePedal = pkgData->u8ThrottlePedal;                            //油门踏板
                cosimuData.driver_ctrl.brakePedal = pkgData->u8BrakePedal;                                  //刹车踏板
                cosimuData.driver_ctrl.accelTgt = pkgData->u8AccelTgt;                                      //期望加速度
                cosimuData.driver_ctrl.gear = pkgData->u1Gear;                                              //挡位
                cosimuData.driver_ctrl.steeringTorque = pkgData->u8SteeringTorque;                          //方向盘扭矩
                cosimuData.driver_ctrl.MtWheel = pkgData->u4MtWheel;                                        //轮端扭矩
                cosimuData.driver_ctrl.speedTgt = pkgData->u8TargetSpeed;                                   //预期目标速度
                cosimuData.driver_ctrl.stopDistance = pkgData->u8StopDistance;                              //停车距离
                cosimuData.driver_ctrl.brkType = pkgData->u1BrkType;                                        //制动类型

                VirtualCityFmiDynamicAdapter::Instance()->useDynamicsModel(&cosimuData);                    //调用动力学更新数据
            }
            else if (pkgHead->u2PkgId == D_SP_MIL_PKG_ID_EGO_DATA)                                          //S_SP_MIL_EGO_STATE
            {
                S_SP_MIL_EGO_STATE *pkgData = (S_SP_MIL_EGO_STATE *)(currentPkg + pkgHead->u4HeaderSize);   //数据部分指针

                //将动力学输出的数据更新到主车
                pkgData->sObjectState.sAccel.u8X = cosimuData.object_state.ext.accel.x;                     //纵向加速度
                pkgData->sObjectState.sAccel.u8Y = cosimuData.object_state.ext.accel.y;                     //横向加速度
                pkgData->sObjectState.sSpeed.u4H = cosimuData.object_state.ext.speed.h;                     //横摆角速度
                pkgData->sObjectState.sSpeed.u8X = cosimuData.object_state.ext.speed.x;                     //速度
                pkgData->sObjectState.sPos.u4H = cosimuData.object_state.base.pos.h;                        //车辆航向角
                pkgData->sObjectState.sPos.u8X = cosimuData.object_state.base.pos.x;                        //世界坐标x
                pkgData->sObjectState.sPos.u8Y = cosimuData.object_state.base.pos.y;                        //世界坐标y

                //更新主车Id
                pkgData->sObjectState.u4Id = ConfigureMngr::getInstance()->getMainVehicleId();              //主车Id
            }
            else if (pkgHead->u2PkgId == D_SP_PKG_ID_END_FRAME)                                             //如果是最后一个Pkg
            {
                log_compnt_mngr->trace("generateMsg 1 pkg end,break");
                break;
            }
            currentPkg += pkgHead->u4HeaderSize + pkgHead->u4DataSize;                                      //指向下一个pkg
        }
    }
    else //不调用动力学
    {
        while (true)
        {
            S_SP_MSG_ENTRY_HDR *pkgHead = (S_SP_MSG_ENTRY_HDR *)currentPkg;                                 //Pkg的头部指针

            if (pkgHead->u2PkgId == D_SP_MIL_PKG_ID_EGO_DATA)                                               //S_SP_MIL_EGO_STATE
            {
                S_SP_MIL_EGO_STATE *pkgData = (S_SP_MIL_EGO_STATE *)(currentPkg + pkgHead->u4HeaderSize);   //数据部分指针

                //更新主车Id
                pkgData->sObjectState.u4Id = ConfigureMngr::getInstance()->getMainVehicleId();              //主车Id
            }
            else if (pkgHead->u2PkgId == D_SP_PKG_ID_END_FRAME)                                             //如果是最后一个Pkg
            {
                log_compnt_mngr->trace("generateMsg 2 pkg end,break");
                break;
            }
            currentPkg += pkgHead->u4HeaderSize + pkgHead->u4DataSize;                                      //指向下一个pkg
        }
    }
    log_compnt_mngr->info("PackMsg::generateMsg end.");
}