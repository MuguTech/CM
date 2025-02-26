#include "parse_ground_truth.h"
#include "../../include/Runtime/coSimu/SimProType.h"
#include "log.h"
#include <cstring>
#include <iostream>

#include "FmiDynamicAdapter/VirtualCityFmiDynamicAdapter.h"
#include "ConfigureMngr.h"

parse_ground_truth::parse_ground_truth(void *msg, int len)
{
    this->_msg = (char *)msg;
    this->_len = len;
    frame_id = 0;
    sim_time = 0.0;
    sender = 0;
    simulate_state = 0;
    xodr = "";
    is_complete = false;
}


parse_ground_truth::~parse_ground_truth()
{
}


void parse_ground_truth::set_frame_id(int id)
{
    frame_id = id;
    return;
}


int parse_ground_truth::get_frame_id()
{
    return frame_id;
}


void parse_ground_truth::set_sim_time(double simTime)
{
    sim_time = simTime;
}


double parse_ground_truth::get_sim_time()
{
    return sim_time;
}


void parse_ground_truth::set_sender(unsigned char _sender)
{
    sender = _sender;
}


unsigned char parse_ground_truth::get_sender()
{
    return sender;
}


int parse_ground_truth::get_simulate_state()
{
    return simulate_state;
}


std::string parse_ground_truth::get_xodr()
{
    return xodr;
}

bool parse_ground_truth::parse()
{
    log_compnt_mngr->info("parse_ground_truth::parse start.");
    int use_len = 0;

    simulate_state = 0;

    if (NULL == _msg || 0 == _len)
    {
        log_compnt_mngr->error("msg is null.");
        return false;
    }

    if (_len - use_len < sizeof(S_SP_MSG_HDR))
    {
        log_compnt_mngr->error("pase gt err, len = {}, less then {}.",
                _len - use_len, sizeof(S_SP_MSG_HDR));
        return false;
    }

    use_len += sizeof(S_SP_MSG_HDR);

    S_SP_MSG_HDR *msgHead = (S_SP_MSG_HDR *)_msg;
    frame_id = msgHead->u4FrameNo;
    sim_time = msgHead->u8SimTime;
    sender = msgHead->u1Sender;
    log_compnt_mngr->debug("parse from simpro msg frameno is {}",frame_id);
    char *currentPkg = NULL;
    currentPkg = _msg + msgHead->u4HeaderSize;                                             //当前Pkg的头部指针
    if (NULL == currentPkg)
    {
        log_compnt_mngr->error("msg head is null.");
        return false;
    }

    if (_len - use_len < sizeof(S_SP_MSG_ENTRY_HDR))
    {
        log_compnt_mngr->error("pase gt err, len = {}, less then {}.",
                _len - use_len, sizeof(S_SP_MSG_ENTRY_HDR));
        return false;
    }

    use_len += sizeof(S_SP_MSG_ENTRY_HDR);

    //第一个pkg为D_SP_PKG_ID_START_FRAME
    S_SP_MSG_ENTRY_HDR *startPkgHead = (S_SP_MSG_ENTRY_HDR *)currentPkg;
    if (startPkgHead->u2PkgId != D_SP_PKG_ID_START_FRAME)
    {
        log_compnt_mngr->error("parse start package error.");
        return false;
    }

    if (_len - use_len < startPkgHead->u4DataSize)
    {
        log_compnt_mngr->error("pase gt err, len = {}, less then {}.",
                _len - use_len, startPkgHead->u4DataSize);
        return false;
    }

    use_len += startPkgHead->u4DataSize;

    currentPkg += startPkgHead->u4HeaderSize + startPkgHead->u4DataSize;                        //指向下一个pkg

    simulate_state = IS_SIMULATE;

    while (true)
    {
        if (NULL == currentPkg)
        {
            log_compnt_mngr->error("cur pkg is null.");
            break;
        }

        if (_len - use_len < sizeof(S_SP_MSG_ENTRY_HDR))
        {
            log_compnt_mngr->error("pase gt err, len = {}, less then {}.",
                    _len - use_len, sizeof(S_SP_MSG_ENTRY_HDR));
            break;
        }

        use_len += sizeof(S_SP_MSG_ENTRY_HDR);

        S_SP_MSG_ENTRY_HDR *pkgHead = (S_SP_MSG_ENTRY_HDR *)currentPkg;                         //Pkg的头部指针

        if (_len - use_len < pkgHead->u4DataSize)
        {
            log_compnt_mngr->error("pase gt err, len = {}, less then {}.",
                    _len - use_len, pkgHead->u4DataSize);
            break;
        }

        use_len += pkgHead->u4DataSize;

        if (pkgHead->u2PkgId == D_SP_PKG_ID_INIT_PARAM)
        {
            simulate_state = IS_START;

            S_SP_INIT_PARAM *pkgData = (S_SP_INIT_PARAM *)(currentPkg + pkgHead->u4HeaderSize); //数据部分指针
            xodr = pkgData->au1OpenDrive;
            log_compnt_mngr->debug("parse IS_START xodr={}.", xodr.c_str());
        }
        else if (pkgHead->u2PkgId == D_SP_PKG_ID_SYNC)
        {
            S_SP_SYNC *pkgData = (S_SP_SYNC *)(currentPkg + pkgHead->u4HeaderSize);             //数据部分指针
            if (pkgData->u4CmdMask & D_SP_SYNC_CMD_FINISH_CURRENT)                              //最低位为1代表当前测试场景结束
            {
                simulate_state = IS_STOP;
                log_compnt_mngr->debug("parse IS_STOP .");
            }
        }
        else if (pkgHead->u2PkgId == D_SP_MIL_PKG_ID_EGO_DATA)
        {
            if (ConfigureMngr::getInstance()->getIsVirtualCity())
            {
                S_SP_MIL_EGO_STATE *pkgData = (S_SP_MIL_EGO_STATE *)(currentPkg + pkgHead->u4HeaderSize);   //数据部分指针
                if (frame_id == 1) //场景运行第一帧，获取主车初始位置、姿态角、速度，给echosim初始化
                {
                    VirtualCityFmiDynamicAdapter::VehicleDynamicModelInit_t initParams = {0.0};

                    initParams.startSpeed = sqrt(pow(pkgData->sObjectState.sSpeed.u8X, 2) + pow(pkgData->sObjectState.sSpeed.u8Y, 2));
                    initParams.initX0 = pkgData->sObjectState.sPos.u8X;
                    initParams.initY0 = pkgData->sObjectState.sPos.u8Y;
                    initParams.initZ0 = pkgData->sObjectState.sPos.u8Z;
                    initParams.initYaw =  pkgData->sObjectState.sPos.u4H;
                    initParams.initPitch =  pkgData->sObjectState.sPos.u4P;
                    initParams.initRoll =  pkgData->sObjectState.sPos.u4R;

                    log_compnt_mngr->debug("parse initParams startSpeed={}, initX0={}, initY0={}, initZ0={}, initYaw={}, initPitch={}, initRoll={}.",
                    initParams.startSpeed, initParams.initX0, initParams.initY0, initParams.initZ0, initParams.initYaw, initParams.initPitch, initParams.initRoll);

                    VirtualCityFmiDynamicAdapter::Instance()->setInitParams(initParams);  //设置动力学初始化参数
                }
            }
        }
        else if (pkgHead->u2PkgId == D_SP_PKG_ID_ENVIRONMENT)
        {
            if (ConfigureMngr::getInstance()->getIsVirtualCity())
            {
                S_SP_ENVIRONMENT *pkgData = (S_SP_ENVIRONMENT *)(currentPkg + pkgHead->u4HeaderSize);   //数据部分指针

                VirtualCityFmiDynamicAdapter::Instance()->setEnvParams(pkgData->u8RainIntensity, pkgData->u8SnowIntensity);  //设置降雨量和降雪量
            }
        }
        else if (pkgHead->u2PkgId == D_SP_PKG_ID_END_FRAME)                                                  //如果是最后一个Pkg
        {
            is_complete = true;
            log_compnt_mngr->debug("pase gt ok.");
            break;
        }

        currentPkg += pkgHead->u4HeaderSize + pkgHead->u4DataSize;                              //指向下一个pkg
    }

    log_compnt_mngr->info("parse_ground_truth::parse end.");

    return is_complete;
}


bool parse_ground_truth::is_complete_pkg()
{
    if (true == is_complete)
    {
        return is_complete;
    }

    if (_len < sizeof(S_SP_MSG_HDR))
    {
        log_compnt_mngr->error("pase gt err, len = {}, less then {}.",
                _len, sizeof(S_SP_MSG_HDR));
        return is_complete;
    }

    S_SP_MSG_HDR *msgHead = (S_SP_MSG_HDR *)_msg;

    if (_len < msgHead->u4HeaderSize + msgHead->u4DataSize)
    {
        log_compnt_mngr->error("pase gt err, len = {}, less then {}.",
                _len, msgHead->u4HeaderSize + msgHead->u4DataSize);
        return is_complete;
    }

    is_complete = true;

    return is_complete;
}


// 返回当前帧数据的长度
bool parse_ground_truth::get_len_frame_data(size_t & len)
{
    log_compnt_mngr->info("parse_ground_truth::get_len_frame_data start.");
    if (true != is_complete_pkg())
    {
        log_compnt_mngr->error("is not compelet data.");
        return false;
    }

    S_SP_MSG_HDR *msgHead = (S_SP_MSG_HDR *)_msg;
    len = msgHead->u4HeaderSize + msgHead->u4DataSize;

    log_compnt_mngr->debug("parse_ground_truth::get_len_frame_data len = {}.",len);
    log_compnt_mngr->info("parse_ground_truth::get_len_frame_data end.");
    return true;
}

