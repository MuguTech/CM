#include "FrameCtrlSocket.h"
#include "ConfigureMngr.h"
#include "log.h"
#include<cstring>

#include "FrameCtrlMsgGenerator.h"
#include "Sensor/SensorManager.h"
#include "common/MMMsgType.h"

FrameCtrlSocket *FrameCtrlSocket::_pInstance = NULL;


FrameCtrlSocket::FrameCtrlSocket(std::string serverIp, uint16_t serverPort, std::string type)
    :SocketClient(serverIp, serverPort, type)
{
    if (sem_init(&sem, 0, 0) == -1)
    {
        log_compnt_mngr->error("frame ctrl socket sem init failed.");
    }

    if(sem_init(&sem_resp, 0, 0) == -1)
    {
      log_compnt_mngr->error("sem_resp init failed.");
    }

    memset(p_data, 0, SOCKET_BUF_SIZE);
    recvLen = 0;

    m_msgBuffer = (char *)malloc(m_msgBufferAllocSize);

    m_frame_ctrl_msg_parser = std::make_unique<FrameCtrlMsgParser>();
}


FrameCtrlSocket::~FrameCtrlSocket()
{
    if(m_msgBuffer)
    {
      free(m_msgBuffer);
      m_msgBuffer = nullptr;
    }

    sem_destroy(&sem);
    sem_destroy(&sem_resp);
}


FrameCtrlSocket *FrameCtrlSocket::get_instance()
{
    if (NULL == _pInstance)
    {
        _pInstance = new FrameCtrlSocket(ConfigureMngr::getInstance()->get_frame_ctl_socket().ip, 
                                    atoi(ConfigureMngr::getInstance()->get_frame_ctl_socket().port.c_str()), 
                                    ConfigureMngr::getInstance()->get_frame_ctl_socket().type);
    }

    return _pInstance;
}

void FrameCtrlSocket::send_init_data() {
  log_compnt_mngr->info("FrameCtrlSocket::send_init_data start.");
  memset(m_msgBuffer, 0, m_msgBufferAllocSize);
  m_msgBufferUsedSize = 0;
  FrameCtrlMsgGenerator msg_generator;
  if (msg_generator.generateMsg(m_msgBuffer, m_msgBufferUsedSize)) {
    log_compnt_mngr->debug("send_init_data.");
    send_msg(m_msgBuffer, m_msgBufferUsedSize);
  } else {
    log_compnt_mngr->error("generate send_init_data error.");
  }
  log_compnt_mngr->info("FrameCtrlSocket::send_init_data end.");
}

void FrameCtrlSocket::send_frame_ctrl_resp_msg(
    bool is_consumed_data,
    const std::map<std::string, bool> &sensor_running_map,
    const std::map<std::string, int> &sensor_frame_map) {
  log_compnt_mngr->info("FrameCtrlSocket::send_frame_ctrl_resp_msg start.");
  memset(m_msgBuffer, 0, m_msgBufferAllocSize);
  m_msgBufferUsedSize = 0;
  FrameCtrlMsgGenerator msg_generator;
  if (msg_generator.generateFrameCtrlRespMsg(
          m_msgBuffer, m_msgBufferUsedSize, is_consumed_data,
          sensor_running_map, sensor_frame_map)) {
    log_compnt_mngr->debug("send_frame_ctrl_resp_msg.");
    send_msg(m_msgBuffer, m_msgBufferUsedSize);
  } else {
    log_compnt_mngr->error("generate send_frame_ctrl_resp_msg error.");
  }
  log_compnt_mngr->info("FrameCtrlSocket::send_frame_ctrl_resp_msg end.");
}

void FrameCtrlSocket::send_stop_resp_msg() {
  log_compnt_mngr->info("FrameCtrlSocket::send_stop_resp_msg start.");
  memset(m_msgBuffer, 0, m_msgBufferAllocSize);
  m_msgBufferUsedSize = 0;
  FrameCtrlMsgGenerator msg_generator;
  if (msg_generator.generateFrameCtrlStopRespMsg(m_msgBuffer,
                                                 m_msgBufferUsedSize)) {
    log_compnt_mngr->debug("send_stop_resp_msg.");
    send_msg(m_msgBuffer, m_msgBufferUsedSize);
  } else {
    log_compnt_mngr->error("generate send_stop_resp_msg error.");
  }
  log_compnt_mngr->info("FrameCtrlSocket::send_stop_resp_msg start.");
}

void FrameCtrlSocket::recv_task()
{
    log_compnt_mngr->info("FrameCtrlSocket::recv_task start.");
    unsigned int sin_size = sizeof(struct sockaddr_in);

    struct sockaddr_in s = get_server_addr();

    if (false == get_conn_stat())
    {
        log_compnt_mngr->error("recv frame ctrl task: pls check server state.");
        return;
    }

    while (get_client() > 0 && true == get_conn_stat())
    {
        memset(p_data, 0, SOCKET_BUF_SIZE);

        if (get_type() == "tcp" || get_type() == "TCP")
        {
            recvLen = recv(get_client(), p_data, SOCKET_BUF_SIZE - 1, 0);
        }
        else if (get_type() == "udp" || get_type() == "UDP")
        {
            recvLen = recvfrom(get_client(), p_data, SOCKET_BUF_SIZE , 0, (struct sockaddr *)&s, &sin_size);
        }
        else
        {
            log_compnt_mngr->error("frame ctrl socket type {} err.", get_type().c_str());
            break;
        }

        log_compnt_mngr->debug("recv frame ctrl len = {}.", recvLen);

        if (recvLen == 0)
        {
            log_compnt_mngr->warn("recv frame ctrl data is 0.");
            break;
        }

        m_frame_ctrl_msg_parser->addRecvMsg(p_data, recvLen);
    }

    log_compnt_mngr->info("FrameCtrlSocket::recv_task end.");
    return;
}

void FrameCtrlSocket::processOneMsg() {
  log_compnt_mngr->info("FrameCtrlSocket::processOneMsg start.");
  // 将收到的sensor list放至sensor manager running list管理
  const auto& sensor_list = m_frame_ctrl_msg_parser->getSensorList();
  log_compnt_mngr->debug("FrameCtrlSocket::processOneMsg revice sensor_list size is = {}",sensor_list.size());
  // reset sensor running list
  if(sensor_list.size() > 0)
  {
    for(const auto& sensor_info : sensor_list)
    {
      std::string sensor_id = sensor_info.first;
      int frame_number = sensor_info.second;
      log_compnt_mngr->debug("FrameCtrlSocket::processOneMsg sensor_id = {},frame_number = {}",sensor_id,frame_number);
      SensorManager::Instance()->setRunning(sensor_id);
      SensorManager::Instance()->setSensorFrameNumber(sensor_id, frame_number);
      SensorManager::Instance()->setIsSyncMode(m_frame_ctrl_msg_parser->getIsSyncMode());
    }
    m_frame_ctrl_msg_parser->clearSensorList();
  }

  sem_post(&sem);

  sem_wait(&sem_resp);
  log_compnt_mngr->info("FrameCtrlSocket::processOneMsg end.");
}

void FrameCtrlSocket::post_sem_resp()
{
  sem_post(&sem_resp);
}

bool FrameCtrlSocket::get_recv_data()
{
    log_compnt_mngr->info("FrameCtrlSocket::get_recv_data start.");
    if (false == get_conn_stat())
    {
        return false;
    }

    log_compnt_mngr->debug("wait frame ctrl data:");

    if (sem_wait(&sem) == -1)
    {
        log_compnt_mngr->error("frame ctrl sem_wait failed!");
        return false;
    }

    log_compnt_mngr->info("FrameCtrlSocket::get_recv_data end.");
    return true;
}

