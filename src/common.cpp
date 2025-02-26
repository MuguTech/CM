#include "common.h"
#include "log.h"
#include <mutex>
#include <unistd.h>
#include <cstring>


// 保存一份 v-traffic 数据
MSG_MUTEX v_traffic_data;


CoSimuMsg recvMsgFromAd;              //存储接收到的AD数据
CoSimuMsg sendMsgToSimpro;            //准备发送给simpro的数据

CoSimuMsg &getRecvMsgFromAd()
{
    return recvMsgFromAd;
}

CoSimuMsg &getSendMsgToSimpro()
{
    return sendMsgToSimpro;
}


bool is_file_exist(std::string &filepath)
{
    return !access(filepath.c_str(), F_OK);
}



bool update_v_traffic_data(void *msg, size_t len)
{
    if (NULL == msg || 0 == len)
    {
        log_compnt_mngr->error("input para err.");
        return false;
    }

    if (len > MSG_BUFF_SIZE)
    {
        log_compnt_mngr->error("input len {} err to update v-traffic data.", len);
        return false;
    }

    v_traffic_data.msgMutex.lock();
    memcpy(v_traffic_data.ch, (char *)msg, len);
    v_traffic_data.len = (int)len;
    v_traffic_data.msgMutex.unlock();

    return true;
}


bool get_v_traffic_data(void *msg, size_t &len)
{
    if (NULL == msg || v_traffic_data.len > len)
    {
        log_compnt_mngr->error("input para err.");
        return false;
    }

    memset(msg, 0, len);

    v_traffic_data.msgMutex.lock();
    memcpy((char *)msg, v_traffic_data.ch, v_traffic_data.len);
    len = (size_t)v_traffic_data.len;
    v_traffic_data.msgMutex.unlock();

    return true;
}


bool is_sensor_type(const std::string& plugin_type)
{
  if (plugin_type == LIDAR_T || plugin_type == RADAR_T ||
      plugin_type == CAMERA_T || plugin_type == ULTRASONIC_T ||
      plugin_type == V2X_T)
  {
    return true;
  }

  return false;
}

//释放数据，防止CM向AD多次发送结束消息
void resetSimproMsg()
{
    memset(v_traffic_data.ch, 0, MSG_BUFF_SIZE);
    v_traffic_data.len = 0;
}