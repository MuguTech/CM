#ifndef __HMI_SENSOR_CFG_TCP_CLIENT_H__
#define __HMI_SENSOR_CFG_TCP_CLIENT_H__

#include "../SocketClient.h"
#include "HmiPraseMsg.h"
#include <memory>
#include <semaphore.h>

/**
 * @class HmiSensorCfgTcpClient
 * @brief CM 与 simpro socket通信client和消息接收，用于接收来自HMI传感器配置
 */
class HmiSensorCfgTcpClient: public SocketClient
{
private:
    static HmiSensorCfgTcpClient *_pInstance;
    char p_data[SOCKET_BUF_SIZE];
    ssize_t recvLen;
    bool recfristtime;
    std::unique_ptr<HmiSensorCfgTcpClient> m_frame_ctrl_msg_parser;

public:
    ~HmiSensorCfgTcpClient();
    static HmiSensorCfgTcpClient *get_instance();
    void resetSensor();
    protected:
    void recv_task();
private:
    HmiSensorCfgTcpClient(std::string serverIp, uint16_t serverPort, std::string type = "tcp");

};



#endif  // __FRAME_CTRL_SOCKET_H__

