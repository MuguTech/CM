#include "../log.h"
#include<cstring>

#include "HmiSensorCfgTcpClient.h"

HmiSensorCfgTcpClient *HmiSensorCfgTcpClient::_pInstance = NULL;

HmiSensorCfgTcpClient::HmiSensorCfgTcpClient(std::string serverIp, uint16_t serverPort, std::string type)
    :SocketClient(serverIp, serverPort, type)
{
    memset(p_data, 0, SOCKET_BUF_SIZE);
    recvLen = 0;
    recfristtime = false;
}

HmiSensorCfgTcpClient::~HmiSensorCfgTcpClient()
{

}

HmiSensorCfgTcpClient *HmiSensorCfgTcpClient::get_instance()
{
    if (NULL == _pInstance)
    {
        _pInstance = new HmiSensorCfgTcpClient(ConfigureMngr::getInstance()->get_cm_sensor_server().ip, 
                                    atoi(ConfigureMngr::getInstance()->get_cm_sensor_server().port.c_str()), 
                                    ConfigureMngr::getInstance()->get_cm_sensor_server().type);
    }

    return _pInstance;
}

void HmiSensorCfgTcpClient::recv_task()
{
    log_compnt_mngr->info("HmiSensorCfgTcpClient::recv_task start.");
    unsigned int sin_size = sizeof(struct sockaddr_in);

    struct sockaddr_in s = get_server_addr();

    if (false == get_conn_stat())
    {
        log_compnt_mngr->error("recv hmi sensorcfg task: pls check server state.");
        return;
    }
    log_compnt_mngr->debug("get_conn_stat() is {} ,get_client() is {}", get_conn_stat(),get_client());
    while (get_client() > 0 && true == get_conn_stat())
    {
        memset(p_data, 0, SOCKET_BUF_SIZE);
        recvLen = 0;
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
            log_compnt_mngr->error("hmi sensorcfg socket type {} err.", get_type().c_str());
            break;
        }

        log_compnt_mngr->debug("recv hmi sensorcfg len = {}.", recvLen);

        if (recvLen == 0)
        {
            log_compnt_mngr->warn("recv hmi sensorcfg data is 0.");
            break;
        }
        
        //下一次接收数据前对sensor list 清空
        if (recfristtime)
        {
            log_compnt_mngr->critical("clear sensor list");
            recfristtime = false;
            HmiPraseMsg::Instance()->clearSensorList();
            resetSensor();
        }

        //接收数据
        HmiPraseMsg::Instance()->addRecvMsg(p_data, recvLen);

        if (HmiPraseMsg::Instance()->getEndSensorMsg())
        {
            log_compnt_mngr->critical("parse over HMI sensor cfg msg, start creat socket server.");
            AdadpterSocket::get_instance()->createSocketServer();
            log_compnt_mngr->critical("createSocketServer OVER.");
            PluginMngr::get_instance()->init();
            log_compnt_mngr->critical("PluginMngr init OVER.");
            FrameCtrlSocket::get_instance()->send_init_data();
            recfristtime = true;
        } 
    }

    log_compnt_mngr->info("HmiSensorCfgTcpClient::recv_task end.");
    return;
}

void HmiSensorCfgTcpClient::resetSensor()
{
    PluginMngr::get_instance()->Destroy();

    SensorManager::Instance()->Destroy();
}