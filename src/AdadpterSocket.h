#ifndef __AD_ADPTER_SOCKET__
#define __AD_ADPTER_SOCKET__

#include "SocketFw/SocketTcpServerMultiClient.h"
#include <map>
#include <thread>
#include <arpa/inet.h>
#include "PluginCfg.h"
#include <semaphore.h>

#define MAX_SENSOR_PLUGIN_NUM       20
class AdadpterSocket
{
private:
    static AdadpterSocket* _pInstance;

    std::map<std::string, SocketTcpServerMultiClient *>g_sock_obj_server; //map<sensor id ,tcp service class>,存放传感器id和对应的TCP server的类的指针
    std::map<std::string, int> m_epoll_fd;                       //<sensor id,epoll fd> 存放传感器id和对应的epoll创建的fd
    std::map<std::string, std::thread> v_socket_adadapter;       //<sensor id,std::thread> 存放传感器id和对应的接收线程
    std::map<std::string, std::atomic<bool>>g_thread_run_flag;   //<sensor id,原子变量> 存放传感器id和对应的原子变量
    std::map<std::string,struct _socket> m_sensorCommunicationCfgMap;  //<传感器id,结构体包含ip、port、type>，存放传感器id和对应的传感器通信端口配置

    std::map<std::string, int>g_sock_udp_server;
    std::map<std::string, int> m_socket_type; //<sensor id, socket type> socket type：1：TCP 2：UDP
    std::map<int, struct sockaddr_in> m_udp_client_addr; //<socket id, client addr>

    sem_t sem_consume;     //通知消费线程给处理数据
    typedef struct{
        char *buf;
        int allocSize;
        int usedSize;
    } RecvBufInfo;
    RecvBufInfo bufInfo;

public:
    ~AdadpterSocket();
    static AdadpterSocket *get_instance();
    bool send_msg(const std::string sensor_id, void *p_msg, int32_t &len);
    void createSocketServer();

private:
    AdadpterSocket();
    static void thread_func(AdadpterSocket *p, const std::string &sensor_name, const std::string &sensor_id, const int &socketSerfd);
    void recvAdThread(const std::string &sensor_name, const std::string &sensor_id, const int &socketSerfd);
    void closeServerAndManageThreads(const std::string &sensorId);
    bool isSoketChg(const std::string &sensorId,struct _socket sensorComunCfg);

    void consumeThread();    //消费线程
    bool recvBufAddSize(RecvBufInfo &info, int needSize);
    void checkMsgComplete(char *buffer, int bufferSize, bool &isComplete, int &msgSize); //检查buffer中是否包含完整的一帧数据

};
#endif  // __AD_ADPTER_SOCKET__

