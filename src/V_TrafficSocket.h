#ifndef __V_TRAFFIC_SOCKET_H__
#define __V_TRAFFIC_SOCKET_H__

#include "SocketClient.h"
#include<semaphore.h>


// #include "../../TaskControl/include/Runtime/coSimu/SimProType.h"



class V_TrafficSocket: public SocketClient
{
private:
    static V_TrafficSocket *_pInstance;
    char p_data[SOCKET_BUF_SIZE];
    ssize_t recvLen;
    sem_t sem;      // 通知是否recv data over
    sem_t sem1;     // 通知是否使用 p_data over
    sem_t sem_send;             //通知发送线程给simpro发送数据

public:
    ~V_TrafficSocket();
    static V_TrafficSocket *get_instance();
    bool get_recv_data(void **pp_msg, size_t &len);
    void sempost();
    void sem_send_post();



private:
    V_TrafficSocket(std::string serverIp, uint16_t serverPort, std::string type = "tcp");
    void recv_task();
    void recv_task_epoll();
    bool recv_pkg_finished(size_t & min_len);

    void sendIdentityInfo(); //在UDP模式下告知simpro身份，使得simpro能够将数据发给cm模块

    void sendEgoId(); //在多主车情况下给SimPro发送CM对应的主车ID

    void sendThread();   //发送线程
};



#endif  // __V_TRAFFIC_SOCKET_H__

