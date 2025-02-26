#ifndef SAIMOSIM_SOCKETTCPSERVERMULTICLIENT_H
#define SAIMOSIM_SOCKETTCPSERVERMULTICLIENT_H


#include "SocketBase.h"
#include <vector>
#include <string>
#include <sys/epoll.h>

#include <netinet/tcp.h>


#define MAXCONNECTFD (20)
#define TEST_PORT_SERVER    30221



class SocketTcpServerMultiClient : public SocketBase
{

private:
    std::vector<int32_t> mClientFdVec;
    std::vector<std::string> mClientIpVec;
    int32_t m_i4_listenfd;
    bool m_open_for_the_first_time;
    struct epoll_event sEv = {0};
    struct epoll_event sEvent[MAXCONNECTFD] = {0};
    int epoll_msg_num = 0;
    int32_t i4FdEp_G29 = 0;
    int clientMaxNum; //用于限制客户端的最大连接数量，数值大于0表示启用此功能

private:
    void checkbuff(int s = -1);

public:
    SocketTcpServerMultiClient();

    ~SocketTcpServerMultiClient();

    virtual bool open(const uint16_t &in_u2_port);

    virtual void closed();

    virtual int32_t getListenFd(uint16_t const &in_u2_port, bool isLoopbackAddress = true, bool isSpecifyBindIp = false, const std::string bindIp = "0.0.0.0");

    virtual int32_t getConnFd();

    bool writeToAll(uint32_t &in_u4_size, const int8_t *in_p_buf);
    /*向所有tcp client发送消息*/

    void getAllConnFd(std::vector<int32_t> &clientFdVec, std::vector<std::string> &clientIpVec);
    /*获取所有client的socket fd和IP地址*/

    bool readSingleClient(int clientFdIndex, int32_t *out_p_size, char **out_p_buf);
    /*指定从某个client接收数据，clientFdIndex代表client fd在vector中的序号*/

    bool writeSingleClient(int clientFdIndex, int32_t &in_p_size, char *in_p_buf);
    /*指定向某个client发送数据，clientFdIndex代表client fd在vector中的序号*/

    bool readAnyClient(int &clientFdIndex, int32_t *out_p_size, char **out_p_buf);
    /*指定从任意一个client接收数据，clientFdIndex代表client fd在vector中的序号*/

    bool epollCreate();
    /*创建epoll句柄*/

    bool epollDelete();
    /*关闭epoll句柄*/

    bool epollAdd(int epoll_listen_fd);
    /*添加epoll监听的连接句柄*/

    bool epollDel(int32_t epoll_listen_fd);
    /*删除epoll监听的连接句柄*/

    bool readSingleClient(int32_t clientFd, int32_t *out_p_size, char **out_p_buf, bool &isConnected);
    /*指定向某个client发送数据，isConnected返回该client是否断开连接*/

    void setClientMaxNum(int num);
    /*设置可以连接的最大客户端数量*/

    bool closeAcceptClient(int32_t accept_connfd);
    /*关闭当前连接的客户client fd*/

};

#endif //SAIMOSIM_SOCKETTCPSERVERMULTICLIENT_H
