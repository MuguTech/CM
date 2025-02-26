#ifndef SAIMOSIM_SOCKETTCPSERVER_H
#define SAIMOSIM_SOCKETTCPSERVER_H


#include "SocketBase.h"

class SocketTcpServer : public SocketBase
{
public:
    SocketTcpServer();

    ~SocketTcpServer();

    virtual bool open(const uint16_t &in_u2_port);

    virtual void closed();

	virtual int32_t getListenFd(uint16_t const &in_u2_port, bool isLoopbackAddress = true, bool isSpecifyBindIp = false, const std::string bindIp = "0.0.0.0");

    virtual int32_t getConnFd();

private:
    int32_t m_i4_listenfd;
    bool m_open_for_the_first_time;
};

#endif //SAIMOSIM_SOCKETTCPSERVER_H
