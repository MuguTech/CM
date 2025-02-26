#ifndef SAIMOSIM_SOCKETBASE_H
#define SAIMOSIM_SOCKETBASE_H

#include "SocketIf.h"

#define D_SOCKET_RECV_BUFF_SIZE (5120000)
class SocketBase : public SocketIf
{
public:
    virtual bool open(const uint16_t &in_u2_port) = 0;

    virtual bool read(int32_t *out_p_size, char **out_p_buf);

    virtual bool write(uint32_t &in_u4_size, const int8_t *in_p_buf);

    virtual void closed();

	virtual int32_t getListenFd(uint16_t const &in_u2_port, bool isLoopbackAddress = true, bool isSpecifyBindIp = false, const std::string bindIp = "0.0.0.0") = 0;

    virtual int32_t getConnFd() = 0;

protected:
    SocketBase();

    ~SocketBase();

    int32_t m_i4_connfd;
    char m_buf[D_SOCKET_RECV_BUFF_SIZE];
};
#endif //SAIMOSIM_SOCKETBASE_H
