#ifndef SAIMOSIM_SOCKETIF_H
#define SAIMOSIM_SOCKETIF_H

#include <string>


class SocketIf
{
public:
    virtual ~SocketIf() = default;
    virtual bool open(const uint16_t &in_u2_port) = 0;
    virtual bool read(int32_t *out_p_size, char **out_p_buf) = 0;
    virtual bool write(uint32_t &in_u4_size, const int8_t *in_p_buf) = 0;
    virtual void closed() = 0;
	virtual int32_t getListenFd(uint16_t const &in_u2_port, bool isLoopbackAddress = true, bool isSpecifyBindIp = false, const std::string bindIp = "0.0.0.0") = 0;
    virtual int32_t getConnFd() = 0;
};
#endif //SAIMOSIM_SOCKETIF_H
