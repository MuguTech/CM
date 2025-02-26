#ifndef SAIMOSIM_SOCKETFACTORY_H
#define SAIMOSIM_SOCKETFACTORY_H

#include "SocketIf.h"

enum e_saimosim_protocol
{
    E_TCP,
    E_UDP
};
enum e_saimosim_role
{
    E_SERVER,
    E_CLIENT
};

class SocketFactory
{
public:
    static SocketFactory *getInstance();

    static SocketIf *create(e_saimosim_protocol const &pr, e_saimosim_role const &ro);

private:
    static SocketFactory *m_inst;
};
#endif //SAIMOSIM_SOCKETFACTORY_H
