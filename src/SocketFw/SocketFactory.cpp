#include "SocketFactory.h"
#include "SocketTcpServer.h"

SocketFactory *SocketFactory::m_inst = nullptr;

SocketFactory *SocketFactory::getInstance()
{
    static SocketFactory local_sf;
    if (m_inst == nullptr)
    {
        m_inst = &local_sf;
    }
    return m_inst;
}

SocketIf *SocketFactory::create(e_saimosim_protocol const &pr, e_saimosim_role const &ro)
{
    SocketIf *si = nullptr;

    if ((E_TCP == pr) && (E_SERVER == ro))
    {
        si = new SocketTcpServer();
    }

    return si;
}
