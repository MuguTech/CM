extern "C"
{
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <ifaddrs.h>
}

#include <cstring>
#include <csignal>
#include "SocketTcpServer.h"
#include "../log.h"

#define LISTENQ (1024)

static void getIp(char *p_ip)
{
    struct ifaddrs *ifAddrStruct = NULL;
    struct ifaddrs *ifAddrStructHead = NULL;
    void *tmpAddrPtr = NULL;
    char target[] = "127.0.0.1";

    getifaddrs(&ifAddrStruct);
    ifAddrStructHead = ifAddrStruct;

    while (ifAddrStruct != NULL)
    {
		if ((ifAddrStruct->ifa_name)[0] == 'e') /* 网络接口名：以字母e开头 */
		{
			if (ifAddrStruct->ifa_addr->sa_family == AF_INET) /* 协议族：AF_INET,当前网络接口可以支持：与所属网络中某个节点通信 */
			{
				tmpAddrPtr = &((struct sockaddr_in *)ifAddrStruct->ifa_addr)->sin_addr;
				char addressBuffer[INET_ADDRSTRLEN];
				inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
			    if (0 != strcmp(target, addressBuffer))
		        {
	                memcpy(p_ip, addressBuffer, INET_ADDRSTRLEN);

	                freeifaddrs(ifAddrStructHead);
					return ;
				}
			}
		}
        ifAddrStruct = ifAddrStruct->ifa_next;
    }
    memcpy(p_ip, target, strlen(target));
    freeifaddrs(ifAddrStructHead);
    return ;
}

SocketTcpServer::SocketTcpServer()
{
    m_open_for_the_first_time = true;
    m_i4_listenfd = -1;
}

SocketTcpServer::~SocketTcpServer()
{
}

bool SocketTcpServer::open(const uint16_t &in_u2_port)
{
    bool ret = false;

    if (m_open_for_the_first_time)
    {
        m_open_for_the_first_time = false;

        (void)getListenFd(in_u2_port);
    }

    (void)getConnFd();
    if (0 <= m_i4_connfd)
    {
        ret = true;
    }

    return ret;
}

void SocketTcpServer::closed() /* 主动关闭socket服务句柄 */
{
    SocketBase::closed();

    if (0 <= m_i4_listenfd)
    {
        (void)close(m_i4_listenfd);
		m_i4_listenfd = -1; /* 初始化socket服务句柄 */
		m_open_for_the_first_time = true; /* init */
    }
}

int32_t SocketTcpServer::getListenFd(uint16_t const &in_u2_port, bool isLoopbackAddress, bool isSpecifyBindIp, const std::string bindIp)
{
    log_compnt_mngr->info("SocketTcpServer::getListenFd start.");
    if (m_i4_listenfd == -1)
    {
        /* Create a socket descriptor */
        m_i4_listenfd = socket(AF_INET, SOCK_STREAM, 0);
        if (m_i4_listenfd < 0)
        {
        }
        else
        {
            const int32_t optval = 1;
            /* Eliminates "Address already in use" error from bind */
            if (setsockopt(m_i4_listenfd, SOL_SOCKET, SO_REUSEADDR,
                           reinterpret_cast<const void *>(&optval), sizeof(int)) < 0)
            {
            }
            else
            {
                struct sockaddr_in serveraddr;
                /* Listenfd will be an end point for all requests to port
                     * on any IP address for this host */
                bzero(reinterpret_cast<int8_t *>(&serveraddr), sizeof(serveraddr));
                serveraddr.sin_family = AF_INET;
                if (isLoopbackAddress) {
                    const char LocalIP[] = "127.0.0.1";
                    serveraddr.sin_addr.s_addr = inet_addr(LocalIP);
                } else {
					if (isSpecifyBindIp) { /* [不以回环地址为绑定IP的情况下] true值:要求使用指定的绑定IP */
						serveraddr.sin_addr.s_addr = inet_addr(bindIp.c_str());
					} else { /* [不以回环地址为绑定IP的情况下] false值:使用默认的绑定IP（网络通信节点IP） */
						char LocalIP[INET_ADDRSTRLEN] = {0};
						getIp(LocalIP); /* 获得节点网卡IP */
						serveraddr.sin_addr.s_addr = inet_addr(LocalIP);
					}
                }
                serveraddr.sin_port = htons(in_u2_port);
                if (bind(m_i4_listenfd, reinterpret_cast<struct sockaddr *>(&serveraddr), sizeof(serveraddr)) < 0)
                {
                }
                else
                {
                    /* Make it a listening socket ready to accept connection requests */
                    if (listen(m_i4_listenfd, LISTENQ) < 0)
                    {
                    }
                }
            }
        }
    }

    log_compnt_mngr->debug("SocketTcpServer::getListenFd m_i4_listenfd = {}.",m_i4_listenfd);
    log_compnt_mngr->info("SocketTcpServer::getListenFd end.");
    return m_i4_listenfd;
}

int32_t SocketTcpServer::getConnFd()
{
    log_compnt_mngr->info("SocketTcpServer::getConnFd start.");
    struct sockaddr_in clientaddr;
    socklen_t clientlen = sizeof(struct sockaddr_in);

    m_i4_connfd = -1; /* init */

    if (0 <= m_i4_listenfd)
    {
        m_i4_connfd = accept(m_i4_listenfd, reinterpret_cast<struct sockaddr *>(&clientaddr), &clientlen);
    }

    log_compnt_mngr->debug("SocketTcpServer::getConnFd m_i4_connfd = {}.",m_i4_connfd);
    log_compnt_mngr->info("SocketTcpServer::getConnFd end.");
    return m_i4_connfd;
}
