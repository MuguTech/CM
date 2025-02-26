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
#include <sys/epoll.h>
#include <ifaddrs.h>
}
#include <cstring>
#include <csignal>
#include<algorithm>

#include "SocketTcpServerMultiClient.h"
#include "../log.h"

#define LISTENQ (1024)
#define MAXCONNECTFD (20)

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

SocketTcpServerMultiClient::SocketTcpServerMultiClient()
{
    m_open_for_the_first_time = true;
    m_i4_listenfd = -1;
    clientMaxNum = -1; //用于限制客户端的最大连接数量，数值大于0表示启用此功能
}

SocketTcpServerMultiClient::~SocketTcpServerMultiClient()
{
}


bool SocketTcpServerMultiClient::open(const uint16_t &in_u2_port)
{
    bool ret = false;

    if (m_open_for_the_first_time)
    {
        m_open_for_the_first_time = false;

        log_compnt_mngr->debug("SocketTcpServerMultiClient::open creat ListenFd.");

        (void)getListenFd(in_u2_port);
    }

    log_compnt_mngr->debug("SocketTcpServerMultiClient::open creat ConnFd.");

    (void)getConnFd();
    if (0 <= m_i4_connfd)
    {
        ret = true;
    }

    return ret;
}

void SocketTcpServerMultiClient::closed() /* 主动关闭socket服务句柄 */
{
    log_compnt_mngr->info("SocketTcpServerMultiClient::closed start.");
    std::vector<int32_t>::iterator it;

    if (m_i4_listenfd > 0)
    {
        (void)close(m_i4_listenfd);
    }

    for (it = mClientFdVec.begin(); it != mClientFdVec.end(); it++)
    {
        m_i4_listenfd = *it;
        if (0 <= m_i4_listenfd)
        {
            (void)close(m_i4_listenfd);
        }
    }
    std::vector<int32_t>(mClientFdVec).swap(mClientFdVec);
    std::vector<std::string>(mClientIpVec).swap(mClientIpVec);
    log_compnt_mngr->info("SocketTcpServerMultiClient::closed end.");
}

int32_t SocketTcpServerMultiClient::getListenFd(uint16_t const &in_u2_port, bool isLoopbackAddress, bool isSpecifyBindIp, const std::string bindIp)
{
    log_compnt_mngr->info("SocketTcpServerMultiClient::getListenFd start.");
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
                    log_compnt_mngr->error("bind ip: {}, port {} err.", inet_ntoa(serveraddr.sin_addr), ntohs(serveraddr.sin_port));
                }
                else
                {
                    /* Make it a listening socket ready to accept connection requests */
                    if (listen(m_i4_listenfd, LISTENQ) < 0)
                    {
                        log_compnt_mngr->error("listen fd err.");
                    }
                    else
                    {

                        checkbuff(m_i4_listenfd);

                        epollAdd(m_i4_listenfd);
                        log_compnt_mngr->debug("start listen: ip: {}, port {}.", inet_ntoa(serveraddr.sin_addr), ntohs(serveraddr.sin_port));
                    }
                }
            }
        }
    }

    log_compnt_mngr->debug("SocketTcpServerMultiClient::getListenFd m_i4_listenfd = {}.",m_i4_listenfd);
    log_compnt_mngr->info("SocketTcpServerMultiClient::getListenFd end.");
    return m_i4_listenfd;
}

int32_t SocketTcpServerMultiClient::getConnFd()
{
    log_compnt_mngr->info("SocketTcpServerMultiClient::getConnFd start.");
    struct sockaddr_in clientaddr;
    socklen_t clientlen = sizeof(struct sockaddr_in);

    m_i4_connfd = -1; /* init */

    if (0 > m_i4_listenfd)
    {
        log_compnt_mngr->error("m i4 listenfd err, {}.", m_i4_listenfd);
        return m_i4_connfd;
    }

    // log_compnt_mngr->info("accept .. .");
    m_i4_connfd = accept(m_i4_listenfd, reinterpret_cast<struct sockaddr *>(&clientaddr), &clientlen);
    if (m_i4_connfd < 0)
    {
        log_compnt_mngr->error("acpt err, {}.", m_i4_connfd);
        return m_i4_connfd;
    }
    log_compnt_mngr->debug("accept client fd is {}", m_i4_connfd);

    if (clientMaxNum > 0) //当客户端连接数量有限制时,连接数量已满后连接的客户端直接close
    {
        if ((mClientFdVec.size() + 1) > clientMaxNum)
        {
            close(m_i4_connfd);
            m_i4_connfd = -1;
            log_compnt_mngr->error("connit client over the clientMaxNum.");
            return m_i4_connfd;
        }
    }
    std::string IpAddr = inet_ntoa(clientaddr.sin_addr);
    mClientIpVec.push_back(IpAddr);
    mClientFdVec.push_back(m_i4_connfd);
    epollAdd(m_i4_connfd);

    log_compnt_mngr->debug("SocketTcpServerMultiClient::getConnFd m_i4_connfd = {}",m_i4_connfd);
    log_compnt_mngr->info("SocketTcpServerMultiClient::getConnFd end.");
    return m_i4_connfd;
}

/*向所有tcp client发送消息*/
bool SocketTcpServerMultiClient::writeToAll(uint32_t &in_u4_size, const int8_t *in_p_buf)
{
    log_compnt_mngr->info("SocketTcpServerMultiClient::writeToAll start.");
    int32_t i4_sent_size = -1;
    bool b_ret = false;
    std::vector<int32_t>::iterator it;

    for (it = mClientFdVec.begin(); it != mClientFdVec.end(); it++)
    {
        log_compnt_mngr->debug("SocketTcpServerMultiClient::writeToAll mClientFdVec.begin().");
        if (0 <= *it) 
        {
            log_compnt_mngr->debug("SocketTcpServerMultiClient::writeToAll 0 <= *it.");
            try{
                i4_sent_size =static_cast<int32_t>(send(*it, in_p_buf, in_u4_size, MSG_NOSIGNAL));
                if (-1 == i4_sent_size)
                {
                    log_compnt_mngr->debug("-1 == i4_sent_size.");
                    if (104 == errno) { /* 104:Connection reset by peer,表明你在对一个对端socket已经关闭的的连接调用write或send方法 */
                        log_compnt_mngr->error("SocketTcpServerMultiClient::writeToAll errno = 104 Connection reset by peer.");

                        (void)close(*it); /* 被动关闭连接句柄 */
                        epollDel(*it);
                        *it = -1; /* 初始化连接句柄 */
                    }
                }
                else
                {
                    log_compnt_mngr->debug("SocketTcpServerMultiClient::writeToAll success.");

                    b_ret = true;
                }
            } catch(...) {
                log_compnt_mngr->error("SocketBase::write exception.");
            }
        }
    }

    log_compnt_mngr->info("SocketTcpServerMultiClient::writeToAll end.");
    return b_ret;
}

/*获取所有client的socket fd和IP地址*/
void SocketTcpServerMultiClient::getAllConnFd(std::vector<int32_t> &clientFdVec, std::vector<std::string> &clientIpVec)
{
    clientFdVec = mClientFdVec;
    clientIpVec = mClientIpVec;
}

/*指定从某个client接收数据，clientFdIndex代表client fd在vector中的序号*/
bool SocketTcpServerMultiClient::readSingleClient(int clientFdIndex, int32_t *out_p_size, char **out_p_buf)
{
    log_compnt_mngr->info("SocketTcpServerMultiClient::readSingleClient start.");

    int32_t i4_recv_size = -1;
    bool b_ret = false;
    int32_t m_client_connfd = mClientFdVec[clientFdIndex];

    (void)memset(m_buf, 0x00, D_SOCKET_RECV_BUFF_SIZE);

    if (0 <= m_client_connfd)
    {
        i4_recv_size = static_cast<int32_t>(recv(m_client_connfd, m_buf, D_SOCKET_RECV_BUFF_SIZE, 0));
        if (-1 == i4_recv_size)
        {
        }
        else if (0 == i4_recv_size)
        {
            log_compnt_mngr->error("SocketTcpServerMultiClient::readSingleClient fail.");

			(void)close(m_client_connfd); /* 被动关闭连接句柄 */
            epollDel(m_client_connfd);
            m_client_connfd = -1; /* EOF,end-of-file */
        }
        else
        {
            log_compnt_mngr->debug("SocketTcpServerMultiClient::readSingleClient success.");

            *out_p_size = i4_recv_size;
            *out_p_buf = m_buf;
            b_ret = true;
        }
    }

    log_compnt_mngr->info("SocketTcpServerMultiClient::readSingleClient end.");
    return b_ret;
}

/*指定向某个client发送数据，clientFdIndex代表client fd在vector中的序号*/
bool SocketTcpServerMultiClient::writeSingleClient(int clientFdIndex, int32_t &in_p_size, char *in_p_buf)
{
    log_compnt_mngr->info("SocketTcpServerMultiClient::writeSingleClient start.");
    int32_t i4_sent_size = -1;
    bool b_ret = false;

    if (clientFdIndex >= mClientFdVec.size())
    {
        log_compnt_mngr->error("cli index {} is err.", clientFdIndex);
        return false;
    }

    int32_t m_client_connfd = mClientFdVec[clientFdIndex];
    if (0 <= m_client_connfd) 
    {
		try{
            log_compnt_mngr->debug("SocketTcpServerMultiClient::writeSingleClient 开始发送 m_client_connfd = {}, len = {}.", m_client_connfd, in_p_size);

			i4_sent_size = static_cast<int32_t>(send(m_client_connfd, in_p_buf, in_p_size, MSG_NOSIGNAL));
            log_compnt_mngr->debug("SocketTcpServerMultiClient::writeSingleClient 查看发送结果.");

			if (-1 == i4_sent_size)
			{
                log_compnt_mngr->error("SocketTcpServerMultiClient::writeSingleClient 发送失败.");

				if (104 == errno) { /* 104:Connection reset by peer,表明你在对一个对端socket已经关闭的的连接调用write或send方法 */
					(void)close(m_client_connfd); /* 被动关闭连接句柄 */
                    epollDel(m_client_connfd);
					m_client_connfd = -1; /* 初始化连接句柄 */
				}
			}
			else
			{
                log_compnt_mngr->debug("SocketTcpServerMultiClient::writeSingleClient 发送成功.");

				b_ret = true;
			}
		} catch(...) {
            log_compnt_mngr->error("SocketBase::write exception .");
		}
    }

    log_compnt_mngr->info("SocketTcpServerMultiClient::writeSingleClient end.");
    return b_ret;
}

/*指定从任意一个client接收数据，clientFdIndex代表client fd在vector中的序号*/
bool SocketTcpServerMultiClient::readAnyClient(int &clientFdIndex, int32_t *out_p_size, char **out_p_buf)
{
    log_compnt_mngr->info("SocketTcpServerMultiClient::readAnyClient start.");
    int32_t i4_recv_size = -1;
    bool b_ret = false;
    static int epoll_count = 0;

    if (i4FdEp_G29)
    {
        log_compnt_mngr->debug("SocketTcpServerMultiClient::readAnyClient 存在epoll句柄.");
    }
    else
    {
        log_compnt_mngr->error("SocketTcpServerMultiClient::readAnyClient 不存在epoll句柄.");
    }

    if (epoll_count == 0 && i4FdEp_G29 != 0)
    {
        log_compnt_mngr->debug("SocketTcpServerMultiClient::readAnyClient 开始epoll_wait.");

        epoll_msg_num = epoll_wait(i4FdEp_G29, sEvent, MAXCONNECTFD, -1);
    }
    else
    {
        log_compnt_mngr->error("SysCtrl error i4FdEp_G29 {}",i4FdEp_G29);
    }

    if (epoll_msg_num <= 0)
    {
        log_compnt_mngr->error("SysCtrl error epoll_msg_num {}",epoll_msg_num);
    }
    else
    {
        for ( ; epoll_count < epoll_msg_num; epoll_count++)
        {
            (void)memset(m_buf, 0x00, D_SOCKET_RECV_BUFF_SIZE);
            int32_t m_client_connfd = sEvent[epoll_count].data.fd;
            log_compnt_mngr->debug("SocketTcpServerMultiClient::readAnyClient m_client_connfd = {}.", m_client_connfd);

            if (std::find(mClientFdVec.begin(), mClientFdVec.end(), m_client_connfd) == mClientFdVec.end())
            {
                if (m_client_connfd != m_i4_listenfd)
                {
                    log_compnt_mngr->error("wait err hd: {}.", m_client_connfd);
                    break;
                }

                getConnFd();
                break;
            }

            if (0 <= m_client_connfd)
            {
                i4_recv_size = static_cast<int32_t>(recv(m_client_connfd, m_buf, D_SOCKET_RECV_BUFF_SIZE, 0));
                if (-1 == i4_recv_size)
                {
                    auto it = std::find(mClientFdVec.begin(), mClientFdVec.end(), m_client_connfd);
                    if (it != mClientFdVec.end()) //recv反馈错误信息,删除错误套接字
                    {
                        mClientFdVec.erase(it);
                        epollDel(m_client_connfd);
                    }

                    log_compnt_mngr->error("SocketTcpServerMultiClient::readAnyClient recv nothing.");
                }
                else if (0 == i4_recv_size)
                {
                    log_compnt_mngr->error("SocketTcpServerMultiClient::readAnyClient i4_recv_size为0.");

                    (void)close(m_client_connfd); /* 被动关闭连接句柄 */

                    auto it = std::find(mClientFdVec.begin(), mClientFdVec.end(), m_client_connfd);
                    if (it != mClientFdVec.end()) //recv = 0,删除已关闭套接字
                    {
                        mClientFdVec.erase(it);
                        epollDel(m_client_connfd);
                    }

                    m_client_connfd = -1; /* EOF,end-of-file */
                    i4_recv_size = -1;
                }
                else
                {
                    log_compnt_mngr->debug("SocketTcpServerMultiClient::readAnyClient 返回收到的消息.");

                    *out_p_size = i4_recv_size;
                    *out_p_buf = m_buf;

                    std::vector<int32_t>::iterator it = find(mClientFdVec.begin(),mClientFdVec.end(), m_client_connfd);
		            if(it != mClientFdVec.end())
			            clientFdIndex = it - mClientFdVec.begin();
		            else
			            clientFdIndex = -1;

                    b_ret = true;
                    break;
                }
            }
        }
        if (epoll_count == epoll_msg_num)
        {
            log_compnt_mngr->debug("SocketTcpServerMultiClient::readAnyClient 初始化接收新的消息.");

            epoll_count = 0;
            epoll_msg_num = 0;
        }
    }

    log_compnt_mngr->info("SocketTcpServerMultiClient::readAnyClient end.");
    return b_ret;
}

/*创建epoll句柄*/
bool SocketTcpServerMultiClient::epollCreate()
{
    log_compnt_mngr->info("SocketTcpServerMultiClient::epollCreate start.");

    if (i4FdEp_G29 != 0)
    {
        (void)close(i4FdEp_G29);
    }
    i4FdEp_G29 = epoll_create(MAXCONNECTFD);

    (void)memset(&sEv, 0, sizeof(sEv));
    for (std::vector<int32_t>::iterator it = mClientFdVec.begin(); it != mClientFdVec.end(); it++)
    {
        if (*it != 0)
        {
            sEv.events = EPOLLIN | EPOLLET;
            sEv.data.fd = *it;
            int32_t i8Ret = epoll_ctl(i4FdEp_G29, EPOLL_CTL_ADD, *it, &sEv);
            if (i8Ret != 0)
            {
                log_compnt_mngr->error("SysCtrl error epollCreate i8Ret:{}.",i8Ret);
            }
        }
    }

    log_compnt_mngr->info("SocketTcpServerMultiClient::epollCreate end.");
    return true;
}

/*关闭epoll句柄*/
bool SocketTcpServerMultiClient::epollDelete()
{
    log_compnt_mngr->info("SocketTcpServerMultiClient::epollDelete start.");

    if (i4FdEp_G29 != 0)
    {
        (void)close(i4FdEp_G29);
    }

    log_compnt_mngr->info("SocketTcpServerMultiClient::epollDelete end.");
    return true;
}


/*添加epoll监听的连接句柄*/
bool SocketTcpServerMultiClient::epollAdd(int32_t epoll_listen_fd)
{
    log_compnt_mngr->info(" SocketTcpServerMultiClient::epollAdd start.");

    sEv.events = EPOLLIN | EPOLLET;
    sEv.data.fd = epoll_listen_fd;

    if (0 == i4FdEp_G29)
    {
        log_compnt_mngr->error("epoll add err.");
        return false;
    }
    
    int32_t i8Ret = epoll_ctl(i4FdEp_G29, EPOLL_CTL_ADD, epoll_listen_fd, &sEv);
    if (i8Ret != 0)
    {
        log_compnt_mngr->error("SysCtrl error epollAdd i8Ret:{}.",i8Ret);
        return false;
    }

    log_compnt_mngr->info(" SocketTcpServerMultiClient::epollAdd end.");
    return true;
}


/*删除epoll监听的连接句柄*/
bool SocketTcpServerMultiClient::epollDel(int32_t epoll_listen_fd)
{
    log_compnt_mngr->info("SocketTcpServerMultiClient::epollDel start.");

    sEv.events = EPOLLIN | EPOLLET;
    sEv.data.fd = epoll_listen_fd;
    int32_t i8Ret = epoll_ctl(i4FdEp_G29, EPOLL_CTL_DEL, epoll_listen_fd, &sEv);
    if (i8Ret != 0)
    {
        log_compnt_mngr->error(" SysCtrl error epollDel i8Ret:{}.",i8Ret);
    }

    log_compnt_mngr->info("SocketTcpServerMultiClient::epollDel end.");
    return true;
}

/*指定向某个client发送数据，isConnected返回该client是否断开连接*/
bool SocketTcpServerMultiClient::readSingleClient(int32_t clientFd, int32_t *out_p_size, char **out_p_buf, bool &isConnected)
{
    log_compnt_mngr->info("SocketTcpServerMultiClient::readSingleClient start.");
    int32_t i4_recv_size = -1;
    bool b_ret = false;
    (void)memset(m_buf, 0x00, D_SOCKET_RECV_BUFF_SIZE);

    for (auto it = mClientFdVec.begin(); it != mClientFdVec.end(); ++it)
    {
        if ((*it == clientFd) && (*it > 0))
        {
            i4_recv_size = static_cast<int32_t>(recv(clientFd, m_buf, D_SOCKET_RECV_BUFF_SIZE, 0));
            if (-1 == i4_recv_size)
            {
                if (errno == 104)
                {
                    log_compnt_mngr->error("SocketTcpServerMultiClient::readSingleClient fail.");
                    (void)close(clientFd);
                    isConnected = false;
                    mClientFdVec.erase(it);
                }
            }
            else if (0 == i4_recv_size)
            {
                log_compnt_mngr->error("SocketTcpServerMultiClient::readSingleClient fail.");
                (void)close(clientFd); /* 被动关闭连接句柄 */
                isConnected = false;
                mClientFdVec.erase(it);
            }
            else
            {
                *out_p_size = i4_recv_size;
                *out_p_buf = m_buf;
                b_ret = true;
            }
            break;
        }
    }

    log_compnt_mngr->info("SocketTcpServerMultiClient::readSingleClient end.");
    return b_ret;
}

/*设置可以连接的最大客户端数量*/
void SocketTcpServerMultiClient::setClientMaxNum(int num)
{
    clientMaxNum = num;
}


/*检查和设置 TCP 套接字的接收和发送缓冲区大小，并禁用 Nagle 算法。其目的是优化套接字的性能，确保缓冲区大小符合预期，并根据实际需求调整网络传输行为。*/
void SocketTcpServerMultiClient::checkbuff(int s)
{
    log_compnt_mngr->info("SocketTcpServerMultiClient::checkbuff start.");
    int value = D_SOCKET_RECV_BUFF_SIZE;
    int tmpCode = 0;
    int result = 0;
    socklen_t len = 4;

    if (s < 0)
    {
        log_compnt_mngr->error("socket init err");
        return;
    }

    // before
    tmpCode=::getsockopt(s, SOL_SOCKET, SO_RCVBUF, (char*)&result, &len);
    log_compnt_mngr->debug("before set socket opt: recv buf = {}, tmp code = {}.", result, tmpCode);
    tmpCode=::getsockopt(s, SOL_SOCKET, SO_SNDBUF, (char*)&result, &len);
    log_compnt_mngr->debug("before set socket opt: send buf = {}, tmp code = {}.", result, tmpCode);

    // update
    tmpCode=::setsockopt(s, SOL_SOCKET, SO_RCVBUF, (char*)&value, sizeof(value));
    tmpCode=::setsockopt(s, SOL_SOCKET, SO_SNDBUF, (char*)&value, sizeof(value));

    // after
    tmpCode=::getsockopt(s, SOL_SOCKET, SO_RCVBUF, (char*)&result, &len);
    log_compnt_mngr->debug("after set socket opt: recv buf = {}, tmp code = {}.", result, tmpCode);
    tmpCode=::getsockopt(s, SOL_SOCKET, SO_SNDBUF, (char*)&result, &len);
    log_compnt_mngr->debug("after set socket opt: send buf = {}, tmp code = {}.", result, tmpCode);

    // set nagle nodelay: 1 - on, 0 - off
    int nagle_status = 1;
    result = setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (char *) &nagle_status, sizeof(int));
    if (result < 0)
    {
        log_compnt_mngr->error("failed to change the nagle.");
    }

    log_compnt_mngr->info("SocketTcpServerMultiClient::checkbuff end.");
    return;
}

/*关闭当前连接的客户client fd*/
bool SocketTcpServerMultiClient::closeAcceptClient(int32_t accept_connfd)
{
    log_compnt_mngr->info("SocketTcpServerMultiClient::closeAcceptClient start.");
    if (std::find(mClientFdVec.begin(), mClientFdVec.end(), accept_connfd) != mClientFdVec.end())
    {
        if (close(accept_connfd) < 0 )
        {
            log_compnt_mngr->error("SocketTcpServerMultiClient::closeAcceptClient close accept fd error");
            return false;
        } 
    }
    else
    {
        log_compnt_mngr->error("SocketTcpServerMultiClient::closeAcceptClient not find accept fd ");
        return false;
    }

    log_compnt_mngr->info("SocketTcpServerMultiClient::closeAcceptClient end.");
    return true;
}
