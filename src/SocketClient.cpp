#include "SocketClient.h"
#include "log.h"
#include <string.h>
#include <arpa/inet.h>


SocketClient::SocketClient(std::string serverIp, uint16_t serverPort, std::string type)
{
    server_ip = serverIp; 
    server_port = serverPort;
    this->type = type;

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(serverIp.c_str());
    server_addr.sin_port = htons(serverPort);

    client = -1;
    connect_stat = false;

    i4FdEp = 0;

    init_epoll();
    init_socket();
}



SocketClient::~SocketClient()
{
    recv_thread_handle.join();
    connect_stat = false;
    close(client);

    if (0 != i4FdEp)
    {
        close(i4FdEp);
    }
}


void SocketClient::init_epoll()
{
    i4FdEp = epoll_create(MAXCLIETFD);
    if (0 == i4FdEp)
    {
        log_compnt_mngr->error("init epoll error");
    }
}


/* https://zhuanlan.zhihu.com/p/21374980 */
bool SocketClient::epollAdd(int fd)
{
    struct epoll_event sEv = {0, {0}};

    // sEv.events = EPOLLIN | EPOLLET;                  // 有消息时触发
    sEv.events = EPOLLIN;                               // 有数据时触发
    sEv.data.fd = fd;

    if (0 == i4FdEp)
    {
        log_compnt_mngr->error("epoll add err");
        return false;
    }

    int ret = epoll_ctl(i4FdEp, EPOLL_CTL_ADD, fd, &sEv);
    if (ret != 0)
    {
        log_compnt_mngr->error("epoll add err ret {}.", ret);
        return false;
    }

    return true;
}


bool SocketClient::epollDel(int fd)
{
    struct epoll_event sEv = {0, {0}};

    // sEv.events = EPOLLIN | EPOLLET;
    sEv.events = EPOLLIN;
    sEv.data.fd = fd;

    if (0 == i4FdEp)
    {
        log_compnt_mngr->error("epoll delete err");
        return false;
    }

    int ret = epoll_ctl(i4FdEp, EPOLL_CTL_DEL, fd, &sEv);
    if (0 != ret)
    {
        log_compnt_mngr->error("epoll delete err ret {}", ret);
        return false;
    }

    return true;
}


void SocketClient::init_socket()
{
    log_compnt_mngr->info("SocketClient::init_socket start.");
    if (type == "tcp" || type == "TCP")
    {
        if ((client = socket(AF_INET, SOCK_STREAM, 0)) < 0)
        {
            log_compnt_mngr->error("socket error");
            return;
        }

        // 仅 TCP 特殊处理
        checkbuff(client);

        if (connect(client, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) 
        {
            log_compnt_mngr->error("CONNECT IP {}, PORT {} BY TCP ERROR.", 
                    inet_ntoa(server_addr.sin_addr), ntohs(server_addr.sin_port));
            return;
        }
    }
    else if (type == "udp" || type == "UDP")
    {
        if ((client = socket(PF_INET, SOCK_DGRAM, 0)) < 0)
        {
            log_compnt_mngr->error("socket error");
            return;
        }
    }
    else
    {
        log_compnt_mngr->error("socket type {} err.", type.c_str());
        return;
    }

    log_compnt_mngr->debug("CONNECT IP {}, PORT {} TYPE {} OK.",
            inet_ntoa(server_addr.sin_addr), 
            ntohs(server_addr.sin_port), type.c_str());
    connect_stat = true;

    epollAdd(client);
    log_compnt_mngr->info("SocketClient::init_socket end.");
    return;
}


void SocketClient::init_thread()
{
    recv_thread_handle = std::thread(thread_func, this);
}


void SocketClient::thread_func(SocketClient *p)
{
    p->recv_task();
}


bool SocketClient::send_msg(char *msg, size_t len)
{
    log_compnt_mngr->info("SocketClient::send_msg start.");
    int i = 0;
    char data[SOCKET_BUF_SIZE] = {0};
    ssize_t l = 0;

    while (false == connect_stat && i++ < RETRY_OVER_TIME)
    {
        log_compnt_mngr->debug("wait for conn to server.");
        sleep(2);
    }

    if (false == connect_stat)
    {
        log_compnt_mngr->error("connect to server falie.");
        return false;
    }

    if (NULL == msg || 0 == len)
    {
        log_compnt_mngr->error("input err.");
        return false;
    }

    if (type == "tcp" || type == "TCP")
    {
        if ((l = send(client, msg, len, 0)) < 0)
        {
            log_compnt_mngr->error("send error!");
            return false;
        }
    }
    else if (type == "udp" || type == "UDP")
    {
        if ((l = sendto(client, msg, len, 0, (struct sockaddr *)&server_addr, sizeof(struct sockaddr))) < 0)
        {
            log_compnt_mngr->error("send error!");
            return false;
        }
    }
    else
    {
        log_compnt_mngr->error("err type {}.", type.c_str());
        return false;
    }

    log_compnt_mngr->debug("send to v-traffic {} ok.", l);
    log_compnt_mngr->info("SocketClient::send_msg end.");
    return true;
}


void SocketClient::checkbuff(int s)
{
    log_compnt_mngr->info("SocketClient::checkbuff start.");
    int value = SOCKET_BUF_SIZE;
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

    log_compnt_mngr->info("SocketClient::checkbuff end.");
    return;
}
