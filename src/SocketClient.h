#ifndef __SOCKET_CLIENT__
#define __SOCKET_CLIENT__


#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <thread>
#include <sys/epoll.h>

#include "log.h"

#define SOCKET_BUF_SIZE        (2 * 1024 * 1024)

#define RETRY_OVER_TIME         3

#define MAXCLIETFD              5




class SocketClient
{
private:
    std::string server_ip;
    uint16_t server_port;
    std::string type;
    struct sockaddr_in server_addr;
    int client;
    bool connect_stat;
    std::thread recv_thread_handle;

public:
    SocketClient(std::string serverIp, uint16_t serverPort, std::string type = "tcp");
    ~SocketClient();
    bool send_msg(char *msg, size_t len);
    void init_thread();

protected:
    int32_t i4FdEp;
    virtual void recv_task() {
      log_compnt_mngr->info("in base client recv task.");
    }
    bool get_conn_stat(){return connect_stat;}
    void set_conn_stat(bool is){connect_stat = is;}
    int get_client(){return client;}
    struct sockaddr_in get_server_addr(){return server_addr;}
    std::string get_type(){return type;}

private:
    void init_epoll();
    void init_socket();
    static void thread_func(SocketClient *p);
    bool epollAdd(int fd);
    bool epollDel(int fd);
    void checkbuff(int s = -1);

};





#endif // __SOCKET_CLIENT__

