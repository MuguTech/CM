#include <stdio.h>
#include <sys/types.h>  
#include <sys/socket.h>  
#include <sys/un.h>
#include <unistd.h>
#include <iostream>
#include <string>
#ifndef SOCKETLOCALCLIENT_H
#define SOCKETLOCALCLIENT_H

class SocketLocalClient
{
public:
    SocketLocalClient();
    bool Connect();
    bool Send(const std::string &data);
    bool Recieve(std::string* buf);
    ~SocketLocalClient();
private:
    struct sockaddr_un srv_addr;  
    int socket_fd;
    char snd_buf[1024];
    char rec_buf[1025];
};

#endif