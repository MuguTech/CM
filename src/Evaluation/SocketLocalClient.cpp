#include<SocketLocalClient.h>
#include "../log.h"
#define MYSQLCONN "/tmp/MYSQLCONN"

SocketLocalClient::SocketLocalClient()
{

}

bool SocketLocalClient::Connect()
{
    // 连接到服务器  
    socket_fd=socket(PF_UNIX,SOCK_STREAM,0);  
    srv_addr.sun_family=AF_UNIX;  
    strncpy(srv_addr.sun_path,MYSQLCONN,sizeof(MYSQLCONN));  
    int ret = connect(socket_fd,(struct sockaddr*)&srv_addr,sizeof(srv_addr));
    log_evaluation->info("SocketLocalClient::Connect ret= {}",ret);
    if(ret < 0)
    {
        return false;
    }
    return true;
}

bool SocketLocalClient::Send(const std::string &data)
{
    //TODO 包头插入长度
    int datalen = data.length();
    char strdatalen[9] = "";
    snprintf(strdatalen,9,"%08d",datalen);
    std::string senddata = std::string(strdatalen)+data;

    log_evaluation->info("SocketLocalClient::Send data= {}",senddata);
    int ret = send(socket_fd,senddata.c_str(),senddata.size(),0);
    log_evaluation->info("SocketLocalClient::Send ret= {}",ret);
    if(ret < 0)
    {
        log_evaluation->error("SocketLocalClient::Send failed");
        return false;
    }

    log_evaluation->info("SocketLocalClient::Send success");
    return true;
}

bool SocketLocalClient::Recieve(std::string* buf)
{
    memset(rec_buf,0,1025);
    int ret = recv(socket_fd,rec_buf,8,0);
    //std::cout<<"SocketLocalClient::Recieve rec_buf="<<rec_buf<<std::endl;
    if(ret < 0)
    {
        //发生异常
        log_evaluation->error("SocketLocalClient::Recieve 服务器发生异常ret= {}",ret);
        return false;
    }
    else if(ret == 0)
    {
        //服务器断开连接
        log_evaluation->error("SocketLocalClient::Recieve 服务器断开连接");
        return false;
    }
    int datalen = atoi(rec_buf);
    buf->assign("");
    while(datalen > 1024)
    {
        memset(rec_buf,0,1025);
        ret = recv(socket_fd,rec_buf,1024,0);
        //std::cout<<"SocketLocalClient::Recieve2 rec_buf="<<rec_buf<<std::endl;
        datalen -= ret;
        *buf += std::string(rec_buf);
    }
    memset(rec_buf,0,1025);
    ret = recv(socket_fd,rec_buf,datalen,0);
    //std::cout<<"SocketLocalClient::Recieve3 rec_buf="<<rec_buf<<std::endl;
    *buf += std::string(rec_buf);
    
    log_evaluation->info("SocketLocalClient::Recieve datalen= {}",datalen);
    log_evaluation->info("SocketLocalClient::Recieve : {}",*buf);
    return true;
}

SocketLocalClient::~SocketLocalClient()
{
    close(socket_fd);
}
