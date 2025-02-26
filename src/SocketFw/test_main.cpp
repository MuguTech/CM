
#include "SocketTcpServerMultiClient.h"
#include <thread>
#include <unistd.h>


static SocketTcpServerMultiClient *g_sock_obj_master[100] = {NULL};

// 通信线程句柄2: 作为服务端监听 AD 连接
std::vector<std::thread> socket_adadapter;

void recvThread(int id);
void sendThread();

int main()
{
    std::vector<uint16_t> in_u2_port = {32001, 32002, 32003};

    for (size_t i = 0; i < in_u2_port.size(); i++)
    {
        g_sock_obj_master[i] = new SocketTcpServerMultiClient();
        g_sock_obj_master[i]->epollCreate();

        // 两种方式启动 server 
        int32_t i4ServeFd = g_sock_obj_master[i]->getListenFd(in_u2_port[i], false, true, "127.0.0.1");
        int32_t i4ClidFd = g_sock_obj_master[i]->getConnFd();
        // bool bret = g_sock_obj_master[i]->open(in_u2_port[i]);

        socket_adadapter.push_back(std::thread(recvThread, i));
    }

    socket_adadapter.push_back(std::thread(sendThread));

    for (size_t i = 0; i < in_u2_port.size(); i++)
    {
        socket_adadapter[i].join();

        g_sock_obj_master[i]->closed();
        g_sock_obj_master[i]->epollDelete();

        delete g_sock_obj_master[i];
        g_sock_obj_master[i] = NULL;
    }

    return 0;
}


void sendThread()
{
    char data_tmp[100] = {0};

    while (true)
    {
        sleep(5);

        for (size_t i = 0; i < 3; i++)
        {
            int32_t in_p_size = 100;
            sleep(1);
            sprintf(data_tmp, "send data: %ld.", i + 10);
            g_sock_obj_master[i]->writeSingleClient(0, in_p_size, data_tmp);
            printf("send %ld over\n", i);
        }
    }
}


void recvThread(int id)
{
    int clientFdIndex = -1;     // client端序号 从0开始
    int32_t out_p_size = 0;     // 大小
    char *recvBuf = NULL;       // buffer

    printf("start recv thread %d ok ..\n", id);

    while (true)
    {
        if (NULL == g_sock_obj_master[id])
        {
            sleep(1);
            printf("socket master is not init..\n");
            continue;
        }

        const bool ret = g_sock_obj_master[id]->readAnyClient(clientFdIndex, &out_p_size, &recvBuf);

        printf("recv cli idx = %d, size = %d\n", clientFdIndex, out_p_size);
    }

    return;
}

