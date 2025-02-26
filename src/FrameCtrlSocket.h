#ifndef __FRAME_CTRL_SOCKET_H__
#define __FRAME_CTRL_SOCKET_H__

#include "SocketClient.h"
#include <memory>
#include <semaphore.h>

#include "FrameCtrlMsgParser.h"

class FrameCtrlSocket: public SocketClient
{
private:
    static FrameCtrlSocket *_pInstance;
    char p_data[SOCKET_BUF_SIZE];
    ssize_t recvLen;
    sem_t sem;      // 通知是否recv data over

    sem_t sem_resp; // 通知 frame ctrl socket data接收成功

    // send data buffer
    char* m_msgBuffer = nullptr;
    unsigned int m_msgBufferUsedSize = 0;
    int m_msgBufferAllocSize  = 1024 * 40;

    std::unique_ptr<FrameCtrlMsgParser> m_frame_ctrl_msg_parser;

public:
    ~FrameCtrlSocket();
    static FrameCtrlSocket *get_instance();
    bool get_recv_data();
    void send_init_data();
    void post_sem_resp();
    void send_frame_ctrl_resp_msg(
        bool is_consumed_data,
        const std::map<std::string, bool> &sensor_running_map,
        const std::map<std::string, int> &sensor_frame_map);
    void send_stop_resp_msg();

    // 处理一帧 system control 发过来的消息
    void processOneMsg();

  protected:
    void recv_task();

private:
    FrameCtrlSocket(std::string serverIp, uint16_t serverPort, std::string type = "tcp");


};



#endif  // __FRAME_CTRL_SOCKET_H__

