#include "V_TrafficSocket.h"
#include "parse_ground_truth.h"
#include "ConfigureMngr.h"
#include "log.h"
#include <cstring>
#include "../../include/Runtime/coSimu/SimProType.h"
#include "./OSIGroundTruthGenerator/OSIGroundTruthGeneratorThread.h"



V_TrafficSocket *V_TrafficSocket::_pInstance = NULL;


V_TrafficSocket::V_TrafficSocket(std::string serverIp, uint16_t serverPort, std::string type)
    :SocketClient(serverIp, serverPort, type)
{
    if (sem_init(&sem, 0, 0) == -1)
    {
        log_compnt_mngr->error("V-traffic socket sem init failed .");
    }

    if (sem_init(&sem1, 0, 0) == -1)
    {
        log_compnt_mngr->error("V-traffic socket sem 1 init failed .");
    }

    memset(p_data, 0, SOCKET_BUF_SIZE);
    recvLen = 0;

    if (type == "udp" || type == "UDP")
    {
        sendIdentityInfo();
    }

    if (ConfigureMngr::getInstance()->getIsVirtualCity())
    {
        sendEgoId(); //在多主车情况下给SimPro发送CM对应的主车ID
    }

    if (ConfigureMngr::getInstance()->getIsVirtualCity())
    {
        if (sem_init(&sem_send, 0, 0) == -1)
        {
            log_compnt_mngr->error("V_TrafficSocket sem_send init failed .");
        }

        std::thread send_thread(&V_TrafficSocket::sendThread, this);    //创建发送线程
        send_thread.detach();
    }
}


V_TrafficSocket::~V_TrafficSocket()
{
  sem_destroy(&sem);
  sem_destroy(&sem1);
}


V_TrafficSocket *V_TrafficSocket::get_instance()
{
    if (NULL == _pInstance)
    {
        _pInstance = new V_TrafficSocket(ConfigureMngr::getInstance()->get_simulator_socket().ip, 
                                    atoi(ConfigureMngr::getInstance()->get_simulator_socket().port.c_str()), 
                                    ConfigureMngr::getInstance()->get_simulator_socket().type);
    }

    return _pInstance;
}

void V_TrafficSocket::recv_task()
{
    // epoll 监听socket fd
    recv_task_epoll();
    return;
}


void V_TrafficSocket::recv_task_epoll()
{
    struct sockaddr_in s = get_server_addr();
    unsigned int sin_size = sizeof(struct sockaddr_in);
    size_t min_len_data = 0;

    if (i4FdEp == 0)
    {
        log_compnt_mngr->error("epoll create err.");
        return;
    }

    log_compnt_mngr->debug("get_conn_stat() is {}", get_conn_stat());
    log_compnt_mngr->critical("V_TrafficSocket::recv_task_epoll start");
    while (true == get_conn_stat())
    {
        log_compnt_mngr->debug("epoll wait v-traffic data");

        struct epoll_event sEvent[MAXCLIETFD];
        int msg_num = epoll_wait(i4FdEp, sEvent, MAXCLIETFD, -1);

        log_compnt_mngr->debug("epoll wait event num = {} ", msg_num);

        for (int i = 0; i < msg_num; i++)
        {
            int fd = sEvent[i].data.fd;
            if (fd != get_client())
            {
                log_compnt_mngr->error("fd {} not is client {}.", fd, get_client());
                continue;
            }

            if (get_type() == "tcp" || get_type() == "TCP")
            {
                // TCP 粘包解决
                ssize_t l = recv(fd, p_data + recvLen, SOCKET_BUF_SIZE - recvLen, 0);
                if (l <= 0)
                {
                    // socket 连接中断, windows recv()返回 -1, Linux recv() 返回 0
                    log_compnt_mngr->error("fd {} recv len = {} error", fd, l);
                    log_compnt_mngr->warn("fd {} dis connect and restart cm repair it", fd);
                    set_conn_stat(false);
                    break;
                }
                log_compnt_mngr->debug("fd {} recv len = {}", fd, l);
                recvLen += l;
            }
            else if (get_type() == "udp" || get_type() == "UDP")
            {
                ssize_t len = recvfrom(fd, p_data + recvLen, SOCKET_BUF_SIZE , 0, (struct sockaddr *)&s, &sin_size);
                if (len <= 0)
                {
                    log_compnt_mngr->error("fd {} recv len = {} error", fd, len);
                    set_conn_stat(false);
                }
                recvLen += len;
            }
            else
            {
                log_compnt_mngr->error("v-traffic socket type {} err.", get_type().c_str());
                break;
            }

            if (recvLen == 0)
            {
                log_compnt_mngr->warn("recv v-traffic data is 0.");
                break;
            }

            if (recvLen > SOCKET_BUF_SIZE)
            {
                log_compnt_mngr->warn("recv v-traffic data buff is no free space.");
                break;
            }

            log_compnt_mngr->debug("recv v-traffic len = {}.", recvLen);
            log_compnt_mngr->debug("timestamp recv from simpro {} len = {} ", get_cur_time_ms(), recvLen);

            // 解决 TCP 粘包问题
            if (true == recv_pkg_finished(min_len_data))
            {
                // 把指定的信号量 sem 的值加 1，通知主线程
                log_compnt_mngr->debug("recv a complete package.");
                sem_post(&sem);

                log_compnt_mngr->debug("wait comsume complete package.");
                // 等待主线程使用 p_data
                if (sem_wait(&sem1) == -1)
                {
                    log_compnt_mngr->error("v-traffic sem_wait 1 failed!");
                    break;
                }

                log_compnt_mngr->debug("recv from simpro data len {}, one frame data len {}"
                            , recvLen, min_len_data);

                // 异步通信：消费线程每次仅消费一帧数据
                memset(p_data, 0, min_len_data); // SOCKET_BUF_SIZE
                recvLen -= min_len_data;

                if (recvLen > 0)
                {
                    log_compnt_mngr->debug("recv from simpro data len {} greate then one frame data len {}"
                                , recvLen + min_len_data, min_len_data);
                    memmove(p_data, p_data + min_len_data, recvLen);
                    memset(p_data + recvLen, 0, min_len_data);
                }
            }
        }   // for (int i = 0; i < msg_num; i++)

        // 缓存内存保存
        if (recvLen > SOCKET_BUF_SIZE)
        {
            log_compnt_mngr->warn("recv v-traffic data buff is no free space.");
            break;
        }
    }   // while

    log_compnt_mngr->critical("V_TrafficSocket::recv_task_epoll end.");

    return;
}


void V_TrafficSocket::sempost()
{
    sem_post(&sem1);
}


bool V_TrafficSocket::get_recv_data(void **pp_msg, size_t &len)
{
    log_compnt_mngr->info("V_TrafficSocket::get_recv_data start.");
    if (false == get_conn_stat())
    {
        log_compnt_mngr->error("v-traffic socket conn err.");
        return false;
    }

    log_compnt_mngr->debug("wait v-traffic data:");

    if (sem_wait(&sem) == -1)
    {
        log_compnt_mngr->error("v-traffic sem_wait failed!");
        return false;
    }

    *pp_msg = p_data;
    len = recvLen;

    log_compnt_mngr->debug("V_TrafficSocket get_recv_data recvlen = {}.",len);

    log_compnt_mngr->info("V_TrafficSocket::get_recv_data end.");
    return true;
}


bool V_TrafficSocket::recv_pkg_finished(size_t & min_len)
{
    parse_ground_truth pgt(p_data, recvLen);

    if (true != pgt.is_complete_pkg())
    {
        log_compnt_mngr->error("is not complete pkg");
        return false;
    }

    pgt.get_len_frame_data(min_len);
    return true;
}

//在UDP模式下告知simpro身份，使得simpro能够将数据发给cm模块
void V_TrafficSocket::sendIdentityInfo()
{
    log_compnt_mngr->info("V_TrafficSocket::sendIdentityInfo start.");
    memset(p_data, 0, SOCKET_BUF_SIZE);
    int usedSize = 0;

    //生成Msg头部
    S_SP_MSG_HDR *msgHead = (S_SP_MSG_HDR *)p_data;

    //填充Msg头部
    msgHead->u8SimTime = 0.0;
    msgHead->u4HeaderSize = sizeof(S_SP_MSG_HDR);
    msgHead->u4FrameNo = 0;
    msgHead->u1Sender = D_SP_SENDER_COMPONENT_MANAGER;
    usedSize += sizeof(S_SP_MSG_HDR);

    //生成D_SP_PKG_ID_START_FRAME
    S_SP_MSG_ENTRY_HDR *pkgHead = (S_SP_MSG_ENTRY_HDR *)(p_data + usedSize);

    //填充PKG头部
    pkgHead->u4HeaderSize = sizeof(S_SP_MSG_ENTRY_HDR);
    pkgHead->u4DataSize = 0;
    pkgHead->u4ElementSize = 0;
    pkgHead->u2PkgId = D_SP_PKG_ID_START_FRAME;

    usedSize += pkgHead->u4HeaderSize + pkgHead->u4DataSize;

    //生成D_SP_PKG_ID_END_FRAME
    S_SP_MSG_ENTRY_HDR* pkgEnd = (S_SP_MSG_ENTRY_HDR *)(p_data + usedSize);

    //填充PKG头部
    pkgEnd->u4HeaderSize = sizeof(S_SP_MSG_ENTRY_HDR);
    pkgEnd->u4DataSize = 0;
    pkgEnd->u4ElementSize = 0;
    pkgEnd->u2PkgId = D_SP_PKG_ID_END_FRAME;

    usedSize += pkgEnd->u4HeaderSize + pkgEnd->u4DataSize;

    msgHead->u4DataSize = usedSize - msgHead->u4HeaderSize;

    send_msg(p_data, usedSize);

    memset(p_data, 0, SOCKET_BUF_SIZE);
    log_compnt_mngr->info("V_TrafficSocket::sendIdentityInfo end.");
}

void V_TrafficSocket::sendEgoId()
{
    log_compnt_mngr->info("V_TrafficSocket::sendEgoId start.");
    memset(p_data, 0, SOCKET_BUF_SIZE);
    int usedSize = 0;

    //生成Msg头部
    S_SP_MSG_HDR *msgHead = (S_SP_MSG_HDR *)p_data;

    //填充Msg头部
    msgHead->u8SimTime = 0.0;
    msgHead->u4HeaderSize = sizeof(S_SP_MSG_HDR);
    msgHead->u4FrameNo = 1;
    msgHead->u1Sender = D_SP_SENDER_COMPONENT_MANAGER;
    usedSize += sizeof(S_SP_MSG_HDR);

    //生成D_SP_PKG_ID_START_FRAME
    S_SP_MSG_ENTRY_HDR *pkgHead = (S_SP_MSG_ENTRY_HDR *)(p_data + usedSize);

    //填充PKG头部
    pkgHead->u4HeaderSize = sizeof(S_SP_MSG_ENTRY_HDR);
    pkgHead->u4DataSize = 0;
    pkgHead->u4ElementSize = 0;
    pkgHead->u2PkgId = D_SP_PKG_ID_START_FRAME;

    usedSize += pkgHead->u4HeaderSize + pkgHead->u4DataSize;

    //生成D_SP_PKG_ID_EGO_ID
    pkgHead = (S_SP_MSG_ENTRY_HDR *)(p_data + usedSize);
    pkgHead->u4HeaderSize = sizeof(S_SP_MSG_ENTRY_HDR);
    pkgHead->u4DataSize = sizeof(S_SP_EGO_ID);
    pkgHead->u4ElementSize = sizeof(S_SP_EGO_ID);
    pkgHead->u2PkgId = D_SP_PKG_ID_EGO_ID;

    S_SP_EGO_ID *pkgBody = (S_SP_EGO_ID *)(pkgHead + 1);
    const char *envValue = std::getenv("MAIN_VEHICLE_ID");
    if (envValue != NULL)
    {
        pkgBody->u4EgoId = std::stoi(envValue);
    }
    else
    {
        pkgBody->u4EgoId = 0;
    }
    log_compnt_mngr->debug("sendEgoId id = {}", pkgBody->u4EgoId);
    usedSize += pkgHead->u4HeaderSize + pkgHead->u4DataSize;

    //生成D_SP_PKG_ID_END_FRAME
    S_SP_MSG_ENTRY_HDR* pkgEnd = (S_SP_MSG_ENTRY_HDR *)(p_data + usedSize);

    //填充PKG头部
    pkgEnd->u4HeaderSize = sizeof(S_SP_MSG_ENTRY_HDR);
    pkgEnd->u4DataSize = 0;
    pkgEnd->u4ElementSize = 0;
    pkgEnd->u2PkgId = D_SP_PKG_ID_END_FRAME;

    usedSize += pkgEnd->u4HeaderSize + pkgEnd->u4DataSize;

    msgHead->u4DataSize = usedSize - msgHead->u4HeaderSize;

    send_msg(p_data, usedSize);

    memset(p_data, 0, SOCKET_BUF_SIZE);
    log_compnt_mngr->info("V_TrafficSocket::sendEgoId end.");
}

void V_TrafficSocket::sem_send_post()
{
    sem_post(&sem_send);
}

void V_TrafficSocket::sendThread()
{
    log_compnt_mngr->critical("V_TrafficSocket::sendThread start.");
    while (true)
    {
        int ret = sem_wait(&sem_send); //不带等待时间，一直阻塞
        if (ret == -1) //失败
        {
            int errNum = errno;
            log_compnt_mngr->error("V_TrafficSocket::sendThread() sem_wait error, errno={}, reason={}", errNum, strerror(errNum));
        }

        bool bRet = false;

        COSIMU_DATA data = {0};

        while (getSendMsgToSimpro().msgList.size() != 0)
        {
            getSendMsgToSimpro().msgMutex.lock();
            const auto &it = getSendMsgToSimpro().msgList.begin();
            data = *it;
            getSendMsgToSimpro().msgList.pop_front();
            getSendMsgToSimpro().msgMutex.unlock();

            bRet = send_msg(data.msg, data.msgLen); //发送数据给simpro

            if (!bRet)
            {
                log_compnt_mngr->error("V_TrafficSocket::sendThread(), bRet={}", bRet);
            }
        }
    }
    log_compnt_mngr->critical("V_TrafficSocket::sendThread end.");
}
