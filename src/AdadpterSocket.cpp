#include "AdadpterSocket.h"
#include "ConfigureMngr.h"
#include "log.h"
#include <thread>

#include "common.h"
#include <atomic> 

#include "V_TrafficSocket.h"
#include "parse_ground_truth.h"
#include "PackMsg.h"

#define RECV_BUFFER_SIZE (4096)
#define UDP_MAX_BUFFER_SIZE (65507)

AdadpterSocket *AdadpterSocket::_pInstance = NULL;


AdadpterSocket::AdadpterSocket()
{
    if (ConfigureMngr::getInstance()->getIsVirtualCity())
    {
        bufInfo = {0};

        if (sem_init(&sem_consume, 0, 0) == -1)
        {
            log_compnt_mngr->error("AdadpterSocket sem_consume init failed .");
        }

        std::thread consume_thread(&AdadpterSocket::consumeThread, this);    //创建消费线程
        consume_thread.detach();
    }
}


AdadpterSocket::~AdadpterSocket()
{
    //TCP 资源回收
    for (auto& iter : g_sock_obj_server)
    {
        if (iter.second != NULL)
        {
            iter.second->closed();
            iter.second->epollDelete();

            delete iter.second;
            iter.second = NULL;
        }
    }

    //UDP 资源回收
    for (auto& iter : g_sock_udp_server)
    {
        if (iter.second > 0)
        {
            close(iter.second);
        }
    }

    //释放每个传感器的接收线程
    for (auto& iter : v_socket_adadapter)
    {
        if (iter.second.joinable())
        {
            iter.second.join();
        }
    }

    if (ConfigureMngr::getInstance()->getIsVirtualCity())
    {
        if (bufInfo.buf != NULL)
        {
            free(bufInfo.buf);
            bufInfo.buf = NULL;
        }
    }
}


AdadpterSocket *AdadpterSocket::get_instance()
{
    if (NULL == _pInstance)
    {
        _pInstance = new AdadpterSocket();
    }

    return _pInstance;    
}


void AdadpterSocket::thread_func(AdadpterSocket *p, const std::string &sensor_name, const std::string &sensor_id, const int &socketSerfd)
{
    p->recvAdThread(sensor_name, sensor_id, socketSerfd);
}

void AdadpterSocket::recvAdThread(const std::string &sensor_name, const std::string &sensor_id, const int &socketSerfd)
{
    log_compnt_mngr->info("AdadpterSocket::recvAdThread {} start.",sensor_name);
    log_compnt_mngr->debug("AdadpterSocket::recvAdThread m_socket_type sensor_name = {},sensor_id={}, socketfd={}",sensor_name,sensor_id,socketSerfd);
    if (m_socket_type.find(sensor_id) == m_socket_type.end())
    {
        return;
    }

    int32_t i4FdEp = epoll_create(MAX_SENSOR_PLUGIN_NUM);
    if (i4FdEp < 0)
    {
        log_compnt_mngr->error("AdadpterSocket::recvAdThread epoll_create,i4FdEp=,{}", i4FdEp);
    }
    m_epoll_fd[sensor_id] = i4FdEp;
    struct epoll_event sEv = {0};
    (void)memset(&sEv, 0, sizeof(sEv));
    sEv.events = EPOLLIN;
    sEv.data.fd = socketSerfd;
    int32_t i8Len = epoll_ctl(i4FdEp, EPOLL_CTL_ADD, socketSerfd, &sEv);
    if (i8Len == -1)
    {
        log_compnt_mngr->error("AdadpterSocket::recvAdThread,epoll add,i8Len=,{}", i8Len);
    }
    struct epoll_event sEvent[MAX_SENSOR_PLUGIN_NUM] = {0};

    //map中传感器id对应传感器的socket类型,1：TCP 2：UDP
    if (m_socket_type[sensor_id] == 1)
    {
        if (g_sock_obj_server[sensor_id] == NULL)
        {
            log_compnt_mngr->error("socket tcp server {} is not init.", sensor_id);
            return; //指针为空
        }

        int clientFdIndex = -1;     // client端序号 从0开始
        int32_t out_p_size = 0;     // 大小
        char *recvBuf = NULL;       // buffer

        log_compnt_mngr->debug("start recv thread for {} ok.", sensor_name.c_str());

        while (g_thread_run_flag[sensor_id].load())
        {
            int32_t fdNEp = epoll_wait(i4FdEp, sEvent, MAX_SENSOR_PLUGIN_NUM, -1);
            if (fdNEp <= 0)
            {
                log_compnt_mngr->error("AdadpterSocket::recvAdThread,fdNEp=,{}", fdNEp);
            }
            else
            {
                for (int32_t i4Loop = 0; i4Loop < fdNEp; i4Loop++)
                {
                    if (sEvent[i4Loop].data.fd == socketSerfd)
                    {
                        int32_t acceptFd = g_sock_obj_server[sensor_id]->getConnFd(); //accept等待接收连接
                        if (acceptFd > 0)
                        {
                            (void)memset(&sEv, 0, sizeof(sEv));
                            sEv.events = EPOLLIN | EPOLLET;
                            sEv.data.fd = acceptFd;
                            i8Len = epoll_ctl(i4FdEp, EPOLL_CTL_ADD, acceptFd, &sEv);
                            if (i8Len != 0)
                            {
                                log_compnt_mngr->error("AdadpterSocket::recvAdThread,epoll add,i8Len=,{}", i8Len);
                            }
                        }
                    }
                    else
                    {
                        int32_t recvFd = sEvent[i4Loop].data.fd;
                        const bool ret = g_sock_obj_server[sensor_id]->readAnyClient(clientFdIndex, &out_p_size, &recvBuf);
                        if (ret == false) //通信断开连接，epoll删除监听句柄
                        {
                            (void)memset(&sEv, 0, sizeof(sEv));
                            sEv.events = EPOLLIN | EPOLLET;
                            sEv.data.fd = recvFd;
                            i8Len = epoll_ctl(i4FdEp, EPOLL_CTL_DEL, recvFd, &sEv);
                            if (i8Len != 0)
                            {
                                log_compnt_mngr->error("Create_TCP_service,epoll del,i8Len=,{}", i8Len);
                            }
                            g_sock_obj_server[sensor_id]->closeAcceptClient(recvFd);
                        }
                        else
                        {
                            if (ConfigureMngr::getInstance()->getIsVirtualCity())
                            {
                                COSIMU_DATA recvData = {0};
                                recvData.msg = new char[MSG_BUFF_SIZE];
                                if (NULL == recvData.msg)
                                {
                                    log_compnt_mngr->error("AdadpterSocket::recvAdThread err for malloc.");
                                }
                                else
                                {
                                    memcpy(recvData.msg, recvBuf, out_p_size);
                                    recvData.msgLen = (size_t)out_p_size;

                                    getRecvMsgFromAd().msgMutex.lock();
                                    getRecvMsgFromAd().msgList.push_back(recvData);
                                    getRecvMsgFromAd().msgMutex.unlock();

                                    sem_post(&sem_consume); //通知消费线程
                                }
                            }
                        }
                        log_compnt_mngr->debug("recv cli tcp idx = {}, size = {}.", clientFdIndex, out_p_size);
                    }
                }
            }
        }
    }
    //map中传感器id对应传感器的socket类型,1：TCP 2：UDP
    else if (m_socket_type[sensor_id] == 2)
    {
        if (g_sock_udp_server[sensor_id] <= 0)
        {
            return;
        }
        struct sockaddr_in cliAddr;
        (void)memset(&cliAddr, 0, sizeof(cliAddr));
        socklen_t addrLen = sizeof(cliAddr);
        char buf[RECV_BUFFER_SIZE] = {0};

        while (g_thread_run_flag[sensor_id].load())
        {
            int32_t fdNEp = epoll_wait(i4FdEp, sEvent, MAX_SENSOR_PLUGIN_NUM, -1);
            if (fdNEp <= 0)
            {
                log_compnt_mngr->error("Create_UDP_service,fdNEp=,{}", fdNEp);
            }
            else
            {
                (void)memset(buf, 0, RECV_BUFFER_SIZE);
                int recvSize = static_cast<int>(recvfrom(g_sock_udp_server[sensor_id], buf, RECV_BUFFER_SIZE - 1, 0, reinterpret_cast<struct sockaddr *>(&cliAddr), &addrLen));
                if (recvSize > 0)
                {
                    if (cliAddr.sin_port != 0)
                    {
                        m_udp_client_addr[g_sock_udp_server[sensor_id]] = cliAddr;
                        log_compnt_mngr->debug("recv cli udp port = {}, size = {}.", cliAddr.sin_port, recvSize);
                    }
                    if (ConfigureMngr::getInstance()->getIsVirtualCity())
                    {
                        COSIMU_DATA recvData = {0};
                        recvData.msg = new char[MSG_BUFF_SIZE];
                        if (NULL == recvData.msg)
                        {
                            log_compnt_mngr->error("AdadpterSocket::recvAdThread udp err for malloc.");
                        }
                        else
                        {
                            memcpy(recvData.msg, buf, recvSize);
                            recvData.msgLen = (size_t)recvSize;

                            getRecvMsgFromAd().msgMutex.lock();
                            getRecvMsgFromAd().msgList.push_back(recvData);
                            getRecvMsgFromAd().msgMutex.unlock();

                            sem_post(&sem_consume); //通知消费线程
                        }
                    }
                }
            }
        }
    }
    if (close(i4FdEp) < 0)
    {
        log_compnt_mngr->error("AdadpterSocket::recvAdThread,epoll close fd {} error",i4FdEp);
    }
    log_compnt_mngr->debug("exit recive thread sensor id is = {}",sensor_id);
    log_compnt_mngr->info("AdadpterSocket::recvAdThread {} end.",sensor_name);

}


// 给 client 发送消息
bool AdadpterSocket::send_msg(const std::string sensor_id, void *p_msg, int32_t &len)
{
    log_compnt_mngr->info("AdadpterSocket::send_msg start.");
    log_compnt_mngr->debug("AdadpterSocket::send_msg sensor_id is = {},msg len is = {}",sensor_id,len);
    if (NULL == p_msg || 0 == len)
    {
        log_compnt_mngr->error("input err.");
        return false;
    }

    if (m_socket_type[sensor_id] == 1) //TCP
    {
        if (NULL == g_sock_obj_server[sensor_id])
        {
            log_compnt_mngr->error("socket for sensor {} is null.", sensor_id.c_str());
            return false;
        }

        if(true != g_sock_obj_server[sensor_id]->writeSingleClient(0, len, (char *)p_msg))
        {
            log_compnt_mngr->error("sensor {} send to ad err.", sensor_id.c_str());
            return false;
        }
    }
    else if (m_socket_type[sensor_id] == 2) //UDP
    {
        if ((g_sock_udp_server[sensor_id] <= 0) || (m_udp_client_addr.find(g_sock_udp_server[sensor_id]) == m_udp_client_addr.end()))
        {
            log_compnt_mngr->error("udp socket for sensor {} is null.", sensor_id.c_str());
            return false; 
        }

        struct sockaddr_in cliAddr = m_udp_client_addr[g_sock_udp_server[sensor_id]];
        int sendLen = len;
        char *msg = (char *)p_msg;
        while (sendLen > UDP_MAX_BUFFER_SIZE)
        {
            int ret = static_cast<int>(sendto(g_sock_udp_server[sensor_id], msg, UDP_MAX_BUFFER_SIZE, 0, reinterpret_cast<struct sockaddr *>(&cliAddr), sizeof(struct sockaddr)));
            msg += UDP_MAX_BUFFER_SIZE;
            sendLen -= UDP_MAX_BUFFER_SIZE;
            if (ret < 0)
            {
                log_compnt_mngr->error("sensor {} udp send to ad err.", sensor_id.c_str());
                return false;
            }
        }
        int ret = static_cast<int>(sendto(g_sock_udp_server[sensor_id], msg, sendLen, 0, reinterpret_cast<struct sockaddr *>(&cliAddr), sizeof(struct sockaddr)));
        if (ret < 0)
        {
            log_compnt_mngr->error("sensor {} udp send to ad err.", sensor_id.c_str());
            return false;
        }


    }

    log_compnt_mngr->debug("{} SEND TO AD LENGTH {} OK.", sensor_id.c_str(), len);
    log_compnt_mngr->info("AdadpterSocket::send_msg end.");
    return true;
}

//创建socket 服务端
void AdadpterSocket::createSocketServer()
{
    log_compnt_mngr->info("AdadpterSocket::createSocketServer start.");
    //获取CM传感器的控制方式，1:一般配置文件模式 2:HMI模式
    if (ConfigureMngr::getInstance()->getSensorControllerMode() == 2)
    {
        //获取关闭或删除的传感器
        std::list<std::string> disablesensorList = HmiPraseMsg::Instance()->getSensorDisableList();
        if (disablesensorList.size() > 0)
        {
            //将hmi关闭和删除的传感器的资源释放
            for (auto p : disablesensorList)
            {
                closeServerAndManageThreads(p);
                m_socket_type.erase(p);
                m_sensorCommunicationCfgMap.erase(p);
            }
        }
    }
    //获取传感器列表
    for (auto p : ConfigureMngr::getInstance()->get_plugin_list())
    {
        if (NULL == p)
        {
            continue;
        }
        // 判断传感器开关是否打开
        if (is_sensor_type(p->get_type()))
        {
            SensorCfg *p_tmp = dynamic_cast<SensorCfg*>(p);
            int socketfd = -1;
            // 判断传感器通信开关是否打开
            if (!p_tmp->get_socket_enable())
            {
                log_compnt_mngr->warn("AdadpterSocket sensor socket disable.");
                continue;
            }
            //判断传感器的Port、Ip和Type是否发生变化
            if (isSoketChg(p_tmp->get_id(),p_tmp->get_socket_info()))
            {
                log_compnt_mngr->warn("AdadpterSocket sensor socket is exist.");
                continue;
            }

            m_sensorCommunicationCfgMap[p_tmp->get_id()] = p_tmp->get_socket_info();
            std::string socketType = p_tmp->get_socket_info().type;
            //创建TCP服务
            if ((socketType == "TCP") || (socketType == "tcp"))
            {
                g_sock_obj_server[p_tmp->get_id()] = new SocketTcpServerMultiClient();
                g_sock_obj_server[p_tmp->get_id()]->epollCreate();

                // 启动监听
                int32_t i4ServeFd = g_sock_obj_server[p_tmp->get_id()]->getListenFd(
                                            atoi(p_tmp->get_socket_info().port.c_str()),
                                            false, true, p_tmp->get_socket_info().ip);
                m_socket_type[p_tmp->get_id()] = 1; //TCP
                socketfd = i4ServeFd;
            }
            //创建UDP服务
            else if ((socketType == "UDP") || (socketType == "udp"))
            {
                struct sockaddr_in sAddr;
                (void)memset(&sAddr, 0, sizeof(sAddr));
                sAddr.sin_family = AF_INET;
                sAddr.sin_addr.s_addr = inet_addr(p_tmp->get_socket_info().ip.c_str());
                sAddr.sin_port = htons(atoi(p_tmp->get_socket_info().port.c_str()));
                int sockFd = socket(PF_INET, SOCK_DGRAM, 0);
                socketfd = sockFd;
                bind(sockFd, reinterpret_cast<struct sockaddr *>(&sAddr), sizeof(struct sockaddr));
                g_sock_udp_server[p_tmp->get_id()] = sockFd; //存储UDP的socket fd
                m_socket_type[p_tmp->get_id()] = 2; //UDP
            }
            // 使用空间换时间，建立连接失败时，统一回收内存和线程资源
            log_compnt_mngr->debug("AdadpterSocket sensor socket fd = {}.",socketfd);
            g_thread_run_flag[p_tmp->get_id()].store(true);
            v_socket_adadapter[p_tmp->get_id()] = std::thread(thread_func, this, p_tmp->get_name(), p_tmp->get_id(), socketfd);
        }
    }
    log_compnt_mngr->info("AdadpterSocket::createSocketServer end.");
}

//关闭socket server服务和线程回收
void AdadpterSocket::closeServerAndManageThreads(const std::string &sensorId)
{
    log_compnt_mngr->info("AdadpterSocket::closeServerAndManageThreads start.");
    //改变对应sensor id 的原子变量
    g_thread_run_flag[sensorId].store(false);
    log_compnt_mngr->debug("AdadpterSocket deleteSocket g_thread_run_flag[{}] = {}",sensorId,g_thread_run_flag[sensorId]);
    if (close(m_epoll_fd[sensorId]) < 0)
    {
        log_compnt_mngr->error("AdadpterSocket::deleteSocket,epoll close error");
    }
    m_epoll_fd.erase(sensorId);

    //关闭TCP的server和epoll fd
    if (g_sock_obj_server[sensorId] != NULL)
    {
        g_sock_obj_server[sensorId]->epollDelete();
        g_sock_obj_server[sensorId]->closed();

        delete g_sock_obj_server[sensorId];
        g_sock_obj_server[sensorId] = NULL;
        g_sock_obj_server.erase(sensorId);
    }
    //关闭UDP的server
    else if (g_sock_udp_server[sensorId] > 0)
    {
        close(g_sock_udp_server[sensorId]);
        g_sock_udp_server.erase(sensorId);
    }

    //释放线程
    if (v_socket_adadapter.find(sensorId) != v_socket_adadapter.end())
    {
        v_socket_adadapter[sensorId].detach();
        v_socket_adadapter.erase(sensorId);
      
    }
    log_compnt_mngr->info("AdadpterSocket::closeServerAndManageThreads end.");
}

//判断传感器的通信IP、Port和Type 是否发生变化，未变化返回Ture。
bool AdadpterSocket::isSoketChg(const std::string &sensorId,struct _socket sensorComunCfg)
{
    if (m_socket_type.find(sensorId) != m_socket_type.end()) 
    {
        if (m_sensorCommunicationCfgMap.find(sensorId) != m_sensorCommunicationCfgMap.end())
        {
            auto tmp = m_sensorCommunicationCfgMap[sensorId];
            log_compnt_mngr->debug("sensorComunCfg.type is = {},tmp.type is ={}",sensorComunCfg.type,tmp.type);
            log_compnt_mngr->debug("sensorComunCfg.port is = {},tmp.port is ={}",sensorComunCfg.port,tmp.port);
            log_compnt_mngr->debug("sensorComunCfg.ip is = {},tmp.ip is ={}",sensorComunCfg.ip,tmp.ip);
            if (sensorComunCfg.type == tmp.type && sensorComunCfg.port == tmp.port && sensorComunCfg.ip == tmp.ip)
            {
                return true;
            }
        }
        closeServerAndManageThreads(sensorId);
        m_socket_type.erase(sensorId);
        m_sensorCommunicationCfgMap.erase(sensorId);
    }
    return false;
}

void AdadpterSocket::consumeThread()
{
    log_compnt_mngr->info("AdadpterSocket::consumeThread start.");
    while (true)
    {
        int ret = sem_wait(&sem_consume); //不带等待时间，一直阻塞
        if (ret == -1) //失败
        {
            int errNum = errno;
            log_compnt_mngr->error("AdadpterSocket::consumeThread() sem_wait error, errno={}, reason={}", errNum, strerror(errNum));
        }

        COSIMU_DATA data = {0};

        getRecvMsgFromAd().msgMutex.lock();
        const auto &it = getRecvMsgFromAd().msgList.begin();
        data = *it;
        getRecvMsgFromAd().msgList.pop_front();
        getRecvMsgFromAd().msgMutex.unlock();

        if (!recvBufAddSize(bufInfo, data.msgLen + bufInfo.usedSize))
        {
            log_compnt_mngr->error("AdadpterSocket::consumeThread(), recvBufAddSize error");
            //释放缓存区数据
            delete [] data.msg;
            data.msg = NULL;
            continue;
        }

        memcpy(bufInfo.buf + bufInfo.usedSize, data.msg, data.msgLen);
        bufInfo.usedSize += data.msgLen;
        bool isComplete = false;    //buffer是否包含完整的一帧数据
        int firstMsgSize = 0;       //buffer中第一帧数据的大小
        checkMsgComplete(bufInfo.buf, bufInfo.usedSize, isComplete, firstMsgSize);

        while (isComplete)
        {
            PackMsg::generateMsg(bufInfo.buf, (size_t)firstMsgSize);  //生成数据

            COSIMU_DATA sendData = {0};

            sendData.msg = bufInfo.buf;
            sendData.msgLen = (size_t)firstMsgSize;

            getSendMsgToSimpro().msgMutex.lock();
            getSendMsgToSimpro().msgList.push_back(sendData);
            getSendMsgToSimpro().msgMutex.unlock();

            V_TrafficSocket::get_instance()->sem_send_post(); //通知发送线程

            if (bufInfo.usedSize == firstMsgSize)
            {
                bufInfo.usedSize = 0;
            }
            else if (bufInfo.usedSize > firstMsgSize)
            {
                int remainMsgSize = bufInfo.usedSize - firstMsgSize;
                memmove(bufInfo.buf, bufInfo.buf + firstMsgSize, remainMsgSize);
                bufInfo.usedSize = remainMsgSize;
            }
            //再次检查buffer是否包含完整的一帧数据
            checkMsgComplete(bufInfo.buf, bufInfo.usedSize, isComplete, firstMsgSize);
        }

        if (NULL != data.msg)
        {
            //释放缓存区数据
            delete [] data.msg;
            data.msg = NULL;
        }
    }
    log_compnt_mngr->info("AdadpterSocket::consumeThread end.");
}

bool AdadpterSocket::recvBufAddSize(RecvBufInfo &info, int needSize)
{
    if (info.allocSize < needSize)
    {
        char *newBuffer = reinterpret_cast<char*>(realloc(info.buf, needSize));
        if(newBuffer == NULL)
        {
            return false;
        }
        else
        {
            info.buf = newBuffer;
            info.allocSize = needSize;
            (void)memset(info.buf + info.usedSize, 0, info.allocSize - info.usedSize);
        }
    }
    return true;
}

void AdadpterSocket::checkMsgComplete(char *buffer, int bufferSize, bool &isComplete, int &msgSize)
{
    isComplete = false; //返回值：是否包含完整的一帧数据
    msgSize = 0; //返回值：当buffer中包含完整的一帧数据的头部时，返回第一帧数据的大小

    if ((buffer != nullptr) && (bufferSize > 0))
    {
        if (static_cast<unsigned int>(bufferSize) >= sizeof(S_SP_MSG_HDR)) //说明包含了完整的MsgHeader
        {
            S_SP_MSG_HDR *msgHeader = (S_SP_MSG_HDR *)buffer; //msg 头部指针

            uint64_t msgLargeSize = msgHeader->u4HeaderSize + msgHeader->u4DataSize; // 返回值：msg总大小

            if (static_cast<uint64_t>(bufferSize) >= msgLargeSize)  //说明包含了完整的Msg
            {
                msgSize = static_cast<int>(msgLargeSize);
                isComplete = true; //返回值：是否包含完整的一帧数据
            }
        }
    }
}


