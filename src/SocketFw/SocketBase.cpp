#include "SocketBase.h"


extern "C"
{
// #include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <errno.h>
#ifdef ONLOGLEVEL /* dingjiajia 2021.8.27 [Print errno] [ADD] */
#include <string.h>
#endif
}

#include <cstring>

#ifdef CT_UT
    #include "SocketTcpServer.h"
#endif
#include "../log.h"

SocketBase::SocketBase()
{
    m_i4_connfd = -1;
    (void)memset(m_buf,0,sizeof(m_buf));
}

SocketBase::~SocketBase()
{
}

bool SocketBase::read(int32_t *out_p_size, char **out_p_buf)
{
    log_compnt_mngr->info("SocketBase::read start.");
    int32_t i4_recv_size = -1;
    bool b_ret = false;

    (void)memset(m_buf, 0x00, D_SOCKET_RECV_BUFF_SIZE);

    if (0 <= m_i4_connfd)
    {
        i4_recv_size = static_cast<int32_t>(recv(m_i4_connfd, m_buf, D_SOCKET_RECV_BUFF_SIZE, 0));
        if (-1 == i4_recv_size)
        {
        }
        else if (0 == i4_recv_size)
        {
			/* When a stream socket peer has performed an orderly shutdown, the
				return value will be 0 (the traditional "end-of-file" return). */
			(void)close(m_i4_connfd); /* 被动关闭连接句柄 */
            m_i4_connfd = -1; /* EOF,end-of-file */
        }
        else
        {
            *out_p_size = i4_recv_size;
            *out_p_buf = m_buf;
            b_ret = true;
        }
    }

    log_compnt_mngr->info("SocketBase::read end.");
    return b_ret;
}

bool SocketBase::write(uint32_t &in_u4_size, const int8_t *in_p_buf)
{
    log_compnt_mngr->info("SocketBase::write start.");
    int32_t i4_sent_size = -1;
    bool b_ret = false;

    if (0 <= m_i4_connfd) 
    {

		try{
			//std::cout << "SocketBase::write tID=" << pthread_self() << " in_p_buf="<< std::hex << (long)in_p_buf << " m_i4_connfd=" << std::hex << m_i4_connfd << " in_u4_size=" << in_u4_size << std::endl;
			i4_sent_size =static_cast<int32_t>(send(m_i4_connfd, in_p_buf, in_u4_size, MSG_NOSIGNAL));
			//std::cout << "SocketBase::write i4_sent_size=" << i4_sent_size << std::endl;
			if (-1 == i4_sent_size)
			{
				if (104 == errno) { /* 104:Connection reset by peer,表明你在对一个对端socket已经关闭的的连接调用write或send方法 */
					(void)close(m_i4_connfd); /* 被动关闭连接句柄 */
					m_i4_connfd = -1; /* 初始化连接句柄 */
				}
                log_compnt_mngr->error("socket write err {}",errno);
#ifdef ONLOGLEVEL /* dingjiajia 2021.8.27 [Print errno] [ADD] */
                printf("{}",errno);
#endif
			}
			else
			{
				b_ret = true;
			}
		} catch(...) {
            log_compnt_mngr->error("SocketBase::write exception");
		}
    }

    log_compnt_mngr->info("SocketBase::read end.");
    return b_ret;
}

void SocketBase::closed() /* 主动关闭连接句柄 */
{
    if (0 <= m_i4_connfd)
    {
        (void)close(m_i4_connfd);
		m_i4_connfd = -1; /* 初始化连接句柄 */
    }
}
