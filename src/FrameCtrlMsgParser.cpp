#include "FrameCtrlMsgParser.h"

#include "ConfigureMngr.h"
#include "common.h"
#include "common/MMMsgType.h"
#include "log.h"
#include "FrameCtrlSocket.h"

void FrameCtrlMsgParser::addRecvMsg(char *msgBuff, int msgSize) {
  log_compnt_mngr->info("FrameCtrlMsgParser::addRecvMsg start.");
  log_compnt_mngr->debug("msgSize is {}, recvMsgBufUsedSize is {}", msgSize,
                         recvMsgBufUsedSize);

  if ((msgBuff != NULL) && (msgSize > 0)) {
    log_compnt_mngr->debug("msgBuff is not null");

    // recvMsgBuf剩余大小
    int recvMsgBufFreeSize = RECV_BUF_SIZE - recvMsgBufUsedSize;

    //如果该client的MsgBuffer剩余大小 足够多
    if (recvMsgBufFreeSize >= msgSize) {
      log_compnt_mngr->debug("recvMsgBufFreeSize >= msgSize");

      //将新消息 追加到 recvMsgBuf 末尾
      memcpy(recvMsgBuf + recvMsgBufUsedSize, msgBuff, msgSize);

      //该client的MsgBuffer已使用大小
      recvMsgBufUsedSize += msgSize;

      //是否包含完整的一帧数据
      bool isComplete = false;
      //第一帧数据的大小
      int firstMsgSize = 0;

      //检查 该client的MsgBuffer 是否包含完整的一帧数据
      checkMsgComplete(recvMsgBuf, recvMsgBufUsedSize, isComplete,
                       firstMsgSize);

      //如果包含完整的一帧数据
      while (isComplete) {
        log_compnt_mngr->debug(
            "isComplete={}, firstMsgSize={}, recvMsgBufUsedSize={}", isComplete,
            firstMsgSize, recvMsgBufUsedSize);

        // 解析数据
        parseMsg(recvMsgBuf, static_cast<unsigned int>(firstMsgSize));

        FrameCtrlSocket::get_instance()->processOneMsg();

        //将第一帧数据移除数据缓冲区
        if (recvMsgBufUsedSize ==
            firstMsgSize) //如果碰巧，recvMsgBuf里的数据不多不少正好一帧
        {
          log_compnt_mngr->debug(
              "isComplete={}, firstMsgSize={}, recvMsgBufUsedSize={}",
              isComplete, firstMsgSize, recvMsgBufUsedSize);

          //只要将clientMsgBufUsedSize重置为0
          recvMsgBufUsedSize = 0; //该client的MsgBuffer已使用大小
        } else if (recvMsgBufUsedSize >
                   firstMsgSize) //如果 recvMsgBuf里的数据 大于 一帧的数据量
        {
          //借助交换空间，将第一帧数据移除数据缓冲区

          // recvMsgBuf里剩余的数据
          char *remainMsgBuf = recvMsgBuf + firstMsgSize;

          // recvMsgBuf里剩余的数据
          int remainMsgSize = recvMsgBufUsedSize - firstMsgSize;

          log_compnt_mngr->debug("isComplete={}, firstMsgSize={}, "
                                 "recvMsgBufUsedSize={}, remainMsgSize={}",
                                 isComplete, firstMsgSize, recvMsgBufUsedSize,
                                 remainMsgSize);

          //将第一帧数据移除数据缓冲区
          // remainMsgSize一定小于swapBuf的空间(RECV_BUF_SIZE)
          memcpy(swapBuf, remainMsgBuf, remainMsgSize);
          memcpy(recvMsgBuf, swapBuf, remainMsgSize);
          // recvMsgBuf已使用大小
          recvMsgBufUsedSize = remainMsgSize;
        } else //如果 recvMsgBuf 小于 一帧的数据量
        {
          //不应该出现，说明checkMsgComplete()工作不正常
          log_compnt_mngr->error("recvMsgBufUsedSize < firstMsgSize");
        }

        //再次检查 recvMsgBuf 是否包含完整的一帧数据
        checkMsgComplete(recvMsgBuf, recvMsgBufUsedSize, isComplete,
                         firstMsgSize);
      }
    } else //如果recvMsgBuf剩余大小 不足
    {
      //严重错误，丢弃掉新的消息
      log_compnt_mngr->error(
          "recvMsgBufFreeSize not enough. recvMsgBufFreeSize={}, msgSize={}",
          recvMsgBufFreeSize, msgSize);
    }
  }
  log_compnt_mngr->info("FrameCtrlMsgParser::addRecvMsg end.");
}

void FrameCtrlMsgParser::parseMsg(char *pkgbuff, unsigned int pkgLen) {
  log_compnt_mngr->info("FrameCtrlMsgParser::parseMsg start.");
  MM_MSG_HDR_t *pMsgHead = (MM_MSG_HDR_t *)pkgbuff; // msg头部指针

  if (pMsgHead->dataSize == 0) {
    return;
  }

  log_compnt_mngr->debug("msgHeadsize={}, msgDataSize={}, no={}",
                         pMsgHead->headerSize, pMsgHead->dataSize,
                         pMsgHead->frameNo);

  if (pkgLen != pMsgHead->headerSize + pMsgHead->dataSize) {
    log_compnt_mngr->error("pkgLen error.");
  }

  char *currentPkg = pkgbuff + pMsgHead->headerSize; //当前Pkg的头部指针
  if (pMsgHead->pkgId == D_MM_PKG_ID_CONTROL_MM_WAKE_UP_REQ) {
    int element_size = sizeof(CONTROL_MM_WAKE_UP_t);
    int pkg_num = pMsgHead->dataSize / element_size;
    std::string sensor_id;

    CONTROL_MM_WAKE_UP_t *pkg_ptr =
        reinterpret_cast<CONTROL_MM_WAKE_UP_t *>(currentPkg);
    for (int i = 0; i < pkg_num; i++) {
      if (!pkg_ptr->is_wakeup_all) {
        sensor_id = pkg_ptr->sensor_id;
        int frame_number = pkg_ptr->frame_number;
        log_compnt_mngr->debug("FrameCtrlMsgParser::parseMsg sensor is {}, consume simpro frame_number is = {}",sensor_id,frame_number);
        m_sensor_list[sensor_id] = frame_number;
        m_is_sync_mode = pkg_ptr->is_sync_mode;
      } else {
        // 获取所有的sensor id
        auto plugin_list = ConfigureMngr::getInstance()->get_plugin_list();
        int frame_number = pkg_ptr->frame_number;
        log_compnt_mngr->debug("FrameCtrlMsgParser::parseMsg all sensor consume simpro frame_number is = {}",frame_number);
        m_is_sync_mode = pkg_ptr->is_sync_mode;
        for (auto plugin : plugin_list) {
          if (NULL == plugin) {
            continue;
          }

          if (true == plugin->get_name().empty() ||
              true == plugin->get_type().empty() ||
              true == plugin->get_id().empty() ||
              true == plugin->get_lib().empty()) {
            log_compnt_mngr->warn("plugin cfg err.");
            continue;
          }

          if (is_sensor_type(plugin->get_type())) {
            sensor_id = plugin->get_id();
            m_sensor_list[sensor_id] = frame_number;
          }
        }
      }
      pkg_ptr += 1;
    }
  }
  log_compnt_mngr->info("FrameCtrlMsgParser::parseMsg end.");
}

void FrameCtrlMsgParser::checkMsgComplete(char *buffer, int bufferSize,
                                          bool &isComplete, int &msgSize) {
  log_compnt_mngr->info("FrameCtrlMsgParser::checkMsgComplete start.");
  log_compnt_mngr->debug("start bufferSize={}", bufferSize);

  //返回值：是否包含完整的一帧数据
  isComplete = false;
  //返回值：当buffer中包含完整的一帧数据的头部时，返回第一帧数据的大小
  msgSize = 0;

  if ((buffer != nullptr) && (bufferSize > 0)) {
    //说明包含了完整的MsgHeader
    if (static_cast<unsigned int>(bufferSize) >= sizeof(MM_MSG_HDR_t)) {
      // msg 头部指针
      MM_MSG_HDR_t *msgHeader = reinterpret_cast<MM_MSG_HDR_t *>(buffer);
      // 返回值：msg总大小
      msgSize = msgHeader->headerSize + msgHeader->dataSize;

      //说明包含了完整的Msg
      if (bufferSize >= msgSize) {
        //返回值：是否包含完整的一帧数据
        isComplete = true;
      }
    }
  }
  log_compnt_mngr->debug("checkMsgComplete isComplete ={}, msgSzie = {}",isComplete, msgSize);
  log_compnt_mngr->info("FrameCtrlMsgParser::checkMsgComplete end.");
}