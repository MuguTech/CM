#ifndef FRAME_CTRL_MSG_PARSER_H
#define FRAME_CTRL_MSG_PARSER_H

#include <string>
#include <map>

#define RECV_BUF_SIZE 1024 * 1024 * 2

class FrameCtrlMsgParser {
public:
  void addRecvMsg(char *msgBuff, int msgSize);
  const std::map<std::string, int> &getSensorList() { return m_sensor_list; }
  void clearSensorList() { m_sensor_list.clear(); }
  bool getIsSyncMode() { return m_is_sync_mode; }

private:
  void checkMsgComplete(char *buffer, int bufferSize, bool &isComplete,
                        int &msgSize);
  void parseMsg(char *pkgbuff, unsigned int pkgLen);

  char recvMsgBuf[RECV_BUF_SIZE]; //接收数据的缓冲区
  int recvMsgBufUsedSize = 0;     // recvMsgBuf已使用空间
  char swapBuf[RECV_BUF_SIZE];    //交换空间

  // sensor id, frame number
  std::map<std::string, int> m_sensor_list;

  // 是否同步模式
  bool m_is_sync_mode = false;;
};

#endif
