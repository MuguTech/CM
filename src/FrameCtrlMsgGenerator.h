#ifndef FRAME_CTRL_MSG_GENERATOR_H
#define FRAME_CTRL_MSG_GENERATOR_H


#include <map>

class FrameCtrlMsgGenerator {
public:
  bool generateMsg(char *msgBuffer, unsigned int &msgBufferUsedSize);
  bool generateFrameCtrlRespMsg(
      char *msgBuffer, unsigned int &msgBufferUsedSize, bool is_consumed_data,
      const std::map<std::string, bool> &sensor_running_map,
      const std::map<std::string, int> &sensor_frame_map);

  bool generateFrameCtrlStopRespMsg(char *msgBuffer,
                                    unsigned int &msgBufferUsedSize);
};

#endif