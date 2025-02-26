#include "./FrameCtrlMsgGenerator.h"

#include <stdio.h>
#include <string.h>

#include "ConfigureMngr.h"
#include "common/MMMsgType.h"

#include "common.h"
#include "log.h"

bool FrameCtrlMsgGenerator::generateMsg(char *msgBuffer,
                                        unsigned int &msgBufferUsedSize) {
  log_compnt_mngr->info("FrameCtrlMsgGenerator::generateMsg start.");
  MM_MSG_HDR *msgHead = (MM_MSG_HDR *)msgBuffer;
  msgHead->dataSize = 0;
  msgHead->frameNo = 0;
  msgHead->headerSize = sizeof(MM_MSG_HDR_t);
  msgHead->simTime = 0;
  msgHead->pkgId = D_MM_PKG_ID_CONTROL_MM_FRAME_CONTROL_REQ;
  CONTROL_MM_FRAME_CONTROL_t *mm_frame_ctrl =
      reinterpret_cast<CONTROL_MM_FRAME_CONTROL_t *>(msgHead + 1);

  const auto frame_ctrl_struct_size = sizeof(CONTROL_MM_FRAME_CONTROL_t);

  // 遍历sensor list，封包
  const auto &plugin_list = ConfigureMngr::getInstance()->get_plugin_list();
  for (auto plugin : plugin_list) {
    if (!plugin) {
      log_compnt_mngr->warn("plugin cfg err.");
      continue;
    }

    if (is_sensor_type(plugin->get_type())) {
      std::string plugin_id = plugin->get_id();
      SensorCfg *sensor_cfg = dynamic_cast<SensorCfg *>(plugin);
      int frame_rate = sensor_cfg->getFrmRate();
      mm_frame_ctrl->frame_rate = frame_rate;
      if (plugin_id.size() >= SENSOR_NAME_SIZE) {
        log_compnt_mngr->error("plugin id {} is too long", plugin_id);
        return false;
      }
      strncpy(mm_frame_ctrl->sensor_id, plugin_id.c_str(),
              SENSOR_NAME_SIZE - 1);
      log_compnt_mngr->debug("sensor id is {}, frame rate is {}", plugin_id,
                             frame_rate);
      mm_frame_ctrl->sensor_id[SENSOR_NAME_SIZE - 1] = '\0';
      msgHead->dataSize += frame_ctrl_struct_size;
      mm_frame_ctrl += 1;
    }
  }

  msgBufferUsedSize = msgHead->dataSize + msgHead->headerSize;

  log_compnt_mngr->info("FrameCtrlMsgGenerator::generateMsg end.");
  return true;
}

bool FrameCtrlMsgGenerator::generateFrameCtrlRespMsg(
    char *msgBuffer, unsigned int &msgBufferUsedSize, bool is_consumed_data,
    const std::map<std::string, bool> &sensor_running_map,
    const std::map<std::string, int> &sensor_frame_map) {
  log_compnt_mngr->info("FrameCtrlMsgGenerator::generateFrameCtrlRespMsg start.");
  MM_MSG_HDR *msgHead = (MM_MSG_HDR *)msgBuffer;
  msgHead->dataSize = 0;
  msgHead->frameNo = 0;
  msgHead->headerSize = sizeof(MM_MSG_HDR_t);
  msgHead->simTime = 0;
  msgHead->pkgId = D_MM_PKG_ID_CONTROL_MM_WAKE_UP_RESP;
  CONTROL_MM_WAKE_UP_RESP_t *mm_wakeup_resp =
      reinterpret_cast<CONTROL_MM_WAKE_UP_RESP_t *>(msgHead + 1);

  for (auto &sensor_info : sensor_running_map) {
    mm_wakeup_resp->consumed_data = is_consumed_data;
    mm_wakeup_resp->frame_number = sensor_frame_map.at(sensor_info.first);
    mm_wakeup_resp->is_wakeup_all = false;

    strncpy(mm_wakeup_resp->sensor_id, sensor_info.first.c_str(),
            SENSOR_NAME_SIZE - 1);

    mm_wakeup_resp->sensor_id[SENSOR_NAME_SIZE - 1] = '\0';
    mm_wakeup_resp += 1;
    msgHead->dataSize += sizeof(CONTROL_MM_WAKE_UP_RESP_t);
    log_compnt_mngr->debug("generateFrameCtrlRespMsg sensor id is {}, consume simpro frame_number is {}",sensor_info.first.c_str(),sensor_frame_map.at(sensor_info.first));
  }

  msgBufferUsedSize = msgHead->dataSize + msgHead->headerSize;

  log_compnt_mngr->info("FrameCtrlMsgGenerator::generateFrameCtrlRespMsg end.");
  return true;
}

bool FrameCtrlMsgGenerator::generateFrameCtrlStopRespMsg(
    char *msgBuffer, unsigned int &msgBufferUsedSize) {
  log_compnt_mngr->info("FrameCtrlMsgGenerator::generateFrameCtrlStopRespMsg start.");
  MM_MSG_HDR *msgHead = (MM_MSG_HDR *)msgBuffer;
  msgHead->dataSize = 1;
  msgHead->frameNo = 0;
  msgHead->headerSize = sizeof(MM_MSG_HDR_t);
  msgHead->simTime = 0;
  msgHead->pkgId = D_MM_PKG_ID_CONTROL_MM_STOP_SIMU_RESP;

  strncpy(reinterpret_cast<char *>(msgHead + 1), "1", 1);

  msgBufferUsedSize = msgHead->dataSize + msgHead->headerSize;

  log_compnt_mngr->info("FrameCtrlMsgGenerator::generateFrameCtrlStopRespMsg end.");
  return true;
}