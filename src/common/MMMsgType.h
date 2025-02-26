#ifndef MM_MSG_TYPE_H
#define MM_MSG_TYPE_H

#include <stdint.h>

// 1字节对齐
#pragma pack(1)

/**
 * @addtogroup MM_COMMON
 * @brief 通用定义
 *
 */
#define SENSOR_NAME_SIZE 32


/**
 * @addtogroup MM_PKG_ID
 * @brief MM 消息类型
 *
 */
/**
 * @brief MM请求帧率控制包
 * CONTROL_MM_FRAME_CONTROL
*/
#define D_MM_PKG_ID_CONTROL_MM_FRAME_CONTROL_REQ 1
/**
 * @brief 回复MM帧率控制包
 * CONTROL_MM_FRAME_CONTROL
 */
#define D_MM_PKG_ID_CONTROL_MM_FRAME_CONTROL_RESP 2
/**
 * @brief 请求唤醒MM sensor包
 * CONTROL_MM_WAKE_UP
 */
#define D_MM_PKG_ID_CONTROL_MM_WAKE_UP_REQ 3
/**
 * @brief 回复唤醒MM sensor包
 * CONTROL_MM_WAKE_UP
 */
#define D_MM_PKG_ID_CONTROL_MM_WAKE_UP_RESP 4

/**
 * @brief 请求消费stop消息包
 * 
 */
#define D_MM_PKG_ID_CONTROL_MM_STOP_SIMU_REQ 5

/**
 * @brief 相应消费stop消息包
 * 
 */
#define D_MM_PKG_ID_CONTROL_MM_STOP_SIMU_RESP 6


typedef struct MM_MSG_HDR {
  // size of this header structure when transmitted   @unit byte
  uint32_t headerSize;
  // size of data following the header   @unit byte
  uint32_t dataSize;
  // number of the simulation frame    @unit _
  uint32_t frameNo;
  // package identifier @unit @MM_PKG_ID
  uint16_t pkgId;
  // simulation time      @unit s
  double simTime;
} MM_MSG_HDR_t;

typedef struct CONTROL_MM_FRAME_CONTROL {
  uint32_t frame_rate = 0;
  // sensor 名称   @unit _
  char sensor_id[SENSOR_NAME_SIZE];
} CONTROL_MM_FRAME_CONTROL_t;

typedef struct CONTROL_MM_WAKE_UP {
  // 是否唤醒所有sensor
  bool is_wakeup_all;
  // 是否同步
  bool is_sync_mode;
  // sensor 名称   @unit _
  char sensor_id[SENSOR_NAME_SIZE];
  // 消费数据的仿真帧号
  uint32_t frame_number;
} CONTROL_MM_WAKE_UP_t;


typedef struct CONTROL_MM_WAKE_UP_RESP {
  // 是否唤醒所有sensor
  bool is_wakeup_all;
  // sensor 名称   @unit _
  char sensor_id[SENSOR_NAME_SIZE];
  // sensor是否成功消费groundtruth数据
  bool consumed_data;
  // 消费数据的仿真帧号
  int frame_number;
} CONTROL_MM_WAKE_UP_RESP_t;


#pragma pack()
#endif
