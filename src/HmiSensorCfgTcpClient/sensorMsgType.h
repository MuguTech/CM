#ifndef SENSOR_MSG_TYPE_H
#define SENSOR_MSG_TYPE_H
#include <stdint.h>
#include <string>
enum CM_SENSOR_TYPE
{
	CM_SENSOR_TYPE_UNKNOWN,	
	CM_SENSOR_TYPE_LIDAR,
	CM_SENSOR_TYPE_IMU,
	CM_SENSOR_TYPE_CAMERA,
	CM_SENSOR_TYPE_BASIC_SENSOR,
	CM_SENSOR_TYPE_GPS,
	CM_SENSOR_TYPE_RADAR,
	CM_SENSOR_TYPE_ULTRASONIC,
	CM_SENSOR_TYPE_V2X_OBU,
	CM_SENSOR_TYPE_V2X_RSU,
    CM_SENSOR_TYPE_UWB
};

/**
 * @struct CM_SENSOR_MSG_HDR
 * @brief HMI sensor配置消息传输的结构体
 */
typedef struct CM_SENSOR_MSG_HDR
{
    // size of this header structure when transmitted   @unit byte
    uint32_t headerSize;
    // size of data following the header   @unit byte
    uint32_t dataSize;
    //传感器类型
    int  sensorType;
	//用于传感器是否发送完成的标志位
	bool sendover;
	//用于传感器端口是否使用的标志位
	bool isPortUse;
} CM_SENSOR_MSG_HDR_t;
#endif