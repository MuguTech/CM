#ifndef SensorDescription_h
#define SensorDescription_h

#include <string>
#include <CJsonObject.hpp>

//车辆
#define SENSOR_OBJECT_DETECTION_TYPE_VEHICLE (0x00000001)

//行人
#define SENSOR_OBJECT_DETECTION_TYPE_PEDESTRIAN (0x00000002)

//障碍物
#define SENSOR_OBJECT_DETECTION_TYPE_OBSTACLE (0x00000004)

//交通标识
#define SENSOR_OBJECT_DETECTION_TYPE_TRAFICSIGN (0x00000008)

//道路信息
#define SENSOR_OBJECT_DETECTION_TYPE_LANEINFO (0x00000010)

//道路标识
#define SENSOR_OBJECT_DETECTION_TYPE_ROADMARK (0x00000020)

//交通灯
#define SENSOR_OBJECT_DETECTION_TYPE_TRFICLIGHT (0x00000040)

//所有
#define SENSOR_OBJECT_DETECTION_TYPE_ALL (0xffffffff)

enum SENSOR_TYPE
{
	SENSOR_TYPE_UNKNOWN,	
	SENSOR_TYPE_LIDAR,
	SENSOR_TYPE_IMU,
	SENSOR_TYPE_CAMERA,
	SENSOR_TYPE_BASIC_SENSOR,
	SENSOR_TYPE_GPS,
	SENSOR_TYPE_RADAR,
	SENSOR_TYPE_ULTRASONIC,
	SENSOR_TYPE_V2X_OBU,
	SENSOR_TYPE_V2X_RSU
    ,SENSOR_TYPE_UWB
};

enum SENSOR_MODEL_TYPE
{
	SENSOR_MODEL_IDEAL,
	SENSOR_MODEL_PROBABLY,
	SENSOR_MODEL_PHYSIC
};

struct SensorDescription
{
    int type;
	int sensorOutputType; // 包络线[Add]传感器输出类型 2022.08.22 LS
    char name[128];
	bool enable;
	char dummy[3];
	float assemblePositionX;
	float assemblePositionY;
	float assemblePositionZ;
	float heading;
	float pitch;
	float roll;
	SENSOR_MODEL_TYPE modelType = SENSOR_MODEL_IDEAL;	// 传感器模型类型，初始化为IDEAL

	int object_detection_type = SENSOR_OBJECT_DETECTION_TYPE_ALL;	//物体识别类别

    int frameRate;              // 每秒的帧率 10Hz
    float hFov;                 // 水平fov 0~ 30
    float vFov;                 // 垂直fov 0~30
    float minimumDetectRange;   // 最近探测距离
    float range;                // 最远探测距离 10

    bool boundingBoxEnable = false;
	char BuiltIn[2] = "2";

public:
	virtual int size()
	{
		return sizeof(SensorDescription);
	};
};



#endif