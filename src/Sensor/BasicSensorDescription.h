#ifndef BasicSensorDescription_h
#define BasicSensorDescription_h

#include <string>
#include <string.h>
#include "SensorDescription.h"

using namespace std;

struct BasicSensorDescription : public SensorDescription
{
    BasicSensorDescription(string n = "untitled BasicSensor", float aPositionX = 0.0, float aPositionY = 0.0, float aPositionZ = 0.0,
     float bHeading = 0.0, float bRoll = 0.0, float bPitch = 0.0, float bhFov = 50.0, float bvFov = 30.0,
     float bMinimumDetectRange = 0.0, float bMaximumDetectRange = 100.0, float bMaximumDetectTargets = 10.0)
    {
		type = SENSOR_TYPE_BASIC_SENSOR;
		strcpy(name, n.c_str());
		enable = false;
        sensorOutputType = 0; // 包络线[Add] 2022.08.22 LS

		assemblePositionX = aPositionX;
		assemblePositionY = aPositionY;
		assemblePositionZ = aPositionZ;
		heading = bHeading;
		pitch = bPitch;
		roll = bRoll;
		modelType = SENSOR_MODEL_IDEAL;

        frameRate = 60;            // 每秒的帧率 10Hz
        hFov = bhFov;                 // 水平fov 0~ 30
        vFov = bvFov;                 // 垂直fov 0~30
        minimumDetectRange = bMinimumDetectRange;   //最近探测距离
        range = bMaximumDetectRange;                // 最远探测距离 10

        maximumDetectTargets = bMaximumDetectTargets;
		object_detection_type = SENSOR_OBJECT_DETECTION_TYPE_ALL;
        enableConeRender = false; // 包络线[Add] 2022.08.22 LS -> [bug9111] 默认为false LS 08.30

        boundingBoxEnable = false;
    } 

    //std::string name; //����
    float leftView; //camera���
    float rightView; //��װλ�ã�X��
    float topView; //��װλ�ã�Y��
    float bottomView; //��װλ�ã�Z��
    float sensorTick;
    float maximumDetectTargets;   //��ת��
    bool enableConeRender; // 包络线开关
    char dummy[3];

public:
    int size()
    {
      return sizeof(BasicSensorDescription);
    };
};

#endif