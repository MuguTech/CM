/**
 *  @file   2021.12.14
 *  @author LS
 *  @brief  Ultrasonic Description 
 */ 

#ifndef simproUltrasonicDescription_h
#define simproUltrasonicDescription_h

#include <string.h>
#include <string>
#include <iostream>
#include "simproSensorDescription.h"

using namespace std;

struct UltrasonicDescription_simpro : public SensorDescription_simpro
{
    UltrasonicDescription_simpro(string n = "untitled Ultrasonic", float aPositionX = 0.0, float aPositionY = 0.0, float aPositionZ = 1.5, float aRoll = 0.0, float aRange = 10,
        float aHeading = 0.0, float aPitch = 0.0, float aMinimumDetectRange = 0.0, float aHFov = 30.0, float aVFov = 30.0,
        float aThresholdVoltage = 5, float aVoltage = 1, float aNoiseVariance = 7.5, int aFrameRate = 10, float hResolution = 0.09, float fAzimuthGaussDistribution = 0.09, 
        float fSPL = 0.09, float fDecayTime = 0.09, float fNoiseLevel = 0.09, float fTemp = 0.09, float fHumidity = 0.09, 
        float afExistProb = 100, SENSOR_MODEL_TYPE amodelType = SENSOR_MODEL_IDEAL
        ):
        range(aRange),

        minimumDetectRange(aMinimumDetectRange),
        hFov(aHFov),
        vFov(aVFov),
        frameRate(aFrameRate),
        horizontalResolution(hResolution), 
        azimuthGaussDistribution(fAzimuthGaussDistribution),
        SPL(fSPL),
        decayTime(fDecayTime), 
        noiseLevel(fNoiseLevel), 
        temp(fTemp), 
        humidity(fHumidity), 
        thresholdVoltage(aThresholdVoltage),
        voltage(aVoltage),
        noiseVariance(aNoiseVariance),
        fExistProb(afExistProb)
    {
        type = SENSOR_TYPE_ULTRASONIC;
        strcpy(name, n.c_str()); 
        enable = false;
        sensorOutputType = 0; // 包络线[Add] 2022.08.22 LS
        enableConeRender = false; // 包络线[Add] 2022.08.22 -> [bug9111] 默认为false LS 08.30
        advSettingsEnable = 0;
        bExistProbInput = false;
		modelType = amodelType;
		assemblePositionX = aPositionX;
		assemblePositionY = aPositionY;
		assemblePositionZ = aPositionZ;
		heading = aHeading;
		pitch = aPitch;
		roll = aRoll;
		object_detection_type = SENSOR_OBJECT_DETECTION_TYPE_ALL;
    }

    float range;                // 最远探测距离 10
    float minimumDetectRange;    //最近探测距离 

    float hFov;                 // 水平fov 
    float vFov;                 // 垂直fov 
    int frameRate;
    float horizontalResolution;     // 水平分辨率
    float azimuthGaussDistribution; // 方位角高斯分布
    float SPL;                      // 发射 Sound Pressure Level
    float decayTime;                // 衰变时间
    float noiseLevel;               // 噪声水平
    float temp;                     // 环境温度
    float humidity;                 // 空气湿度

    float thresholdVoltage;         // 门限电压
    float voltage;                  // 电压
    float noiseVariance;            // 噪声方差
    bool  advSettingsEnable;        // 高级选项开关

    bool enableConeRender;          // 包络线开关
    bool bExistProbInput;           // 自定义目标存在可能性开关
    float fExistProb;               // 目标存在可能性
    char dummy[2];

public:
    int size()
    {
        return sizeof(UltrasonicDescription);
    }
    int getFrameRate()
    {
        return frameRate;
    }

};
#endif