/**
 *  @file   2021.12.14
 *  @author LS
 *  @brief  Radar Description 
 */ 

#ifndef simproRadarDescription_h
#define simproRadarDescription_h

#include <string.h>
#include <string>
#include <iostream>
#include "simproSensorDescription.h"

using namespace std;

struct RadarDescription_simpro : public SensorDescription_simpro
{
    RadarDescription_simpro(string n = "untitled Radar", float aPositionX = -8.0, float aPositionY = 0.0, float aPositionZ = 2.5, float aRange = 10,
        int aOpeFreq = 77, int aFrameRate = 10 , float aHeading = 0.0, float aPitch = 0.0, float aRoll = 0.0, float aHFov = 30.0, 
        float aVFov = 30.0, float aMinimumDetectRange = 0.0, int aPointsPerSecond = 1500, float atransmitterPower = 160.0,float  aeffectiveArea = 1.0, float atransmitGain = 1.0, 
        float abandwidth = 100.0, float anoiseFigure = 7.2, float atotalLosses = 2.0, float afalseAlarmRate = 0.1, double aideal_Freq = 10, SENSOR_MODEL_TYPE amodelType = SENSOR_MODEL_IDEAL,
        int aSimMode = 0, int aRxAntennaMode = 0, int aTxAntennaMode = 0, float aFMbandwidth = 500, float aReceiveGain = 18.0, float aVelocityResolution = 1.0, 
        float aAngleResolution = 1.0, int aMaxReflectedBeam = 100, int aMaxAbsorbedBeam = 100,
        float fAzimuthMSENoise = 1.0, float fElevationMSENoise = 1.0, float fRangeMSENoise = 1.0, float hResolution = 0.09, int iSensitivityThreshold = 0, 
        float afExistProb = 100):
        range(aRange),
        opeFreq(aOpeFreq),
        frameRate(aFrameRate), 
        hFov(aHFov),
        vFov(aVFov),
        minimumDetectRange(aMinimumDetectRange),
        pointsPerSecond(aPointsPerSecond),
        azimuthMSENoise(fAzimuthMSENoise),
        elevationMSENoise(fElevationMSENoise),
        rangeMSENoise(fRangeMSENoise),
        horizontalResolution(hResolution), 
        sensitivityThreshold(iSensitivityThreshold),
        transmitterPower(atransmitterPower),
        effectiveArea(aeffectiveArea),
        transmitGain(atransmitGain),
        bandwidth(abandwidth),
        noiseFigure(anoiseFigure),
        totalLosses(atotalLosses),
        falseAlarmRate(afalseAlarmRate),
        fExistProb(afExistProb)
        // simMode(aSimMode),
        // antennaMode(aAntennaMode),
        // FMbandwidth(aFMbandwidth),
        // receiveGain(aReceiveGain),
        // rangeResolution(aRangeResolution),
        // velocityResolution(aVelocityResolution),
        // angleResolution(aAngleResolution),
        // maxReflectedBeam(aMaxReflectedBeam),
        // maxAbsorbedBeam(aMaxAbsorbedBeam)
    {
        type = SENSOR_TYPE_RADAR;
        strcpy(name, n.c_str()); 
        enable = false;
        sensorOutputType = 0; // 包络线[Add] 2022.08.22 LS
        dumpRawDataType = 0;
        temperature = 290;
        advSettingsEnable = 0;
        isSavePointCloud = false;
        enableConeRender = false; // 包络线[Add] 2022.08.22 LS -> [bug9111] 默认为false LS 08.30
        bExistProbInput = false;
		modelType = amodelType;
		assemblePositionX = aPositionX;
		assemblePositionY = aPositionY;
		assemblePositionZ = aPositionZ;
		heading = aHeading;
		pitch = aPitch;
		roll = aRoll;
		object_detection_type = SENSOR_OBJECT_DETECTION_TYPE_ALL;
        /* 物理级参数 */
        phySettingsEnable = false;     // 物理开关
        boundingBoxEnable = 0;
    }

    // string name; 
    float range;                // 最远探测距离 10
    int opeFreq;                // 工作频率 77GHz
    int frameRate;            // 每秒的帧率 10Hz

    float hFov;                 // 水平fov 0~ 30
    float vFov;                 // 垂直fov 0~30
    float minimumDetectRange;   //最近探测距离 
    int pointsPerSecond;      // 每秒点云数量
    int dumpRawDataType;
    float azimuthMSENoise;      // 方位角均方差噪声
    float elevationMSENoise;    // 高低角均方差噪声
    float rangeMSENoise;        // 距离均方差噪声
    float horizontalResolution; // 水平分辨率
    int sensitivityThreshold;   // 敏感性阈值

    float transmitterPower;     // 发射功率 千瓦
    float effectiveArea;        // 天线有效面积 平方米
    float transmitGain;         // 发射天线增益 分贝
    float bandwidth;            // 接收机噪声带宽 兆赫兹
    float noiseFigure;          // 接收机噪声系数 分贝
    float totalLosses;          // 雷达损耗  分贝
    float falseAlarmRate;       // 虚警概率  
    float temperature;          // 环境温度
    bool advSettingsEnable;     // 高级设置使能

    bool isSavePointCloud;
    bool enableConeRender;      // 包络线开关
    bool bExistProbInput;       // 自定义目标存在可能性开关
    float fExistProb;           // 目标存在可能性
    char dummy[1];

public:
    int size()
    {
        return sizeof(RadarDescription);
    }

    int getFrameRate()
    {
        return frameRate;
    }

    /* 物理级参数 */
    bool phySettingsEnable;
    int simMode;                // 仿真模式
    int rxAntennaMode;            // 天线极化方式
    int txAntennaMode;            // 天线极化方式
    float FMbandwidth;          // 调频带宽
    float receiveGain;          // 接收天线增益
    float rangeResolution;      // 多普勒回波距离精度
    float velocityResolution;   // 多普勒回波速度精度
    float angleResolution;      // 角分辨率
    int maxReflectedBeam;       // 最大反射波束
    int maxAbsorbedBeam;        // 最大吸收波束
    /* 物理级参数 且包含：工作频率, 带宽, 发射天线增益, 天线有效面积, 发射功率 */
    float TxHHalfWaveBandwidth;   //水平半波带宽（0~3.14rad）
    float TxVHalfWaveBandwidth;   //垂直半波带宽（0~3.14rad）A
    float RxHHalfWaveBandwidth;   //水平半波带宽（0~3.14rad）
    float RxVHalfWaveBandwidth;   //垂直半波带宽（0~3.14rad）A
};
#endif