#ifndef LidarDescription_h
#define LidarDescription_h

#include <string>
#include <iostream>
#include <string.h>
#include "SensorDescription.h"

using namespace std;

struct LidarDescription : public SensorDescription
{
	//参照KITTI的velodyne参数，设置默认值
    LidarDescription(string n = "untitled lidar", float aPositionX = 0.0, float aPositionY = 0.0, float aPositionZ = 1.5, int c = 64,
        float aHeading = 0.0, float aPitch = 0.0, float aRoll = 0.0, float ahFov = 360.0, float avFov = 40.0, float aMinimumDetectRange = 0.0,
        float r = 250.0, float rFrequency = 10.0, float hResolution = 0.09, float uFov = 10.0, float lFov = -16.8, float aAttenuationRate = 0.004,
        float dGeneralRate = 0.45, float dIntensityLimit = 0.8, float dZeroIntensity = 0.4, float nStddev = 0.0, int aFrameRate = 10,
        float aWavelength = 1.0, float aVBeamDivAngle = 1.0, float aHBeamDivAngle = 1.0, float aTAperture = 1.0, float aRAperture = 1.0, 
        float aTTransmissivitye = 1.0, float aRTransmissivity = 1.0, float aNEP = 1.0, float aNoiseBandwidth = 1.0, float apeakOpticalPower = 90.0, 
        float areceiverAperture = 50.0, float areceiverOpticalEfficiency = 0.8, float aunitGain = 10000000.0, float aAPD = 100.0, 
        float abandwidth = 18,float aphotonFluxes = 3, float areflectivity = 0.5, float aexcessNoiseFactor = 5.0, float afalseAlarmRate = 0.1,  
        float afExistProb = 100, SENSOR_MODEL_TYPE amodelType = SENSOR_MODEL_IDEAL):
        channels(c), 
        rotationFrequency(rFrequency), 
        horizontalResolution(hResolution), 
        upperFov(uFov), 
        lowerFov(lFov), 
        atmosphereAttenuationRate(aAttenuationRate), 
        dropoffGeneralRate(dGeneralRate), 
        dropoffIntensityLimit(dIntensityLimit), 
        dropoffZeroIntensity(dZeroIntensity), 
        noiseStddev(nStddev),
        fWavelength(aWavelength),
        fVBeamDivergenceAngle(aVBeamDivAngle),
        fHBeamDivergenceAngle(aHBeamDivAngle),
        fTransmittingAperture(aTAperture),
        fReceivingAperture(aRAperture),
        fTransmitTransmissivity(aTTransmissivitye),
        fReceiveTransmissivity(aRTransmissivity),
        fNEP(aNEP),
        fNoiseBandwidth(aNoiseBandwidth),

        peakOpticalPower(apeakOpticalPower),
        receiverAperture(areceiverAperture),
        receiverOpticalEfficiency(areceiverOpticalEfficiency),
        unitGain(aunitGain),
        APD(aAPD),
        bandwidth(abandwidth),
        excessNoiseFactor(aexcessNoiseFactor),
        reflectivity(areflectivity),
        falseAlarmRate(afalseAlarmRate),
        photonFluxes(aphotonFluxes),

        fExistProb(afExistProb)

    {
        type = SENSOR_TYPE_LIDAR;
        strcpy(name, n.c_str()); 
        enable = false;
        sensorOutputType = 0; // 包络线[Add] 2022.08.22 LS
        isSavePointCloud = false;
        dumpRawDataType = 0;
        isVisibleCloudPoint = false;
        advSettingsEnable = false;
        thermalNoise = 0.0;
        darkcurrent = 0.00000002;
        solarNoise = 0.0000001;
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
        frameRate = aFrameRate;            // 每秒的帧率 10Hz
        hFov = ahFov;                 // 水平fov 0~ 30
        vFov = avFov;                 // 垂直fov 0~30
        minimumDetectRange = aMinimumDetectRange;   //最近探测距离
        range = r;                // 最远探测距离 10

        boundingBoxEnable = false;
    }

	int channels;	// 线束
    float rotationFrequency; //扫描帧频
    float horizontalResolution; //水平分辨率
    float upperFov; //上垂直方位角
    float lowerFov; //下垂直方位角
    float atmosphereAttenuationRate; //大气衰减率
    float dropoffGeneralRate; //一般衰减率
    float dropoffIntensityLimit; //衰减强度极限
    float dropoffZeroIntensity; //0衰减率
    float noiseStddev; //加性高斯噪声
    int dumpRawDataType;
    float fWavelength;               // 波长
    float fVBeamDivergenceAngle;     // 光束垂直发散角
    float fHBeamDivergenceAngle;     // 光束水平发散角
    float fTransmittingAperture;     // 发射孔径
    float fReceivingAperture;        // 接收孔径
    float fTransmitTransmissivity;   // 发射孔径透射率
    float fReceiveTransmissivity;    // 接收孔径透射率
    float fNEP;                      // 噪声等效功率
    float fNoiseBandwidth;           // 噪声带宽
    float peakOpticalPower; //峰值光功率
    float receiverAperture; //接收器孔径
    float receiverOpticalEfficiency; //接收器光学系统效率
    float unitGain; //单位增益响应
    float APD; //APD
    float bandwidth;//带宽
    double darkcurrent; //暗电流
    double solarNoise; //太阳辐照度背景噪声
    float excessNoiseFactor; //过量噪声系数
    double thermalNoise; //热噪声
    float reflectivity; //目标反射率
    float falseAlarmRate; //虚警概率
    float photonFluxes;  //光子通量
    bool  advSettingsEnable; //高级选项
	bool isVisibleCloudPoint;
    bool isSavePointCloud;
    bool enableConeRender; // 包络线开关
    bool bExistProbInput;     // 自定义目标存在可能性开关
    float fExistProb;        // 目标存在可能性

public:
    int size()
    {
        return sizeof(LidarDescription);
    }
};



#endif