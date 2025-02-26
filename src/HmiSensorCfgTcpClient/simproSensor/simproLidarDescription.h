#ifndef simproLidarDescription_h
#define simproLidarDescription_h

#include <string>
#include <iostream>
#include <string.h>
#include "simproSensorDescription.h"

using namespace std;

struct LidarDescription_simpro : public SensorDescription_simpro
{
	//参照KITTI的velodyne参数，设置默认值
    LidarDescription_simpro(string n = "untitled lidar", float aPositionX = 0.0, float aPositionY = 0.0, float aPositionZ = 1.5, int c = 64,
        float aHeading = 0.0, float aPitch = 0.0, float aRoll = 0.0, float hFov = 360.0, float vFov = 40.0, float aMinimumDetectRange = 0.0,
        float r = 250.0, float rFrequency = 10.0, float hResolution = 0.09, float uFov = 10.0, float lFov = -16.8, float aAttenuationRate = 0.004,
        float dGeneralRate = 0.45, float dIntensityLimit = 0.8, float dZeroIntensity = 0.4, float nStddev = 0.0, float apeakOpticalPower = 90.0,
        float aVBeamDivAngle = 1.0, float aHBeamDivAngle = 1.0, float aTAperture = 1.0, float aRAperture = 1.0,
        float aTTransmissivitye = 1.0, float aRTransmissivity = 1.0, float aNEP = 1.0, float aNoiseBandwidth = 1.0,
        float areceiverAperture = 50.0, float areceiverOpticalEfficiency = 0.8, float aunitGain = 10000000.0,
        float aAPD = 100.0, float abandwidth = 18,float aphotonFluxes = 3, float areflectivity = 0.5,
        float aexcessNoiseFactor = 5.0, float afalseAlarmRate = 0.1, int aFrameRate = 10, double aideal_Freq = 10, 
        float angleOfBeamDivergence = 0.001,/* float pulseWireHarness = 64,*/ int pulseWaveForm = 2, float pulseDivergenceAngle = 0.00555, 
        float waveLength = 1000.0, float delayOfPulse = 20.0, float peakPower = 100.0,  float speed = 0.0, float frequency = 10.0, 
        float gainOfEmission = 1.0, float sizeOfAperture = 0.01, float rotaryPulseResolution = 00555, float scanningPulseResolution = 00555,
        float atmosphericExtinctionCoefficient = 1.0, float OpticalSystemExtinctionCoefficient = 1.0, int operatingMode = 0,
        float afExistProb = 100, SENSOR_MODEL_TYPE amodelType = SENSOR_MODEL_IDEAL):
        channels(c), 
        horizontalFov(hFov),
        verticalFov(vFov),
        minimumDetectRange(aMinimumDetectRange),
        range(r), 
        rotationFrequency(rFrequency), 
        horizontalResolution(hResolution), 
        upperFov(uFov), 
        lowerFov(lFov), 
        atmosphereAttenuationRate(aAttenuationRate), 
        dropoffGeneralRate(dGeneralRate), 
        dropoffIntensityLimit(dIntensityLimit), 
        dropoffZeroIntensity(dZeroIntensity), 
        noiseStddev(nStddev),
        frameRate(aFrameRate),
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
        bPhySettingsEnable = false; 
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

        // bPhySettingsEnable = false;                                            //物理参数开关
        fAngleOfBeamDivergence = angleOfBeamDivergence;                           //光束发散角
        // fPulseWireHarness = pulseWireHarness;                                     //脉冲线束
        iPulseWaveForm = pulseWaveForm;                                           //脉冲波形
        fPulseDivergenceAngle = pulseDivergenceAngle;                             //脉冲发散角
        fPhysicalWaveLength = waveLength;                                                 //波长
        fDelayOfPulse = delayOfPulse;                                             //脉冲延时
        fPeakPower = peakPower;                                                   //峰值功率
        fSpeed = speed;                                                           //转速
        fFrequency = frequency;                                                   //频率
        fGainOfEmission = gainOfEmission;                                         //发射增益
        fSizeOfAperture = sizeOfAperture;                                         //光阑大小
        fRotaryPulseResolution = rotaryPulseResolution;                           //旋转式脉冲分辨率
        fScanningPulseResolution = scanningPulseResolution;                       //扫描式脉冲分辨率
        fAtmosphericExtinctionCoefficient = atmosphericExtinctionCoefficient;     //大气消光系数
        fOpticalSystemExtinctionCoefficient = OpticalSystemExtinctionCoefficient; //光学系统消光系数
        iOperatingMode = operatingMode;                                           //运行模式
    }
    float horizontalFov; //水平视场角 
    float verticalFov; //垂直视场角 
    float minimumDetectRange;
	int channels;	// 线束
    float range; //最大探测距离
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
    int frameRate;
    float hFov;                 // 水平fov 0~ 30
    float vFov;                 // 垂直fov 0~30
    float fWaveLength;               // 波长
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
	double ideal_Freq;	// 理想的频率参数

    bool bPhySettingsEnable;                   //物理参数开关
    float fAngleOfBeamDivergence;              //光束发散角
    // float fPulseWireHarness;                   //脉冲线束
    int iPulseWaveForm;                        //脉冲波形
    float fPulseDivergenceAngle;               //脉冲发散角
    float fPhysicalWaveLength;                         //波长
    float fDelayOfPulse;                       //脉冲延时
    float fPeakPower;                          //峰值功率
    float fSpeed;                              //转速
    float fFrequency;                          //频率
    float fGainOfEmission;                     //发射增益
    float fSizeOfAperture;                     //光阑大小
    float fRotaryPulseResolution;              //旋转式脉冲分辨率
    float fScanningPulseResolution;            //扫描式脉冲分辨率
    float fAtmosphericExtinctionCoefficient;   //大气消光系数
    float fOpticalSystemExtinctionCoefficient; //光学系统消光系数
    int iOperatingMode;                        //运行模式
    int MaxNumOfReflections;		           //最大反射数（默认为1）	

public:
    int size()
    {
        return sizeof(LidarDescription);
    }

    int getFrameRate()
    {
        return frameRate;
    }
};


#endif