#ifndef CameraDescription_h
#define CameraDescription_h

#include <string>
#include <iostream>
#include <string.h>
#include "SensorDescription.h"

using namespace std;

struct CameraDescription : public SensorDescription
{
    CameraDescription(string n = "untitled Camera", int cID = 0, float aPositionX = 0.0, float aPositionY = 0.0, float aPositionZ = 1.5, float aHeading = 32,
        float aRoll = 10.0, float aPitch = 10.0, float ahFov = 60.0, float avFov = 28.0, float aMinimumDetectRange = 0.0, float t = 0.005, float detDis = 100.0,
        string aModle = "untitled Camera Modle", float aProbabilityDistribution = 2.0, float aThreshold = 0.9, int aFrameRate = 15,
		uint32_t aresolutionL = 640, uint32_t aresolutionV = 480, double asensorSizeL = 12.8, double asensorSizeW = 9.6,
		double asensorFD = 12, double adistCenterL = 6.4, double adistCenterV = 4.8, double adistParamK1 = 1, double adistParamK2 = 1,
        double apixelSize = 0.512, double agaussSigma = 0, bool adistSw = false,
		SENSOR_MODEL_TYPE amodelType = SENSOR_MODEL_IDEAL ) :
        cameraID(cID),
        tick(t),
        probabilityDistribution(aProbabilityDistribution),
        threshold(aThreshold),
        resolutionL(aresolutionL),
		resolutionV(aresolutionV),
		sensorSizeL(asensorSizeL),
		sensorSizeW(asensorSizeW),
		sensorFD(asensorFD),
		distCenterL(adistCenterL),
		distCenterV(adistCenterV),
		distParamK1(adistParamK1),
		distParamK2(adistParamK2),
        pixelSize(apixelSize),
        gaussSigma(agaussSigma),
        distSw(adistSw)
    {
        type = SENSOR_TYPE_CAMERA;
        strcpy(name, n.c_str());
        enable = false;
        sensorOutputType = 0; // 包络线[Add] 2022.08.22 LS
        isSaveRGB = false; 
        strcpy(model, aModle.c_str());
        enableConeRender = false; // 包络线[Add] 2022.08.22 LS -> [bug9111] 默认为false LS 08.30
        advSettingsEnable = 0;

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
        range = detDis;                // 最远探测距离 10

        boundingBoxEnable = false; //输出BoundingBox开关
    }

    int   cameraID; //camera���
    int frameRate;
    float tick; //����������ʱ����
    float probabilityDistribution;  // 目标的概率分布参数
    float threshold;                // 阈值
    bool isSaveRGB;
    bool enableConeRender; // 包络线开关
    bool  advSettingsEnable;        // 高级选项开关
    char dummy[1];
    char model[128];

public:
    int size()
    {
        return sizeof(CameraDescription);
    };

    uint32_t resolutionL;		// 水平分辨率，初始化为640
	uint32_t resolutionV;		// 垂直分辨率，初始化为480
	double sensorSizeL;	// 传感器长度，初始化为12.8mm，按照一英寸大小
	double sensorSizeW;	// 传感器宽度，初始化为9.6mm，按照一英寸大小
	double sensorFD;		// 传感器焦距，初始化为12mm
	double distCenterL;	// 横向畸变中心，初始化为6.4 mm
	double distCenterV;	// 垂直畸变中心，初始化为4.8 mm
	double distParamK1;	// 畸变参数K1，初始化为1
	double distParamK2;	// 畸变参数K2，初始化为1
    double pixelSize;   // 像素尺寸，初始化为0.512 mm^2
    double gaussSigma;  // 高斯标准差，初始化为0
    bool distSw;    // 畸变开关，初始化为false
};

#endif