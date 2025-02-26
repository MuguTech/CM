#ifndef simproSensorDescription_h
#define simproSensorDescription_h

#include <string>
#include <CJsonObject.hpp>

struct SensorDescription_simpro
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
	SENSOR_MODEL_TYPE modelType;	// 传感器模型类型，初始化为IDEAL
	int object_detection_type;	//物体识别类别

    int EnableCommunicationPort;   //通信端口是否开启字段：EnableCommunicationPort  '0'关闭（默认）  '1'开启
    char CommunicationPortIP[40]; //通信端口IP字段：CommunicationPortIP 默认为空，长度不超过40
    int CommunicationPort;  //通信端口端口字段：CommunicationPort 默认为空，输入范围[0,65535]
    int CommunicationPortType; //通信端口方式：CommunicationPortType '1'TCP  '2'UDP（默认）
    char model[128];				//传感器型号
    bool  boundingBoxEnable = false;   

public:
	virtual int size()
	{
		return sizeof(SensorDescription);
	};

	virtual int getFrameRate()
	{
		return 0;
	};
};



#endif