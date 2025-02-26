#ifndef MQEVALUATION_H
#define MQEVALUATION_H

#include <iostream>
#include <list>

#define D_QUE_EVALUATION_CMD_RUN_OFFLINE_TOOL (1)
#define D_QUE_EVALUATION_CMD_CALC_RSS         (2)
#define D_QUE_EVALUATION_CMD_RUN_DATA_ANALYZE (3)
#define D_QUE_SCENARIO_CAPTURE_NG_CSV_READY_CMD (4)

#define D_PARAM_PRESS_START_TIME_SIZE  (1024)
#define D_PARAM_SCENARIO_NAME_SIZE  (1024)
#define D_PARAM_NGINFO_CSV_PATH_SIZE  (1024)
#define D_PARAM_OSI_PB_PATH_SIZE  (1024)

#define D_PARAM_XODR_PATH_SIZE (1024)

#define D_ASIM_SCK_API_REQ_DYN_PATH (1024)
#define D_ASIM_SCK_API_REQ_AD_PATH (1024)

typedef unsigned int      uint32_t;
typedef unsigned char      uint8_t;

/* S_QUE --> parameters */
typedef struct s_mq_param_
{
	uint32_t cmdId;                 /* : 命令ID */
	uint32_t scenarioTestTimes;     /* :  测试轮数 */
	uint8_t pressStartTime[D_PARAM_PRESS_START_TIME_SIZE];     /* :  按下开始按钮的时间 */

    uint8_t scenarioName[D_PARAM_SCENARIO_NAME_SIZE];// TODO

	char NGinformationCsvPath[D_PARAM_NGINFO_CSV_PATH_SIZE]; //存储NG场景信息的CSV文件路径
	char OSIGroundTruthPbPath[D_PARAM_OSI_PB_PATH_SIZE]; //OSIGroundTruth的Pb文件地址
	char scenarioFilePath[D_PARAM_SCENARIO_NAME_SIZE]; //存储泛化前的初始场景文件路径

	double physicalTime; //场景实际用时
	double simulationTime; //场景仿真用时

    char dynamicsDataPath[D_ASIM_SCK_API_REQ_DYN_PATH];
    char algorithmDataCsvPath[D_ASIM_SCK_API_REQ_AD_PATH];

    std::list<std::tuple<double, double>> *p_TTCTimeList; //需要输出TTC的时间段列表 tuple<startTime, endTime>
    uint32_t isFinish; // 是否是本场景的最后一个OSI pb文件
    char xodrPath[D_PARAM_XODR_PATH_SIZE]; // 存储地图文件路径
    double egoDistance; // 主车行驶的里程数   
} S_MQ_EVALUATION_PARAM;

#endif /* MQEVALUATION_H */
