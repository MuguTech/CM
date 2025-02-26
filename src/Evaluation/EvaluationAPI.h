#ifndef EVALUATIONAPI_H
#define EVALUATIONAPI_H

#define D_EGOVEHTYPE    (0)
#define D_GNSSIMUTYPE   (1)
#define D_OBJTYPE       (2)
#define D_TRAFSIGTYPE   (3)
#define D_TRAFLIGHTTYPE (4)
#define D_LANETYPE      (5)


#include "MqEvaluation.h"
#include <set>
#include <map>
#include <list>
#include <vector>
#include <mutex>
//#include <mysql/mysql.h> // mysql文件

#include "SocketLocalClient.h"

#define D_EVALUATION_API_SIMU_DATA_SIZE (102400*6) /* 6個type數據，每個type數據最大102400字節 */

class EvaluationAPI //: public WsAdapterInterface
{
public:
    static EvaluationAPI *Instance();
    static void Destroy();
	void setIsFirstFlag();
	void clearIsFirstFlag();
	void setIsSimulatingFlag();
	void clearIsSimulatingFlag();
	bool isSimulatingDone();
	int getEnvCount(){return Env_count;}
	void addEnvCount(){Env_count++;}
	void clearEnvCount(){Env_count = 0;}

	/* 非功能安全 start */
	void notify(const S_MQ_EVALUATION_PARAM* param); /* from message queue */

protected:
    EvaluationAPI();
    ~EvaluationAPI();
	
private:
    static EvaluationAPI *_instance;
	bool isFirstFlag = true;
	bool isSimulatingFlag = false;
	int Env_count = 0;

    std::string execCmd(const char* cmd);
	/* 功能安全 start */
	void wrapperMemcpyToSimuData(const char * src);
    // int8_t mStrSimuData[D_EVALUATION_API_SIMU_DATA_SIZE];
    // uint32_t mU4SimuDataCurCount; /* 當前mStrSimuData的字節數 */
	/* 功能安全 end */

	/* 非功能安全 start */
	void init_non_fs();

	void runOfflineEvalTool(const S_MQ_EVALUATION_PARAM* param);

	std::string mSaveReportPath; //测试报告保存地址
	std::string mAbsolutePathStr;
	std::vector<std::string> mEvaluationRuleVec; //所有的评估指标
    std::set<int> mSelectedEvalRuleSet; //已选中的评估指标


	/* 数据库连接需要使用的数据结构 */
	// MYSQL mysql;//数据库句柄
	// MYSQL_RES* res;//查询结果集
	// MYSQL_ROW row;//记录结构体

	/* 记录已经完成的且可以用于页面显示的Group UUID */
	std::string finished_Group_UUID;

	// std::string temp_Group_UUID;

	/*显示HMI粗略总评信息*/
	void showOverallWebInfo();

	/*生成NG场景文件*/
	void produceNGxosc(std::string ngcsvPath);
	/*进行总评PDF生成*/
	void createOverallPDF(std::string groupID);
	/*进行指标表格刷新*/
	void updateMetricTable();
	/*进行指标表格刷新*/
	void updateWebMetric();
    /*进行算法信号设定发送*/
    void updateWebAlgorithm();
    void sendAlgorithm();
	/*将算法信号变化时的时间帧号保存到csv中*/
	void insertTTCtoCsv(std::list<std::tuple<double, double>> *p_TTCTimeList,std::string OSIGroundTruthTTCCsvPath);

	/* 非功能安全 end */
	std::list<std::string> taskList;
	std::string config_host;
    std::string config_user;
    std::string config_password;
    std::string config_database;
    std::string config_port;
	SocketLocalClient slclient;
	std::mutex sqlClientMutex;
	bool _isUpdateMetricTable;
};
#endif // EVALUATIONAPI_H
