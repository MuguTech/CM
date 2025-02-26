// TODO
// #include "Common.h"
// #include "CoSimuMaster.h"
#include <unistd.h>
#include "ConfigManager.h"
#include "EvaluationAPI.h"
#include <iostream>
#include <string>
#include <fstream>
// #include "TestController.h"
// TODO
// #include "../Runtime/Setup/WorkspaceManager/WorkspaceManager.cpp"
#include <map>

#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <functional>
#include <random>
#include <CJsonObject.hpp>
#include "SocketLocalClient.h"
// #include "EvpAES.h"
#include "WsAdapter.h"
#include <experimental/filesystem>

#include "MetricName.h"

#include "../log.h"

namespace fs = std::experimental::filesystem;

//simpro从ini配置文件中获取mysql连接信息
// 定义一个类型别名，用于存储配置项的映射
// string 转 unsigned char*
// TODO std::hex 恢复
void decodeUnit(const std::string &str, unsigned char *buf)
{
    log_evaluation->critical("decodeUnit");
    if(!str.empty())
    {
        int size=str.size()/2;
        int i=0;
        std::string szchar;
        std::stringstream ss;
        int itemp;
        for(i=0;i<size;i++)
        {
            szchar=str.substr(i*2,2);
            ss<<std::hex<<szchar<<std::endl;
            ss>>itemp;
            buf[i]=itemp;
        }
        //buf[i]='\0';
    }
    log_evaluation->critical("decodeUnit end");
}

using ConfigMap = std::map<std::string, std::map<std::string, std::string>>;
// #ifndef ENABLE_EVA_PASSWORD
ConfigMap readIniFile(const std::string& filename) {
    ConfigMap config;

    std::ifstream file(filename);
    if (!file) {
        log_evaluation->critical("Error opening file: {}", filename);
        return config;
    }

    std::string currentSection;
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string key, value;

        // 跳过空行和注释行
        if (line.empty() || line[0] == ';' || line[0] == '#') {
            continue;
        }

        // 解析Section
        if (line[0] == '[' && line.back() == ']') {
            currentSection = line.substr(1, line.size() - 2);
            continue;
        }

        // 解析键值对
        if (std::getline(iss, key, '=') && std::getline(iss, value)) {
            config[currentSection][key] = value;
        }
    }

    return config;
}

/* 根据当前时钟创建UUID */
std::string create_uuid()
{
    std::stringstream stream;
    auto random_seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 seed_engine(random_seed);
    std::uniform_int_distribution<std::size_t> random_gen ;
    std::size_t value = random_gen(seed_engine);
    stream << std::hex << value;
    
    std::time_t nowC = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()); /* 当前时间转换为time_t类型 */
    std::tm nowTm = *std::localtime(&nowC); /* 转换为tm结构体 */
    std::string retStr = stream.str() + std::to_string(nowTm.tm_min) + std::to_string(nowTm.tm_sec);
    return retStr;
}

void EvaluationAPI::init_non_fs()
{
    log_evaluation->critical("enter");
    _isUpdateMetricTable = true;
    fs::path current_dir = fs::current_path();
    // const std::string workspacePath = current_dir.string();
    mSaveReportPath = current_dir.string() + "/output/Report"; //测试报告保存地址
}

void EvaluationAPI::notify(const S_MQ_EVALUATION_PARAM* param)
{
    log_evaluation->critical("评估工具收到请求,评估工具执行命令: {}", param->cmdId);

	// TODO
    if (D_QUE_EVALUATION_CMD_RUN_OFFLINE_TOOL == param->cmdId)  
	{
        log_evaluation->critical(" 评估命令： D_QUE_EVALUATION_CMD_RUN_OFFLINE_TOOL");
        runOfflineEvalTool(param);
	}
}

std::string EvaluationAPI::execCmd(const char* cmd) {
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }

    std::string result;
    char buffer[128];
    while (!feof(pipe.get())) {
        if (fgets(buffer, 128, pipe.get()) != nullptr) {
            result += buffer;
        }
    }

    return result;
}

void EvaluationAPI::runOfflineEvalTool(const S_MQ_EVALUATION_PARAM* param)
{
	if (!param) {
        log_evaluation->critical("param is null, please check.");
		return;
	}
// #ifdef ENABLEOFFLINEEVAL
	if (!_isUpdateMetricTable)
	{ /* 用户没有进入指标选择画面的操作时，也没有初始化AllMetricTable时，这里完成首次初始化AllMetricTable。 */
        log_evaluation->critical("[runOfflineEvalTool] call updateMetricTable");
		// updateMetricTable();
		
		_isUpdateMetricTable = true; /* 为了不重复进入本if分支，而立flag。 */
	}

    log_evaluation->critical(" 进入评估工具");
    if(isFirstFlag)
    /*判断是否需要清空评估任务列表*/
    {taskList.clear();}
    std::string task_uuid = create_uuid();
    taskList.push_back(task_uuid);
    std::string EvalRuleIndex = "111111111";
    std::string NewDBNeeded = " -s 1";//置为1说明需要新建DB文件
    std::string NoNewDBNeeded = " -s 0";//置为0说明不需要新建DB文件

    std::string pressStartTime;
    if(param->pressStartTime)
    {
        pressStartTime = (char *) param->pressStartTime;
    }
    log_evaluation->critical("press start time: {}", pressStartTime);

    std::string CSVPath = "";

    std::string scenarioName;
    if(param->scenarioName)
    {
        scenarioName = (char *) param->scenarioName;
    }
    log_evaluation->critical("scenarioName: {}", scenarioName);

    std::string ngInfoCsvPath;
    if(param->NGinformationCsvPath)
    {
        ngInfoCsvPath = (char *) param->NGinformationCsvPath;
    }
    log_evaluation->critical("ngInfoCsvPath: {}", ngInfoCsvPath);

    std::string osiPbPath;
    if(param->OSIGroundTruthPbPath)
    {
        osiPbPath = (char *) param->OSIGroundTruthPbPath;
    }
    log_evaluation->critical("osiPbPath: {}", osiPbPath);
    std::string OSIGroundTruthTTCCsvPath;
    // 查找最后一个'/'字符的位置  
    size_t lastSlashPos = osiPbPath.find_last_of('/');  
    // 如果找到了'/'字符  
    if (lastSlashPos != std::string::npos) {  
        // 截取最后一个'/'字符左边的子串  
        OSIGroundTruthTTCCsvPath = osiPbPath.substr(0, lastSlashPos);   
    } 
    OSIGroundTruthTTCCsvPath.append("/TTCtime.csv");

/* zhangziyi 2023.11.20 [开发案件：评测新增输出算法和动力学信号] */
    std::string dynamicsDataPath;
    if(param->dynamicsDataPath)
    {
        dynamicsDataPath = (char *) param->dynamicsDataPath;
    }
    log_evaluation->critical("rec dynamicsDataPath: {}", dynamicsDataPath);

    std::string algorithmDataCsvPath;
    if(param->algorithmDataCsvPath)
    {
        algorithmDataCsvPath = (char *) param->algorithmDataCsvPath;
    }
    log_evaluation->critical("rec algorithmDataCsvPath: {}", algorithmDataCsvPath);

    std::string scenarioFilePath;
    if(param->scenarioFilePath)
    {
        scenarioFilePath = (char *) param->scenarioFilePath;
    }
    log_evaluation->critical(scenarioFilePath);

/* liujing 2024.2.22 [开发案件：SimPro-支持算法控制TTC输出需求] */
    // TODO
    // std::cout << " liujing1111111111" << '\n';
    // insertTTCtoCsv(param->p_TTCTimeList, OSIGroundTruthTTCCsvPath);
    // std::cout << " liujing222222222" << '\n';

/*  使用MQ方式获取仿真实际时长 */
    //获取仿真时长
    double simulationTime = 0.0;
    if (param->simulationTime)
    {
        simulationTime = param->simulationTime;
    }
    //获取实际用时
    double physicalTime = 0.0;
    if (param->physicalTime)
    {
        physicalTime = param->physicalTime;
    }

    //redmine 2506 & 2507 仿真时间、里程以及录制视频路径追加 2023.03.14
    //获取主车里程
    // AgentVehicle *ego = VehicleManager::Instance()->getEgo();
	// TODO
    // double egoDistance = ScenarioRuntime::instance()->getEgoDistance();
    double egoDistance = param->egoDistance;

    //std::string MP4PathName = ScenarioRuntime::instance()->getMP4recoderPathName();
    std::string MP4PathName;
    if (MP4PathName.empty())
    {   /*处理MP4录制路径为空的情况*/
        MP4PathName = "null";
    }

    std::string pythonParament;
    std::string path = CSVPath;
    int index = static_cast<int>(path.rfind('/'));

    // 这里的pdfName没有生效在offline-evaluation-tool中被截去了
    std::string pdfName = path.erase(0,index+1) + "_" + "第" +std::to_string(param->scenarioTestTimes) + "轮";
    std::string creatNewFilePath;

    /*采用相对路径方式*/
    pythonParament = "python3 ../PythonEval/bin/offlineEvalMain.py -i ";
    // pythonParament = "python3 /opt/simpro/simulator/offline-evaluation-tool/base/offlineEvalMain.py -i ";

    creatNewFilePath = "mkdir -p \"" + mSaveReportPath + "/" + pressStartTime + "/" + std::to_string(param->scenarioTestTimes) + "\"";

    //【评估refine】输入文件只支持PB文件 yangxiaocheng 2023.3.2
    pythonParament += "\"" + osiPbPath + "\"";
 
    pythonParament += " -o \"" + mSaveReportPath + "/" + pressStartTime  + "/" + std::to_string(param->scenarioTestTimes) + "\" " + "\"" + scenarioName + "\" " + EvalRuleIndex;    

    static std::string temp_Group_UUID = create_uuid();
    if(isFirstFlag)//需要新建DB文件
    {
        pythonParament += NewDBNeeded;
        clearIsFirstFlag();
        clearEnvCount();
        addEnvCount();
    }
    else
    {
        pythonParament += NoNewDBNeeded;
        addEnvCount();
    }
    log_evaluation->critical("在groupid: {}, 插入taskid: {}", temp_Group_UUID, task_uuid);

    // if (ConfigManager::Instance()->getNgIntercept())
    if (0)
    {
        pythonParament += " -c \"" + ngInfoCsvPath + "\"" + " -p " + "\"" + osiPbPath + "\"" + " -t " + "\"" + pressStartTime + "\"" + " -f " + "\"" + scenarioFilePath + "\"";
    }

    //redmine 2506 & 2507 仿真时间、里程以及录制视频路径追加 2023.03.14
    pythonParament += " -sumutime \"" + std::to_string(simulationTime) + "\"" + " -mileage " + "\"" + std::to_string(egoDistance) + "\"" + " -mp4path " + "\"" + MP4PathName + "\"" + " -physicalTime " + "\"" + std::to_string(physicalTime) + "\"" ;

    pythonParament += " -dynamics_csv_path \"" + dynamicsDataPath + "\"" + " -algorithm_csv_path " + "\"" + algorithmDataCsvPath + "\"";

    if (temp_Group_UUID != "")
    {
        std::string task_id_str = " -taskid ";
        pythonParament += task_id_str + "\"" + task_uuid + "\"" + " -groupid " + "\"" + temp_Group_UUID + "\"" + " -kpis '" + ConfigManager::GetInstance()->getKpiList() + "' -is_finish " + "\"" + std::to_string(param->isFinish) + "\"" + " -xodr " + "\""  + std::string(param->xodrPath) + "\"" + " -scenarioid " + "\""  + scenarioName + "\""  + " -f " + "\"" + param->scenarioFilePath + "\"" + " -c " + "\"" +  ngInfoCsvPath + "\"" + " -t " + pressStartTime;
    }

    pythonParament += " >/dev/null 2>&1";
    log_evaluation->critical("评估工具执行命令, pythonParament= {}.", pythonParament);

    // TODO check
    (void)std::system(creatNewFilePath.c_str());

    try {
        std::string output = execCmd(pythonParament.c_str());
        log_evaluation->critical("命令输出：{}", output);
    } catch (const std::exception& e) {
        log_evaluation->critical("命令执行出错：{}", e.what());
    }

}

/*将算法信号变化时的时间帧号保存到csv中*/
void EvaluationAPI::insertTTCtoCsv(std::list<std::tuple<double, double>> *p_TTCTimeList,std::string OSIGroundTruthTTCCsvPath)
{
    log_evaluation->critical("insertTTCtoCsv start  OSIGroundTruthTTCCsvPath= {}", OSIGroundTruthTTCCsvPath);
    std::ofstream csvFile(OSIGroundTruthTTCCsvPath);  
    if (csvFile.is_open()) {  
        log_evaluation->critical("scvfile.is_open");
        csvFile << std::fixed << std::setprecision(9);
        csvFile << "starttime";
        csvFile << ","; 
        csvFile << "endtime";
        csvFile << "\n";  
        // 遍历数据并写入到CSV文件中  
            // 使用范围基础的for循环遍历列表  
        for (const auto& item : *p_TTCTimeList) {  
            log_evaluation->critical(" liujing item");
            csvFile << std::get<0>(item);
            // 如果不是行内最后一个元素，则添加一个逗号 
            csvFile << ","; 
            csvFile << std::get<1>(item);
            log_evaluation->critical(" std::get<0>(item)= {}, std::get<1>(item)= {}", std::get<0>(item), std::get<1>(item));
            // 行结束后添加一个换行符  
            csvFile << "\n";  
        }  
    
        // 不要忘记在程序结束前删除动态分配的内存  
        delete p_TTCTimeList; 
        }  
  
    csvFile.close(); // 关闭文件  
    log_evaluation->critical("数据已成功写入CSV文件。");
}

 void EvaluationAPI::setIsFirstFlag()
{
    log_evaluation->critical("初始化测试");
    isFirstFlag = true;
}

void EvaluationAPI::clearIsFirstFlag()
{
    isFirstFlag = false;
}

void EvaluationAPI::setIsSimulatingFlag()
{
    log_evaluation->critical("开始仿真");
    isSimulatingFlag = true;
}

void EvaluationAPI::clearIsSimulatingFlag()
{
    log_evaluation->critical("仿真结束");
    isSimulatingFlag = false;
}

bool EvaluationAPI::isSimulatingDone()
{
    if(isSimulatingFlag == false)
    {
        log_evaluation->critical("所有场景全部执行完了，需要进行数据分析");
        return true;
    }
    else
    {
        log_evaluation->critical("所有场景还没有全部执行完");
        return false;
    }
}


//评估工具refine 接收到评估工具完成后返回消息的后处理
// TODO delete ?
/*进行指标表格刷新*/
void EvaluationAPI::updateMetricTable()
{
    log_evaluation->critical("评估命令： ASIM_SCK_API_ASK_INDEX_LIST");
    std::string pythonParament = "python3 ../PythonEval/bin/offlineEvalMain.py -s 6 ";

    pythonParament += " -taskid " + create_uuid() + " -groupid " + create_uuid();
    pythonParament += " >/dev/null 2>&1";

    /*发送命令*/
    log_evaluation->critical(" 评估工具执行命令: {}", pythonParament);
    WsAdapter::Instance()->startApp(pythonParament.c_str(), "offlineEvalMain\0");
}
