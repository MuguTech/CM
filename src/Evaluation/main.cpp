#include "MqEvaluation.h"
#include "EvaluationAPI.h"
#include "ConfigManager.h"
#include <cstring>
#include <string>
#include "../log.h"


int main() {
	ConfigManager::GetInstance()->init("../data.xml");
    log_evaluation->info("kpi list: {}",ConfigManager::GetInstance()->getKpiList());
    S_MQ_EVALUATION_PARAM param;
    param.cmdId = D_QUE_EVALUATION_CMD_RUN_OFFLINE_TOOL;
    param.scenarioTestTimes = 123;
    strcpy((char*)param.pressStartTime, "2024-3-20_08_10_30_213");    // -t
    strcpy((char*)param.xodrPath, "/tmp/12.xodr");             // -xodr
    strcpy((char*)param.OSIGroundTruthPbPath, "/VirtualCity/12345/GroundTruthPB/123/SimProGroundTruth_1.pb");  // -i
    std::string scenario_path = "/tmp/1.xosc";
    strcpy((char*)param.scenarioName, scenario_path.c_str());   // -scenarioid
    strcpy((char*)param.scenarioFilePath, scenario_path.c_str());  // -f
    strcpy((char*)param.NGinformationCsvPath, "/tmp/ng_info.csv");  // -c
    param.isFinish = 0;      // -is_finish
    param.egoDistance = 300; // -mileage

    EvaluationAPI::Instance()->notify(&param);

    return 0;
}
