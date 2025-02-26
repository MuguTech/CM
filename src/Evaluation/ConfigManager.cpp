#include <iostream>
#include "ConfigManager.h"
#include "../log.h"

ConfigManager *ConfigManager::GetInstance()
{
    static ConfigManager instance;
    return &instance;
}

ConfigManager::ConfigManager() {}
ConfigManager::~ConfigManager() {}

bool ConfigManager::init(const std::string& config_file_path)
{
    const bool result = parseConfigFile(config_file_path);

    return result;
}

const std::string &ConfigManager::getKpiList() const
{
    return _kpiList;
}


bool ConfigManager::parseConfigFile(const std::string& config_file_path) {
    simproxml::XMLDocument doc;
    const simproxml::XMLError loadResult = doc.LoadFile(config_file_path.c_str());

    if (loadResult != simproxml::XML_SUCCESS) {
        // std::cout << "ConfigManager::parseConfigFile Error: load file error" << '\n';
        log_evaluation->error("ConfigManager::parseConfigFile Error: load file error");
        return false;
    }

    simproxml::XMLElement* root = doc.RootElement();
    if (root == NULL) {
        // std::cout << "ConfigManager::parseConfigFile Error: root == NULL" << '\n';
        log_evaluation->error("ConfigManager::parseConfigFile Error: root == NULL");
        return false;
    }

    // 遍历二级节点
    for (simproxml::XMLElement* level2Node = root->FirstChildElement(); level2Node != NULL; level2Node = level2Node->NextSiblingElement()) {
        if (strncmp(level2Node->Name(), "Kpilist", 8) == 0) {
            const char* valueAttribute = level2Node->Attribute("value");
            // std::cout << "Kpilist = " << valueAttribute << std::endl;
            log_evaluation->info("Kpilist = {}",valueAttribute);
            if (valueAttribute != NULL) { // value属性存在
                _kpiList = valueAttribute;
            }
        }
    }

    return true;
}

