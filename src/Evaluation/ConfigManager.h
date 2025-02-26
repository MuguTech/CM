#ifndef SAIMOSIM_CONFIGMANAGER_H
#define SAIMOSIM_CONFIGMANAGER_H

#include <string>
#include <simproxml.h>


class ConfigManager
{
public:
    static ConfigManager* GetInstance();
	const std::string &getKpiList() const;
	bool init(const std::string& config_file_path);

protected:
    ConfigManager();
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;
    ~ConfigManager();

private:
    bool parseConfigFile(const std::string& config_file_path);
    std::string _kpiList;
};


#endif //SAIMOSIM_CONFIGMANAGER_H
