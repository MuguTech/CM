#include "ConfigureMngr.h"
#include "simproxml/simproxml.h"
#include "log.h"
#include <algorithm>
#include "OSIGroundTruthGenerator/OSIGroundTruthGeneratorThread.h"



ConfigureMngr *ConfigureMngr::_instance = NULL;


ConfigureMngr::ConfigureMngr()
{
    simu_frame_ctl = -1;
    frame_rate = -1;
    sensorControllerMode = 0;
    init = false;

    isVirtualCity = false; //是否开启虚拟城市

    if (parseVirtualCityConfigFile() == false)
    {
        log_compnt_mngr->error("parseVirtualCityConfigFile false");
    }

    isOutputOsiGroundTruthLogicalLane = true;
    coSimuInterfaceType = 1;
    controlInLoopValue = 0;
    controlInLoopType = 1;
    frameRate = 50;
    coSimuSocketMode = 1;

    mainVehicleId = 0;

    load_cfg();
    print_cfg();
}


ConfigureMngr::~ConfigureMngr()
{
    while (true != p_plugin_list.empty())
    {
        delete *(p_plugin_list.begin());
        p_plugin_list.erase(p_plugin_list.begin());
    }

    if (NULL != _instance)
    {
        delete _instance;
        _instance = NULL;
    }
}



ConfigureMngr *ConfigureMngr::getInstance()
{
    if (NULL == _instance)
    {
        _instance = new ConfigureMngr();
    }

    return _instance;
}



void ConfigureMngr::update_cfg()
{
    load_cfg();
    print_cfg();
}


void ConfigureMngr::load_cfg()
{
    log_compnt_mngr->info("ConfigureMngr::load_cfg start.");

    if (isVirtualCity)
    {
        // 获取环境变量CO_SIMU_INTERFACE_TYPE
        std::string env_value = "";
        char* env_p = std::getenv("CO_SIMU_INTERFACE_TYPE");
        if (env_p)
        {
            env_value = env_p;  // 转换为 std::string
            if (env_value == "2")
            {
                coSimuInterfaceType = 2;
            }
            else
            {
                coSimuInterfaceType = 1;
            }
        }
        else
        {
            log_compnt_mngr->error("get env CO_SIMU_INTERFACE_TYPE error.");
        }

        // 获取环境变量CONTROL_IN_LOOP_VALUE
        env_p = std::getenv("CONTROL_IN_LOOP_VALUE");
        if (env_p)
        {
            env_value = env_p;  // 转换为 std::string
            if (env_value == "1")
            {
                controlInLoopValue = 1;
            }
            else
            {
                controlInLoopValue = 0;
            }
        }
        else
        {
            log_compnt_mngr->error("get env CONTROL_IN_LOOP_VALUE error.");
        }

        // 获取环境变量CONTROL_IN_LOOP_TYPE
        env_p = std::getenv("CONTROL_IN_LOOP_TYPE");
        if (env_p)
        {
            env_value = env_p;  // 转换为 std::string
            if (env_value == "2")
            {
                controlInLoopType = 2;
            }
            else
            {
                controlInLoopType = 1;
            }
        }
        else
        {
            log_compnt_mngr->error("get env CONTROL_IN_LOOP_TYPE error.");
        }

        // 获取环境变量FRAME_RATE
        env_p = std::getenv("FRAME_RATE");
        if (env_p)
        {
            env_value = env_p;  // 转换为 std::string
            frameRate = std::stoi(env_value);
        }
        else
        {
            log_compnt_mngr->error("get env FRAME_RATE error.");
        }

        // 获取环境变量CO_SIMU_SOCKET_MODE
        env_p = std::getenv("CO_SIMU_SOCKET_MODE");
        if (env_p)
        {
            env_value = env_p;  // 转换为 std::string
            if (env_value == "2")
            {
                coSimuSocketMode = 2;
            }
            else
            {
                coSimuSocketMode = 1;
            }
        }
        else
        {
            log_compnt_mngr->error("get env CO_SIMU_SOCKET_MODE error.");
        }

        // 获取环境变量MAIN_VEHICLE_ID
        env_p = std::getenv("MAIN_VEHICLE_ID");
        if (env_p)
        {
            env_value = env_p;  // 转换为 std::string
            mainVehicleId = std::stoi(env_value);
        }
        else
        {
            log_compnt_mngr->error("get env MAIN_VEHICLE_ID error.");
        }
    }

    std::string cfg_file = CONFIGURE_FILE;
    simproxml::XMLDocument xml_document;
    simproxml::XMLElement *root = NULL;

    if (true != is_file_exist(cfg_file))
    {
        log_compnt_mngr->error("CONFIGURE FILE {} NOT EXIST", cfg_file);
        return;
    }

    /* 加载配置文件 */
    const simproxml::XMLError loadResult = xml_document.LoadFile(cfg_file.c_str());
    if (loadResult != simproxml::XML_SUCCESS)
    {
        log_compnt_mngr->error("OPEN CFG FILE {} ERROR.", cfg_file.c_str());
        log_compnt_mngr->error("loadResult = {}", loadResult);
        return;
    }

    root = xml_document.RootElement();
    if (root == NULL)
    {
        log_compnt_mngr->error("CFG XML DOC ROOT NULL");
        return;
    }

    for (simproxml::XMLElement *level2Node = root->FirstChildElement(); 
        level2Node != NULL;
        level2Node = level2Node->NextSiblingElement())
    {
        if (0 == strncmp(level2Node->Name(), "SimuFrameCtrl", strlen("SimuFrameCtrl")))
        {
            const char *value = level2Node->Attribute("value");
            if (NULL == value)
            {
                log_compnt_mngr->error("get frame ctrl err");
                continue;
            }

            set_frame_ctl(atoi(value));

            const char *ip = level2Node->Attribute("ip");
            if (NULL == ip)
            {
                log_compnt_mngr->error("get simulator ip err");
                continue;
            }

            const char *port = level2Node->Attribute("port");
            if (NULL == port)
            {
                log_compnt_mngr->error("get simulator port err");
                continue;
            }

            const char *type = level2Node->Attribute("type");
            if (NULL == type)
            {
                log_compnt_mngr->error("get simulator type err");
                continue;
            }

            std::string typeStr = type;
            transform(typeStr.begin(), typeStr.end(), typeStr.begin(), ::tolower); //转化为小写字母
            set_frame_ctl_socket(ip, port, typeStr);
        }

        if (0 == strncmp(level2Node->Name(), "FrameRate", strlen("FrameRate")))
        {
            const char *value = level2Node->Attribute("value");
            if (NULL == value)
            {
                log_compnt_mngr->error("get frame rate err");
                continue;
            }

            set_frame_rate(atoi(value));
        }

        if (0 == strncmp(level2Node->Name(), "Simulator", strlen("Simulator")))
        {
            const char *ip = level2Node->Attribute("ip");
            if (NULL == ip)
            {
                log_compnt_mngr->error("get simulator ip err");
                continue;
            }

            const char *port = level2Node->Attribute("port");
            if (NULL == port)
            {
                log_compnt_mngr->error("get simulator port err");
                continue;
            }

            const char *type = level2Node->Attribute("type");
            if (NULL == type)
            {
                log_compnt_mngr->error("get simulator type err");
                continue;
            }

            std::string typeStr = type;
            transform(typeStr.begin(), typeStr.end(), typeStr.begin(), ::tolower); //转化为小写字母
            set_simulator_socket(ip, port, typeStr);
        }

        if (0 == strncmp(level2Node->Name(), "LibDir", strlen("LibDir")))
        {
            const char *value = level2Node->Attribute("value");
            if (NULL == value)
            {
                log_compnt_mngr->error("get lib dir err");
                continue;
            }

            set_so_dir(value);
        }

        if (0 == strncmp(level2Node->Name(), "OSIGroundTruthLogicalLane", strlen("OSIGroundTruthLogicalLane")))
        {
            const char *value = level2Node->Attribute("enable");
            if (NULL == value)
            {
                log_compnt_mngr->error("get output osi groundtruth logical lane err");
                continue;
            }

            std::string strIsOutputOsiGroundTruthLogicalLane = std::string(value);

            if (strIsOutputOsiGroundTruthLogicalLane == "0")
            {
                isOutputOsiGroundTruthLogicalLane = false;
            }
            else
            {
                isOutputOsiGroundTruthLogicalLane = true;
            }
        }

        // 1000HZ使用动力学插件
        if (0 == strncmp(level2Node->Name(), "DynamicsPlugin", strlen("DynamicsPlugin")))
        {
            const char *name = level2Node->Attribute("name");
            if (NULL == name)
            {
                log_compnt_mngr->error("get dynmc plugin name err.");
                continue;
            }

            const char *type = level2Node->Attribute("type");
            if (NULL == type)
            {
                log_compnt_mngr->error("get dynmc type err.");
                continue;
            }

            const char *id = level2Node->Attribute("id");
            if (NULL == id)
            {
                log_compnt_mngr->error("get dynmc type err.");
                continue;
            }

            DynamicVehicleCfg *p_dynmcCfg = new DynamicVehicleCfg();
            p_dynmcCfg->set_name(name);
            p_dynmcCfg->set_type(type);
            p_dynmcCfg->set_id(id);

            for (simproxml::XMLElement *level3Node = level2Node->FirstChildElement(); level3Node != NULL; level3Node = level3Node->NextSiblingElement())
            {
                if (0 == strncmp(level3Node->Name(), "Load", strlen("Load")))
                {
                    const char *lib = level3Node->Attribute("lib");
                    if (NULL == lib)
                    {
                        log_compnt_mngr->error("get dynmc lib err.");
                        continue;
                    }

                    p_dynmcCfg->set_lib(lib);
                }
            }

            add_plugin_list(p_dynmcCfg);
        }
        if (0 == strncmp(level2Node->Name(),"CmSensorServer",strlen("CmSensorServer")))
        {
            const char *ip = level2Node->Attribute("ip");
            if (NULL == ip)
            {
                log_compnt_mngr->error("get CmSensorServer ip err");
                continue;
            }

            const char *port = level2Node->Attribute("port");
            if (NULL == port)
            {
                log_compnt_mngr->error("get CmSensorServer port err");
                continue;
            }

            set_cm_sensor_server(ip,port,"tcp");
        }

        if (0 == strncmp(level2Node->Name(),"SensorControllerMode",strlen("SensorControllerMode")))
        {
            const char *value = level2Node->Attribute("value");
            if (NULL == value)
            {
                log_compnt_mngr->error("get CmSensorServer ip err");
                continue;
            }
            else
            {
                sensorControllerMode = atoi(value);
            }
        }

#ifndef V1000HZ
        if (0 == strncmp(level2Node->Name(), "Sensor", strlen("Sensor")))
        {
            const char *name = level2Node->Attribute("name");
            if (NULL == name)
            {
                log_compnt_mngr->error("get sensor plugin name err.");
                continue;
            }

            const char *type = level2Node->Attribute("type");
            if (NULL == type)
            {
                log_compnt_mngr->error("get sensor type err.");
                continue;
            }

            const char *id = level2Node->Attribute("id");
            if (NULL == id)
            {
                log_compnt_mngr->error("get sensor type err.");
                continue;
            }

            const char *s_freq = level2Node->Attribute("freq");
            if (!s_freq) {
                log_compnt_mngr->error("get sensor freq err.");
                continue;
            }

            SensorDescription *pDesc = nullptr;
            std::string typeStr = type;
            if (typeStr == "lidar")
            {
                pDesc = (SensorDescription *)(new LidarDescription());
            }
            else if (typeStr == "camera")
            {
                pDesc = (SensorDescription *)(new CameraDescription());
            }
            else if (typeStr == "radar")
            {
                pDesc = (SensorDescription *)(new RadarDescription());
            }
            else if (typeStr == "ultrasonic")
            {
                pDesc = (SensorDescription *)(new UltrasonicDescription());
            }
            else
            {
                pDesc = new SensorDescription();
            }
            SensorCfg *p_sensorcfg = new SensorCfg(pDesc);

            p_sensorcfg->set_name(name);
            p_sensorcfg->set_type(type);
            p_sensorcfg->set_id(id);
            p_sensorcfg->setFrmRate(atoi(s_freq));

            for (simproxml::XMLElement *level3Node = level2Node->FirstChildElement(); 
                level3Node != NULL;
                level3Node = level3Node->NextSiblingElement())
            {
                if (0 == strncmp(level3Node->Name(), "Load", strlen("Load")))
                {
                    const char *lib = level3Node->Attribute("lib");
                    if (NULL == lib)
                    {
                        log_compnt_mngr->error("get sensor lib err.");
                        continue;
                    }

                    p_sensorcfg->set_lib(lib);
                }

                if (0 == strncmp(level3Node->Name(), "Position", strlen("Position")))
                {
                    const char *dx = NULL;
                    const char *dy = NULL;
                    const char *dz = NULL;
                    const char *dh = NULL;
                    const char *dp = NULL;
                    const char *dr = NULL;

                    dx = level3Node->Attribute("dx");
                    dy = level3Node->Attribute("dy");
                    dz = level3Node->Attribute("dz");
                    dh = level3Node->Attribute("dh");
                    dp = level3Node->Attribute("dp");
                    dr = level3Node->Attribute("dr");

                    if (NULL == dx || NULL == dy || NULL == dz || 
                        NULL == dh || NULL == dp || NULL == dr)
                    {
                        log_compnt_mngr->error("get sensor position err.");
                        continue;
                    }

                    p_sensorcfg->set_position(atof(dx), atof(dy), atof(dz), atof(dh), atof(dp), atof(dr));
                }

                if (0 == strncmp(level3Node->Name(), "Port", strlen("Port")))
                {
                    const char *enable = level3Node->Attribute("enable");
                    if (NULL == enable)
                    {
                        log_compnt_mngr->error("get sensor socket enable err.");
                        continue;
                    }
                    p_sensorcfg->set_socket_enable(enable);
                    const char *name = NULL;
                    const char *ip = NULL;
                    const char *port = NULL;
                    const char *type = NULL;

                    name = level3Node->Attribute("name");
                    ip = level3Node->Attribute("ip");
                    port = level3Node->Attribute("port");
                    type = level3Node->Attribute("type");

                    if (isVirtualCity)
                    {
                        if (coSimuSocketMode == 2)
                        {
                            type = "tcp";
                        }
                        else
                        {
                            type = "udp";
                        }
                    }

                    if (NULL == name || NULL == ip || NULL == port || NULL == type)
                    {
                        log_compnt_mngr->error("get sensor socket err.");
                        continue;
                    }

                    std::string typeStr = type;
                    transform(typeStr.begin(), typeStr.end(), typeStr.begin(), ::tolower); //转化为小写字母
                    p_sensorcfg->set_socket(name, ip, port, typeStr);
                }

                if (0 == strncmp(level3Node->Name(), "Filter", strlen("Filter")))
                {
                    const char *objectType = level3Node->Attribute("objectType");
                    if (NULL == objectType)
                    {
                        log_compnt_mngr->error("get sensor filter obj type err.");
                        continue;
                    }

                    p_sensorcfg->add_filter(objectType);
                }

                if (0 == strncmp(level3Node->Name(), "FOV", strlen("FOV"))) {
                    const char* hFOV = level3Node->Attribute("hFOV");
                    const char* vFOV = level3Node->Attribute("vFOV");
                    if (!hFOV || !vFOV) {
                        log_compnt_mngr->error("get sensor filter FOV type err.");
                        continue;
                    }
                    else {
                        p_sensorcfg->set_fov(atof(hFOV), atof(vFOV));
                    }
                }
                if (0 == strncmp(level3Node->Name(), "Frustum", strlen("Frustum"))) {
                    const char* near = level3Node->Attribute("near");
                    const char* far = level3Node->Attribute("far");
                    if (!near || !far) {
                        log_compnt_mngr->error("get sensor Frustum obj type err.");
                        continue;
                    }
                    else {
                        p_sensorcfg->set_detect_dist(atof(near), atof(far));
                    }
                }
                if (0 == strncmp(level3Node->Name(), "BoundingBox", strlen("BoundingBox"))) {
                    const char* enable = level3Node->Attribute("enable");
                    if (!enable ) {
                        log_compnt_mngr->error("get sensor boundingbox obj type err.");
                        continue;
                    }
                    else {
                        log_compnt_mngr->debug("get sensor boundingbox enable={}", enable);
                        p_sensorcfg->set_boundingbox_enable(enable);
                    }
                }
                log_compnt_mngr->debug("get sensor desc boundingbox enable={}", p_sensorcfg->get_boundingbox_enable() );
            }

            add_plugin_list(p_sensorcfg);
        }
#endif

    }

    init = true;
    log_compnt_mngr->info("ConfigureMngr::load_cfg end.");
    return;
}



void ConfigureMngr::print_cfg()
{
    log_compnt_mngr->info("ConfigureMngr::print_cfg start.");
    if (true != init)
    {
        log_compnt_mngr->error("ConfigureMngr::print_cfg init is flase,return.");
        return;
    }

    log_compnt_mngr->debug("PRINT CONFIGURE INFO");
    log_compnt_mngr->debug("simulator frame ctrl = {}, ip = {}, port = {}, type = {}."
            , get_frame_ctl(), get_frame_ctl_socket().ip.c_str()
            , get_frame_ctl_socket().port.c_str()
            , get_frame_ctl_socket().type.c_str());
    log_compnt_mngr->debug("frame rate = {}.", get_frame_rate());
    log_compnt_mngr->debug("simulator: ip {}, port {}, type {}."
            , get_simulator_socket().ip.c_str()
            , get_simulator_socket().port.c_str()
            , get_simulator_socket().type.c_str());
    log_compnt_mngr->debug("lib dir = {}.", get_so_dir().c_str());

    log_compnt_mngr->debug("env coSimuInterfaceType = {}.", coSimuInterfaceType);
    log_compnt_mngr->debug("env controlInLoopValue = {}.", controlInLoopValue);
    log_compnt_mngr->debug("env controlInLoopType = {}.", controlInLoopType);
    log_compnt_mngr->debug("env frameRate = {}.", frameRate);
    log_compnt_mngr->debug("env coSimuSocketMode = {}.", coSimuSocketMode);
    log_compnt_mngr->debug("output osi groundtruth logical lane = {}.", isOutputOsiGroundTruthLogicalLane);

    for (auto p : get_plugin_list())
    {
        log_compnt_mngr->debug("name = {}.", p->get_name().c_str());
        log_compnt_mngr->debug("type = {}.", p->get_type().c_str());
        log_compnt_mngr->debug("id = {}.", p->get_id().c_str());
        log_compnt_mngr->debug("lib = {}.", p->get_lib().c_str());

        if (p->get_type() == "DynamicsVehicle")
        {
            DynamicVehicleCfg *p_tmp = dynamic_cast<DynamicVehicleCfg*>(p);
        }
#ifndef V1000HZ
        else if (p->get_type() == "camera" || 
            p->get_type() == "lidar" || 
            p->get_type() == "ultrasonic" || 
            p->get_type() == "radar")
        {
            SensorCfg *p_tmp = dynamic_cast<SensorCfg*>(p);

            log_compnt_mngr->debug("pos = ({}, {}, {}, {}, {}, {}).",
                    p_tmp->get_position().dx, p_tmp->get_position().dy, 
                    p_tmp->get_position().dz, p_tmp->get_position().dh, 
                    p_tmp->get_position().dp, p_tmp->get_position().dr);

            log_compnt_mngr->debug("port: name = {}, ip = {}, port = {}, type = {}, enable = {}",
                    p_tmp->get_socket_info().name.c_str(), 
                    p_tmp->get_socket_info().ip.c_str(), 
                    p_tmp->get_socket_info().port.c_str(), 
                    p_tmp->get_socket_info().type.c_str(),
                    p_tmp->get_socket_enable());

            for (auto f : p_tmp->get_filter())
            {
                log_compnt_mngr->debug("filter: {}.", f.c_str());
            }
        }
#endif
    }

    log_compnt_mngr->info("ConfigureMngr::print_cfg end.");
    return;
}


bool ConfigureMngr::is_sys_ready()
{
    return init;
}


void ConfigureMngr::set_frame_ctl(int ctl)
{
    simu_frame_ctl = ctl;
}


// 0: 内部控制; 大于0: 外部控制;  小于0: 无仿真
int ConfigureMngr::get_frame_ctl() const
{
    return simu_frame_ctl;
}


void ConfigureMngr::set_frame_rate(int rate)
{
    frame_rate = rate;
}


// 0: 数据驱动; 大于0: 定时驱动; 小于0: 无仿真
int ConfigureMngr::get_frame_rate() const
{
    return frame_rate;
}


void ConfigureMngr::set_frame_ctl_socket(const std::string ip, const std::string port, const std::string type)
{
    frame_ctl_srv.name = "simu_frame_ctl";
    frame_ctl_srv.ip = ip;
    frame_ctl_srv.port = port;
    frame_ctl_srv.type = type;
}


struct _socket ConfigureMngr::get_frame_ctl_socket()
{
    return frame_ctl_srv;
}


void ConfigureMngr::set_simulator_socket(const std::string ip, const std::string port, const std::string type)
{
    simulator.name = "simulator";
    simulator.ip = ip;
    simulator.port = port;
    simulator.type = type;

    if (isVirtualCity)
    {
        const char *envIpValue = std::getenv("SIMPRO_IP");
        if (envIpValue != NULL)
        {
            simulator.ip = envIpValue;
            log_compnt_mngr->debug("getenv SIMPRO_IP = {}", envIpValue);
        }
    }
}


struct _socket ConfigureMngr::get_simulator_socket()
{
    return simulator;
}


void ConfigureMngr::add_plugin_list(PluginCfg *plugin)
{
    p_plugin_list.push_back(plugin);
}


std::vector<PluginCfg*> ConfigureMngr::get_plugin_list()
{
    //获取CM传感器的控制方式，1:一般配置文件模式 2:HMI模式
    if (sensorControllerMode == 2)
    {
        p_plugin_list = HmiPraseMsg::Instance()->getSensorList();
    }
    log_compnt_mngr->debug("ConfigureMngr::get_plugin_list p_plugin_list size = {}",p_plugin_list.size());

    return p_plugin_list;
}


void ConfigureMngr::set_so_dir(const std::string &dir)
{
    so_dir = dir;
}


std::string ConfigureMngr::get_so_dir() const
{
    return so_dir;
}

void ConfigureMngr::set_cm_sensor_server(const std::string ip, const std::string port, const std::string type)
{
    cm_sensor_server.name = "cm_sensor_server";
    cm_sensor_server.ip = ip;
    cm_sensor_server.port = port;
    cm_sensor_server.type = type;
}

struct _socket ConfigureMngr::get_cm_sensor_server()
{
    return  cm_sensor_server;
}

//1:一般配置文件模式 2:HMI模式
int ConfigureMngr::getSensorControllerMode()
{
    return sensorControllerMode;
}

//解析虚拟城市enable.xml文件
bool ConfigureMngr::parseVirtualCityConfigFile()
{
    //解析文件
    simproxml::XMLDocument doc;
    std::string configFilePath = "../resource/VirtualCity/enable.xml";

    const simproxml::XMLError loadResult = doc.LoadFile(configFilePath.c_str());

    if (loadResult != simproxml::XML_SUCCESS)
    {   
        log_compnt_mngr->error("ConfigManager::parseConfigFile Error: load file error");
        return false;
    }

    simproxml::XMLElement *root = doc.RootElement();
    if (root == NULL)
    {
        log_compnt_mngr->error("ConfigManager::parseConfigFile Error: root == NULL");

        return false;
    }

    //遍历二级节点
    simproxml::XMLElement *node = root->FirstChildElement();
    if (node != nullptr)
    {
        if (strncmp(node->Value(), "Enable", 6) == 0)//此标签存在
        {
            const char *_value = node->Attribute("value");

            if (_value != nullptr)
            {
                std::string enableValue = _value;
                if (enableValue == "1")
                {
                    isVirtualCity = true;
                    return true;
                }
            }
        }
    }

    return false;
}

//获取是否开启虚拟城市
bool ConfigureMngr::getIsVirtualCity()
{
    return isVirtualCity;
}

//获取是否输出OSI数据
bool ConfigureMngr::getIsOutputOsiGroundTruthLogicalLane()
{
    return isOutputOsiGroundTruthLogicalLane;
}

//获取联仿接口模式 1:C-API 2:OSI
int ConfigureMngr::getCoSimuInterfaceType()
{
    return coSimuInterfaceType;
}

//获取控制在环开关 0:关闭 1:打开
int ConfigureMngr::getControlInLoopValue()
{
    return controlInLoopValue;
}

//获取控制在环类型 1:动力学挂载内部 2:动力学挂载外部
int ConfigureMngr::getControlInLoopType()
{
    return controlInLoopType;
}

//获取仿真帧率
int ConfigureMngr::getFrameRate()
{
    return frameRate;
}

//获取通信方式 1:UDP 2:TCP
int ConfigureMngr::getCoSimuSocketMode()
{
    return coSimuSocketMode;
}

//获取主车ID
int ConfigureMngr::getMainVehicleId()
{
    return mainVehicleId;
}