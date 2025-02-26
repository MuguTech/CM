#include "PluginMngr.h"
#include "ConfigureMngr.h"
#include "common.h"
#include "DynamicVehicleMgr.h"
#ifndef V1000HZ
    #include "AdadpterSocket.h"
    #include "PackMsg.h"
    #include "Sensor/SensorManager.h"
    #include "../../Runtime/TrafficSimulation/TrafficSimulation.h"
    #include "parse_ground_truth.h"
    #include "../../APF/RoadSystem/RoadSystem.h"
#endif
#include "log.h"
#include <stdlib.h>
#include <dlfcn.h>
#include "./OSIGroundTruthGenerator/OSIGroundTruthGeneratorThread.h"

#include "FmiDynamicAdapter/VirtualCityFmiDynamicAdapter.h"


PluginMngr *PluginMngr::p_instance = NULL;


PluginMngr::PluginMngr()
{
    p_buff = NULL;
    len = 0;
    frame_id = 0;
    sim_time = 0.0;
    sender = 0;
    xodr = "";
    is_new_xodr = false;
    simulate_state = 0;

}


PluginMngr::~PluginMngr()
{
    // 释放 so 句柄
    for (auto so : so_list)
    {
        dlclose(so.so_handle);
    }
}


PluginMngr * PluginMngr::get_instance()
{
    if (NULL == p_instance)
    {
        p_instance = new PluginMngr();
    }

    return p_instance;
}

void PluginMngr::Destroy()
{
    if (p_instance != nullptr)
    {
        delete p_instance;
        p_instance = nullptr;
    }
}


void PluginMngr::init()
{
    log_compnt_mngr->info("PluginMngr::init start.");
    if (so_list.size())
    {
        for (auto so : so_list)
        {
            dlclose(so.so_handle);
        }
        so_list.clear();
    }

    std::vector<PluginCfg*> plugin_list;
    std::string so_file_dir;

    plugin_list = ConfigureMngr::getInstance()->get_plugin_list();
    so_file_dir = ConfigureMngr::getInstance()->get_so_dir();

    // 加载so
    for (auto plugin : plugin_list)
    {
        std::string so_path;
        void *so_handle = NULL;
        INIT_FUNC init_func = NULL;
        UPDATA_FUNC update_func = NULL;
        STOP_FUNC stop_func = NULL;
        ISWORKING_FUNC isworking_func = NULL;

        if (NULL == plugin)
        {
            log_compnt_mngr->warn("plugin is null.");
            continue;
        }

        if (true == plugin->get_name().empty() ||
            true == plugin->get_type().empty() ||
            true == plugin->get_id().empty() ||
            true == plugin->get_lib().empty())
        {
            log_compnt_mngr->warn("plugin cfg err.");
            continue;
        }

        if (so_file_dir.substr(so_file_dir.size() - 1, so_file_dir.size()) != "/")
        {
            so_file_dir += "/";
        }

        so_path = so_file_dir + plugin->get_lib();

        if (true != load_so(so_path, &so_handle, init_func, update_func, stop_func, isworking_func) ||
            NULL == so_handle ||
            NULL == init_func || NULL == update_func || NULL == stop_func || NULL == isworking_func)
        {
            log_compnt_mngr->error("LOAD SO {} ERROR.", plugin->get_name().c_str());
            continue;
        }

        struct so_info soInfo;

        soInfo.so_name = plugin->get_name();
        soInfo.so_type = plugin->get_type();
        soInfo.so_id = plugin->get_id();
        soInfo.so_handle = so_handle;
        soInfo.init = init_func;
        soInfo.update = update_func;
        soInfo.stop = stop_func;
        soInfo.isworking = isworking_func;

        so_list.push_back(soInfo);

        log_compnt_mngr->debug("LOAD SO {} OK.", plugin->get_name().c_str());

        uint32_t pluginId = 0;
        //获取CM传感器的控制方式，1:一般配置文件模式 2:HMI模式
        if (ConfigureMngr::getInstance()->getSensorControllerMode() == 2)
        {
            //将字符串的传感器id转换成哈希值
            size_t hash_value = hasher(soInfo.so_id);
            pluginId = hash_value % std::numeric_limits<int>::max();
            log_compnt_mngr->debug("init pluginId is ={},hash_value is= {}",pluginId,hash_value);
        }
        else
        {
            pluginId = static_cast<uint32_t>(TrafficSimulation::stoi(soInfo.so_id));
        }

        if (plugin->get_type() == "camera" ||
            plugin->get_type() == "lidar" ||
            plugin->get_type() == "ultrasonic" ||
            plugin->get_type() == "radar")
        {
#ifndef V1000HZ
            SensorCfg *p_tmp = dynamic_cast<SensorCfg*>(plugin);
            SensorDescription *p_desc = p_tmp->getDesc();
            soInfo.init((void *)p_desc, p_tmp->space(), pluginId);
#endif
        }
        else if (plugin->get_type() == "DynamicsPlugin")
        {
            DynamicVehicleCfg *p_tmp = dynamic_cast<DynamicVehicleCfg*>(plugin);
            soInfo.init((void *)p_tmp, sizeof(DynamicVehicleCfg), pluginId);
        }
    }

    /* SensorMngr::init() 移动到类的构造函数 */

    log_compnt_mngr->info("PluginMngr::init end.");
    return;
}


void PluginMngr::timerEvent()
{
    // update();
}

bool PluginMngr::isSensorRunning(
    const std::map<std::string, bool> &sensor_running_map,
    const std::string& sensor_id) {
bool is_running = true;
if (sensor_running_map.find(sensor_id) == sensor_running_map.end()) {
    is_running = false;
    log_compnt_mngr->error("not find sensor {}", sensor_id);
} else {
    is_running = sensor_running_map.at(sensor_id);
    log_compnt_mngr->debug("is_running is {}", is_running);
}

return is_running;
}

// 输入来自 V-traffic 自定义协议
void PluginMngr::update(const std::map<std::string, bool>& sensor_running_map)
{
    log_compnt_mngr->info("PluginMngr::update start.");
    void *p_gt = NULL;
    size_t gt_len = 0;

    if (true != get_ground_truth(&p_gt, gt_len))
    {
        log_compnt_mngr->error("get gt err.");
        return;
    }

    log_compnt_mngr->debug("plugin update frame id {} len = {}, so list size = {}."
            , get_frame_id(), gt_len, so_list.size());

    for (auto so : so_list)
    {
        char *sensorInfoPkgBuffer = nullptr; //sensorInfoPkg的buffer指针
        unsigned int sensorInfoPkgSize = 0; //sensorInfoPkg的buffer大小
        char sensor_output[OSI_BUF_SIZE] = {0};
        char c_api_output[OSI_BUF_SIZE] = {0};
        size_t osi_len = OSI_BUF_SIZE;
        size_t sensor_output_len = OSI_BUF_SIZE;
        size_t c_api_output_len = OSI_BUF_SIZE;

        log_compnt_mngr->debug("start so: {}.", so.so_name.c_str());

        if (so.so_type == "camera" ||
            so.so_type == "lidar" ||
            so.so_type == "ultrasonic" ||
            so.so_type == "radar")
        {
#ifndef V1000HZ
            if (!isSensorRunning(sensor_running_map, so.so_id))
            {
                log_compnt_mngr->error("sensor {} is not running.", so.so_id);
                continue;
            }

            // 获取该传感器的SensorInfoPkg
            SensorManager::Instance()->getSensorInfoPkg(so.so_id, &sensorInfoPkgBuffer, &sensorInfoPkgSize);
            if ((sensorInfoPkgBuffer == nullptr) || (sensorInfoPkgSize == 0))
            {
                log_compnt_mngr->error("PluginMngr::update Error: cannot get SensorInfoPkg, sensor id={}", so.so_id);
                continue;
            }
#endif
        }
        else if (so.so_type == "DynamicsPlugin")
        {
            if (true != DynamicVehicleMgr().isRun((void *)(so.so_name.c_str())))
            {
                log_compnt_mngr->error("is running err.");
                continue;
            }

        }
        else
        {
            log_compnt_mngr->error("so type {} err.", so.so_type.c_str());
            continue;
        }

        log_compnt_mngr->debug("cover data len {} to {} over.", gt_len, osi_len);

        log_compnt_mngr->debug("timestamp sensor start {} ", get_cur_time_ms());

        uint32_t pluginId = 0;
        //获取CM传感器的控制方式，1:一般配置文件模式 2:HMI模式
        if (ConfigureMngr::getInstance()->getSensorControllerMode() == 2)
        {
            //将字符串的传感器id转换成哈希值
            size_t hash_value = hasher(so.so_id);
            pluginId = hash_value % std::numeric_limits<int>::max();
            log_compnt_mngr->debug("update pluginId is ={},hash_value is= {}",pluginId,hash_value);
        }
        else
        {
            pluginId = static_cast<uint32_t>(TrafficSimulation::stoi(so.so_id));
        }
        so.update(pluginId, p_gt, gt_len, sensor_output, sensor_output_len);
        if (0 == sensor_output_len || OSI_BUF_SIZE == sensor_output_len)
        {
            log_compnt_mngr->error("so {} update err.", so.so_name.c_str());
            continue;
        }

        log_compnt_mngr->debug("timestamp sensor end {} ", get_cur_time_ms());

        log_compnt_mngr->debug("so name {} so id {} update data len {} to {} over.", so.so_name.c_str(), so.so_id, osi_len, sensor_output_len);

#ifndef V1000HZ
        //存放所有要打包的pkg <buffer, size>
        std::list<std::tuple<char *, unsigned int>> inputBufferList;
        inputBufferList.push_back(std::make_tuple(sensorInfoPkgBuffer, sensorInfoPkgSize)); //SensorInfoPkg
        inputBufferList.push_back(std::make_tuple(sensor_output, sensor_output_len)); //SensorDetectionInfoPkg

        PackMsg::getPkgData(p_gt, gt_len, inputBufferList, c_api_output, c_api_output_len);
        if (0 == c_api_output_len || OSI_BUF_SIZE == c_api_output_len)
        {
            log_compnt_mngr->error("osi 2 gt pkg err.");
            continue;
        }

        log_compnt_mngr->debug(" plugin mngr update send_msg id = {},c_api_output_len = {}",so.so_id,c_api_output_len);
        AdadpterSocket::get_instance()->send_msg(so.so_id, c_api_output,
                                                (int32_t &)c_api_output_len);
#endif

        log_compnt_mngr->debug("timestamp send to ad {} len = {} ", get_cur_time_ms(), c_api_output_len);

    }

    log_compnt_mngr->info("PluginMngr::update end.");
    return;
}




// 结束仿真
void PluginMngr::stop_simulator()
{
    log_compnt_mngr->info("PluginMngr::stop_simulator start.");
    void *p_gt = NULL;
    size_t gt_len = 0;

    if (true != get_ground_truth(&p_gt, gt_len))
    {
        log_compnt_mngr->error("get gt err.");
        return;
    }

    log_compnt_mngr->debug("plugin stop frame id {} len = {}, so list size = {}."
            , get_frame_id(), gt_len, so_list.size());

    for (auto so : so_list)
    {
        char c_api_output[OSI_BUF_SIZE] = {0};
        size_t c_api_output_len = OSI_BUF_SIZE;

        log_compnt_mngr->debug("stop so: {}.", so.so_name.c_str());
        so.stop();

        if (so.so_type == "camera" ||
            so.so_type == "lidar" ||
            so.so_type == "ultrasonic" ||
            so.so_type == "radar")
        {
#ifndef V1000HZ
            PackMsg::getStopPkgData(p_gt, gt_len, c_api_output, c_api_output_len);
            if (0 == c_api_output_len || OSI_BUF_SIZE == c_api_output_len)
            {
                log_compnt_mngr->warn("osi 2 gt pkg err.");
                continue;
            }

            AdadpterSocket::get_instance()->send_msg(
                so.so_id, c_api_output, (int32_t &)c_api_output_len);
#endif
        }
    }

    log_compnt_mngr->info("PluginMngr::stop_simulator end.");
    return;
}



bool PluginMngr::load_so
(
    std::string &soFilePath,
    void **so_handle,
    INIT_FUNC &init_func,
    UPDATA_FUNC &update_func,
    STOP_FUNC &stop_func,
    ISWORKING_FUNC &isworking_func
)
{
    log_compnt_mngr->info("PluginMngr::load_so start.");
    void *handle = NULL;
    char *error = NULL;

    if (true != is_file_exist(soFilePath))
    {
        log_compnt_mngr->error("FILE NOT EXIST: {}.", soFilePath.c_str());
        return false;
    }

    handle = dlopen(soFilePath.c_str(), RTLD_LAZY);
    if (!handle)
    {
        log_compnt_mngr->error("dlopen err.");
        log_compnt_mngr->error("{}", dlerror());
        return false;
    }

    // 清除之前存在的错误
    dlerror();

    // 获取函数 init
    *(void **) (&init_func) = dlsym(handle, "init");
    if ((error = dlerror()) != NULL)  {
        log_compnt_mngr->error("dlsym init err.");
        log_compnt_mngr->error("{}.", error);
        return false;
    }

    // 获取函数 update
    *(void **) (&update_func) = dlsym(handle, "update");
    if ((error = dlerror()) != NULL)  {
        log_compnt_mngr->error("dlsym update err.");
        log_compnt_mngr->error("{}.", error);
        return false;
    }

    // 获取函数 stop
    *(void **) (&stop_func) = dlsym(handle, "stop");
    if ((error = dlerror()) != NULL)  {
        log_compnt_mngr->error("dlsym stop err.");
        log_compnt_mngr->error("{}.", error);
        return false;
    }

    // 获取函数 isWorking
    *(void **) (&isworking_func) = dlsym(handle, "isWorking");
    if ((error = dlerror()) != NULL)  {
        log_compnt_mngr->error("dlsym is working err.");
        log_compnt_mngr->error("{}.", error);
        return false;
    }

    *so_handle = handle;

    log_compnt_mngr->info("PluginMngr::load_so end.");
    return true;
}


// 仅保存指针
void PluginMngr::set_ground_truth(void *msg, size_t len)
{
    if (NULL == msg || 0 == len)
    {
        log_compnt_mngr->error("set ground truth err.");
        return;
    }

    p_buff = (char *)msg;
    this->len = len;
    return;
}


// 返回指针
bool PluginMngr::get_ground_truth(void **pp_buff, size_t &len)
{
    if (NULL == p_buff)
    {
        log_compnt_mngr->error("get ground truth err.");
        return false;
    }

    *pp_buff = p_buff;
    len = this->len;
    return true;
}


void PluginMngr::set_frame_id(int id)
{
    frame_id = id;
}


int PluginMngr::get_frame_id()
{
    return frame_id;
}


void PluginMngr::set_sim_time(double simTime)
{
    sim_time = simTime;
}


double PluginMngr::get_sim_time()
{
    return sim_time;
}


void PluginMngr::set_sender(unsigned char _sender)
{
    sender = _sender;
}


unsigned char PluginMngr::get_sender()
{
    return sender;
}


void PluginMngr::set_xodr(const std::string xodr)
{
    if (!ConfigureMngr::getInstance()->getIsVirtualCity())
    {
        this->xodr = xodr;
    }
    else
    {
        std::size_t index = xodr.rfind('/');
        char *workPath = getcwd(NULL, 0);
        std::string workPathStr = "";
        if (workPath != NULL)
        {
            workPathStr = workPath;
            free(workPath);
        }
        if (index != std::string::npos)
        {
            this->xodr = workPathStr + "/" + xodr.substr(index + 1);
        }
        else
        {
            this->xodr = workPathStr + "/" + xodr;
        }
    }
}


std::string PluginMngr::get_xodr()
{
    return xodr;
}


void PluginMngr::set_isnew_xodr(const bool is)
{
    is_new_xodr = is;
}


bool PluginMngr::get_isnew_xodr()
{
    return is_new_xodr;
}


void PluginMngr::set_simulate_state(const int state)
{
    simulate_state = state;
}


int PluginMngr::get_simulate_state()
{
    return simulate_state;
}


void PluginMngr::parse_gt(void *gt, int len)
{
    log_compnt_mngr->info("PluginMngr::parse_gt start.");
#ifndef V1000HZ
    // 当帧率由内部控制或者外部控制时，执行解析数据流程
    if (ConfigureMngr::getInstance()->get_frame_ctl() >= 0)
    {
        parse_ground_truth parseGT(gt, len);

        parseGT.parse();

        set_simulate_state(parseGT.get_simulate_state());
        set_frame_id(parseGT.get_frame_id());
        set_sim_time(parseGT.get_sim_time());
        set_sender(parseGT.get_sender());

        if (get_xodr() == parseGT.get_xodr())
        {
            set_isnew_xodr(false);
        }
        else
        {
            set_isnew_xodr(true);
            set_xodr(parseGT.get_xodr());
        }
    }
#endif

    set_ground_truth(gt, len);

    if (ConfigureMngr::getInstance()->getIsVirtualCity() &&
        (ConfigureMngr::getInstance()->getControlInLoopValue() == 1) &&
        (ConfigureMngr::getInstance()->getControlInLoopType() == 1)) //虚拟城市并且控制在环打开，动力学挂载内部
    {
        if (frame_id == 1) //场景运行第一帧
        {
            VirtualCityFmiDynamicAdapter::Instance()->init();  //初始化动力学
        }
    }

    log_compnt_mngr->info("PluginMngr::parse_gt end.");
    return;
}


// 更新 road system 模块
bool update_road_system()
{
    if (IS_START != PluginMngr::get_instance()->get_simulate_state())
    {
        return true;
    }

    if (true != PluginMngr::get_instance()->get_isnew_xodr())
    {
        log_compnt_mngr->debug("is not new xodr, dont need update road system module.");
        return true;
    }

    if (true == PluginMngr::get_instance()->get_xodr().empty())
    {
        log_compnt_mngr->error("get xodr file name err.");
        return false;
    }

#ifndef V1000HZ
    RoadSystem::Instance()->Destroy();

    log_compnt_mngr->debug("start parse open drive.");
    if (true != RoadSystem::Instance()->parseOpenDrive(PluginMngr::get_instance()->get_xodr()))
    {
        log_compnt_mngr->error("parse open drive error.");
        return false;
    }
#endif

    if ((ConfigureMngr::getInstance()->getIsVirtualCity()) && (ConfigureMngr::getInstance()->getCoSimuInterfaceType() == 2) && (ConfigureMngr::getInstance()->getIsOutputOsiGroundTruthLogicalLane()))
    {
        RoadSystem::Instance()->samplingPoints();
    }

    if (ConfigureMngr::getInstance()->getIsVirtualCity())
    {
        OSIGroundTruthGeneratorThread::Instance()->informParseXodrComplete();
    }

    return true;
}

//虚拟城市对接大疆算法时只生成和发送OSIPkg数据
void PluginMngr::virtualCityOsiUpdate()
{
    log_compnt_mngr->info("PluginMngr::virtualCityOsiUpdate start.");
    void *p_gt = NULL;
    size_t gt_len = 0;

    if (true != get_ground_truth(&p_gt, gt_len))
    {
        log_compnt_mngr->error("get gt err.");
        return;
    }

    log_compnt_mngr->debug("plugin virtual city osi update frame id {} len = {}, so list size = {}."
            , get_frame_id(), gt_len, so_list.size());

    for (auto so : so_list)
    {
        char c_api_output[OSI_BUF_SIZE] = {0};
        size_t c_api_output_len = OSI_BUF_SIZE;
        char *osiGroundTruthPkgbuffer = nullptr; //osiGroundTruthPkg的buffer指针
        unsigned int osiGroundTruthPkgSize = 0;  //osiGroundTruthPkg的buffer大小

        OSIGroundTruthGeneratorThread::Instance()->getCoSimOSIGroundTruthPkg(osiGroundTruthPkgbuffer, osiGroundTruthPkgSize);

#ifndef V1000HZ
        //存放所有要打包的pkg <buffer, size>
        std::list<std::tuple<char *, unsigned int>> inputBufferList;
        inputBufferList.push_back(std::make_tuple(osiGroundTruthPkgbuffer, osiGroundTruthPkgSize));

        PackMsg::getPkgData(p_gt, gt_len, inputBufferList, c_api_output, c_api_output_len);

        if (0 == c_api_output_len || OSI_BUF_SIZE == c_api_output_len)
        {
            log_compnt_mngr->error("osi 2 gt pkg err.");
            continue;
        }

        log_compnt_mngr->debug(" plugin mngr virtual city osi update send_msg id = {},c_api_output_len = {}",so.so_id,c_api_output_len);
        AdadpterSocket::get_instance()->send_msg(so.so_id, c_api_output,
                                                (int32_t &)c_api_output_len);
#endif

        log_compnt_mngr->debug("timestamp send to ad {} len = {} ", get_cur_time_ms(), c_api_output_len);

    }

    log_compnt_mngr->info("PluginMngr::virtualCityOsiUpdate end.");
    return;
}