#include "ConfigureMngr.h"
#include "PluginMngr.h"
#include "V_TrafficSocket.h"
#include "FrameCtrlSocket.h"
#include "common.h"
#include "log.h"

#ifndef V1000HZ
    #include "AdadpterSocket.h"
    #include "../../APF/RoadSystem/RoadSystem.h"
#endif

#include "./parse_ground_truth.h"
#include <thread>
#include <map>
#include <cstring>
#include "Sensor/SensorManager.h"
#include "./Evaluation/ConfigManager.h"
#include "./OSIGroundTruthGenerator/OSIGroundTruthGeneratorThread.h"
#include "./HmiSensorCfgTcpClient/HmiSensorCfgTcpClient.h"
#include "FmiDynamicAdapter/VirtualCityFmiDynamicAdapter.h"

// 加载配置, 创建通信线程, 加载so
void init_system();


// 根据配置驱动不同工作线程
void start_work();


// 阻塞接收 v-traffic 模块的数据
bool recv_gt_block();
void recv_data_task();
void recv_ctrl_task();

bool data_driven_work(const std::map<std::string, bool> &sensor_running_map,
                    const std::map<std::string, int> &sensor_frame_map,
                    bool is_sync_mode);

void timer_driven_work(long interval_ms);

// 发送数据给pb文件生成线程
bool sendToGeneratorOsiPbFileThread();
// 虚拟城市数据驱动工作方式
bool vc_data_driven_work(const std::map<std::string, bool> &sensor_running_map);

//生成osiPkg打包所需数据
bool generatorOsiGroundTruthPkgData();

int main()
{

    init_system();

    start_work();

    return 0;
}


void init_system()
{
    // 初始化 日志
    log_init();
    log_compnt_mngr->critical("INIT LOG OVER.");

    // 初始化配置
    ConfigureMngr::getInstance();
    log_compnt_mngr->critical("INIT CONFIGURE OVER.");

    if (true != ConfigureMngr::getInstance()->is_sys_ready())
    {
        log_compnt_mngr->error("CONFIGURE PARSE ERROR.");
        return;
    }

    if (ConfigureMngr::getInstance()->get_frame_ctl() < 0 || 
        ConfigureMngr::getInstance()->get_frame_rate() < 0)
    {
        log_compnt_mngr->error("CONFIGURE FRAME RATE NOT SIMULATOR.");
        return;
    }

    //获取CM传感器的控制方式，1:一般配置文件模式 2:HMI模式
    if (ConfigureMngr::getInstance()->getSensorControllerMode() == 2)
    {
        HmiSensorCfgTcpClient::get_instance()->init_thread();
        log_compnt_mngr->critical("INIT HMI SENSOR CFG SOCKET OVER.");
    }

#ifndef V1000HZ
    // 初始化 road system 模块
    RoadSystem::Instance();
    log_compnt_mngr->critical("INIT ROADSYSTEM OVER.");
#endif

    // 建立 frame ctrl socket client 连接
    if (ConfigureMngr::getInstance()->get_frame_ctl() > 0)
    {
        FrameCtrlSocket::get_instance()->init_thread();
        log_compnt_mngr->critical("INIT FRAME CONTROL SOCKET OVER.");
    }

    // 建立 v-traffic socket client 连接
    V_TrafficSocket::get_instance()->init_thread();
    log_compnt_mngr->critical("INIT V-TRAFFIC SOCKET OVER.");

#ifndef V1000HZ
    // 建立 socket server 监听 AD连接
    //获取CM传感器的控制方式，1:一般配置文件模式 2:HMI模式
    if (ConfigureMngr::getInstance()->getSensorControllerMode() != 2)
    {
        AdadpterSocket::get_instance()->createSocketServer();
        log_compnt_mngr->critical("INIT AD ADAPTER SOCKET OVER.");
    }
#endif

    // load so
    //获取CM传感器的控制方式，1:一般配置文件模式 2:HMI模式
    if (ConfigureMngr::getInstance()->getSensorControllerMode() != 2)
    {
        // load so
        PluginMngr::get_instance()->init();
        log_compnt_mngr->critical("INIT PLUGIN MNGR OVER.");
        // 发送sensor配置给frame control
        if (ConfigureMngr::getInstance()->get_frame_ctl() > 0)
        {
            FrameCtrlSocket::get_instance()->send_init_data();
        }
    }

    if (ConfigureMngr::getInstance()->getIsVirtualCity())
    {
        // 获取环境变量
        char* env_var = std::getenv("KPI_XML_URL");

        if (env_var != nullptr)
        {
            // 将获取到的环境变量转换为C++的字符串类型
            std::string path(env_var);

            // 查找最后一个路径分隔符的位置
            size_t last_separator = path.find_last_of('/');

            // 截取最后的字符串
            if (last_separator != std::string::npos)
            {
                std::string last_part = path.substr(last_separator + 1);
                log_compnt_mngr->debug("getenv last_part={}.", last_part);
                // 解析评估指标文件
                ConfigManager::GetInstance()->init(last_part);
            }
            else
            {
                log_compnt_mngr->error("getenv separator not found.");
            }
        }
        else
        {
            log_compnt_mngr->error("getenv KPI_XML_URL is null.");
        }

        OSIGroundTruthGeneratorThread::Instance()->creatGeneratorOsiPbFileThread();        
    }
}



void start_work()
{
    log_compnt_mngr->critical("CM start_work start.");
    std::thread recv_data_handle;
    std::thread recv_ctrl_handle;

    if (true != ConfigureMngr::getInstance()->is_sys_ready())
    {
        log_compnt_mngr->error("CONFIGURE PARSE ERROR.");
        return;
    }

    // 仿真帧率由外部控制, 例如1000帧需求
    if (ConfigureMngr::getInstance()->get_frame_ctl() > 0)
    {
        // 接收控制信息触发处理流程
        recv_ctrl_handle = std::thread(recv_ctrl_task);

        // 接收数据信息更新本地数据
        while (recv_gt_block()) {
        }
    }
    // 仿真帧率由内部控制
    else if (ConfigureMngr::getInstance()->get_frame_ctl() == 0)
    {
        // 数据驱动
        if (ConfigureMngr::getInstance()->get_frame_rate() == 0)
        {
            log_compnt_mngr->critical("vc_data_driven_work start.");
            frame_rate_tj frame_rate(60);

            auto plugin_list = ConfigureMngr::getInstance()->get_plugin_list();
            for (auto plugin : plugin_list)
            {
                SensorManager::Instance()->setRunning(plugin->get_id());
            }

            while (recv_gt_block())
            {
                // 帧率统计
                frame_rate.print();

                std::map<std::string, bool> sensor_running_map =
                    SensorManager::Instance()->getSensorRunningMap();

                // 数据驱动工作: 解析数据、update
                if (true != vc_data_driven_work(sensor_running_map))
                {
                    log_compnt_mngr->error("vc_data_driven_work error.");
                }
            }
            log_compnt_mngr->critical("vc_data_driven_work end.");
        }
        // 定时触发
        else if (ConfigureMngr::getInstance()->get_frame_rate() > 0)
        {
            log_compnt_mngr->critical("timer_driven_work start.");
            // 独立线程
            long interval_ms = 1 * 1000.0 / ConfigureMngr::getInstance()->get_frame_rate();

            while (recv_gt_block())
            {
                timer_driven_work(interval_ms);
            }
            log_compnt_mngr->critical("timer_driven_work end.");
        }
        else
        {
            log_compnt_mngr->warn("cfg simulator frame rate {}, no simuloator.", 
                    ConfigureMngr::getInstance()->get_frame_rate());
        }
    }
    else
    {
        log_compnt_mngr->warn("cfg simulator frame ctrol {}, no simuloator.", 
                ConfigureMngr::getInstance()->get_frame_ctl());
    }

    if (recv_data_handle.joinable())
    {
        recv_data_handle.join();
    }

    if (recv_ctrl_handle.joinable())
    {
        recv_ctrl_handle.join();
    }

    if (ConfigureMngr::getInstance()->getIsVirtualCity())
    {
        OSIGroundTruthGeneratorThread::Instance()->joinThread(); //回收cm生成pb文件线程    
    }

    log_compnt_mngr->critical("CM start_work end.");

    return;
}



bool recv_gt_block()
{
    log_compnt_mngr->info("recv_gt_block start.");
    char *p_buff = NULL;
    size_t len = 0;
    size_t min_len = 0;

    if (true != V_TrafficSocket::get_instance()->get_recv_data((void **)&p_buff, len))
    {
        log_compnt_mngr->error("get data from v-traffic err.");
        return false;
    }

    // 从缓存中取一帧数据
    parse_ground_truth pgt(p_buff, len);
    if (true != pgt.get_len_frame_data(min_len))
    {
        log_compnt_mngr->error("get len of one frame data err.");
        return false;
    }

    S_SP_MSG_HDR *msgHead = (S_SP_MSG_HDR *)p_buff;
    log_compnt_mngr->debug("recv_gt_block update gt frame number is {}", msgHead->u4FrameNo);

    // 保存一份本地数据
    if (true != update_v_traffic_data(p_buff, min_len))
    {
        log_compnt_mngr->error("update v-traffic data err.");
    }

    // 释放接收线程数据区
    V_TrafficSocket::get_instance()->sempost();

    log_compnt_mngr->info("recv_gt_block end.");
    return true;
}


// 接收(v-traffic)数据信息线程
void recv_data_task()
{
    while (recv_gt_block());
    return;
}


// 接收(system-ctrl)控制信息线程
void recv_ctrl_task()
{
    log_compnt_mngr->critical("recv_ctrl_task start.");
    // 外部触发
    while (FrameCtrlSocket::get_instance()->get_recv_data()) {
        std::map<std::string, bool> sensor_running_map =
            SensorManager::Instance()->getSensorRunningMap();
        std::map<std::string, int> sensor_frame_map =
            SensorManager::Instance()->getSensorFrameMap();
        bool is_sync_mode = SensorManager::Instance()->getIsSyncMode();

        SensorManager::Instance()->resetSensorRunningMap();

        FrameCtrlSocket::get_instance()->post_sem_resp();
        data_driven_work(sensor_running_map, sensor_frame_map, is_sync_mode);
    }

    log_compnt_mngr->critical("recv_ctrl_task end.");
    return;
}

bool data_driven_work(const std::map<std::string, bool> &sensor_running_map,
                    const std::map<std::string, int> &sensor_frame_map,
                    bool is_sync_mode) {
    log_compnt_mngr->info("data_driven_work start.");
    char *p_temp = NULL;
    size_t len = 0;

    p_temp = new char[MSG_BUFF_SIZE];
    bool ret = true;
    do {

        if (NULL == p_temp) {
            log_compnt_mngr->error("start work err for malloc.");
            ret = false;
            break;
        }

        len = MSG_BUFF_SIZE;
        // 使用临时数据进行处理
        if (true != get_v_traffic_data(p_temp, len)) {
            log_compnt_mngr->error("get v-traffic data err.");
            ret = false;
            break;
        }
        // 解析数据
        PluginMngr::get_instance()->parse_gt(p_temp, len);
        if (IS_STOP == PluginMngr::get_instance()->get_simulate_state()) {
            //释放仿真结束数据，防止CM向AD多次发送结束消息
            resetSimproMsg();

            PluginMngr::get_instance()->stop_simulator();
            log_compnt_mngr->debug("recv stop sig from v-traffic.");

            FrameCtrlSocket::get_instance()->send_stop_resp_msg();
            log_compnt_mngr->debug("send stop resp.");

            break;
        }
        // 更新 road system 模块
        if (true != update_road_system()) {
            log_compnt_mngr->error("UPDATE ROAD SYSTEM ERR.");
            ret = false;
            break;
        }

        if (is_sync_mode)
        {
            // 如果vtraffic data 的frame number < sensor 的frame
            // number，则返回失败消息给system control
            int gt_frame_id = PluginMngr::get_instance()->get_frame_id();
            std::map<std::string, bool> sensor_running_map_local;

            std::map<std::string, bool> sensor_running_map_valid;

            for (auto &sensor_info : sensor_running_map) {
                if (sensor_info.second) {
                    std::string current_sensor_id = sensor_info.first;
                    int sensor_frame = sensor_frame_map.at(current_sensor_id);
                    log_compnt_mngr->debug(
                        "current_sensor_id is {}, sensor_frame is {}, gt_frame_id is {}",
                        current_sensor_id, sensor_frame, gt_frame_id);
                    if (sensor_frame != gt_frame_id) {
                        sensor_running_map_local[current_sensor_id] = true;
                    }
                    else
                    {
                        sensor_running_map_valid[current_sensor_id] = true;
                    }
                }
            }

            if (!sensor_running_map_local.empty()) {
                bool is_consumed_data = false;
                FrameCtrlSocket::get_instance()->send_frame_ctrl_resp_msg(
                    is_consumed_data, sensor_running_map_local, sensor_frame_map);
                log_compnt_mngr->debug("send not consumed data.");
            }
            if(!sensor_running_map_valid.empty())
            {
                // 执行 plugin update
                PluginMngr::get_instance()->update(sensor_running_map_valid);
                bool is_consumed_data = true;
                FrameCtrlSocket::get_instance()->send_frame_ctrl_resp_msg(
                    is_consumed_data, sensor_running_map_valid, sensor_frame_map);
                log_compnt_mngr->debug("send consumed data.");
            }
        }
        else {
            // 执行 plugin update
            PluginMngr::get_instance()->update(sensor_running_map);
        }

    } while (false);

    if (NULL != p_temp) {
        delete[] p_temp;
        p_temp = NULL;
    }

    log_compnt_mngr->debug("leave data_driven_work ret is {}", ret);
    log_compnt_mngr->info("data_driven_work end.");
    return ret;
}

// 定时触发
void timer_driven_work(long interval_ms)
{
    log_compnt_mngr->info("timer_driven_work start.");
    char *p_temp = NULL;
    size_t len = 0;

    p_temp = new char[MSG_BUFF_SIZE];
    if (NULL == p_temp)
    {
        log_compnt_mngr->error("start work err for malloc.");
        return;
    }

    len = MSG_BUFF_SIZE;

    // 使用临时数据进行处理
    if (true != get_v_traffic_data(p_temp, len))
    {
        log_compnt_mngr->error("get v-traffic data err.");
        return;
    }

    // 解析数据
    PluginMngr::get_instance()->parse_gt(p_temp, len);

    // 停止任务
    if (IS_STOP == PluginMngr::get_instance()->get_simulate_state())
    {
        PluginMngr::get_instance()->stop();
        PluginMngr::get_instance()->stop_simulator();
        log_compnt_mngr->debug("recv stop sig from v-traffic.");
        return;
    }

    // 更新 road system 模块，重启 plugin update
    if (IS_START == PluginMngr::get_instance()->get_simulate_state())
    {
        // 保险起见，调用一下 stop
        PluginMngr::get_instance()->stop();

        if (true != update_road_system())
        {
            log_compnt_mngr->error("UPDATE ROAD SYSTEM ERR.");
            return;
        }

        // 启动定时线程工作
        PluginMngr::get_instance()->startms(interval_ms);
    }

    if (NULL != p_temp)
    {
        delete[] p_temp;
        p_temp = NULL;
    }
    log_compnt_mngr->info("timer_driven_work end.");
    return;
}

// 虚拟城市数据驱动工作方式
bool vc_data_driven_work(const std::map<std::string, bool> &sensor_running_map) 
{
    log_compnt_mngr->info("vc_data_driven_work start.");
    char *p_temp = NULL;
    size_t len = 0;

    p_temp = new char[MSG_BUFF_SIZE];
    bool ret = true;
    do 
    {
        if (NULL == p_temp) 
        {
            log_compnt_mngr->error("start work err for malloc.");
            ret = false;
            break;
        }

        len = MSG_BUFF_SIZE;
        // 使用临时数据进行处理
        if (true != get_v_traffic_data(p_temp, len)) 
        {
            log_compnt_mngr->error("get v-traffic data err.");
            ret = false;
            break;
        }

        // 解析数据
        PluginMngr::get_instance()->parse_gt(p_temp, len);

        if (IS_STOP == PluginMngr::get_instance()->get_simulate_state())
        {
            log_compnt_mngr->debug("OSIGroundTruthGeneratorThread simulationStop.");
            OSIGroundTruthGeneratorThread::Instance()->simulationStop();
        }

        //发送数据给pb文件生成线程
        if (ConfigureMngr::getInstance()->getCoSimuInterfaceType() == 1)
        {
            if (sendToGeneratorOsiPbFileThread() == false)
            {
                log_compnt_mngr->error("sendToGeneratorOsiPbFileThread error.");
                ret = false;
                break;
            }
        }

        if (IS_STOP == PluginMngr::get_instance()->get_simulate_state()) 
        {
            PluginMngr::get_instance()->stop_simulator();
            if ((ConfigureMngr::getInstance()->getControlInLoopValue() == 1) && (ConfigureMngr::getInstance()->getControlInLoopType() == 1)) //控制在环打开，动力学挂载内部
            {
                VirtualCityFmiDynamicAdapter::Instance()->finish(); //结束动力学
            }
            log_compnt_mngr->debug("recv stop sig from v-traffic.");
        }

        // 更新 road system 模块
        if (true != update_road_system()) 
        {
            log_compnt_mngr->error("UPDATE ROAD SYSTEM ERR.");
            ret = false;
            break;
        }

        if (ConfigureMngr::getInstance()->getCoSimuInterfaceType() == 2)
        {
            //生成osiPkg打包所需数据
            if (generatorOsiGroundTruthPkgData() == false)
            {
                log_compnt_mngr->error("generatorOsiGroundTruthPkgData error.");
                ret = false;
                break;
            }            
        }

        // 执行 plugin update
        if (ConfigureMngr::getInstance()->getCoSimuInterfaceType() == 1)
        {
            PluginMngr::get_instance()->update(sensor_running_map);
        }
        else
        {
            PluginMngr::get_instance()->virtualCityOsiUpdate();
        }
    } while (false);

    if (NULL != p_temp) 
    {
        delete[] p_temp;
        p_temp = NULL;
    }

    log_compnt_mngr->debug("leave data_driven_work ret is {}", ret);
    log_compnt_mngr->info("vc_data_driven_work end.");
    return ret;
}

//发送数据给pb文件生成线程
bool sendToGeneratorOsiPbFileThread()
{
    log_compnt_mngr->info("sendToGeneratorOsiPbFileThread start.");
    double time = get_cur_time_ms();
    log_compnt_mngr->debug("main.cpp send_to_GeneratorOsiPbFileThread start time = {}.", time);

    OSI_PB_DATA data = {0};
    bool result = true;

    data.msg = new char[MSG_BUFF_SIZE];
    if (NULL == data.msg)
    {
        log_compnt_mngr->error("start work err for malloc.");
    }

    data.msgLen = MSG_BUFF_SIZE;

    // 处理最新数据
    if (true != get_v_traffic_data(data.msg, data.msgLen))
    {
        log_compnt_mngr->error("get v-traffic data err.");
        result = false;
    }

    if (OSIGroundTruthGeneratorThread::Instance()->sendMsgToMq(data) == false)
    {
        log_compnt_mngr->error("mq_send err.");
        result = false;        
    }

    time = get_cur_time_ms();
    log_compnt_mngr->debug("main.cpp send_to_GeneratorOsiPbFileThread end time = {}.", time);
    log_compnt_mngr->info("sendToGeneratorOsiPbFileThread end.");
    return result;
}

//生成osiPkg打包所需数据
bool generatorOsiGroundTruthPkgData()
{
    log_compnt_mngr->info("generatorOsiGroundTruthPkgData start.");
    double time = get_cur_time_ms();
    log_compnt_mngr->debug("main.cpp generatorOsiGroundTruthPkgData start time = {}.", time);

    OSI_PB_DATA data = {0};
    bool result = true;

    data.msg = new char[MSG_BUFF_SIZE];
    if (NULL == data.msg)
    {
        log_compnt_mngr->error("start work err for malloc.");
    }

    data.msgLen = MSG_BUFF_SIZE;

    // 处理最新数据
    if (true != get_v_traffic_data(data.msg, data.msgLen))
    {
        log_compnt_mngr->error("get v-traffic data err.");
        result = false;
    }

    if (true != OSIGroundTruthGeneratorThread::Instance()->generatorOsiCosimuData(data))
    {
        log_compnt_mngr->error("generator osi cosimu data err.");
        result = false;
    }

    time = get_cur_time_ms();
    log_compnt_mngr->debug("main.cpp generatorOsiGroundTruthPkgData end time = {}.", time);
    log_compnt_mngr->info("generatorOsiGroundTruthPkgData end.");
    return result;
}