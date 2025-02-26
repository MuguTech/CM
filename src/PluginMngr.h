#ifndef __PLUGIN_MNGR_H__
#define __PLUGIN_MNGR_H__

#include "../../Main/SystemCtrol/CppTimer.h"
#include <string>
#include <vector>
#include "Sensor/SensorManager.h"


#define OSI_BUF_SIZE        (2 * 1024 * 1024)


typedef void (*INIT_FUNC)(void *input, size_t inlen, uint32_t pluginId);
typedef void (*UPDATA_FUNC)(uint32_t pluginId, void *input, size_t inlen, void *output, size_t &outlen);
typedef void (*STOP_FUNC)();
typedef void (*ISWORKING_FUNC)();


bool update_road_system();



class PluginMngr: public CppTimer
{
private:
    struct so_info
    {
        std::string so_name;
        std::string so_id;
        std::string so_type;
        void *so_handle;
        INIT_FUNC init;
        UPDATA_FUNC update;
        STOP_FUNC stop;
        ISWORKING_FUNC isworking;
    };

    std::vector<struct so_info> so_list;

    static PluginMngr *p_instance;

    char *p_buff;    // 保存数据指针，无内存管理
    size_t len;

    int frame_id;
    double sim_time;
    unsigned char sender;
    std::string xodr;
    bool is_new_xodr;
    int simulate_state;
    std::hash<std::string> hasher;  //创建一个哈希类型对象

public:
    ~PluginMngr();

    static PluginMngr *get_instance();

    // 实现 cpp time 类中的 virtual 方法
    void timerEvent();

    void update(const std::map<std::string, bool> &sensor_running_map);
    bool isSensorRunning(const std::map<std::string, bool> &sensor_running_map,
                         const std::string& sensor_id);

    void stop_simulator();

    void parse_gt(void *gt, int len);
    bool get_ground_truth(void **pp_buff, size_t &len);
    int get_frame_id();
    double get_sim_time();
    unsigned char get_sender();
    std::string get_xodr();     // return: RoadSystem::Instance()->parseOpenDrive(filename) 使用的名字
    bool get_isnew_xodr();
    int get_simulate_state();   // return: IS_START IS_STOP IS_SIMULATE

    void Destroy(); //删除单列

    void init();

    //虚拟城市对接大疆算法时只生成和发送OSIPkg数据
    void virtualCityOsiUpdate();

private:
    PluginMngr();

    bool load_so
    (
        std::string &soFilePath,
        void **so_handle,
        INIT_FUNC &init_func,
        UPDATA_FUNC &update_func,
        STOP_FUNC &stop_func,
        ISWORKING_FUNC &isworking_func
    );

    void set_sim_time(double simTime);
    void set_sender(unsigned char sender);
    void set_ground_truth(void *msg, size_t len);
    void set_frame_id(int id);
    void set_xodr(const std::string xodr);
    void set_isnew_xodr(const bool is);
    void set_simulate_state(const int state);

};





#endif // __PLUGIN_MNGR_H__


