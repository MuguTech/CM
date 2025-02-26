#ifndef __SENSOR_CFG_H__
#define __SENSOR_CFG_H__


#include "PluginCfg.h"
#include "Sensor/SensorDescription.h"
#include "Sensor/LidarDescription.h"

#include "Sensor/RadarDescription.h"
#include "Sensor/CameraDescription.h"
#include "Sensor/UltrasonicDescription.h"
#include <algorithm>
#include <vector>
#include <string>


class SensorCfg:public PluginCfg
{
private:
    struct _position
    {
        double dx;
        double dy;
        double dz;
        double dh;
        double dp;
        double dr;
    };

    std::vector<std::string> filter;
    struct _position position;
    struct _socket socket;
    SensorDescription *p_desc;

    bool sensorSocketEnable;    //是否开启socket服务端与AD连接

public:
    SensorCfg(SensorDescription *pDesc);
    ~SensorCfg();
    std::vector<std::string> get_filter() {return filter;}
    struct _position get_position(){return position;}
    struct _socket get_socket_info(){return socket;}

    void  setFrmRate(int f) { this->p_desc->frameRate = f;}
    int  getFrmRate() { return p_desc->frameRate;}
    SensorDescription* getDesc() {return p_desc;}

    void add_filter(const std::string obj_type){filter.push_back(obj_type);}
    void set_type (const std::string type) override ;
    void set_position(const double dx, const double dy, const double dz,
                      const double dh, const double dp, const double dr);

    inline void set_fov(const double h, const double v) { p_desc->hFov = h; p_desc->vFov = v;}
    inline void set_detect_dist(const double n, const double f) { p_desc->minimumDetectRange = n; p_desc->range = f;}

    void set_socket(const std::string name, const std::string ip, const std::string port, const std::string type);

    bool get_socket_enable();                           //获取是否开启socket服务端与AD连接
    void set_socket_enable(const std::string &enable);  //设置是否开启socket服务端与AD连接

    bool get_boundingbox_enable() { return p_desc->boundingBoxEnable; };   
    void set_boundingbox_enable(const std::string &enable);

    int space()
    {
        static std::vector<int> v = {
            sizeof(SensorDescription),
            sizeof(LidarDescription),
            sizeof(RadarDescription),
            sizeof(CameraDescription),
            sizeof(UltrasonicDescription),
        };
        return *max_element(v.begin(), v.end());
    }

   void set_name (const std::string &name) override ;

};




#endif  // __SENSOR_CFG_H__

