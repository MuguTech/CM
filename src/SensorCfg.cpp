
#include "SensorCfg.h"


SensorCfg::SensorCfg(SensorDescription *pDesc)
{
    sensorSocketEnable = false;
    p_desc = nullptr;
    if (pDesc != nullptr)
    {
        p_desc = pDesc;
    }
    else
    {
        p_desc = new SensorDescription();
    }
}


SensorCfg::~SensorCfg()
{
    if (p_desc != nullptr)
    {
        delete p_desc;
        p_desc = nullptr;
    }
}


void SensorCfg::set_position(double dx, double dy, double dz, double dh, double dp, double dr)
{
    position.dx = dx;
    position.dy = dy;
    position.dz = dz;
    position.dh = dh;
    position.dp = dp;
    position.dr = dr;

    p_desc->assemblePositionX   = dx;
    p_desc->assemblePositionY   = dy;
    p_desc->assemblePositionZ   = dz;
    p_desc->heading             = dh;
    p_desc->pitch               = dp;
    p_desc->roll                = dr;
}


void SensorCfg::set_socket(const std::string name, const std::string ip, const std::string port, const std::string type)
{
    socket.name = name;
    socket.ip = ip;
    socket.port = port;
    socket.type = type;
}

bool SensorCfg::get_socket_enable()
{
    return sensorSocketEnable;
}

void SensorCfg::set_socket_enable(const std::string &enable)
{
    if (enable == "false")
    {
        sensorSocketEnable = false;
    }
    else if (enable == "true")
    {
        sensorSocketEnable = true;
    }
    else
    {
        sensorSocketEnable = false;
    }
}

void SensorCfg::set_type(const std::string type)
{
    PluginCfg::set_type(type);
    if(type == "lidar") {
        this->p_desc->type = SENSOR_TYPE_LIDAR;
    }
    else if(type == "camera") {
        this->p_desc->type = SENSOR_TYPE_CAMERA;
    }
    else if(type == "radar") {
        this->p_desc->type = SENSOR_TYPE_RADAR;
    }
    else if(type == "ultrasonic") {
        this->p_desc->type = SENSOR_TYPE_ULTRASONIC;
    }
    else {
        this->p_desc->type = SENSOR_TYPE_UNKNOWN;
    }
    this->p_desc->enable = true;
}
                     
void SensorCfg::set_boundingbox_enable(const std::string &enable)
{
    if (enable == "true")
    {
        this->p_desc->boundingBoxEnable = true;
    }
    else
    {
        this->p_desc->boundingBoxEnable = false;
    }
 
}


void SensorCfg::set_name (const std::string &name)
{
    PluginCfg::set_name(name);
    char * sensorName = const_cast<char*>(name.c_str());
    (void)strncpy(this->p_desc->name, sensorName, sizeof(this->p_desc->name));
}