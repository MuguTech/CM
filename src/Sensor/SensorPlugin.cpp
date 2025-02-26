#include "SensorPlugin.h"

void init(void *desc, int inlen, uint32_t pluginId)
{
    log_compnt_mngr->info("sensorPlugin init start.");
    if (desc == nullptr)
    {
        log_compnt_mngr->error("sensorPlugin init desc is nullptr");
        return;
    }
    std::shared_ptr<Sensor> sensor = nullptr;
    SensorDescription *_description = nullptr;
    _description = (SensorDescription*)desc;
    bIsWorking = _description->enable;

    // 更改SENSOR_DETECTION_INFO sensorID为打开使能的传感器列表
    if (_description->type == SENSOR_TYPE_BASIC_SENSOR && _description->enable == true)
    {
        BasicSensorDescription *description = (BasicSensorDescription*)(_description);
        sensor = std::make_shared<IdealizedSensor>((*description));
    }
    // 更改SENSOR_DETECTION_INFO sensorID为打开使能的传感器列表
    else if (_description->type == SENSOR_TYPE_LIDAR && _description->enable == true)
    {
        LidarDescription *description = nullptr;
        //获取CM传感器的控制方式，1:一般配置文件模式 2:HMI模式
        if (ConfigureMngr::getInstance()->getSensorControllerMode() != 2)
        {
            description = (LidarDescription*)(_description);
            sensor = std::make_shared<Lidar>(*description);
        }
        else
        {
            //获取雷达传感器list
            std::list<LidarDescription*> lidarList = HmiPraseMsg::Instance()->getLidarDescList();
            for(std::list<LidarDescription*>::iterator it = lidarList.begin(); it != lidarList.end(); ++it)
            {
                log_compnt_mngr->debug("lidar description->name is = {},it -> name ={}",_description->name,(*it)->name);
                if ((*it) != nullptr)
                {
                    //根据传感器名称确认雷达list中对应的传感器配置
                    if(strcmp(_description->name,(*it)->name) == 0)
                    {
                        description =  *it;
                        break;
                    }
                }
            }

            if (description != nullptr)
            {
                sensor = std::make_shared<Lidar>(*description);
            }
        }
    }
    // 更改SENSOR_DETECTION_INFO sensorID为打开使能的传感器列表
    else if (_description->type == SENSOR_TYPE_CAMERA && _description->enable == true)
    {
        CameraDescription *description = nullptr;
        //获取CM传感器的控制方式，1:一般配置文件模式 2:HMI模式
        if (ConfigureMngr::getInstance()->getSensorControllerMode() != 2)
        {
            description = (CameraDescription*)(_description);
            sensor = std::make_shared<Camera>(*description);
        }
        else
        {
            //获取摄像机传感器list
           std::list<CameraDescription*> camerList = HmiPraseMsg::Instance()->getCameraDescList();
            for(std::list<CameraDescription*>::iterator it = camerList.begin(); it != camerList.end(); ++it)
            {
                log_compnt_mngr->debug("camera description->name is = {},it -> name ={}",_description->name,(*it)->name);
                if ((*it) != nullptr)
                {
                    //根据传感器名称确认雷达list中对应的传感器配置
                    if(strcmp(_description->name,(*it)->name) == 0)
                    {
                        description =  *it;
                        break;
                    }
                }
            }

            if (description != nullptr)
            {
                sensor = std::make_shared<Camera>(*description);
            }
        }
    }
    // 更改SENSOR_DETECTION_INFO sensorID为打开使能的传感器列表
    else if (_description->type == SENSOR_TYPE_RADAR && _description->enable == true)
    {
        RadarDescription *description = nullptr;
        //获取CM传感器的控制方式，1:一般配置文件模式 2:HMI模式
        if (ConfigureMngr::getInstance()->getSensorControllerMode() != 2)
        {
            description = (RadarDescription*)(_description);
            sensor = std::make_shared<Radar>(*description);
        }
        else
        {
            //获取毫米波传感器list
            std::list<RadarDescription*> raderList = HmiPraseMsg::Instance()->getRadarDescList();
            for(std::list<RadarDescription*>::iterator it = raderList.begin(); it != raderList.end(); ++it)
            {
                log_compnt_mngr->debug("rader description->name is = {},it -> name ={}",_description->name,(*it)->name);
                if ((*it) != nullptr)
                {
                    //根据传感器名称确认雷达list中对应的传感器配置
                    if(strcmp(_description->name,(*it)->name) == 0)
                    {
                        description =  *it;
                        break;
                    }
                }
            }

            if (description != nullptr)
            {
                sensor = std::make_shared<Radar>(*description);
            }
        }

    }
    // 更改SENSOR_DETECTION_INFO sensorID为打开使能的传感器列表
    else if (_description->type == SENSOR_TYPE_ULTRASONIC && _description->enable == true)
    {
        UltrasonicDescription *description = nullptr;
        //获取CM传感器的控制方式，1:一般配置文件模式 2:HMI模式
        if (ConfigureMngr::getInstance()->getSensorControllerMode() != 2)
        {
            description = (UltrasonicDescription*)(_description);
            sensor = std::make_shared<Ultrasonic>(*description);
        }
        else
        {
            //获取超声波传感器list
            std::list<UltrasonicDescription*> UltrasonicList = HmiPraseMsg::Instance()->getUltrasonicDescList();
            for(std::list<UltrasonicDescription*>::iterator it = UltrasonicList.begin(); it != UltrasonicList.end(); ++it)
            {
                log_compnt_mngr->debug("UltrasonicList description->name is = {},it -> name ={}",_description->name,(*it)->name);
                if ((*it) != nullptr)
                {
                    //根据传感器名称确认雷达list中对应的传感器配置
                    if(strcmp(_description->name,(*it)->name) == 0)
                    {
                        description =  *it;
                        break;
                    }
                }
            }
            if (description != nullptr)
            {
                sensor = std::make_shared<Ultrasonic>(*description);
            }
        }
    }

    if (sensor != nullptr)
    {
        sensor->setSensorId(pluginId);

        sensorIdMap[pluginId] = sensor;
    }
    log_compnt_mngr->info("sensorPlugin init end.");
}

void update(uint32_t pluginId, void *input, int inlen, void *output, int &outlen)
{
    log_compnt_mngr->info("sensorPlugin update start.");
    auto it = sensorIdMap.find(pluginId);

    if (it != sensorIdMap.end())
    {
        if (it->second != nullptr)
        {
            it->second->update(input, inlen, output, outlen);
        }
    }
    log_compnt_mngr->info("sensorPlugin update end.");
}

void stop()
{
    log_compnt_mngr->info("sensorPlugin so stop");
    bIsWorking = false;
    return;
}

void isWorking()
{
    if (bIsWorking)
    {
        log_compnt_mngr->info("sensorPlugin so is working");
    }
    else
    {
        log_compnt_mngr->info("sensorPlugin so is not working, so stop");
    }
    return;
}
