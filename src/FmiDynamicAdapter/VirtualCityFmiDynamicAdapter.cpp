#include <iostream>
#include "../log.h"
#include <memory.h>
#include <dlfcn.h>
#include "../../../APF/Unzip/unzip.h"
#include "VirtualCityFmiDynamicAdapter.h"
#include "../ConfigureMngr.h"

VirtualCityFmiDynamicAdapter *VirtualCityFmiDynamicAdapter::_instance = nullptr;

VirtualCityFmiDynamicAdapter *VirtualCityFmiDynamicAdapter::Instance()
{
    if (!_instance)
    {
        _instance = new VirtualCityFmiDynamicAdapter();
    }

    return _instance;
}

void VirtualCityFmiDynamicAdapter::Destroy()
{
    if (_instance != nullptr)
    {
        delete _instance;
        _instance = nullptr;
    }
}


VirtualCityFmiDynamicAdapter::VirtualCityFmiDynamicAdapter()
{
    so_vehicledynamic = nullptr; //动力学模型
    isLoadDynamic = false; //是否加载过了动力学.so
    fmi2InstantiateObj = nullptr; //fmi实例化对象

    //设置给fmu用的回调函数
    callbackFunc.logger = cbLogMessage;
    callbackFunc.allocateMemory = calloc;
    callbackFunc.freeMemory = free;
    callbackFunc.stepFinished = nullptr;
    callbackFunc.componentEnvironment = fmi2InstantiateObj;

    vehicleDynamicsSelect = 1; //纵向控制选择
	vehicleSteeringSelect = 1; //横向控制选择

    simulationTime = 0.0; //当前仿真时间
    stepTime = 0.0; //单帧的时间

    // Bug16525 zxz 2023.6.1 联仿复测数据不一致
    dynOutput = {0}; //结构体初始化

    //bug17650 wwh 23.6.21
    steeringWheel = 0.0; //方向盘转角
    accX = 0.0; //纵向加速度
	speedH = 0.0; //横摆角速度

    //道路支持摩擦系数 wwh 23.12.7
    isXoscSetPavement = false;
    pavementFromXosc = 1.0;
    pavementFromHmi = 0.8;

    /* wanghong 2024.8.27 [SimPro集成EchoSim] [ADD] [START] */
    vehicleName = "default";               //车辆name固定为default
    licenseFilePath = "";                  //license文件路径
    vehicleParameterFilePath = "../resource/VehicleDynamics/VehicleDynamicsParam/test.par"; //车模参数文件路径
    roadFilePath = "";                     //默认地图文件路径
    controlParameterFile = "../resource/VehicleDynamics/VehicleDynamicsParam/ControlParameters.txt"; //加速度控制参数文件路径
    initYaw = 0.0;
    initPitch = 0.0;
    initRoll = 0.0;
    initX0 = 0.0;
    initY0 = 0.0;
    initZ0 = 0.0;
    trajectoryPoint = "default";
    fmiOutputDataX = 0.0;
    fmiOutputDataY = 0.0;
    initVelo = 0.0;
    /* wanghong 2024.8.27 [SimPro集成EchoSim] [ADD] [END] */

    /* wanghong 2024.7.16 [bug30791] [ADD] [START] */
    sndFmiDynamicErrorMsg = false;
    /* wanghong 2024.7.16 [bug30791] [ADD] [END] */

    /* zxz 2024.09.13 [FAW-DEV-474] [虚拟城市对接大疆的OSI算法] [ADD] [START] */
    m_initParams = {0};
    m_rainIntensity = 0.0;
    m_snowIntensity = 0.0;
    /* zxz 2024.09.13 [FAW-DEV-474] [虚拟城市对接大疆的OSI算法] [ADD] [END] */
}

VirtualCityFmiDynamicAdapter::~VirtualCityFmiDynamicAdapter()
{
    if (so_vehicledynamic != nullptr)
    {
        dlclose(so_vehicledynamic);
        so_vehicledynamic = nullptr;
    }
}

void VirtualCityFmiDynamicAdapter::init()
{
    dynOutput = {0}; //结构体初始化

    //加载动力学fmu,只会在第一次初始化时加载一次
    if (!isLoadDynamic)
    {
        bool isLoadSuccessed = loadFmuModel();
        
        if (!isLoadSuccessed) //加载失败
        {
            return;
        }
        else
        {
            isLoadDynamic = true;
        }
    }

    double frameRate = ConfigureMngr::getInstance()->getFrameRate(); //获取仿真帧率
    stepTime = 1.0 / frameRate; //单帧的时间
    simulationTime = 0.0; //当前仿真时间

    if (fmi2InstantiateObj) //成功实例化后
    {
        //设置静态参数
        fmi2Real startSpeed = m_initParams.startSpeed; //初始速度

        initVelo = startSpeed;

        log_compnt_mngr->debug("VirtualCityFmiDynamicAdapter,init,startSpped = {}",startSpeed);

        roadFilePath = PluginMngr::get_instance()->get_xodr();
        if (roadFilePath.empty())
        {
            roadFilePath = "default";
            log_compnt_mngr->error("VirtualCityFmiDynamicAdapter,roadFilePath is empty");
        }

        vehicleDynamicsSelect = 1; //大疆算法使用预期加速度控制方式
        vehicleSteeringSelect = 1; //大疆算法使用方向盘转角控制方式

        initX0 = m_initParams.initX0;
        initY0 = m_initParams.initY0;
        initZ0 = m_initParams.initZ0;

        initYaw = m_initParams.initYaw;
        initPitch = m_initParams.initPitch;
        initRoll = m_initParams.initRoll;
        // std::cout << "initX0 = " << initX0 << " initY0 = " << initY0 << " initZ0 = " << initZ0
        // << " initYaw = " << initYaw << " initPitch = " << initPitch << " initRoll = " << initRoll << std::endl;

        log_compnt_mngr->info("VirtualCityFmiDynamicAdapter::init initX0 = {}, initY0 = {}, initZ0 = {}, vehicleDynamicsSelect = {}, vehicleSteeringSelect = {}",
                initX0, initY0, initZ0, vehicleDynamicsSelect, vehicleSteeringSelect);

        //设置String类型的静态参数,车辆ID、license文件路径、车模参数文件路径、道路文件路径、加速度控制参数文件路径
        const fmi2ValueReference stringValueReferences[FMI_SETTING_STRING_count] = {FMI_PARAMETER_REFERENCE::VEHICLE_NAME, FMI_PARAMETER_REFERENCE::LICENSE_FILE_PATH, FMI_PARAMETER_REFERENCE::VEHICLE_PARAMETER_FILE_PATH, FMI_PARAMETER_REFERENCE::ROAD_FILE_PATH, FMI_PARAMETER_REFERENCE::CONTROL_PARAMETER_FILE_PATH};
        const fmi2String stringValues[FMI_SETTING_STRING_count] = {vehicleName.c_str(), licenseFilePath.c_str(), vehicleParameterFilePath.c_str(), roadFilePath.c_str(), controlParameterFile.c_str()};
        fmi2SetStringTYPE *setString = (fmi2SetStringTYPE *)dlsym(so_vehicledynamic, "fmi2SetString");
        setString(fmi2InstantiateObj, stringValueReferences, FMI_SETTING_STRING_count, stringValues);

        // std::cout << "vehicleName = " << vehicleName << " \nlicenseFilePath = " << licenseFilePath << " \nvehicleParameterFilePath = " << vehicleParameterFilePath << " \nroadFilePath = " << roadFilePath << " \ncontrolParameterFile = " << controlParameterFile << std::endl;

        //设置real类型的静态参数,初始速度、初始航向角、初始俯仰角、初始侧倾角、初始x坐标、初始y坐标、初始z坐标
        const fmi2ValueReference float64ValueReferences[7] = {FMI_PARAMETER_REFERENCE::INIT_VELOCITY, FMI_PARAMETER_REFERENCE::INTI_YAW, FMI_PARAMETER_REFERENCE::INIT_PITCH, FMI_PARAMETER_REFERENCE::INIT_ROLL,
                                                              FMI_PARAMETER_REFERENCE::INIT_X0, FMI_PARAMETER_REFERENCE::INIT_Y0, FMI_PARAMETER_REFERENCE::INIT_Z0};
        const fmi2Real float64Values[7] = {startSpeed, initYaw, initPitch, initRoll, initX0, initY0, initZ0};
        fmi2SetRealTYPE *setReal = (fmi2SetRealTYPE *)dlsym(so_vehicledynamic, "fmi2SetReal");
        fmi2Status setRealStatus = setReal(fmi2InstantiateObj, float64ValueReferences, 7, float64Values);

        if (setRealStatus != fmi2OK)
        {
            log_compnt_mngr->error("VirtualCityFmiDynamicAdapter,init,setReal failed, simulationTime = {}", simulationTime);
        }

        // std::cout << "vehicleDynamicsSelect = " << vehicleDynamicsSelect << " vehicleSteeringSelect = " << vehicleSteeringSelect << std::endl;

        //设置Integer类型的静态参数，纵向控制选择, 横向控制选择
        const fmi2ValueReference int32ValueReferences[FMI_SETTING_INT_count] = {FMI_PARAMETER_REFERENCE::VEHICLE_DYNAMICS_SELECT, FMI_PARAMETER_REFERENCE::VEHICLE_STEERING_SELECT};
        const fmi2Integer int32Values[FMI_SETTING_INT_count] = {vehicleDynamicsSelect, vehicleSteeringSelect};

        fmi2SetIntegerTYPE *setInteger = (fmi2SetIntegerTYPE *)dlsym(so_vehicledynamic, "fmi2SetInteger");
        fmi2Status setIntegerStatus = setInteger(fmi2InstantiateObj, int32ValueReferences, FMI_SETTING_INT_count, int32Values);

        if (setIntegerStatus != fmi2OK)
        {
            log_compnt_mngr->error("VirtualCityFmiDynamicAdapter,init,setInteger failed, simulationTime = {}", simulationTime);
        }

        //初始化Fmu
        fmi2EnterInitializationModeTYPE *enterInitializationMode = (fmi2EnterInitializationModeTYPE *)dlsym(so_vehicledynamic, "fmi2EnterInitializationMode");
        fmi2Status enterInitializationModeStatus = enterInitializationMode(fmi2InstantiateObj);

        if (enterInitializationModeStatus != fmi2OK)
        {
            log_compnt_mngr->error("VirtualCityFmiDynamicAdapter,init,enterInitializationMode failed, simulationTime = {}", simulationTime);
        }

        fmi2ExitInitializationModeTYPE *exitInitializationMode = (fmi2ExitInitializationModeTYPE *)dlsym(so_vehicledynamic, "fmi2ExitInitializationMode");
        fmi2Status exitInitializationModeStatus = exitInitializationMode(fmi2InstantiateObj);

        if (exitInitializationModeStatus != fmi2OK)
        {
            log_compnt_mngr->error("VirtualCityFmiDynamicAdapter,init,exitInitializationMode failed, simulationTime = {}", simulationTime);
        }

        /*Aaron 2022.07.01 [ADD] 解决车辆质心初始位置偏移问题 start */
        //用于保存初始质心坐标以及初始航向角信息
        m_Xrc0 = m_initParams.initX0;
        m_Yrc0 = m_initParams.initY0;
        /*Aaron 2022.04.14 [ADD] 解决车辆质心初始位置偏移问题 end*/
    }
}


void VirtualCityFmiDynamicAdapter::finish()
{
    if ((so_vehicledynamic != nullptr) && (fmi2InstantiateObj != nullptr))
    {
        fmi2ResetTYPE *reset = (fmi2ResetTYPE *)dlsym(so_vehicledynamic, "fmi2Reset");
        reset(fmi2InstantiateObj);
    }

    /* Aaron 2022.8.22 重置车身垂向位移、车身俯仰角、车身侧倾角、各车轮垂向位移 [start] */
	verticalDistance_Vehicle = 0;				/**< 车身垂向位移            @unit m*/
	pitchTheta_Vehicle = 0;						/**< 车身俯仰角             @unit rad*/
	rollTheta_Vehicle = 0;						/**< 车身侧倾角             @unit rad*/
	verticalDistance_LF = 0;					/**< 左前轮位垂向位移        @unit m*/
	verticalDistance_RF = 0;					/**< 右前轮位垂向位移        @unit m*/
	verticalDistance_LR = 0;					/**< 左后轮位垂向位移        @unit m*/
	verticalDistance_RR = 0;					/**< 右后轮位垂向位移        @unit m*/
    /* Aaron 2022.8.22 重置车身垂向位移、车身俯仰角、车身侧倾角、各车轮垂向位移 [end] */

    //道路支持摩擦系数 wwh 23.12.7
    isXoscSetPavement = false;
    pavementFromXosc = 1.0;

    /* wanghong 2024.7.16 [bug30791] [CHG] [START] */
    sndFmiDynamicErrorMsg = false;
    /* wanghong 2024.7.16 [bug30791] [CHG] [END] */

    /* wanghong 2024.8.27 [SimPro集成EchoSim] [CHG] [START] */
    initVelo = 0.0;
    /* wanghong 2024.8.27 [SimPro集成EchoSim] [CHG] [END] */
}


/**
 * @decription: 加载动力学Fmu模型
 * @return {*}	true    加载成功
 * 				false   加载失败
 */
bool VirtualCityFmiDynamicAdapter::loadFmuModel()
{
    bool isLoadSuccessed = true;

    std::string fmuZipFilePath = "../resource/VehicleDynamics/FMU/SimProVehicleDynamic.fmu"; //fmu压缩包路径
    /* wanghong 2024.1.4 [SimPro集成EchoSim] [ADD] [START] */
    std::string fmuGUID = "{4d91f554-11fb-4bd0-8640-02f621eccbf9}"; //fmu的GUID
    /* wanghong 2024.1.4 [SimPro集成EchoSim] [ADD] [END] */
    std::string fmuUnzipFilePath = "/tmp/" + fmuGUID; //解压后保存的路径

    //解压fmu文件
    bool isUnzipSuccessed = unzip(fmuZipFilePath, fmuUnzipFilePath);

    //解压完后加载动力学.so
    if(isUnzipSuccessed)
    {
        if (so_vehicledynamic == nullptr)
        {
            /* wanghong 2024.1.4 [SimPro集成EchoSim] [CHG] [START] */
            std::string vehicledynamicPath = fmuUnzipFilePath + "/libEchoVeh_FMI.so";
            /* wanghong 2024.1.4 [SimPro集成EchoSim] [CHG] [END] */
            so_vehicledynamic = dlopen(vehicledynamicPath.c_str(), RTLD_LAZY);
        }

        if (so_vehicledynamic == nullptr)
        {
            log_compnt_mngr->error("VirtualCityFmiDynamicAdapter,init,dlopen NG");
            isLoadSuccessed = false;
        }
        else
        {
            fmi2InstantiateTYPE *func = (fmi2InstantiateTYPE *)dlsym(so_vehicledynamic, "fmi2Instantiate");

            //检查获取函数是否成功
            if (func != nullptr)
            {
                std::string instanceName = "SimproDynamicFmu"; //实例化名称
                std::string fmuResourceLocation = "file:///" + fmuUnzipFilePath; //解压后的fmu文件路径
                fmi2InstantiateObj = func(instanceName.c_str(), fmi2CoSimulation, fmuGUID.c_str(), fmuResourceLocation.c_str(), &callbackFunc, fmi2False, fmi2False);
            }

            //检查是否实例化成功
            if (fmi2InstantiateObj == nullptr)
            {
                //实例化失败
                log_compnt_mngr->error("VirtualCityFmiDynamicAdapter,init,fmi2Instantiate NG");
                isLoadSuccessed = false;
            }
        }
    }

    /* wanghong 2024.1.4 [SimPro集成EchoSim] [ADD] [START] */
    if (isLoadSuccessed) //加载动力学成功设置license和车模参数文件路径
    {
        licenseFilePath = fmuUnzipFilePath + "/license.lic";                  //license文件路径
    }
    /* wanghong 2024.1.4 [SimPro集成EchoSim] [ADD] [END] */

    return isLoadSuccessed;
}


/**
 * @decription: 用于fmu使用的日志函数
 */
void VirtualCityFmiDynamicAdapter::cbLogMessage(fmi2ComponentEnvironment componentEnvironment, fmi2String instanceName, fmi2Status status, fmi2String category, fmi2String _message, ...)
{
    std::string categoryStatus = category;

    if (status != fmi2OK)
    {
        log_compnt_mngr->error("VirtualCityFmiDynamicAdapter,cbLogMessage,status={}, _message={}", status, _message);

        if (status == fmi2Error)
        {
            log_compnt_mngr->error("VirtualCityFmiDynamicAdapter,cbLogMessage,category={}", category);
        }
    }   
}

/**
 * @decription: 使用动力学模型来更新数据
 * @param  cosimuData	来自AD的数据
 */
void VirtualCityFmiDynamicAdapter::useDynamicsModel(COSIMU_AD_DATA_MANAGER_t *cosimuData)
{
	log_compnt_mngr->critical("useDynamicsModel input,throttlePedal=,speed.x=,accelTgt=,steeringTgt=,gear=,speedTgt=,brakePedal=,steeringWheel=,steeringSpeed=,steeringTorque=,{},{},{},{},{},{},{},{},{},{}",
					 cosimuData->driver_ctrl.throttlePedal,cosimuData->object_state.ext.speed.x,cosimuData->driver_ctrl.accelTgt,cosimuData->driver_ctrl.steeringTgt,cosimuData->driver_ctrl.gear,
					 cosimuData->driver_ctrl.speedTgt,cosimuData->driver_ctrl.brakePedal,cosimuData->driver_ctrl.steeringWheel,cosimuData->driver_ctrl.steeringSpeed,cosimuData->driver_ctrl.steeringTorque);

    if ((fmi2InstantiateObj != nullptr) && (so_vehicledynamic != nullptr))
    {
        //Step1.input
        VehicleDynamicModelInput_t inputdata = {0};

        inputdata.acc = cosimuData->driver_ctrl.accelTgt; //预期纵向加速度输入
        inputdata.throttle = cosimuData->driver_ctrl.throttlePedal; //油门开度输入
        inputdata.Wheelcylinderpressure = cosimuData->driver_ctrl.MasterCylinderPressure; //制动主缸压力输入
        inputdata.Veh_SteeringWheelAngle = cosimuData->driver_ctrl.steeringWheel*180.0 / M_PI; //方向盘转角输入
        inputdata.Veh_SteeringWheelTorque = cosimuData->driver_ctrl.steeringTorque; //方向盘力矩输入
        inputdata.frictionHmi = pavementFromHmi; //HMI摩擦系数
        inputdata.frictionOpenScenario = pavementFromXosc; //场景文件摩擦系数比例因子

        double rainIntensity = m_rainIntensity;
        double snowIntensity = m_snowIntensity;
        if (rainIntensity < 0.0)
        {
            rainIntensity = 0.0;
        }
        else if (rainIntensity > 0.99)
        {
            rainIntensity = 0.99;
        }

        if (snowIntensity < 0.0)
        {
            snowIntensity = 0.0;
        }
        else if (snowIntensity > 0.99)
        {
            snowIntensity = 0.99;
        }

        inputdata.rainIntensity = rainIntensity; //降雨量(0-1)
        inputdata.snowIntensity = snowIntensity; //降雪量(0-1)
        inputdata.targetSpeed = cosimuData->driver_ctrl.speedTgt; //目标速度输入
        inputdata.stopDistance = cosimuData->driver_ctrl.stopDistance; //剩余停车距离输入
        inputdata.gearShift = cosimuData->driver_ctrl.gear; //档位输入
        inputdata.BrkType = cosimuData->driver_ctrl.brkType;//制动类型输入
        if (isXoscSetPavement)
        {
            inputdata.enableGlobalFriction = 1; //摩擦系数是否启动场景的全局配置(0:关闭,1:开启)
        }
        else
        {
            inputdata.enableGlobalFriction = 0; //摩擦系数是否启动场景的全局配置(0:关闭,1:开启)
        }

        inputdata.trajecoryPoint = trajectoryPoint;

        log_compnt_mngr->info("useDynamicsModel inputdata acc = {}, throttle = {}, Wheelcylinderpressure = {}, Veh_SteeringWheelAngle = {}, Veh_SteeringWheelTorque = {}\
                        , frictionHmi = {}, frictionOpenScenario = {}, rainIntensity = {}, snowIntensity = {}, enableGlobalFriction = {}\
                        , Mt_LF = {}, Mt_RF = {}, Mt_LR = {}, Mt_RR = {}, targetSpeed = {}, stopDistance = {}, gearShift = {}, BrkType = {}\
                        , trajectoryPoint = {}, STEPTIME = {}",
                        inputdata.acc, inputdata.throttle, inputdata.Wheelcylinderpressure, inputdata.Veh_SteeringWheelAngle, inputdata.Veh_SteeringWheelTorque,
                        inputdata.frictionHmi, inputdata.frictionOpenScenario, inputdata.rainIntensity, inputdata.snowIntensity, inputdata.enableGlobalFriction,
                        inputdata.Mt_LF, inputdata.Mt_RF, inputdata.Mt_LR, inputdata.Mt_RR, inputdata.targetSpeed, inputdata.stopDistance, inputdata.gearShift, inputdata.BrkType,
                        inputdata.trajecoryPoint, simulationTime);

        //设置real类型的input参数
        const fmi2ValueReference float64ValueReferences[FMI_INPUT_DOUBLE_count] = {FMI_INPUT_REFERENCE::ACC, FMI_INPUT_REFERENCE::THROTTLE, FMI_INPUT_REFERENCE::WHEELCYLINDERPRESSURE, FMI_INPUT_REFERENCE::VEH_STEERING_WHEEL_ANGLE,
                                                               FMI_INPUT_REFERENCE::VEH_STEERING_WHEEL_TORQUE, FMI_INPUT_REFERENCE::FRICTION_HMI, FMI_INPUT_REFERENCE::FRICTION_OPEN_SCENARIO, FMI_INPUT_REFERENCE::RAIN_INTENSITY,
                                                           FMI_INPUT_REFERENCE::SNOW_INTENSITY, FMI_INPUT_REFERENCE::MT_LF, FMI_INPUT_REFERENCE::MT_RF, FMI_INPUT_REFERENCE::MT_LR,
                                                               FMI_INPUT_REFERENCE::MT_RR, FMI_INPUT_REFERENCE::TARGET_SPEED, FMI_INPUT_REFERENCE::STOP_DISTANCE};
        const fmi2Real float64Values[FMI_INPUT_DOUBLE_count] = {inputdata.acc, inputdata.throttle, inputdata.Wheelcylinderpressure, inputdata.Veh_SteeringWheelAngle, inputdata.Veh_SteeringWheelTorque,
                                            inputdata.frictionHmi, inputdata.frictionOpenScenario, inputdata.rainIntensity, inputdata.snowIntensity, 
                                            inputdata.Mt_LF, inputdata.Mt_RF, inputdata.Mt_LR, inputdata.Mt_RR, inputdata.targetSpeed, inputdata.stopDistance};

        fmi2SetRealTYPE *setReal = (fmi2SetRealTYPE *)dlsym(so_vehicledynamic, "fmi2SetReal");
        fmi2Status setRealStatus = setReal(fmi2InstantiateObj, float64ValueReferences, FMI_INPUT_DOUBLE_count, float64Values);

        if (setRealStatus != fmi2OK)
        {
            log_compnt_mngr->debug("VirtualCityFmiDynamicAdapter,useDynamicsModel,setReal failed, simulationTime = {}", simulationTime);
        }

        //设置Integer类型的静态参数，档位输入, 制动类型输入, 摩擦系数是否启动场景的全局配置
        const fmi2ValueReference int32ValueReferences[FMI_INPUT_INT_count] = {FMI_INPUT_REFERENCE::GEAR_SHIFT_INPUT, FMI_INPUT_REFERENCE::BRK_TYPE, FMI_INPUT_REFERENCE::ENABLE_GLOBAL_FRICTION};
        const fmi2Integer int32Values[FMI_INPUT_INT_count] = {inputdata.gearShift, inputdata.BrkType, inputdata.enableGlobalFriction};

        fmi2SetIntegerTYPE *setInteger = (fmi2SetIntegerTYPE *)dlsym(so_vehicledynamic, "fmi2SetInteger");
        fmi2Status setIntegerStatus = setInteger(fmi2InstantiateObj, int32ValueReferences, FMI_INPUT_INT_count, int32Values);

        if (setIntegerStatus != fmi2OK)
        {
            log_compnt_mngr->debug("VirtualCityFmiDynamicAdapter,useDynamicsModel,setInteger failed, simulationTime = {}", simulationTime);
        }

        //设置string类型的input参数, 预测轨迹点坐标
        const fmi2ValueReference stringValueReferences[FMI_INPUT_STRING_count] = {FMI_INPUT_REFERENCE::TRAJECTORY_POINT};
        const fmi2String stringValues[FMI_INPUT_STRING_count] = {inputdata.trajecoryPoint.c_str()};

        fmi2SetStringTYPE *setString = (fmi2SetStringTYPE *)dlsym(so_vehicledynamic, "fmi2SetString");
        setString(fmi2InstantiateObj, stringValueReferences, FMI_INPUT_STRING_count, stringValues);

        //Step2.doStecp
        fmi2DoStepTYPE *doStep = (fmi2DoStepTYPE *)dlsym(so_vehicledynamic, "fmi2DoStep");
        fmi2Status doStepStatus = doStep(fmi2InstantiateObj, simulationTime, stepTime, fmi2True);

        //计算当前的仿真时间
        simulationTime += stepTime;

        if (doStepStatus != fmi2OK)
        {
            log_compnt_mngr->debug("VirtualCityFmiDynamicAdapter,useDynamicsModel,doStep failed, simulationTime = {}", simulationTime);
        }

        //Step3.output
        //获取real类型的output参数
        const fmi2ValueReference getFloat64ValueReferences[FMI_OUTPUT_DOUBLE_count] = {FMI_OUTPUT_REFERENCE::VX, FMI_OUTPUT_REFERENCE::AX, FMI_OUTPUT_REFERENCE::WR, FMI_OUTPUT_REFERENCE::AY, FMI_OUTPUT_REFERENCE::W_LF_WHEEL_SPD,
                                                                  FMI_OUTPUT_REFERENCE::W_RF_WHEEL_SPD, FMI_OUTPUT_REFERENCE::W_LR_WHEEL_SPD, FMI_OUTPUT_REFERENCE::W_RR_WHEEL_SPD, FMI_OUTPUT_REFERENCE::WHEELCYLINDERPRESSURE_OUT,
                                                                  FMI_OUTPUT_REFERENCE::X_LF, FMI_OUTPUT_REFERENCE::Y_LF, FMI_OUTPUT_REFERENCE::X_RF, FMI_OUTPUT_REFERENCE::Y_RF, FMI_OUTPUT_REFERENCE::X_LR, FMI_OUTPUT_REFERENCE::Y_LR,
                                                                  FMI_OUTPUT_REFERENCE::X_RR, FMI_OUTPUT_REFERENCE::Y_RR, FMI_OUTPUT_REFERENCE::X_MC, FMI_OUTPUT_REFERENCE::Y_MC, FMI_OUTPUT_REFERENCE::VERTICAL_DISTANCE_VEHICLE,
                                                                  FMI_OUTPUT_REFERENCE::PITCH_THETA_VEHICLE, FMI_OUTPUT_REFERENCE::ROLL_THETA_VEHICLE, FMI_OUTPUT_REFERENCE::VERTICAL_DISTANCE_LF, FMI_OUTPUT_REFERENCE::VERTICAL_DISTANCE_RF,
                                                                  FMI_OUTPUT_REFERENCE::VERTICAL_DISTANCE_LR, FMI_OUTPUT_REFERENCE::VERTICAL_DISTANCE_RR, FMI_OUTPUT_REFERENCE::STEERING_WHEEL_ANGLE_OUTPUT, FMI_OUTPUT_REFERENCE::STEERING_WHEEL_TORQUE_OUTPUT,
                                                                  FMI_OUTPUT_REFERENCE::THROTTLE_OUTPUT, FMI_OUTPUT_REFERENCE::BRAKE_PEDAL_TRAVEL, FMI_OUTPUT_REFERENCE::VEHICLE_CURV, FMI_OUTPUT_REFERENCE::STEERING_WHEEL_ANGULAR_RATE,
                                                                  FMI_OUTPUT_REFERENCE::DRV_TORQ_ACT, FMI_OUTPUT_REFERENCE::YAW_THETA_VEHICLE, FMI_OUTPUT_REFERENCE::PITCH_RATE_VEHICLE, FMI_OUTPUT_REFERENCE::ROLL_RATE_VEHICLE,
                                                                  FMI_OUTPUT_REFERENCE::V_LF_WHEEL_SPD, FMI_OUTPUT_REFERENCE::V_RF_WHEEL_SPD, FMI_OUTPUT_REFERENCE::V_LR_WHEEL_SPD, FMI_OUTPUT_REFERENCE::V_RR_WHEEL_SPD,
                                                                  FMI_OUTPUT_REFERENCE::SIN_SLOPE_OUT, FMI_OUTPUT_REFERENCE::TMOTOR_CMD, FMI_OUTPUT_REFERENCE::TMOTOR, FMI_OUTPUT_REFERENCE::ENERGY_SOURCE_LEVEL, FMI_OUTPUT_REFERENCE::W_GEAR_SPD,
                                                                  FMI_OUTPUT_REFERENCE::AZ, FMI_OUTPUT_REFERENCE::XRC_GLOBAL, FMI_OUTPUT_REFERENCE::YRC_GLOBAL, FMI_OUTPUT_REFERENCE::VX_GLOBAL, FMI_OUTPUT_REFERENCE::VY_GLOBAL};
        fmi2Real getFloat64Values[FMI_OUTPUT_DOUBLE_count] = {0};
        fmi2GetRealTYPE *getReal = (fmi2GetRealTYPE *)dlsym(so_vehicledynamic, "fmi2GetReal");
        fmi2Status getRealStatus = getReal(fmi2InstantiateObj, getFloat64ValueReferences, FMI_OUTPUT_DOUBLE_count, getFloat64Values);

        if (getRealStatus != fmi2OK)
        {
            log_compnt_mngr->debug("VirtualCityFmiDynamicAdapter,useDynamicsModel,getReal failed, simulationTime = {}", simulationTime);
        }

        //获取Integer类型的output参数
        const fmi2ValueReference getInt32ValueReferences[FMI_OUTPUT_INT_count] = {FMI_OUTPUT_REFERENCE::GEAR_SHIFT_OUTPUT, FMI_OUTPUT_REFERENCE::LF_WHEEL_ROTATED_DIR, FMI_OUTPUT_REFERENCE::RF_WHEEL_ROTATED_DIR, FMI_OUTPUT_REFERENCE::LR_WHEEL_ROTATED_DIR,
                                                                  FMI_OUTPUT_REFERENCE::RR_WHEEL_ROTATED_DIR, FMI_OUTPUT_REFERENCE::BRAKE_PEDAL_STATUS, FMI_OUTPUT_REFERENCE::VEHICLE_DIR};
        fmi2Integer getInt32Values[FMI_OUTPUT_INT_count] = {0};
        fmi2GetIntegerTYPE *getInteger = (fmi2GetIntegerTYPE *)dlsym(so_vehicledynamic, "fmi2GetInteger");
        fmi2Status getIntegerStatus = getInteger(fmi2InstantiateObj, getInt32ValueReferences, FMI_OUTPUT_INT_count, getInt32Values);
        
        if (getIntegerStatus != fmi2OK)
        {
            log_compnt_mngr->debug("VirtualCityFmiDynamicAdapter,useDynamicsModel,getInteger failed, simulationTime = {}", simulationTime);
        }

        //将获取的值赋值给output
        VehicleDynamicModelOutput_t outputdata;
        outputdata.Vx = getFloat64Values[0]; //车辆纵向车速
        outputdata.ax = getFloat64Values[1]; //车辆纵向加速度
        outputdata.wr = getFloat64Values[2]; //车辆横摆角速度
        outputdata.Ay = getFloat64Values[3]; //车辆横向加速度

        outputdata.wLFWheelSpd = getFloat64Values[4]; //左前轮转速
        outputdata.wRFWheelSpd = getFloat64Values[5]; //右前轮转速
        outputdata.wLRWheelSpd = getFloat64Values[6]; //左后轮转速
        outputdata.wRRWheelSpd = getFloat64Values[7]; //右后轮转速
        outputdata.Wheelcylinderpressure_out = getFloat64Values[8]; //制动主缸压力

        outputdata.xLF = getFloat64Values[9]; //前左轮X坐标
        outputdata.yLF = getFloat64Values[10]; //前左轮Y坐标
        outputdata.xRF = getFloat64Values[11]; //前右轮X坐标
        outputdata.yRF = getFloat64Values[12]; //前右轮Y坐标
        outputdata.xLR = getFloat64Values[13]; //后左轮X坐标
        outputdata.yLR = getFloat64Values[14]; //后左轮Y坐标
        outputdata.xRR = getFloat64Values[15]; //后右轮X坐标
        outputdata.yRR = getFloat64Values[16]; //后右轮Y坐标
        outputdata.Xmc = getFloat64Values[17]; //车辆质心X坐标
        outputdata.Ymc = getFloat64Values[18]; //车辆质心Y坐标

        outputdata.VerticalDistance_Vehicle = getFloat64Values[19]; //车辆质心垂向位移
        outputdata.PitchTheta_Vehicle = getFloat64Values[20]; //车身俯仰角度
        outputdata.RollTheta_Vehicle = getFloat64Values[21]; //车身侧倾角度

        outputdata.VerticalDistance_LF = getFloat64Values[22]; //前左轮垂向位移
        outputdata.VerticalDistance_RF = getFloat64Values[23]; //前右轮垂向位移 
        outputdata.VerticalDistance_LR = getFloat64Values[24]; //后左轮垂向位移
        outputdata.VerticalDistance_RR = getFloat64Values[25]; //后右轮垂向位移

        outputdata.steeringWheelAngle_output = getFloat64Values[26]; //方向盘转向角度
        outputdata.steeringWheelTorque_output = getFloat64Values[27]; //方向盘转向力矩
        outputdata.Throttle_output = getFloat64Values[28]; //油门开度

        outputdata.BrakePedal_Travel = getFloat64Values[29]; //制动踏板行程
        outputdata.vehicleCurv = getFloat64Values[30]; //车辆曲率
        outputdata.steeringWheelAngularRate = getFloat64Values[31]; //方向盘转向角速度
        outputdata.DrvTorqAct = getFloat64Values[32]; //车辆驱动扭矩

        outputdata.YawTheta_Vehicle = getFloat64Values[33]; //车辆横摆角
        outputdata.PitchRate_Vehicle = getFloat64Values[34]; //车辆俯仰角速度
        outputdata.RollRate_Vehicle = getFloat64Values[35]; //车辆侧倾角速度

        outputdata.vLFWheelSpd = getFloat64Values[36]; //左前轮轮速
        outputdata.vRFWheelSpd = getFloat64Values[37]; //右前轮轮速
        outputdata.vLRWheelSpd = getFloat64Values[38]; //左后轮轮速 
        outputdata.vRRWheelSpd = getFloat64Values[39]; //右后轮轮速

        outputdata.sinSlope_out = getFloat64Values[40]; //车辆所在位置坡度
        outputdata.Tmotor_cmd = getFloat64Values[41]; //电机控制扭矩
        outputdata.Tmotor = getFloat64Values[42]; //电机实际输出扭矩
        outputdata.EnergySourceLevel = getFloat64Values[43]; //当前油量
        outputdata.wGearSpd = getFloat64Values[44]; //输出轴转速
        outputdata.Az = getFloat64Values[45]; //车辆垂向加速度

        outputdata.Xrc_Global = getFloat64Values[46]; //车辆后轴中心X坐标（世界坐标系）
        outputdata.Yrc_Global = getFloat64Values[47]; //车辆后轴中心Y坐标（世界坐标系）
        outputdata.Vx_Global = getFloat64Values[48]; //车辆纵向速度（世界坐标系）
        outputdata.Vy_Global = getFloat64Values[49]; //车辆横向速度（世界坐标系）

        outputdata.Gear_Shift = getInt32Values[0]; //档位状态
        outputdata.LFWheelRotatedDir = getInt32Values[1]; //左前轮旋转方向
        outputdata.RFWheelRotatedDir = getInt32Values[2]; //左右轮旋转方向
        outputdata.LRWheelRotatedDir = getInt32Values[3]; //后左轮旋转方向
        outputdata.RRWheelRotatedDir = getInt32Values[4]; //后右轮旋转方向
        outputdata.BrakePedalStatus = getInt32Values[5]; //制动踏板状态
        outputdata.VehicleDir = getInt32Values[6]; //车辆行驶方向

        log_compnt_mngr->info("useDynamicsModel outputdata Vx = {}, ax = {}, wr = {}, Ay = {}, wLFWheelSpd = {}, wRFWheelSpd = {}, wLRWheelSpd = {}, wRRWheelSpd = {}\
                , Wheelcylinderpressure_out = {}, xLF = {}, yLF = {}, xRF = {}, yRF = {}, xLR = {}, yLR = {}, xRR = {}, yRR = {}, Xmc = {}, Ymc = {}, VerticalDistance_Vehicle = {}\
                , PitchTheta_Vehicle = {}, RollTheta_Vehicle = {}, VerticalDistance_LF = {}, VerticalDistance_RF = {}, VerticalDistance_LR = {}, VerticalDistance_RR = {}\
                , steeringWheelAngle_output = {}, steeringWheelTorque_output = {}, Throttle_output = {}, BrakePedal_Travel = {}, vehicleCurv = {}, steeringWheelAngularRate = {}\
                , DrvTorqAct = {}, YawTheta_Vehicle = {}, PitchRate_Vehicle = {}, RollRate_Vehicle = {}, vLFWheelSpd = {}, vRFWheelSpd = {}, vLRWheelSpd = {}, vRRWheelSpd = {}\
                , sinSlope_out = {}, Tmotor_cmd = {}, Tmotor = {}, EnergySourceLevel = {}, wGearSpd = {}, Az = {}\
                , Xrc_Global = {}, Yrc_Global = {}, Vx_Global = {}, Vy_Global = {}, Gear_Shift = {}, LFWheelRotatedDir = {}, RFWheelRotatedDir = {}\
                , LRWheelRotatedDir = {}, RRWheelRotatedDir = {}, BrakePedalStatus = {}, VehicleDir = {}.",
                outputdata.Vx, outputdata.ax, outputdata.wr, outputdata.Ay, outputdata.wLFWheelSpd, outputdata.wRFWheelSpd, outputdata.wLRWheelSpd, outputdata.wRRWheelSpd\
                , outputdata.Wheelcylinderpressure_out, outputdata.xLF, outputdata.yLF, outputdata.xRF, outputdata.yRF, outputdata.xLR, outputdata.yLR, outputdata.xRR, outputdata.yRR\
                , outputdata.Xmc, outputdata.Ymc, outputdata.VerticalDistance_Vehicle, outputdata.PitchTheta_Vehicle, outputdata.RollTheta_Vehicle, outputdata.VerticalDistance_LF\
                , outputdata.VerticalDistance_RF, outputdata.VerticalDistance_LR, outputdata.VerticalDistance_RR, outputdata.steeringWheelAngle_output, outputdata.steeringWheelTorque_output\
                , outputdata.Throttle_output, outputdata.BrakePedal_Travel, outputdata.vehicleCurv, outputdata.steeringWheelAngularRate, outputdata.DrvTorqAct, outputdata.YawTheta_Vehicle\
                , outputdata.PitchRate_Vehicle, outputdata.RollRate_Vehicle, outputdata.vLFWheelSpd, outputdata.vRFWheelSpd, outputdata.vLRWheelSpd, outputdata.vRRWheelSpd\
                , outputdata.sinSlope_out, outputdata.Tmotor_cmd, outputdata.Tmotor, outputdata.EnergySourceLevel, outputdata.wGearSpd, outputdata.Az, outputdata.Xrc_Global\
                , outputdata.Yrc_Global, outputdata.Vx_Global, outputdata.Vy_Global, outputdata.Gear_Shift, outputdata.LFWheelRotatedDir, outputdata.RFWheelRotatedDir, outputdata.LRWheelRotatedDir\
                , outputdata.RRWheelRotatedDir, outputdata.BrakePedalStatus, outputdata.VehicleDir);

        //动力学输出参数异常判断
        if (std::isnan(outputdata.ax) || std::isnan(outputdata.wr) || std::isnan(outputdata.wLFWheelSpd) || std::isnan(outputdata.wRFWheelSpd) || std::isnan(outputdata.wLRWheelSpd) || std::isnan(outputdata.wRRWheelSpd) || std::isnan(outputdata.Wheelcylinderpressure_out)) //车辆动力学输出异常处理
        {
            if (!sndFmiDynamicErrorMsg)
            {
                log_compnt_mngr->error("useDynamicsModel output data error");

                sndFmiDynamicErrorMsg = true;
            }
            else
            {
                // do nothing
            }

            //动力学输出异常使用上一帧数据
            cosimuData->object_state.ext.accel.x = accX;    // 纵向加速度
            cosimuData->object_state.ext.speed.h = speedH;  // 横摆角速度		
            cosimuData->object_state.base.pos.x = fmiOutputDataX;
            cosimuData->object_state.base.pos.y = fmiOutputDataY;
        }
        else
        {
            cosimuData->object_state.ext.accel.x = outputdata.ax; //纵向加速度
            cosimuData->object_state.ext.accel.y = outputdata.Ay; //横向加速度
            cosimuData->object_state.ext.speed.h = outputdata.wr; //横摆角速度
            cosimuData->object_state.ext.speed.x = outputdata.Vx; //速度
            cosimuData->object_state.base.pos.h = outputdata.YawTheta_Vehicle; //车辆航向角
            cosimuData->object_state.base.pos.x = outputdata.Xrc_Global;
            cosimuData->object_state.base.pos.y = outputdata.Yrc_Global;
            fmiOutputDataX = outputdata.Xrc_Global;
            fmiOutputDataY = outputdata.Yrc_Global;

            //动力学输出异常使用上一帧数据
            accX = outputdata.ax;   // 纵向加速度
            speedH = outputdata.wr; // 横摆角速度

            dynOutput = outputdata;
            //动力学新增输出接口
            dynOutput.AccelTgt = cosimuData->driver_ctrl.accelTgt;//预期加速度
            dynOutput.brkType = cosimuData->driver_ctrl.brkType;//制动类型 (0=None 1=Comfort 2=Emergence)
            dynOutput.wGearSpd = outputdata.wGearSpd;//变速箱输出轴转速/电机输出轴转速 单位是rad/s。

            //车轮垂向震动,由于echoSim模型输出的是相对于悬架的跳动,不支持输出车轮自身跳动,采取置0方案
            verticalDistance_LF = 0.0;					/**< 左前轮位垂向位移            							    @unit m*/
            verticalDistance_RF = 0.0;					/**< 右前轮位垂向位移            							    @unit m*/
            verticalDistance_LR = 0.0;					/**< 左后轮位垂向位移            							    @unit m*/
            verticalDistance_RR = 0.0;					/**< 右后轮位垂向位移            							    @unit m*/            
        }

        //用于保存左前、右前、左后、右后轮胎以及质心坐标信息
        m_xLF = outputdata.xLF;
        m_yLF = outputdata.yLF;
        m_xRF = outputdata.xRF;
        m_yRF = outputdata.yRF;
        m_xLR = outputdata.xLR;
        m_yLR = outputdata.yLR;
        m_xRR = outputdata.xRR;
        m_yRR = outputdata.yRR;
        m_Xmc = outputdata.Xmc;
        m_Ymc = outputdata.Ymc;

        //由于echoSim模型输出的是世界坐标系下相对于初始位置的垂向位移量、车身俯仰角、车身侧倾角,不支持输出相对于路面平衡状态的垂向位移跳动量、车身俯仰角、侧倾角,采取置0方案
        verticalDistance_Vehicle = 0.0;            	/**< 车身垂向位移            @unit m*/
        pitchTheta_Vehicle = 0.0;			        /**< 车身俯仰角              @unit rad*/
        rollTheta_Vehicle = 0.0;				    /**< 车身侧倾角              @unit rad*/
    }

    steeringWheel = cosimuData->driver_ctrl.steeringWheel; // rad
}

/* zxz 2024.09.13 [FAW-DEV-474] [虚拟城市对接大疆的OSI算法] [ADD] [START] */
//设置动力学初始化结构体参数
void VirtualCityFmiDynamicAdapter::setInitParams(const VehicleDynamicModelInit_t &initParams)
{
    m_initParams = initParams;
}

//设置环境参数（降雨量、降雪量）
void VirtualCityFmiDynamicAdapter::setEnvParams(double rainIntensity, double snowIntensity)
{
    m_rainIntensity = rainIntensity;
    m_snowIntensity = snowIntensity;
}
/* zxz 2024.09.13 [FAW-DEV-474] [虚拟城市对接大疆的OSI算法] [ADD] [END] */