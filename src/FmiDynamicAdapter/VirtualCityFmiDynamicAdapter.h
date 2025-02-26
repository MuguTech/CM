#ifndef VirtualCityFmiDynamicAdapter_H
#define VirtualCityFmiDynamicAdapter_H

#include <iostream>
#include <fstream> 
#include <sstream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <map>

#include "../../../Runtime/coSimu/CoSimuAdDataType.h"
#include "Fmi/fmi2Functions.h"
#include "Fmi/fmi2FunctionTypes.h"
#include "Fmi/fmi2TypesPlatform.h"

// Setting paramters - string
#define FMI_SETTING_STRING_count 5 //Total amount of setting parameters - string

// Setting parameters - double
#define FMI_SETTING_DOUBLE_count 9 //Total amount of setting parameters - double

// Setting parameters - int
#define FMI_SETTING_INT_count 2 //Total amount of setting parameters - int

// Setting parameters - all
#define FMI_SETTING_count FMI_SETTING_STRING_count + FMI_SETTING_DOUBLE_count + FMI_SETTING_INT_count //Total amount of setting parameters

// Input parameters - double
#define FMI_INPUT_DOUBLE_count 15 //Total amount of input parameters - double

// Input parameters - int
#define FMI_INPUT_INT_count 3 //Total amount of input parameters - int

#define FMI_INPUT_STRING_count 1 //Total amount of input parameters - string

// Input parameters - all
#define FMI_INPUT_count FMI_INPUT_DOUBLE_count + FMI_INPUT_INT_count + FMI_INPUT_STRING_count //Total amount of input parameters - all

// Setting ang input count
#define FMI_SETTING_AND_INPUT_count FMI_INPUT_count + FMI_SETTING_count //Total amount of setting and input parameters - all

// Output parameters - double
#define FMI_OUTPUT_DOUBLE_count 50 //Total amount of output parameters

// Output parameters - int
#define FMI_OUTPUT_INT_count 7 //Total amount of output parameters


class VirtualCityFmiDynamicAdapter
{
public:
    //FMI静态变量的Reference
    enum FMI_PARAMETER_REFERENCE
    {
        VEHICLE_NAME,                   //车辆name
        LICENSE_FILE_PATH,              //license文件路径
        VEHICLE_PARAMETER_FILE_PATH,    //车模参数文件路径
        ROAD_FILE_PATH,                 //道路文件路径
        CONTROL_PARAMETER_FILE_PATH,    //加速度控制参数文件路径
        WHEELBASE,                      //轴距
        TRACK,                          //轮距
        INIT_VELOCITY,                  //初始速度
        INTI_YAW,                       //初始航向角(世界坐标系)
        INIT_PITCH,                     //初始俯仰角(世界坐标系)
        INIT_ROLL,                      //初始侧倾角(世界坐标系)
        INIT_X0,                        //初始位置车辆后轴中心位于地面的投影的初始x坐标
        INIT_Y0,                        //初始位置车辆后轴中心位于地面的投影的初始y坐标
        INIT_Z0,                        //初始位置车辆后轴中心位于地面的投影的初始z坐标
        VEHICLE_DYNAMICS_SELECT,        //纵向控制选择
        VEHICLE_STEERING_SELECT,        //横向控制选择
    };

    //FMI输入变量的Reference
    enum FMI_INPUT_REFERENCE
    {
        ACC = FMI_SETTING_count,    // 预期纵向加速度输入
        THROTTLE,                   // 油门开度输入
        WHEELCYLINDERPRESSURE,      // 制动主缸压力输入
        VEH_STEERING_WHEEL_ANGLE,   // 方向盘转角输入
        VEH_STEERING_WHEEL_TORQUE,  // 方向盘力矩输入
        FRICTION_HMI,               // HMI摩擦系数
        FRICTION_OPEN_SCENARIO,     // 场景文件摩擦系数比例因子
        RAIN_INTENSITY,             // 降雨量(0-1)
        SNOW_INTENSITY,             // 降雪量(0-1)
        MT_LF,                      // 轮端扭矩输入（前左轮）
        MT_RF,                      // 轮端扭矩输入（前右轮）
        MT_LR,                      // 轮端扭矩输入（后左轮）
        MT_RR,                      // 轮端扭矩输入（后右轮）
        TARGET_SPEED,               // 目标速度输入（echoSim速度控制接口）
        STOP_DISTANCE,              // 剩余停车距离输入
        GEAR_SHIFT_INPUT,           // 档位输入
        BRK_TYPE,                   // 制动类型输入
        ENABLE_GLOBAL_FRICTION,     // 摩擦系数是否启动场景的全局配置(0:关闭,1:开启)
        TRAJECTORY_POINT,           // 轨迹点坐标(x1,y1,z1,x2,y2,z2...),共20个轨迹点坐标,每个数值用逗号分隔
    };

    //FMI输出变量的Reference
    enum FMI_OUTPUT_REFERENCE
    {
        VX = FMI_SETTING_AND_INPUT_count,     // 车速,单位是m/s，正值向前负值向后。
        AX,                                   // 纵向加速度,单位是m/s2，正值向前负值向后。
        WR,                                   // 横摆角速度,单位是rad/s，正值：表示俯视图中，车辆围绕质心逆时针旋转。
        AY,                                   // 横向加速度,单位是m/s2，左正右负。
        W_LF_WHEEL_SPD,                       // 左前车轮旋转角速度,单位是rad/s，正值向前负值向后。
        W_RF_WHEEL_SPD,                       // 右前车轮旋转角速度,单位是rad/s，正值向前负值向后。
        W_LR_WHEEL_SPD,                       // 左后车轮旋转角速度,单位是rad/s，正值向前负值向后。
        W_RR_WHEEL_SPD,                       // 右后车轮旋转角速度,单位是rad/s，正值向前负值向后。
        WHEELCYLINDERPRESSURE_OUT,            // 制动压力,单位是Mpa。
        X_LF,                                 // 前左轮X坐标
        Y_LF,                                 // 前左轮Y坐标
        X_RF,                                 // 前右轮X坐标
        Y_RF,                                 // 前右轮Y坐标
        X_LR,                                 // 后左轮X坐标
        Y_LR,                                 // 后左轮Y坐标
        X_RR,                                 // 后右轮X坐标
        Y_RR,                                 // 后右轮Y坐标
        X_MC,                                 // 车辆x坐标,单位是m，正值：表示世界坐标系x正方向。
        Y_MC,                                 // 车辆y坐标,单位是m，正值：表示世界坐标系y正方向。
        VERTICAL_DISTANCE_VEHICLE,            // 车辆质心垂向位移
        PITCH_THETA_VEHICLE,                  // 车身俯仰角度
        ROLL_THETA_VEHICLE,                   // 车身侧倾角度
        VERTICAL_DISTANCE_LF,                 // 前左轮垂向位移
        VERTICAL_DISTANCE_RF,                 // 前右轮垂向位移
        VERTICAL_DISTANCE_LR,                 // 后左轮垂向位移
        VERTICAL_DISTANCE_RR,                 // 后右轮垂向位移
        STEERING_WHEEL_ANGLE_OUTPUT,          // 方向盘转角,单位是deg，左正右负。
        STEERING_WHEEL_TORQUE_OUTPUT,         // 方向盘实际扭矩,单位是N*m，左正右负。
        THROTTLE_OUTPUT,                      // 加速踏板开度，0-1。
        BRAKE_PEDAL_TRAVEL,                   // 制动踏板开度，0-1。
        VEHICLE_CURV,                         // 车辆曲率，单位是1/m，左转为正。
        STEERING_WHEEL_ANGULAR_RATE,          // 方向盘角速度,单位是rad/s，左正右负。
        DRV_TORQ_ACT,                         // 实际驱动扭矩,单位是N*m。
        YAW_THETA_VEHICLE,                    // 车辆航向角,单位是rad，正值：表示俯视图中，车头方向相对世界坐标系x轴逆时针旋转的角度。
        PITCH_RATE_VEHICLE,                   // 车辆俯仰角,单位是rad，正值：表示从左向右观察，车辆逆时针旋转运动。
        ROLL_RATE_VEHICLE,                    // 车辆侧倾角,单位是rad，正值：表示从前向后观察，车辆逆时针旋转运动。
        V_LF_WHEEL_SPD,                       // 左前轮轮速,单位是m/s，正值向前负值向后。
        V_RF_WHEEL_SPD,                       // 右前轮轮速,单位是m/s，正值向前负值向后。
        V_LR_WHEEL_SPD,                       // 左后轮轮速,单位是m/s，正值向前负值向后。
        V_RR_WHEEL_SPD,                       // 右后轮轮速,单位是m/s，正值向前负值向后。
        SIN_SLOPE_OUT,                        // sinSlope_out = sin（坡角），车头比车尾高为正。
        TMOTOR_CMD,                           // 电机的控制扭矩,单位是N*m。
        TMOTOR,                               // 电机实际输出扭矩,单位是N*m。
        ENERGY_SOURCE_LEVEL,                  // 当前油量/电量百分比，单位是%，0-100。
        W_GEAR_SPD,                           // 变速箱输出轴转速/电机输出轴转速 单位是rad/s。
        AZ,                                   // 车辆垂向加速度
        XRC_GLOBAL,                           // 车辆后轴中心X坐标（世界坐标系）
        YRC_GLOBAL,                           // 车辆后轴中心Y坐标（世界坐标系）
        VX_GLOBAL,                            // 车辆纵向速度（世界坐标系）
        VY_GLOBAL,                            // 车辆横向速度（世界坐标系）
        GEAR_SHIFT_OUTPUT,                    // 档位状态（-5：invalid；-4：no request；-3：target gear P；-2：target gear D；-1：target gear R；0：target gear N；1：target gear 1st；2：target gear 2st，......）
        LF_WHEEL_ROTATED_DIR,                 // 左前轮旋转方向（0=Standstill；1=Forward；2=Backward）
        RF_WHEEL_ROTATED_DIR,                 // 右前轮旋转方向（0=Standstill；1=Forward；2=Backward
        LR_WHEEL_ROTATED_DIR,                 // 左后轮旋转方向（0=Standstill；1=Forward；2=Backward）
        RR_WHEEL_ROTATED_DIR,                 // 右后轮旋转方向（0=Standstill；1=Forward；2=Backward）
        BRAKE_PEDAL_STATUS,                   // 制动踏板的状态（0=Not pressed；1=Pressed）
        VEHICLE_DIR,                          // 车辆行驶方向（0=standstill（静止）；1=forward；2=backward）
    };


// External inputs (root inport signals with default storage)
    typedef struct {
        double acc;                        // 预期纵向加速度输入
        double throttle;                   // 油门开度输入
        double Wheelcylinderpressure;      // 制动主缸压力输入
        double Veh_SteeringWheelAngle;     // 方向盘转角输入
        double frictionHmi;                // HMI摩擦系数
        double Veh_SteeringWheelTorque;    // 方向盘力矩输入
        double frictionOpenScenario;       // 场景文件摩擦系数比例因子
        double rainIntensity;              // 降雨量(0-1)
        double snowIntensity;              // 降雪量(0-1)
        double Mt_LF;                      // 轮端扭矩输入（前左轮）
        double Mt_RF;                      // 轮端扭矩输入（前右轮）
        double Mt_LR;                      // 轮端扭矩输入（后左轮）
        double Mt_RR;                      // 轮端扭矩输入（后右轮）
        double targetSpeed;                // 目标速度输入
        double stopDistance;               // 剩余停车距离输入
        signed char gearShift;             // 档位输入
        unsigned char BrkType;             // 制动类型输入
        unsigned char enableGlobalFriction;// 摩擦系数是否启动场景的全局配置(0:关闭,1:开启)
        std::string trajecoryPoint;        // 轨迹点坐标(x1,y1,z1,x2,y2,z2...),共20个轨迹点坐标,每个数值用逗号分隔
    } VehicleDynamicModelInput_t;

    //动力学输出参数
    typedef struct
    {
        double Vx;                         // 车速,单位是m/s，正值向前负值向后。
        double ax;                         // 纵向加速度,单位是m/s2，正值向前负值向后。
        double wr;                         // 横摆角速度,单位是rad/s，正值：表示俯视图中，车辆围绕质心逆时针旋转。
        double Ay;                         // 横向加速度,单位是m/s2，左正右负。
        double wLFWheelSpd;                // 左前车轮旋转角速度,单位是rad/s，正值向前负值向后。
        double wRFWheelSpd;                // 右前车轮旋转角速度,单位是rad/s，正值向前负值向后。
        double wLRWheelSpd;                // 左后车轮旋转角速度,单位是rad/s，正值向前负值向后。
        double wRRWheelSpd;                // 右后车轮旋转角速度,单位是rad/s，正值向前负值向后。
        double Wheelcylinderpressure_out;  // 制动压力,单位是Mpa。
        double xLF;                        // 前左轮X坐标
        double yLF;                        // 前左轮Y坐标
        double xRF;                        // 前右轮X坐标
        double yRF;                        // 前右轮Y坐标
        double xLR;                        // 后左轮X坐标
        double yLR;                        // 后左轮Y坐标
        double xRR;                        // 后右轮X坐标
        double yRR;                        // 后右轮Y坐标
        double Xmc;                        // 车辆x坐标,单位是m，正值：表示世界坐标系x正方向。
        double Ymc;                        // 车辆y坐标,单位是m，正值：表示世界坐标系y正方向。
        double VerticalDistance_Vehicle;   // 车辆质心垂向位移
        double PitchTheta_Vehicle;         // 车身俯仰角度
        double RollTheta_Vehicle;          // 车身侧倾角度
        double VerticalDistance_LF;        // 前左轮垂向位移
        double VerticalDistance_RF;        // 前右轮垂向位移
        double VerticalDistance_LR;        // 后左轮垂向位移
        double VerticalDistance_RR;        // 后右轮垂向位移
        double steeringWheelAngle_output;  // 方向盘转角,单位是deg，左正右负。
        double steeringWheelTorque_output; // 方向盘实际扭矩,单位是N*m，左正右负。
        double Gear_Shift;                 // 档位状态（-5：invalid；-4：no request；-3：target gear P；-2：target gear D；-1：target gear R；0：target gear N；1：target gear 1st；2：target gear 2st，......）
        double Throttle_output;            // 加速踏板开度，0-1。
        double BrakePedal_Travel;          // 制动踏板开度，0-1。
        double vehicleCurv;                // 车辆曲率，单位是1/m，左转为正。
        double steeringWheelAngularRate;   // 方向盘角速度,单位是rad/s，左正右负。
        double DrvTorqAct;                 // 实际驱动扭矩,单位是N*m。
        double YawTheta_Vehicle;           // 车辆航向角,单位是rad，正值：表示俯视图中，车头方向相对世界坐标系x轴逆时针旋转的角度。
        double PitchRate_Vehicle;          // 车辆俯仰角,单位是rad，正值：表示从左向右观察，车辆逆时针旋转运动。
        double RollRate_Vehicle;           // 车辆侧倾角,单位是rad，正值：表示从前向后观察，车辆逆时针旋转运动。
        double vLFWheelSpd;                // 左前轮轮速,单位是m/s，正值向前负值向后。
        double vRFWheelSpd;                // 右前轮轮速,单位是m/s，正值向前负值向后。
        double vLRWheelSpd;                // 左后轮轮速,单位是m/s，正值向前负值向后。
        double vRRWheelSpd;                // 右后轮轮速,单位是m/s，正值向前负值向后。
        double sinSlope_out;               // sinSlope_out = sin（坡角），车头比车尾高为正。
        double Tmotor_cmd;                 // 电机的控制扭矩,单位是N*m。
        double Tmotor;                     // 电机实际输出扭矩,单位是N*m。
        double EnergySourceLevel;          // 当前油量/电量百分比，单位是%，0-100。
        double wGearSpd;                   // 变速箱输出轴转速/电机输出轴转速 单位是rad/s。
        double Az;                         // 车辆垂向加速度
        double Xrc_Global;                 // 车辆后轴中心X坐标（世界坐标系）
        double Yrc_Global;                 // 车辆后轴中心Y坐标（世界坐标系）
        double Vx_Global;                  // 车辆纵向速度（世界坐标系）
        double Vy_Global;                  // 车辆横向速度（世界坐标系）
        unsigned char LFWheelRotatedDir;   // 左前轮旋转方向（0=Standstill；1=Forward；2=Backward）
        unsigned char RFWheelRotatedDir;   // 右前轮旋转方向（0=Standstill；1=Forward；2=Backward
        unsigned char LRWheelRotatedDir;   // 左后轮旋转方向（0=Standstill；1=Forward；2=Backward）
        unsigned char RRWheelRotatedDir;   // 右后轮旋转方向（0=Standstill；1=Forward；2=Backward）
        unsigned char BrakePedalStatus;    // 制动踏板的状态（0=Not pressed；1=Pressed）
        unsigned char VehicleDir;          // 车辆行驶方向（0=standstill（静止）；1=forward；2=backward）
        double AccelTgt;                   //预期加速度
        unsigned char brkType;             //制动类型(0=None 1=Comfort 2=Emergence)
    } VehicleDynamicModelOutput_t;

    //动力学初始化结构体参数
    typedef struct
    {
        double startSpeed; //主车初始化速度
        double initX0;
        double initY0;
        double initZ0;
        double initYaw;
        double initPitch;
        double initRoll;
    } VehicleDynamicModelInit_t;

    static VirtualCityFmiDynamicAdapter *Instance();
    static void Destroy();

    void init();//场景初始化时，获取用户选择第几套动力学模型，初始化对应的动力学模型	
    void finish();
    bool loadFmuModel(); //加载fmu模型
    static void cbLogMessage(fmi2ComponentEnvironment componentEnvironment, fmi2String instanceName, fmi2Status status, fmi2String category, fmi2String message, ...); //用于fmu使用的日志函数
    void useDynamicsModel(COSIMU_AD_DATA_MANAGER_t *cosimuData);

    /* zxz 2024.09.13 [FAW-DEV-474] [虚拟城市对接大疆的OSI算法] [ADD] [START] */
    void setInitParams(const VehicleDynamicModelInit_t &initParams); //设置动力学初始化结构体参数
    void setEnvParams(double rainIntensity, double snowIntensity);   //设置环境参数（降雨量、降雪量）
    /* zxz 2024.09.13 [FAW-DEV-474] [虚拟城市对接大疆的OSI算法] [ADD] [END] */

protected:
    VirtualCityFmiDynamicAdapter();
    ~VirtualCityFmiDynamicAdapter();

private:
    static VirtualCityFmiDynamicAdapter *_instance;
    bool isLoadDynamic; //是否加载过了动力学.so
    void* so_vehicledynamic; //动力学.so
    fmi2Component fmi2InstantiateObj; //fmi实例化对象
    fmi2CallbackFunctions callbackFunc; //用于Fmu使用的回调函数
    double simulationTime; //当前仿真时间
    double stepTime; //单帧的时间

    //动力学模型参数设置v1.0 wwh 2022.2.15
    int vehicleDynamicsSelect;//配置文件中动力学模型的选择

    //动力学 方向盘控制参数 gaoyibo 2022.06.16
    int vehicleSteeringSelect;

    // Xu 2021.9.26 保存方向盘转角信息
    double steeringWheel; // rad

    /*Aaron 2022.04.14 [ADD] OpenCRG对接 start*/
    //用于保存左前、右前、左后、右后轮胎以及质心坐标信息
    double m_xLF;
    double m_yLF;
    double m_xRF;
    double m_yRF;
    double m_xLR;
    double m_yLR;
    double m_xRR;
    double m_yRR;
    double m_Xmc;
    double m_Ymc;
    double m_EgoZ0;
    /*Aaron 2022.04.14 [ADD] OpenCRG对接 end*/

    /*Aaron 2022.07.01 [ADD] 解决车辆质心初始位置偏移问题 start */
    //用于保存初始质心坐标以及初始航向角信息
    double m_Xrc0;
    double m_Yrc0;
    double m_theta0;
	double m_elevation0;
    /*Aaron 2022.04.14 [ADD] 解决车辆质心初始位置偏移问题 end*/

    //动力学输出异常使用上一帧数据 wwh 2022.4.24 
    double accX; //纵向加速度
    double speedH; //横摆角速度


    /* Aaron 2022.4.22 车轮垂向震动 [start] */
    double verticalDistance_Vehicle = 0;            /**< 车身垂向位移            @unit m*/
    double pitchTheta_Vehicle = 0;			        /**< 车身俯仰角             @unit rad*/
    double rollTheta_Vehicle = 0;				    /**< 车身侧倾角             @unit rad*/
	double verticalDistance_LF = 0;					/**< 左前轮位垂向位移        @unit m*/
    double verticalDistance_RF = 0;					/**< 右前轮位垂向位移        @unit m*/
    double verticalDistance_LR = 0;					/**< 左后轮位垂向位移        @unit m*/
    double verticalDistance_RR = 0;					/**< 右后轮位垂向位移        @unit m*/
    /* Aaron 2022.4.22 车轮垂向震动 [end] */

    //动力学输出参数增加 wwh 23.1.11
    VehicleDynamicModelOutput_t dynOutput;//动力学输出数据结构体

    //道路支持摩擦系数 wwh 23.12.7
    bool isXoscSetPavement;//场景文件是否带有摩擦系数
    double pavementFromXosc;//保存场景文件中设置的摩擦系数系数
    double pavementFromHmi;//保存前端设置的摩擦系数

    /* wanghong 2024.8.27 [SimPro集成EchoSim] [ADD] [START] */
    std::string vehicleName;                //车辆name
    std::string licenseFilePath;            //license文件路径
    std::string vehicleParameterFilePath;   //车模参数文件路径
    std::string roadFilePath;               //默认地图文件路径
    std::string controlParameterFile;       //加速度控制参数文件路径
    double initYaw;                         //初始航向角(世界坐标系)
    double initPitch;                       //初始俯仰角(世界坐标系)
    double initRoll;                        //初始侧倾角(世界坐标系)
    double initX0;                          //初始位置车辆后轴中心位于地面的投影的初始x坐标
    double initY0;                          //初始位置车辆后轴中心位于地面的投影的初始y坐标
    double initZ0;                          //初始位置车辆后轴中心位于地面的投影的初始z坐标
    std::string trajectoryPoint;            //车辆预测轨迹点
    double fmiOutputDataX;                  //动力学输出x坐标
    double fmiOutputDataY;                  //动力学输出y坐标
    double initVelo;                        //主车初始速度
    /* wanghong 2024.8.27 [SimPro集成EchoSim] [ADD] [END] */

    /* wanghong 2024.7.16 [bug30791] [ADD] [START] */
    bool sndFmiDynamicErrorMsg;             //是否已向前端发送动力学异常信息
    /* wanghong 2024.7.16 [bug30791] [ADD] [END] */

    /* zxz 2024.09.13 [FAW-DEV-474] [虚拟城市对接大疆的OSI算法] [ADD] [START] */
    VehicleDynamicModelInit_t m_initParams;
    double m_rainIntensity;                 //降雨量(0-1)
    double m_snowIntensity;                 //降雪量(0-1)
    /* zxz 2024.09.13 [FAW-DEV-474] [虚拟城市对接大疆的OSI算法] [ADD] [END] */
};


#endif