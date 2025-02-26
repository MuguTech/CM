#ifndef Sensor_h
#define Sensor_h

#include "../../../include/Runtime/coSimu/SimProType.h"
#include "../../../APF/RoadSystem/Types.h"
#include "LidarDescription.h"
#include "CameraDescription.h"
#include "BasicSensorDescription.h"
#include "RadarDescription.h"
#include "UltrasonicDescription.h"
#endif

#include<cmath>
#include <set>
#include <vector>
#include <random>
#include <stack>

//pi(用于目标识别)
#define SENSOR_OBJECT_DETECTION_PI (3.141592653589793238463)
#define SENSOR_OBJECT_DETECTION_MIN_DISTANCE (5) // 最小距离内，目标识别不受天气影响
#define LIDAR_SNOWFACTOR (5.0) // Lidar雪因子
#define LIDAR_FOGFACTOR (4.0) // Lidar雾因子
#define LIDAR_RAINFACTOR (2.5) // Lidar雨因子
#define LIDAR_PHOTONFLUXES_UNIT (1.0e-20) // Lidar光子通量单位
#define LIDAR_THERMALNOISE (5.0e-23) // Lidar热噪声
#define LIDAR_EXTINCTIONCOEFFICIENT (0) // Lidar大气消光系数
#define RADAR_SNOWFACTOR (2.0) // Radar雪因子
#define RADAR_FOGFACTOR (0.0) // Radar雾因子
#define RADAR_RAINFACTOR (0.5) // Radar雨因子
#define RADAR_BOLTZMANNCONSTANT (1.38e-23) // Radar玻尔兹曼常数
#define CAMERA_SNOWFACTOR (2.5) // CAMERA雪因子
#define CAMERA_FOGFACTOR (4.0) // CAMERA雾因子
#define CAMERA_RAINFACTOR (3.5) // CAMERA雨因子
#define CAMERA_TRUNCATIONNOISE (2.0) // CAMERA截断噪声
#define CAMERA_PIXELNUM (2) // CAMERA截断噪声
#define RADAR_FALSE_ALARM_RANGE (100) // 毫米波雷达虚警范围

enum TRAFIC_SIGN_TYPE{
	TRAFFIC_SIGN_NONE,
	TRAFFIC_SIGN_TRAFFICLIGH
};

//  计算多面体目标的最近点信息
/* 从后方视角：模型右侧为1 左侧为2
侧视图：
		E1 _______ F1
		_C1/_______\D1_
	B1|_______________| G1
		A1            H1
*/
enum MODEL_OUTLINE{
	A1,	B1,	C1,	D1,	E1,	F1,	G1,	H1,
	A2,	B2,	C2,	D2,	E2,	F2,	G2,	H2,
	CNT
};
// 建立kd树结构
struct Node {
	Vector3D point;
	Node* left;
	Node* right;
	Node(const Vector3D& p) : point(p), left(nullptr), right(nullptr) {}
};

// 球坐标系  
struct SphericalCoordinates {
	double r; // 半径  
	double theta; // 极角或方位角 (从z轴到xy平面的夹角)  
	double phi; // 方位角或仰角 (从xy平面到点的夹角)  
};

class Sensor
{

public:
	Sensor();
	virtual ~Sensor();

	//取得指定类型（车辆、行人等）的物体ID列表
	bool getObjectList(int type, std::set<int> &IDs);

	// 筛选所有的遮挡物体ID
	bool getObjectInViewList();
	// 获取筛选所有的遮挡物体ID
	std::map<int, std::set<int>> getInViewIDsList();
	// 设置筛选所有的遮挡物体ID
	void setInViewIDsList(std::map<int, std::set<int>> setInViewIDs);

	virtual bool getBoundingBoxStatus()=0;

	virtual double getAssemblePositionX() = 0;
	virtual double getAssemblePositionY() = 0;
	virtual double getAssemblePositionZ() = 0;

	//取得探测范围
	virtual double getRange() = 0;

	//取得Heading Pitch Roll
	virtual double getHeading() = 0;
	virtual double getPitch() = 0;
	virtual double getRoll() = 0;

	//取得HFOV
	virtual double getHFOV() = 0;
	//取得VFOV
	virtual double getVFOV() = 0;

	//获取最近探测距离
	virtual double getMinimumDetectRange() = 0;

	virtual int getFrameRate() = 0;

	//判定是否是物体识别类型
	virtual bool isDetectionObject(int objectType);

	//置信度增加遮挡比例的影响
	virtual double getDetectedPR(double distance,float radarCrossSection, float weather, float occlusionScale) = 0;

	// 判定是否使能高级设置
	virtual bool isEnableAdvSettings() = 0;

	virtual double getRangeMSENoise()= 0;

	virtual double getAzimuthMSENoise()= 0;

	virtual double getElevationMSENoise()= 0;

	virtual double getRangeResolution()= 0;

	virtual double getAzimuthResolution()= 0;

	virtual double getElevationResolution()= 0;

	virtual double getEnvNoise(double distance)= 0;


	//取得识别物体列表（所有识别类型）
	bool getObjectList(std::vector<S_SP_SENSOR_DETECTION_INFO> &objects);

	//取得传感器状态
	virtual S_SP_SENSOR_INFO getState() = 0;

	//取得道路信息，车道信息
	virtual bool getLaneInfo(std::vector<S_SP_LANE_INFO> &laneInfos);

	//取得道路标志，车道线
	virtual bool getRoadMark(std::vector<S_SP_MIL_ROADMARK> &roadMarks);

	//取得交通标识信息 std::set<std::pair<TrafficSignType, TrafficSignID>>
	std::set<std::pair<std::string,int>> getTrafficSignInfo();
	std::set<std::pair<std::string,int>> trafficSign_infos;

	double calcWeatherNoise(int type, double distance);//计算天气影响因子

	std::vector<S_SP_MIL_ROADMARK> getRoadMarkInfo();  //获取道路信息
	void setRoadMarkInfo(std::vector<S_SP_MIL_ROADMARK> &roadMarks); 

	static double normalizeRad(double angle); //将弧度角值域调整到[0, 2PI)
	static double normalizeRad2(double angle); //将弧度角值域调整到[-PI, PI)

	int getSensorFrameRate(); //获取传感器频率
	bool isOutputData();

	static void VectorRotated(float yaw, float pitch, float raw, Vector3D &SensorCoor);//通过欧拉角旋转得到新坐标

	//坐标系转换
	static void CoordTransform(double egoYaw, double egoPitch, double egoRoll, Transform _egoTransform, Transform _agentTransform, double _sensorX, double _sensorY, double _sensorZ, double _sensorYaw, double _sensorPitch, double _sensorRoll, Vector3D &EgoCoord, Vector3D &SensorCoorTrans);

	//修改相对速度算法为位移对时间求导
	void CoordTransform(Transform _oriTransf, Transform _tgtTransform, Vector3D _oriAngles, Vector3D _senAngles, Vector3D _senMountPos, Vector3D &EgoCoord, Vector3D &SensorCoorTrans);

	static void CoordRotated(float _yaw, float _pitch, float _roll, Vector3D &vCoorTrans);

	void setSensorId(uint32_t _sensorId);

	// 输入GroundTruth，输出SensorDetectionInfoPkg
	void update(void *input, int inlen, void *output, int &outlen);

protected:

	// 高斯误差函数
	double getGaussRand(double sigma);

	uint32_t sensorId;
	S_SP_MIL_EGO_STATE *egoState; //主车数据
	std::list<S_SP_MIL_OBJECT_STATE *> vehicleList; //环境车数据
	std::list<S_SP_MIL_OBJECT_STATE *> pedestrianList; //行人数据
	std::list<S_SP_MIL_OBJECT_STATE *> obstacleList; //障碍物数据
	std::list<S_SP_TRAFFIC_LIGHT *> trafficLightList; //交通灯数据
	std::list<S_SP_TRAFFIC_SIGN *> trafficSignList; //交通标志数据
	S_SP_ENVIRONMENT *environment; //环境数据
	std::map<std::string, std::vector<Vector3D>> m_modelOutline;  //保存车模名称与轮廓数据的对应关系 <车模名称, 轮廓数据>

private:
	// 每一帧需要做的数据重置工作
	void resetData();

	// 解析输入的数据包 返回：是否成功
	bool parseMsg(char *msgBuff, unsigned int msgSize);

	// 打包SensorDetectionInfoPkg 返回：是否成功
	bool generateSensorDetectionInfoPkg(const std::vector<S_SP_SENSOR_DETECTION_INFO> &detectionInfoVec, char *outputBuffer, unsigned int bufferSize, unsigned int &pkgSize);

	// 计算行人包围盒8个顶点
	void computePedestrianBBox(S_SP_MIL_OBJECT_STATE *object, std::vector<Vector3D> &bBoxVec);

	// 计算车辆、障碍物包围盒8个顶点
	void computeVehicleObstacleBBox(S_SP_MIL_OBJECT_STATE *object, std::vector<Vector3D> &bBoxVec);

	//根据车辆转向灯，获取下一条路和在下一条路上的行驶方向
	void getNextRoadByLightMask(Road *&nextRoad, int &nextRoadDir);

	// 判断物体是否在圆内
	static bool InCircleandInSector(S_SP_MIL_EGO_STATE *egoState, Transform _egoTransform,  const std::vector<Vector3D> &_pointVector, float hfov, float vfov, double _sensorX, double _sensorY, double _sensorZ, float _sensorRange, float _sensorMinRange, double _sensorYaw, double _sensorPitch, double _sensorRoll);
	static bool InCircle(S_SP_MIL_EGO_STATE *egoState, Transform _egoTransform, Vector3D _agentTransform, double _sensorX, double _sensorY, double _sensorZ, float _sensorRange, float _sensorMinRange);
	static bool IsIntersection(S_SP_MIL_EGO_STATE *egoState, Transform _egoTransform, const std::vector<Vector3D> &_pointVector, double _sensorX, double _sensorY, double _sensorZ, double _sensorYaw, double _sensorPitch, double _sensorRoll, float _sensorRange, float _sensorMinRange);

	// 判断物体是否在扇形内
	static bool InSector(float hfov, float vfov, double x2, double y2, double z2);

	double CalcNearestDist(double _yaw, double _pitch, double _roll, Vector3D _relativeSensorCoor,
							double _dimensionX, double _dimensionY,double _dimensionZ,
							double _centerX, double _centerY, double _centerZ, Vector3D &vrelativeCoor);

	void CalcNearestEgoCoordSys(double _yaw, double _pitch, double _roll, double _sensorX, double _sensorY, double _sensorZ, Vector3D _relativeObjCoor, Vector3D &vCoorTrans);
	void CalcNearestSensorCoordSys(double _sensorYaw, double _sensorPitch, double _sensorRoll, double _sensorX, double _sensorY, double _sensorZ, Vector3D _egoCoorTrans, Vector3D &vSensorCoorTrans);

	// 计算传感器到目标物最近点距离
	double getObjNearestCoordInfo(S_SP_MIL_OBJECT_STATE *objectState, double _sensorX, double _sensorY, double _sensorZ, double _sensorYaw, double _sensorPitch, double _sensorRoll, Vector3D &vEgoCoorTrans, Vector3D &vSensorCoorTrans);

	// 填充多面体模型特征点分为上下两个部分
	bool fillTwoPartOutlineMap(const std::string modelEntryName, std::map<int, std::tuple<Vector3D,Vector3D,Vector3D,Vector3D>> &m_upperOutline, std::map<int, std::tuple<Vector3D,Vector3D,Vector3D,Vector3D>> &m_bottomOutline, const Vector3D &paramScale);
	// 当前物体最近点是否被遮挡
	bool hasOcclusion(int id, float f_targetHorRad, float f_targetVerRad, float f_occlusionR, float f_occlusionL, float f_occlusionB, float f_occlusionU);
	//计算遮挡后的最近点
	void calcCoordWithOcclusion(int id, Vector3D nearestVertex, Vector3D normalVec, float occlusionR, float occlusionL, Vector3D v_modelOutline, Vector3D &v_modelR, Vector3D &v_modelL);
	// 当最近点在棱上和面上时,出现异常情况
	bool isExceptionCase(Vector3D AP, Vector3D OA, Vector3D vec_AC, Vector3D vec_AD, Vector3D &newCoord);
	// 解决最近点在棱和面判断错误问题 
	bool isEdgeCase(Vector3D OA, Vector3D vec_edge1, Vector3D vec_edge2, Vector3D vec_normal, Vector3D &newCoord);

	// 计算不同情况下的最近点坐标
	void calcNearestCoord(int id, Vector3D vec_AB, Vector3D vec_AC, Vector3D vec_AD, Vector3D v_modelOutline, Vector3D &v_modelOutlineR, Vector3D &v_modelOutlineL);// 计算多面体最近点信息
	// 计算最近点
	void calcNearestInfo(int id, std::map<int, std::tuple<Vector3D,Vector3D,Vector3D,Vector3D>> m_outline, Vector3D sensorCoord, double yaw, double pitch, double roll, Vector3D& v_modelOutlineR, Vector3D& v_modelOutlineL);


#ifdef ENABLE_YIQILOG //传感器最近点方案优化 pjd 2023.09.15
	Vector3D GetSegment(double Occlusion_angle, Vector3D point1, Vector3D point2);
	double distanceToSegment( Vector3D sensor_point, Vector3D point1, Vector3D point2,Vector3D &point);
	bool Line_intersection(std::vector<Vector3D> _vehBBoxVertexVectors,std::vector<Vector3D> &intersectionPoints);
	void GetNearest_Point(int id ,std::vector<Vector3D> &_pointVector, Vector3D &_nearest_point,double &mindistance);
	void GetNearest_boolPoint(double Occlusion_angle,std::vector<Vector3D> &_pointVector, Vector3D &_nearest_point);
	Vector3D getOcclusion_point(int id,  std::vector<Vector3D> &_pointVector,Vector3D &v_modelOutline);
	void calcCoord(int id, std::vector<Vector3D> &_pointVector, float occlusionR, float occlusionL, Vector3D &v_modelR, Vector3D &v_modelL);
#endif

	// enable物体遮挡
	bool bEnableMaskFunction;

	// 判断物体是否被遮挡
	bool IsMaskedView(int count, int id, float theta);
	// 取得车辆筛选后的ID
	bool getVehicleInViewList(std::set<int> &IDs);
	// 取得行人筛选后的ID
	bool getPedestrianInViewList(std::set<int> &IDs);
	// 取得障碍物筛选后的ID
	bool getRoadObjectInViewList(std::set<int> &IDs);

	// 获取范围内物体质点的相对坐标
	std::vector<tuple<int, int, float,float,float>> getSortedAgentCoord(std::vector<tuple<int,int,float,float,float>> &unsortedVec);
	// 获取目标物体的最左边弧度和最右边弧度
	std::map<int, std::tuple<float, float, float, float>> getAgentFourPointMap();

	std::map<int, std::tuple<float, float, float, float>> agentFourPointRadMap; // 获取目标物体的最左边弧度和最右边弧度

	std::map<int, std::set<int>> InViewIDs;	// 筛选所有的遮挡物体ID

	std::vector<std::tuple<float, float, float, float>> occupiedRadVec; // 被遮挡的范围

	// [一汽需求] 遮挡增加Z维度 23.06.21 LS <id,<occlusionScale, horOcclusionScale, verOcclusionScale, occupiedRightRad, occupiedLeftRad, occupiedBottomRad, occupiedUpperRad>>
	std::map<int, std::tuple<float, float, float, float, float, float, float>> objOcclusionScale;

	// 车道线拟合调用优化 LS 23.03.28
	std::vector<S_SP_MIL_ROADMARK> roadMarkInfo;

	// objID, objtype, 主车坐标系x, y, z
	std::vector<tuple<int, int, Vector3D>> sortedObjs;
	std::vector<tuple<int, int, Vector3D>> ghostInfo; // 无效目标

	// 极坐标系转笛卡尔坐标系
	void getCartesianCoord(double _range, double _azimuth, double _elevation, Vector3D &vCoordSys);
	// 笛卡尔坐标系转球坐标系
	void getSphereCoord(double _x, double _y, double _z, Vector3D &vCoordSys);

	// 建立kd树
	Node* build_kdtree(std::vector<tuple<int, int, Vector3D>>& points, int start, int end, int depth);
	// 找到每个点的最近邻点
	void kdtree_nearest_neighbor(Node* root, const Vector3D& target, Vector3D& nearest, double& min_dist);
	// 保存目标id和欧氏距离最短目标的相对距离，相对水平角，相对俯仰角
	void getNearestObjRelCoordToTarget(float insensitiveRadius, std::map<int, Vector3D> &nearestObjRelCoord);

	// 目标分类正确的可能性
	double getClassificationAccuracy(int targetId, std::map<int, Vector3D>& nearestObjRelCoord);
	// 获取范围内物体质点的相对坐标并按照与x轴夹角远近排序
	// void getSortedArc(std::vector<tuple<int,int,float,float,float>> &unsortedVec);

	// 有效性标识 (传感器坐标系下的目标位置，目标的相对传感器姿态角)
	void getValidityLabel(int curID, Vector3D pos, Vector3D ori);
	// 是否是有效物体
	bool isValidObject(Vector3D pos, Vector3D ori);
	// 计算无效目标的位置信息等
	bool getInvalidObjectInfo(std::vector<tuple<int, int, Vector3D>>& ghostInfo);


	//取得（车辆）的ID列表
	bool getVehicleList( std::set<int> &IDs);

	//取得（行人）的ID列表
	bool getPedestrianList(std::set<int> &IDs);

	//取得（障碍物）的ID列表
	bool getRoadObjectList(std::set<int> &IDs);

	//取得（交通灯）的ID列表
	bool getTrafficLightList(std::set<int> &IDs);

	//取得（TrafficSign）的ID列表
	bool getTrafficSignList(std::set<int> &IDs);
	std::vector<double> getRoadMarkCoeff(int laneID);	// 获取左右车道线的三次拟合系数
	std::vector<Vector3D> getXYNearestPois(S_SP_MIL_OBJECT_STATE *object, Vector3D gaussRandErr, Vector3D &sensorCoord);	// 获取X/Y轴左右侧最近点 采用车辆模型轮廓（16点，不包含后视镜）
	std::vector<Vector3D> getXYNearestPois(S_SP_MIL_OBJECT_STATE *object, Vector3D gaussRandErr);	// 获取X/Y轴左右侧最近点（8顶点，包含后视镜）
	S_SP_COORDINATE Vec3dToSPCOORD(Vector3D _Vec3d);					// 将Vector3D转换为S_SP_COORDINATE
	std::vector<Vector3D> getBoxCrossXAxis(std::vector<Vector3D> _BBoxPois);	// 获取目标包围盒和X轴的交点
	std::vector<Vector3D> getBoxCrossYAxis(std::vector<Vector3D> _BBoxPois);	// 获取目标包围盒和Y轴的交点

	//两点拟合直线
	std::vector<double> fitStraightLine(std::vector<double> &_xVec, std::vector<double> &_yVec);

	// 计算目标相对于主车的XY方向速度
	Vector2D getRelaVelToEgo(S_SP_MIL_OBJECT_STATE *objectState);

	// 计算目标相对于主车的XY方向加速度
	Vector2D getRelaAccToEgo(S_SP_MIL_OBJECT_STATE *objectState);

	Vector2D calRelVecByAngleDiff(Vector2D _ori, Vector2D _tgt, double _angDiff);

    // 将单个Vector3D从笛卡尔坐标转换为球坐标  
    SphericalCoordinates CartesianToSpherical(const Vector3D& cart);
    // 判断该点是否在传感器探测范围内
    bool isPoisInSensorView(const Vector3D& cart);

	// 获取车道线点世界坐标
	int getLaneLinePois(double _intv, int _num, Road* _road, int _laneId, double _startu, int _dir, std::vector<Vector3D> &_lPois, std::vector<Vector3D> &_rPois);
	std::vector<double> fitLaneLine(std::vector<Vector3D> &_pois);	// 拟合车道线的三次曲线方程
	virtual Vector3D calDistCoord(Vector3D _poi) = 0;	// 计算三维的坐标点在传感器处理后发生了畸变后的坐标
	int getLaneDir(Road *inRoad, int inLane, int inDir, Road *outRoad, int outLane);
protected:
	//共通传感器配置
	SensorDescription _description;
	Vector2D getRelaValToSensor(const double _dValX, const double _dValY);

};

class Lidar : public Sensor
{
public:
	Lidar(LidarDescription description);

	//取得传感器状态
	virtual S_SP_SENSOR_INFO getState();

	virtual bool getBoundingBoxStatus(); //获取BoundingBox 开关是否打开

	virtual double getAssemblePositionX();
	virtual double getAssemblePositionY();
	virtual double getAssemblePositionZ();

	//取得探测范围
	virtual double getRange();

	//取得Heading Pitch Roll
	virtual double getHeading();
	virtual double getPitch();
	virtual double getRoll();
	//取得HFOV
	virtual double getHFOV();
	//取得VFOV
	virtual double getVFOV();

	//获取最近探测距离
	virtual double getMinimumDetectRange();

	//置信度增加遮挡比例的影响
	virtual double getDetectedPR(double distance,float radarCrossSection, float weather, float occlusionScale);

	virtual int getFrameRate(); //获取传感器频率

	// 判定是否使能高级设置
	virtual bool isEnableAdvSettings();

	//判定是否是物体识别类型
	virtual bool isDetectionObject(int objectType);

	virtual double getRangeMSENoise();

	virtual double getAzimuthMSENoise();

	virtual double getElevationMSENoise();

	virtual double getRangeResolution();

	virtual double getAzimuthResolution();

	virtual double getElevationResolution();

	virtual double getEnvNoise(double distance);

	virtual Vector3D calDistCoord(Vector3D _poi) {return _poi;};	// 计算三维的坐标点在传感器处理后发生了畸变后的坐标

private:
	LidarDescription _description;
};

class Camera : public Sensor
{
public:
	Camera(CameraDescription description);

	//取得传感器状态
	virtual S_SP_SENSOR_INFO getState();

	virtual bool getBoundingBoxStatus();

	virtual double getAssemblePositionX();
	virtual double getAssemblePositionY();
	virtual double getAssemblePositionZ();

	//取得探测范围
	virtual double getRange();

	//取得Heading Pitch Roll
	virtual double getHeading();
	virtual double getPitch();
	virtual double getRoll();

	//取得HFOV
	virtual double getHFOV();
	//取得VFOV
	virtual double getVFOV();

	//获取最近探测距离
	virtual double getMinimumDetectRange();

	// 计算上不gamma函数
	float getIncomGamma(int n, float weather);

	// 计算gamma函数
	float getGamma(int n);

	//置信度增加遮挡比例的影响
	virtual double getDetectedPR(double distance,float radarCrossSection, float weather, float occlusionScale);

	virtual int getFrameRate(); //获取传感器频率

	// 判定是否使能高级设置
	virtual bool isEnableAdvSettings();

	//判定是否是物体识别类型
	virtual bool isDetectionObject(int objectType);

	virtual double getRangeMSENoise();

	virtual double getAzimuthMSENoise();

	virtual double getElevationMSENoise();

	virtual double getRangeResolution();

	virtual double getAzimuthResolution();

	virtual double getElevationResolution();

	virtual double getEnvNoise(double distance);

	Vector3D con3dToCamPlane(Vector3D _poi, double _fd);		// 将三维坐标转换到像平面
	Vector3D conCamPlaneTo3d(Vector3D _poi, double _fd);		// 将像平面上的坐标转换到三维坐标
	void getCamDistCoord(std::vector<Vector3D> &_pois, double _fd, double _k1, double _k2, double _gaussSigma);		// 计算相机畸变/噪音后的点坐标
	virtual Vector3D calDistCoord(Vector3D _poi);	// 计算三维的坐标点在传感器处理后发生了畸变后的坐标
	Vector3D getCamDistCoord2D(Vector3D _poi, double _distParK1, double _distParK2, double distCenterL, double distCenterV);

private:
	CameraDescription _description;
};

class IdealizedSensor : public Sensor
{
public:
	IdealizedSensor(BasicSensorDescription description);

	//取得传感器状态
	virtual S_SP_SENSOR_INFO getState();

	virtual bool getBoundingBoxStatus();

	virtual double getAssemblePositionX();
	virtual double getAssemblePositionY();
	virtual double getAssemblePositionZ();

	//取得探测范围
	virtual double getRange();

	//取得Heading Pitch Roll
	virtual double getHeading();
	virtual double getPitch();
	virtual double getRoll();
	//取得HFOV
	virtual double getHFOV();
	//取得VFOV
	virtual double getVFOV();

	//获取最近探测距离
	virtual double getMinimumDetectRange();


	//判定是否是物体识别类型
	virtual bool isDetectionObject(int objectType);

	//置信度增加遮挡比例的影响
	virtual double getDetectedPR(double distance,float radarCrossSection, float weather, float occlusionScale);

	//获取传感器频率
	virtual int getFrameRate(); 

	// 判定是否使能高级设置
	virtual bool isEnableAdvSettings();

	virtual double getRangeMSENoise();

	virtual double getAzimuthMSENoise();

	virtual double getElevationMSENoise();

	virtual double getRangeResolution();

	virtual double getAzimuthResolution();

	virtual double getElevationResolution();

	virtual double getEnvNoise(double distance);

	virtual Vector3D calDistCoord(Vector3D _poi) {return _poi;};	// 计算三维的坐标点在传感器处理后发生了畸变后的坐标

private:
	BasicSensorDescription _description;
};

class Radar : public Sensor
{
public:
	Radar(RadarDescription description);

	//取得传感器状态
	virtual S_SP_SENSOR_INFO getState();

	virtual bool getBoundingBoxStatus();

	virtual double getAssemblePositionX();
	virtual double getAssemblePositionY();
	virtual double getAssemblePositionZ();

	//取得探测范围
	virtual double getRange();

	//取得Heading Pitch Roll
	virtual double getHeading();
	virtual double getPitch();
	virtual double getRoll();

	//取得HFOV
	virtual double getHFOV();
	//取得VFOV
	virtual double getVFOV();

	//获取最近探测距离
	virtual double getMinimumDetectRange();

	//置信度增加遮挡比例的影响
	virtual double getDetectedPR(double distance,float radarCrossSection, float weather, float occlusionScale);

	//获取传感器频率
	virtual int getFrameRate();

	// 判定是否使能高级设置
	virtual bool isEnableAdvSettings();

	//判定是否是物体识别类型
	virtual bool isDetectionObject(int objectType);

	virtual double getRangeMSENoise();

	virtual double getAzimuthMSENoise();

	virtual double getElevationMSENoise();

	virtual double getRangeResolution();

	virtual double getAzimuthResolution();

	virtual double getElevationResolution();

	virtual double getEnvNoise(double distance);

	virtual Vector3D calDistCoord(Vector3D _poi) {return _poi;};	// 计算三维的坐标点在传感器处理后发生了畸变后的坐标

private:
	RadarDescription _description;
};

class Ultrasonic : public Sensor
{
public:
	Ultrasonic(UltrasonicDescription description);

	//取得传感器状态
	virtual S_SP_SENSOR_INFO getState();

	virtual bool getBoundingBoxStatus();

	virtual double getAssemblePositionX();
	virtual double getAssemblePositionY();
	virtual double getAssemblePositionZ();

	//取得探测范围
	virtual double getRange();

	//取得Heading Pitch Roll
	virtual double getHeading();
	virtual double getPitch();
	virtual double getRoll();

	//取得HFOV
	virtual double getHFOV();
	//取得VFOV
	virtual double getVFOV();

	//获取最近探测距离
	virtual double getMinimumDetectRange();

	virtual int getFrameRate();

	virtual double getDetectedPR(double distance,float radarCrossSection, float weather, float occlusionScale);

	// 判定是否使能高级设置
	virtual bool isEnableAdvSettings();

	//判定是否是物体识别类型
	virtual bool isDetectionObject(int objectType);

	virtual double getRangeMSENoise();

	virtual double getAzimuthMSENoise();

	virtual double getElevationMSENoise();

	virtual double getRangeResolution();

	virtual double getAzimuthResolution();

	virtual double getElevationResolution();

	virtual double getEnvNoise(double distance);

	virtual Vector3D calDistCoord(Vector3D _poi) {return _poi;};	// 计算三维的坐标点在传感器处理后发生了畸变后的坐标

private:
	UltrasonicDescription _description;
};
