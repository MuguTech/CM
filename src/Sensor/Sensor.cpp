#include "Sensor.h"
#include <cfloat>
#include "../log.h"
#include "../../../Runtime/TrafficSimulation/TrafficSimulation.h"
#include "../../../APF/RoadSystem/Lane.h"
#include "../../../APF/RoadSystem/Road.h"
#include "../../../APF/RoadSystem/Junction.h"
#include "../../../APF/RoadSystem/RoadSystem.h"

#include "../../../Runtime/coSimu/PolynomialRegression.h"

#define relu0(a, b) (( ((a - b) < 0) ? 0 : (a - b) )) // 取两者之差大于等于0
#define relu1(a, b) (( ((a - b) < 1) ? (a - b) : 1 )) // 取两者之差小于等于1
#define minPct(a, b)	(((a) > (b)) ? (b) : (a)) //取两者中最小值
#define PI 3.1415926535897932384626

Sensor::Sensor()
{
    // enable物体遮挡
    bEnableMaskFunction = true;
    sensorId = 0;

    // 数据重置
    resetData();
}

Sensor::~Sensor()
{
    occupiedRadVec.clear();
    objOcclusionScale.clear();
}

//获取范围内物体质点的相对坐标
bool Sensor::getObjectInViewList()
{
    log_compnt_mngr->info("Sensor::getObjectInViewList start.");
    struct timespec timeStamp{0, 0};
    (void )clock_gettime(CLOCK_REALTIME, &timeStamp);
    long int sysTimeStartMs = static_cast<long int>(timeStamp.tv_sec * 1000000000) + timeStamp.tv_nsec;

    if (egoState == nullptr)
    {
        log_compnt_mngr->error( "egoState=nullptr.");
        return false;
    }
    Transform ego_transform; //获取Ego车世界坐标
    ego_transform.update(egoState->sObjectState.sPos.u8X, egoState->sObjectState.sPos.u8Y, egoState->sObjectState.sPos.u8Z);

    std::vector<tuple<int, int, float, float, float>> agentCoordVec; // 存储所有目标物体的质点
    /********************************** 1. Prepare vehicle data *************************************/
    //遍历所有车辆
    for (auto vehIt = vehicleList.begin(); vehIt != vehicleList.end(); ++vehIt)
    {
        S_SP_MIL_OBJECT_STATE *objectState = *vehIt;
        if (objectState == nullptr)
        {
            log_compnt_mngr->warn( "vehicle objectState is nullptr, continue.");
            continue;
        }
        // 目标物ID
        uint32_t objectId = objectState->sObjectState.u4Id;

        Transform agent_transform; //获取世界坐标
        agent_transform.update(objectState->sObjectState.sPos.u8X, objectState->sObjectState.sPos.u8Y, objectState->sObjectState.sPos.u8Z);

        log_compnt_mngr->debug("getObjectInViewList,Name= (ID: type:) {},{},{}",objectState->sObjectState.au1Name, objectId, SENSOR_OBJECT_DETECTION_TYPE_VEHICLE);
        std::vector<Vector3D> vehBBoxVertexVector; //记录环境车包围盒八个顶点信息
        computeVehicleObstacleBBox(objectState, vehBBoxVertexVector); //获取环境车包围盒八个顶点信息
        // 世界坐标系转传感器坐标系
        Vector3D relativeCoord(0, 0, 0);
        Vector3D sensorRela(0, 0, 0); //记录物体位于传感器坐标系下点
        CoordTransform(egoState->sObjectState.sPos.u4H, egoState->sObjectState.sPos.u4P, egoState->sObjectState.sPos.u4R, ego_transform, agent_transform, getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(), getHeading(), getPitch(), getRoll(), relativeCoord, sensorRela);

        log_compnt_mngr->trace("relativeCoord[0], relativeCoord[1] relativeCoord[2] {},{},{}", relativeCoord[0],relativeCoord[1],relativeCoord[2]);
        log_compnt_mngr->trace("sensorRela[0], sensorRela[1] sensorRela[2] {},{},{}", sensorRela[0],sensorRela[1],sensorRela[2]);
        //是否在识别范围和FOV内
        if (!InCircleandInSector(egoState, ego_transform, vehBBoxVertexVector, getHFOV(), getVFOV(), getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(),
                                getRange(), getMinimumDetectRange(), getHeading(), getPitch(), getRoll()))
        {
            if (!IsIntersection(egoState, ego_transform, vehBBoxVertexVector, getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(),getHeading(), getPitch(), getRoll(),getRange(), getMinimumDetectRange()))
            {
                log_compnt_mngr->warn("vehicle is not InCircleandInSector and IsIntersection, Name = {} ,id = {},continue.",objectState->sObjectState.au1Name,objectId);
                continue;
            }
        }

        float agentX = sensorRela.x();
        float agentY = sensorRela.y();
        float agentZ = sensorRela.z();
        // 存放所有的目标物体
        agentCoordVec.push_back(std::make_tuple(static_cast<int>(objectId), SENSOR_OBJECT_DETECTION_TYPE_VEHICLE, agentX, agentY,agentZ));

        log_compnt_mngr->debug("agentCoordVec id {} (agentX: agentY:) {},{}",objectId, agentX, agentY);

        // 获取目标物体的最大弧度和最小弧度
        float thetaMin = DBL_MAX;      //目标物物体在传感器坐标系下相对于传感器的最小水平弧度
        float thetaMax = -DBL_MAX;     //目标物物体在传感器坐标系下相对于传感器的最大水平弧度
        float thetaTmp = 0.0;          //目标物物体的某个顶点在传感器坐标系下相对于传感器的水平角度
        float thetaMinZDim = DBL_MAX;  //目标物物体在传感器坐标系下相对于传感器的最小垂直角度
        float thetaMaxZDim = -DBL_MAX; //目标物物体在传感器坐标系下相对于传感器的最大垂直角度

        for (unsigned int i = 0; i < vehBBoxVertexVector.size() ; i++) //遍历被探测车辆的8个顶点
        {
            Vector3D point = vehBBoxVertexVector[i];

            //计算相对坐标，即转换为主车坐标系，x轴向前，y轴向左
            Vector3D relativeFourPointCoord(0, 0, 0); //记录目标物在主车坐标系下位置
            //世界坐标系转主车坐标系
            ego_transform.relative3dCoor(egoState->sObjectState.sPos.u4H, egoState->sObjectState.sPos.u4P, egoState->sObjectState.sPos.u4R, point, relativeFourPointCoord);

            //主车坐标系转换-转传感器坐标系
            Vector3D sensorRelaFourPoint(relativeFourPointCoord[0] - getAssemblePositionX(), relativeFourPointCoord[1] - getAssemblePositionY(), relativeFourPointCoord[2] - getAssemblePositionZ());
            CoordRotated(getHeading(), getPitch(), getRoll(), sensorRelaFourPoint);
            float fourPointX = sensorRelaFourPoint.x();
            float fourPointY = sensorRelaFourPoint.y();
            float fourPointZ = sensorRelaFourPoint.z();
            log_compnt_mngr->trace("fourPointX:{} fourPointY:{} fourPointZ:{} ", fourPointX, fourPointY, fourPointZ);

            thetaTmp = std::atan2(fourPointY, fourPointX); //当前被探测车辆的某个顶点在传感器坐标系下相对于传感器的水平角度
            // 确定最大弧度和最小弧度
            if (thetaTmp > thetaMax)
            {
                thetaMax = thetaTmp; //被探测车辆的 8 个顶点在传感器坐标系下相对于传感器的最大水平角度
            }

            if (thetaTmp < thetaMin)
            {
                thetaMin = thetaTmp;  //被探测车辆的 8 个顶点在传感器坐标系下相对于传感器的最小水平角度
            }

            float range = std::sqrt(std::pow(fourPointX,2) + std::pow(fourPointY,2)); //当前被探测车辆的某个顶点在传感器坐标系下在水平面上（xy 平面）到传感器的距离。
            float thetaZDim = std::atan(fourPointZ/range);  //前被探测车辆的某个顶点在传感器坐标系下相对于传感器的垂直角度
            if (thetaZDim > thetaMaxZDim)
            {
                thetaMaxZDim = thetaZDim;  //被探测车辆的 8 个顶点在传感器坐标系下相对于传感器的最大垂直角度
            }
            if (thetaZDim < thetaMinZDim)
            {
                thetaMinZDim = thetaZDim;  //被探测车辆的 8 个顶点在传感器坐标系下相对于传感器的最小垂直角度
            }
        }

        agentFourPointRadMap[static_cast<int>(objectId)] = std::make_tuple(thetaMin, thetaMax, thetaMinZDim, thetaMaxZDim); // 获取目标水平和垂直方向最大夹角
        log_compnt_mngr->debug("agentFourPointRadMap,(ID:{})(thetaMin:{} thetaMax:{})(thetaMinZDim: {}, thetaMaxZDim: {})", objectId, thetaMin, thetaMax, thetaMinZDim, thetaMaxZDim);
    }

    /********************************** 2. Prepare pedstrian data *************************************/
    //遍历所有行人
    for (auto pedIt = pedestrianList.begin(); pedIt != pedestrianList.end(); ++pedIt)
    {
        S_SP_MIL_OBJECT_STATE *objectState = *pedIt;
        if (objectState == nullptr)
        {
            log_compnt_mngr->warn( "pedestrian objectState is nullptr ,continue.");
            continue;
        }

        // 目标物ID
        uint32_t objectId = objectState->sObjectState.u4Id;

        Transform pedestrianTransform;
        pedestrianTransform.update(objectState->sObjectState.sPos.u8X, objectState->sObjectState.sPos.u8Y, objectState->sObjectState.sPos.u8Z);

        log_compnt_mngr->debug("getObjectInViewList,Name= (ID: type:) {},{},{}",objectState->sObjectState.au1Name, objectId, SENSOR_OBJECT_DETECTION_TYPE_PEDESTRIAN);
        std::vector<Vector3D> pedBBoxVertexVector; //获取行人包围盒8个顶点信息
        computePedestrianBBox(objectState, pedBBoxVertexVector);
        // 世界坐标系转传感器坐标系
        Vector3D relativeCoord(0, 0, 0);
        Vector3D sensorRela(0, 0, 0); //传感器坐标系下点
        CoordTransform(egoState->sObjectState.sPos.u4H, egoState->sObjectState.sPos.u4P, egoState->sObjectState.sPos.u4R, ego_transform, pedestrianTransform, getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(), getHeading(), getPitch(), getRoll(), relativeCoord, sensorRela);

        log_compnt_mngr->trace("relativeCoord[0] , relativeCoord[1]  relativeCoord[2] {},{},{}", relativeCoord[0],relativeCoord[1],relativeCoord[2]);
        log_compnt_mngr->trace("sensorRela[0] , sensorRela[1]  sensorRela[2] {},{},{}", sensorRela[0],sensorRela[1],sensorRela[2]);
        //行人是否在识别范围和FOV内
        if (!InCircleandInSector(egoState, ego_transform, pedBBoxVertexVector, getHFOV(), getVFOV(), getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(),
                                getRange(), getMinimumDetectRange(), getHeading(), getPitch(), getRoll()))
        {
            log_compnt_mngr->warn("pedestrian is not InCircleandInSector, Name = {} ,id = {},continue.",objectState->sObjectState.au1Name,objectId);
            continue;
        }

        //计算相对坐标，即转换为传感器坐标系，x轴向前，y轴向左
        float agentX = sensorRela.x();
        float agentY = sensorRela.y();
        float agentZ = sensorRela.z();
        // 存放所有的目标物体
        agentCoordVec.push_back(std::make_tuple(static_cast<int>(objectId), SENSOR_OBJECT_DETECTION_TYPE_PEDESTRIAN, agentX, agentY, agentZ));
        log_compnt_mngr->debug("agentCoordVec,id {}:(agentX: agentY:) {},{}",objectId, agentX, agentY);

        // 获取目标物体的最大弧度和最小弧度
        float thetaMin = DBL_MAX;      //目标物物体在传感器坐标系下相对于传感器的最小水平弧度
        float thetaMax = -DBL_MAX;     //目标物物体在传感器坐标系下相对于传感器的最大水平弧度
        float thetaTmp = 0.0;          //目标物物体的某个顶点在传感器坐标系下相对于传感器的水平角度
        float thetaMinZDim = DBL_MAX;  //目标物物体在传感器坐标系下相对于传感器的最小垂直角度
        float thetaMaxZDim = -DBL_MAX; //目标物物体在传感器坐标系下相对于传感器的最大垂直角度

        for (unsigned int i = 0; i < pedBBoxVertexVector.size() ; i++) //遍历被探测行人的八个顶点
        {
            Vector3D point = pedBBoxVertexVector[i];

            //计算相对坐标，即转换为主车坐标系，x轴向前，y轴向左
            Vector3D relativeFourPointCoord(0, 0, 0); //记录目标物在主车坐标系下位置
            //世界坐标系转主车坐标系
            ego_transform.relative3dCoor(egoState->sObjectState.sPos.u4H, egoState->sObjectState.sPos.u4P, egoState->sObjectState.sPos.u4R, point, relativeFourPointCoord);

            Vector3D sensorRelaFourPoint(relativeFourPointCoord[0] - getAssemblePositionX(), relativeFourPointCoord[1] - getAssemblePositionY(), relativeFourPointCoord[2] - getAssemblePositionZ());
            //主车坐标系转传感器坐标系
            CoordRotated(getHeading(), getPitch(), getRoll(), sensorRelaFourPoint);
            float fourPointX = sensorRelaFourPoint.x();
            float fourPointY = sensorRelaFourPoint.y();
            float fourPointZ = sensorRelaFourPoint.z();


            thetaTmp = std::atan2(fourPointY, fourPointX);
            log_compnt_mngr->trace("fourPointX  , fourPointY  , thetaTmp {},{},{}",fourPointX, fourPointY, thetaTmp);
            // 确定最大弧度和最小弧度
            if (thetaTmp > thetaMax)
            {
                thetaMax = thetaTmp;
            }
            if (thetaTmp < thetaMin)
            {
                thetaMin = thetaTmp;
            }

            float range = std::sqrt(std::pow(fourPointX,2) + std::pow(fourPointY,2)); //当前被探测行人的某个顶点在传感器坐标系下在水平面上（xy 平面）到传感器的距离。
            float thetaZDim = std::atan(fourPointZ/range); //前被探测行人的某个顶点在传感器坐标系下相对于传感器的垂直角度
            if (thetaZDim > thetaMaxZDim)
            {
                thetaMaxZDim = thetaZDim;
            }
            if (thetaZDim < thetaMinZDim)
            {
                thetaMinZDim = thetaZDim;
            }
        }
        agentFourPointRadMap[static_cast<int>(objectId)] = std::make_tuple(thetaMin, thetaMax, thetaMinZDim, thetaMaxZDim); // 获取目标水平和垂直方向最大夹角
        log_compnt_mngr->debug("agentFourPointRadMap, (ID:)(thetaMin: thetaMax:) {},{},{},{}.{}", objectId, thetaMin, thetaMax,thetaMinZDim,thetaMaxZDim);
    }


    /********************************** 3. Prepare obstacle data *************************************/
    //遍历所有障碍物
    for (auto miscIt = obstacleList.begin(); miscIt != obstacleList.end(); ++miscIt)
    {
        S_SP_MIL_OBJECT_STATE *objectState = *miscIt;
        if (objectState == nullptr)
        {
            log_compnt_mngr->warn( "obstacle objectState is nullptr,continue.");
            continue;
        }

        // 目标物ID
        uint32_t objectId = objectState->sObjectState.u4Id;

        Transform miscObjectTransform;
        miscObjectTransform.update(objectState->sObjectState.sPos.u8X, objectState->sObjectState.sPos.u8Y, objectState->sObjectState.sPos.u8Z);

        log_compnt_mngr->debug("getObjectInViewList,Name= (ID: type:) {},{},{}",objectState->sObjectState.au1Name, objectId, SENSOR_OBJECT_DETECTION_TYPE_OBSTACLE);
        std::vector<Vector3D> objBBoxVertexVector;  //获取障碍物包围盒8个顶点信息
        computeVehicleObstacleBBox(objectState, objBBoxVertexVector);

        // 世界坐标系转传感器坐标系
        Vector3D relativeCoord(0, 0, 0);
        Vector3D sensorRela(0, 0, 0); //记录传感器坐标系下点
        CoordTransform(egoState->sObjectState.sPos.u4H, egoState->sObjectState.sPos.u4P, egoState->sObjectState.sPos.u4R, ego_transform, miscObjectTransform, getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(), getHeading(), getPitch(), getRoll(), relativeCoord, sensorRela);

        log_compnt_mngr->trace("relativeCoord[0] , relativeCoord[1]  relativeCoord[2] {},{},{}", relativeCoord[0],relativeCoord[1],relativeCoord[2]);
        log_compnt_mngr->trace("sensorRela[0] , sensorRela[1]  sensorRela[2] {},{},{}", sensorRela[0],sensorRela[1],sensorRela[2]);
        //障碍物是否在识别范围和FOV内
        if (!InCircleandInSector(egoState, ego_transform, objBBoxVertexVector, getHFOV(), getVFOV(), getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(),
                                getRange(), getMinimumDetectRange(), getHeading(), getPitch(), getRoll()))
        {
            log_compnt_mngr->warn("obstacle is not InCircleandInSector, Name = {} ,id = {},continue.",objectState->sObjectState.au1Name,objectId);
            continue;
        }

        if (objectState->sObjectState.u1Type == D_SP_OBJECT_TYPE_TREE || objectState->sObjectState.u1Type == D_SP_OBJECT_TYPE_POLE) //遮挡算法忽略杆和树模型
        {
            InViewIDs[SENSOR_OBJECT_DETECTION_TYPE_OBSTACLE].insert(objectId);
            sortedObjs.push_back(std::make_tuple(objectId,SENSOR_OBJECT_DETECTION_TYPE_OBSTACLE,sensorRela));
            log_compnt_mngr->warn("obstacle type ={} is tree or pole, sortedObjs.push_back Name = {} ,id = {},continue.",objectState->sObjectState.u1Type, objectState->sObjectState.au1Name,objectId);
            continue;
        }

        //计算相对坐标，即转换为传感器坐标系，x轴向前，y轴向左
        float agentX = sensorRela.x();
        float agentY = sensorRela.y();
        float agentZ = sensorRela.z();
        // 存放所有的目标物体
        agentCoordVec.push_back(std::make_tuple(static_cast<int>(objectId), SENSOR_OBJECT_DETECTION_TYPE_OBSTACLE,agentX, agentY,agentZ));

        log_compnt_mngr->debug("agentCoordVec, (agentX: agentY:). {},{}", agentX, agentY);

        // 获取目标物体的最大弧度和最小弧度
        float thetaMin = DBL_MAX;    //目标物物体在传感器坐标系下相对于传感器的最小水平弧度
        float thetaMax = -DBL_MAX;   //目标物物体在传感器坐标系下相对于传感器的最大水平弧度
        float thetaTmp = 0.0;        //目标物物体的某个顶点在传感器坐标系下相对于传感器的水平角度
        float thetaMinZDim = DBL_MAX;  //目标物物体在传感器坐标系下相对于传感器的最小垂直角度
        float thetaMaxZDim = -DBL_MAX; //目标物物体在传感器坐标系下相对于传感器的最大垂直角度
        float hightDiffMin = -DBL_MAX; //目标物物体在传感器坐标系下相对于传感器的最小高度差

        for(unsigned int i = 0; i < objBBoxVertexVector.size() ; i++) //遍历被探测障碍物的八个顶点
        {
            Vector3D point = objBBoxVertexVector[i];

            //计算相对坐标，即转换为主车坐标系，x轴向前，y轴向左
            Vector3D relativeFourPointCoord(0, 0, 0);
            //世界坐标系转主车坐标系
            ego_transform.relative3dCoor(egoState->sObjectState.sPos.u4H, egoState->sObjectState.sPos.u4P, egoState->sObjectState.sPos.u4R, point, relativeFourPointCoord);

            Vector3D sensorRelaFourPoint(relativeFourPointCoord[0] - getAssemblePositionX(), relativeFourPointCoord[1] - getAssemblePositionY(), relativeFourPointCoord[2] - getAssemblePositionZ());
            //主车坐标系转传感器坐标系
            CoordRotated(getHeading(), getPitch(), getRoll(), sensorRelaFourPoint);
            float fourPointX = sensorRelaFourPoint.x();
            float fourPointY = sensorRelaFourPoint.y();
            float fourPointZ = sensorRelaFourPoint.z();
            hightDiffMin = std::max<float>(sensorRelaFourPoint.z(), hightDiffMin);

            thetaTmp = std::atan2(fourPointY, fourPointX);
            log_compnt_mngr->trace("fourPointX  , fourPointY  , thetaTmp . {},{},{}",fourPointX, fourPointY, thetaTmp);
            // 确定最大弧度和最小弧度
            if (thetaTmp > thetaMax)
            {
                thetaMax = thetaTmp;
            }
            if (thetaTmp < thetaMin)
            {
                thetaMin = thetaTmp;
            }

            float range = std::sqrt(std::pow(fourPointX,2) + std::pow(fourPointY,2)); //当前被探测障碍物的某个顶点在传感器坐标系下在水平面上（xy 平面）到传感器的距离。
            float thetaZDim = std::atan(fourPointZ/range); //前被探测障碍物的某个顶点在传感器坐标系下相对于传感器的垂直角度
            if (thetaZDim > thetaMaxZDim)
            {
                thetaMaxZDim = thetaZDim;
            }
            if (thetaZDim < thetaMinZDim)
            {
                thetaMinZDim = thetaZDim;
            }

        }

        if ( hightDiffMin < 0.1 - getAssemblePositionZ() );  // 过滤高程极低的模型
        {
            InViewIDs[SENSOR_OBJECT_DETECTION_TYPE_OBSTACLE].insert(objectId);
            sortedObjs.push_back(std::make_tuple(objectId,SENSOR_OBJECT_DETECTION_TYPE_OBSTACLE,sensorRela));
            log_compnt_mngr->warn("hightDiffMin= {} < {}; sortedObjs.push_back(ID:)(name: type:).{},{},{}",hightDiffMin, 0.1 - getAssemblePositionZ(), objectId, objectState->sObjectState.au1Name, objectState->sObjectState.u1Type);
            continue;
        }

        agentFourPointRadMap[static_cast<int>(objectId)] = std::make_tuple(thetaMin, thetaMax, thetaMinZDim, thetaMaxZDim); // 获取目标水平和垂直方向最大夹角
        log_compnt_mngr->debug("agentFourPointRadMap, (ID:)(thetaMin: thetaMax:).{},{},{},{},{}", objectId, thetaMin, thetaMax,thetaMinZDim,thetaMaxZDim);
    }

    /********************************** 4. Sort All object data *************************************/
    if (agentCoordVec.empty())
    {
        log_compnt_mngr->error("agentCoordVec is empty.");
        return false;
    }
    // 当目标物体size不等于1时，把目标物体进行排序
    if (agentCoordVec.size() > 1)
    {
        getSortedAgentCoord(agentCoordVec);
    }

    /********************************** 5. Filter Object based on Masking algorithm *************************************/
    for (unsigned int i = 0; i < agentCoordVec.size(); ++i)
    {
        int id = std::get<0>(agentCoordVec[i]);
        int type = std::get<1>(agentCoordVec[i]);
        float x = std::get<2>(agentCoordVec[i]);
        float y = std::get<3>(agentCoordVec[i]);
        float z = std::get<4>(agentCoordVec[i]);

        log_compnt_mngr->debug("Object size:{}",agentCoordVec.size());
        if (agentCoordVec.size() == 1) //当只有一个物体时，无遮挡判断
        {
            InViewIDs[type].insert(id);
            sortedObjs.push_back(std::make_tuple(id,type,Vector3D(x,y,z)));
            break;
        }

        // 取得当前目标的质点弧度
        float theta = std::atan2(y, x);
        log_compnt_mngr->debug(", (ID: type: theta:).{},{},{}",id, type, theta);
        // 如果目标被遮挡
        if( IsMaskedView(i,id,theta))
        {
            log_compnt_mngr->warn("IsMaskedView, id:{} continue.", id);
            continue;
        }
        else // 如果目标没有被遮挡，则保存此ID
        {
            log_compnt_mngr->debug("IsMaskedView, insert(id {}).", id);
            InViewIDs[type].insert(id);
            sortedObjs.push_back(std::make_tuple(id,type,Vector3D(x,y,z)));
        }
    }

    // Set ID List and Update ID List
    setInViewIDsList(InViewIDs);

    (void )clock_gettime(CLOCK_REALTIME, &timeStamp);
    long int sysTimeNs2 = static_cast<long int>(timeStamp.tv_sec * 1000000000) + timeStamp.tv_nsec;

    log_compnt_mngr->debug("Target Object InViewIDs size:{}",InViewIDs.size());
    log_compnt_mngr->info("Sensor::getObjectInViewList end.");
    return true;
}

// 获取筛选后未遮挡的物体ID
std::map<int, std::set<int>> Sensor::getInViewIDsList()
{
    return InViewIDs;
}

// 设置筛选后未遮挡的物体ID
void Sensor::setInViewIDsList(std::map<int, std::set<int>> setInViewIDs)
{
    InViewIDs = setInViewIDs;
}

//判定是否是物体识别类型
bool Sensor::getObjectList(int type, std::set<int> &IDs)
{
    log_compnt_mngr->info("Sensor::getObjectList 2 start.");
    if (_description.enable == false)
    {
        log_compnt_mngr->error("description.enable is false,return.");
        return false;
    }

    if (isDetectionObject(type) == false)
    {
        log_compnt_mngr->error("is not detection object,return.");
        return false;
    }

    if(_description.type == SENSOR_TYPE_V2X_OBU)
    {
        bEnableMaskFunction = false;
    }

    if(_description.type == SENSOR_TYPE_V2X_RSU)
    {
        bEnableMaskFunction = false;
    }

    //取得（车辆）的ID列表
    if (type == SENSOR_OBJECT_DETECTION_TYPE_VEHICLE)
    {

        if(bEnableMaskFunction)
        {
            if (InViewIDs.size() == 0)
            {
                getObjectInViewList();
            }
            getVehicleInViewList(IDs);
        }
        else
        {
            getVehicleList(IDs);
        }
    }
    //取得（行人）的ID列表
    else if (type == SENSOR_OBJECT_DETECTION_TYPE_PEDESTRIAN)
    {
        if(bEnableMaskFunction)
        {
            if (InViewIDs.size() == 0)
            {
                getObjectInViewList();
            }
            getPedestrianInViewList(IDs);
        }
        else
        {
            getPedestrianList(IDs);
        }
    }
    //取得（障碍物）的ID列表
    else if (type == SENSOR_OBJECT_DETECTION_TYPE_OBSTACLE || type == SENSOR_OBJECT_DETECTION_TYPE_ROADMARK)
    {
        if(bEnableMaskFunction)
        {
            if (InViewIDs.size() == 0)
            {
                getObjectInViewList();
            }
            getRoadObjectInViewList(IDs);
        }
        else
        {
            getRoadObjectList(IDs);
        }
    }
    ////取得（交通灯）的ID列表
    else if (type == SENSOR_OBJECT_DETECTION_TYPE_TRFICLIGHT)
    {
        getTrafficLightList(IDs);
    }
    ////取得（TrafficSign）的ID列表
    else if (type == SENSOR_OBJECT_DETECTION_TYPE_TRAFICSIGN)
    {
        getTrafficSignList(IDs);
    }
    else
    {
        //do nothing
    }

    log_compnt_mngr->info("Sensor::getObjectList 2 end.");
    return true;
}


//计算天气影响因子
double Sensor::calcWeatherNoise(int type, double distance)
{
    log_compnt_mngr->info("Sensor::calcWeatherNoise start.");
    double rainIntensity = 0.0;
    double snowIntensity = 0.0;
    double fogIntensity = 0.0;
    if (environment != nullptr)
    {
        rainIntensity = environment->u8RainIntensity; //获得降雨量强度
        snowIntensity = environment->u8SnowIntensity; //获得降雪量强度
        fogIntensity = environment->u8FogIntensity; //获得雾强度
    }

    float snowFactor = 0.0; // 雪因子
    float rainFactor = 0.0; // 雨因子
    float fogFactor = 0.0;  // 雾因子

    // 不同类别的传感器计算环境影响因子
    switch(type)
    {
        case SENSOR_TYPE_RADAR:
            snowFactor = RADAR_SNOWFACTOR;
            rainFactor = RADAR_RAINFACTOR;
            fogFactor = RADAR_FOGFACTOR;
        break;
        case SENSOR_TYPE_LIDAR: //天气影响因子
            snowFactor = LIDAR_SNOWFACTOR;
            fogFactor = LIDAR_FOGFACTOR;
            rainFactor = LIDAR_RAINFACTOR;
        break;
        case SENSOR_TYPE_CAMERA: //天气影响因子
            snowFactor = CAMERA_SNOWFACTOR;
            fogFactor = CAMERA_FOGFACTOR;
            rainFactor = CAMERA_RAINFACTOR;
        break;
        default:
            break;
    }

    double minDetectRange = SENSOR_OBJECT_DETECTION_MIN_DISTANCE; // 目标最小识别范围内不受天气影响
    // 计算天气因子
    double snowPct = pow( (minDetectRange / distance) , (snowFactor * snowIntensity) / 10 );
    double fogPct = pow( (minDetectRange / distance) , (fogFactor * fogIntensity) / 10 );
    double rainPct = pow( (minDetectRange / distance) , (rainFactor * rainIntensity) / 10 );
    if (snowPct == 0.0) snowPct = 1;
    if (fogPct == 0.0) fogPct = 1;
    if (rainPct == 0.0) rainPct = 1;

    double weatherPct = std::min<double>({snowPct, fogPct, rainPct});  //计算影响雨雪雾影响最大的因子
    log_compnt_mngr->info("Sensor::calcWeatherNoise end.");
    return weatherPct;
}

// 填充多面体模型特征点分为上下两个部分
bool Sensor::fillTwoPartOutlineMap(const std::string modelEntryName, std::map<int, std::tuple<Vector3D,Vector3D,Vector3D,Vector3D>> &m_upperOutline, std::map<int, std::tuple<Vector3D,Vector3D,Vector3D,Vector3D>> &m_bottomOutline, const Vector3D &paramScale)
{
    log_compnt_mngr->info("Sensor::fillTwoPartOutlineMap start.");
    log_compnt_mngr->debug("m_modelOutline.size(), {}, modelEntryName = {}" , m_modelOutline.size(),modelEntryName);
    auto iterOutline = m_modelOutline.find(modelEntryName);

    if ((iterOutline == m_modelOutline.end()) && (0 == modelEntryName.compare("Saimo")))
    {
        std::string modelCarName = "默认";
        iterOutline = m_modelOutline.find(modelCarName);
        if (iterOutline == m_modelOutline.end())
        {
            std::string modelCarName = "HongQi";
            iterOutline = m_modelOutline.find(modelCarName);
        }
    }


    // 是否有此模型轮廓点
    if (iterOutline != m_modelOutline.end())
    {
        std::vector<Vector3D> outLine = iterOutline->second;
        //对点坐标按照车模缩放比例进行相应修改
        for (auto &it : outLine)
        {
            it.set(it.x() * paramScale[0], it.y() * paramScale[1], it.z() * paramScale[2]);
        }

        // 特征点的传感器坐标系
        m_bottomOutline[A1] = make_tuple(outLine[A1],outLine[A2],outLine[B1],outLine[H1]); // A1
        m_bottomOutline[B1] = make_tuple(outLine[B1],outLine[B2],outLine[G1],outLine[A1]); // B1
        m_upperOutline[C1]  = make_tuple(outLine[C1],outLine[C2],outLine[D1],outLine[F1]); // C1
        m_upperOutline[D1]  = make_tuple(outLine[D1],outLine[D2],outLine[E1],outLine[C1]); // D1

        m_upperOutline[E1]  = make_tuple(outLine[E1],outLine[E2],outLine[F1],outLine[D1]); // E1
        m_upperOutline[F1]  = make_tuple(outLine[F1],outLine[F2],outLine[C1],outLine[E1]); // F1
        m_bottomOutline[G1] = make_tuple(outLine[G1],outLine[G2],outLine[H1],outLine[B1]); // G1
        m_bottomOutline[H1] = make_tuple(outLine[H1],outLine[H2],outLine[A1],outLine[G1]); // H1


        m_bottomOutline[A2] = make_tuple(outLine[A2],outLine[A1],outLine[H2],outLine[B2]); // A2
        m_bottomOutline[B2] = make_tuple(outLine[B2],outLine[B1],outLine[A2],outLine[G2]); // B2
        m_upperOutline[C2]  = make_tuple(outLine[C2],outLine[C1],outLine[F2],outLine[D2]); // C2
        m_upperOutline[D2]  = make_tuple(outLine[D2],outLine[D1],outLine[C2],outLine[E2]); // D2

        m_upperOutline[E2]  = make_tuple(outLine[E2],outLine[E1],outLine[D2],outLine[F2]); // E2
        m_upperOutline[F2]  = make_tuple(outLine[F2],outLine[F1],outLine[E2],outLine[C2]); // F2
        m_bottomOutline[G2] = make_tuple(outLine[G2],outLine[G1],outLine[B2],outLine[H2]); // G2
        m_bottomOutline[H2] = make_tuple(outLine[H2],outLine[H1],outLine[G2],outLine[A2]); // H2

        log_compnt_mngr->info("Sensor::fillTwoPartOutlineMap end.");
        return true;
    }
    else
    {
        log_compnt_mngr->error("modelEntryName not found");
        return false;
    }

}

// 当前物体最近点是否被遮挡
bool Sensor::hasOcclusion(int id, float f_targetHorRad, float f_targetVerRad, float f_occlusionR, float f_occlusionL, float f_occlusionB, float f_occlusionU)
{

#if ENABLE_YIQILOG //bug 20599 pjd 2023.09.21
    if (f_targetHorRad >= f_occlusionR && f_targetHorRad <= f_occlusionL
        && f_targetVerRad <= f_occlusionU && f_targetVerRad >= f_occlusionB) // 水平方向和垂直方向最近点被遮挡
#else
    if (f_targetHorRad > f_occlusionR && f_targetHorRad < f_occlusionL
        && f_targetVerRad < f_occlusionU && f_targetVerRad > f_occlusionB) // 水平方向和垂直方向最近点被遮挡
#endif
    {
        return true;
    }
    else if (f_occlusionL == 0.0 && f_occlusionR == 0.0
            && f_occlusionU == 0.0 && f_occlusionB == 0.0) // 遮挡参数无效（即最近物体）
    {
        return false;
    }
    else // 无遮挡关系
    {
        return false;
    }
}

//计算遮挡后的最近点
void Sensor::calcCoordWithOcclusion(int id, Vector3D nearestVertex, Vector3D normalVec, float occlusionR, float occlusionL, Vector3D v_modelOutline, Vector3D &v_modelR, Vector3D &v_modelL)
{
    log_compnt_mngr->info("Sensor::calcCoordWithOcclusion start.");
    double dir = normalVec.dot(nearestVertex); // 法向量点乘顶点向量
    Vector3D vec_n (0,0,1);
    if (agentFourPointRadMap.find(id) != agentFourPointRadMap.end())
    {
        float targetR = std::get<0>(agentFourPointRadMap[id]);
        float targetL = std::get<1>(agentFourPointRadMap[id]);
        log_compnt_mngr->debug("target right {}, target left {}", targetR,targetL);
        log_compnt_mngr->debug("normalVec = ({},{},{})", normalVec[0], normalVec[1], normalVec[2]);

        if (dir < 0 && abs(normalVec.dot(vec_n))< 0.99)
        {
            if (occlusionR > targetR && occlusionR < targetL)
            {
                float valR = (dir - v_modelOutline[2] * normalVec[2]) / (cos(occlusionR) * normalVec[0] + sin(occlusionR) * normalVec[1]);
                v_modelR[0] = valR * cos(occlusionR);
                v_modelR[1] = valR * sin(occlusionR);
                v_modelR[2] = v_modelOutline[2];
                log_compnt_mngr->debug("v_modelR ({},{},{})", v_modelR[0],v_modelR[1],v_modelR[2]);
            }

            if (occlusionL < targetL && occlusionL > targetR)
            {
                float valL = (dir - v_modelOutline[2] * normalVec[2]) / (cos(occlusionL) * normalVec[0] + sin(occlusionL) * normalVec[1]);
                v_modelL[0] = valL * cos(occlusionL);
                v_modelL[1] = valL * sin(occlusionL);
                v_modelL[2] = v_modelOutline[2];
                log_compnt_mngr->debug("v_modelL ({},{},{})", v_modelL[0],v_modelL[1],v_modelL[2]);
            }
        }
        else
        {
            // 异常：全部被遮挡，输出原始点
            log_compnt_mngr->error("Non-fulfilment of conditions");
        }
    }
    else
    {
        log_compnt_mngr->error("ID {} not found", id);
    }

    log_compnt_mngr->info("Sensor::calcCoordWithOcclusion end.");
}

// 当最近点在棱上和面上时,出现异常情况
bool Sensor::isExceptionCase(Vector3D AP, Vector3D OA, Vector3D vec_edge1, Vector3D vec_edge2, Vector3D &newCoord)
{
    log_compnt_mngr->info("Sensor::isExceptionCase start.");
    double len_AP = AP.length();
    double len_edge1 = vec_edge1.length();
    double len_edge2 = vec_edge2.length();
    // P点实际应该在棱上
    if ((AP.cross(vec_edge1)).dot(AP.cross(vec_edge2)) > 0)
    {
        double radPAD = AP.dot(vec_edge2) / (len_AP * len_edge2);
        double radPAC = AP.dot(vec_edge1) / (len_AP * len_edge1);
        log_compnt_mngr->debug("radPAD {}, radPAC {}", radPAD,radPAC);
        if (radPAD > radPAC)
        {
            newCoord[0] = OA[0] + (len_AP * radPAD / len_edge2) * vec_edge2[0];
            newCoord[1] = OA[1] + (len_AP * radPAD / len_edge2) * vec_edge2[1];
            newCoord[2] = OA[2] + (len_AP * radPAD / len_edge2) * vec_edge2[2];
        }
        else
        {
            newCoord[0] = OA[0] + (len_AP * radPAC / len_edge1) * vec_edge1[0];
            newCoord[1] = OA[1] + (len_AP * radPAC / len_edge1) * vec_edge1[1];
            newCoord[2] = OA[2] + (len_AP * radPAC / len_edge1) * vec_edge1[2];
        }
        log_compnt_mngr->debug("newCoord ({},{},{})", newCoord[0],newCoord[1],newCoord[2]);
        log_compnt_mngr->info("Sensor::isExceptionCase end.");
        return true;
    }
    // P点实际应该在面上
    else
    {
        log_compnt_mngr->error("point on the surface, return");
        return false;
    }
}

//解决最近点在棱和面判断错误问题
//判断一个特定的点是否处于某种边缘情况
bool Sensor::isEdgeCase(Vector3D OA, Vector3D vec_edge1, Vector3D vec_edge2, Vector3D vec_normal, Vector3D &newCoord)
{
    log_compnt_mngr->info("Sensor::isEdgeCase start.");
    double dir = vec_normal.dot(OA);
    newCoord[0] = vec_normal[0] * dir;
    newCoord[1] = vec_normal[1] * dir;
    newCoord[2] = vec_normal[2] * dir;
    Vector3D AP = newCoord - OA;
    // P点实际应该在面上
    double len_edge1 = vec_edge1.length();
    double len_edge2 = vec_edge2.length();

    if ( ((AP).cross(vec_edge1)).dot(AP.cross(vec_edge2)) < 0
        && AP.dot(vec_edge1 / len_edge1 + vec_edge2 / len_edge2) > 0 )
    {
        log_compnt_mngr->debug("newCoord ({},{},{})", newCoord[0],newCoord[1],newCoord[2]);
        log_compnt_mngr->info("Sensor::isEdgeCase end.");
        return true;
    }
    else
    {
        log_compnt_mngr->error("isEdgeCase false");
        return false;
    }
}

/* 最近点情况：
        case 1 顶点上	AOxAB <= 0 && AOxAC <= 0 && AOxAD <= 0
        case 2 棱边上	AOxAB > 0 || AOxAC > 0 || AOxAD > 0
        case 3 面上		AOxAB > 0 && AOxAC > 0 ||
                        AOxAB > 0 && AOxAD > 0 ||
                        AOxAC > 0 && AOxAD > 0
        case 4 异常		AOxAB > 0 && AOxAC > 0 && AOxAD > 0
*/
void Sensor::calcNearestCoord(int id, Vector3D vec_AB, Vector3D vec_AC, Vector3D vec_AD, Vector3D v_modelOutline, Vector3D &v_modelOutlineR, Vector3D &v_modelOutlineL)
{
    log_compnt_mngr->info("Sensor::calcNearestCoord start.");
    Vector3D OA (v_modelOutline[0],v_modelOutline[1],v_modelOutline[2]);
    Vector3D AO (-OA[0],-OA[1],-OA[2]);
    // 向量点乘
    double AOxAB = AO.dot(vec_AB);
    double AOxAC = AO.dot(vec_AC);
    double AOxAD = AO.dot(vec_AD);
    // 各面的法向量
    Vector3D normalABC = vec_AC.cross(vec_AB) / (vec_AC.cross(vec_AB)).length();
    Vector3D normalADB = vec_AB.cross(vec_AD) / (vec_AB.cross(vec_AD)).length();
    Vector3D normalACD = vec_AD.cross(vec_AC) / (vec_AD.cross(vec_AC)).length();

    log_compnt_mngr->debug("AO ({},{},{})", AO[0],AO[1],AO[2]);
    log_compnt_mngr->debug("vec_AB = ({},{},{})", vec_AB[0], vec_AB[1], vec_AB[2]);
    log_compnt_mngr->debug("vec_AD = ({},{},{})", vec_AD[0], vec_AD[1], vec_AD[2]);
    log_compnt_mngr->debug("vec_AC = ({},{},{})", vec_AC[0], vec_AC[1], vec_AC[2]);

    if (AOxAB <= 0 && AOxAC <= 0 && AOxAD <= 0) // case 1 A点
    {
        log_compnt_mngr->debug("case 1");
    }
    else if (AOxAB > 0) // case2 & 3
    {
        if (AOxAC <= 0 && AOxAD <= 0) // case2 AB棱
        {
            log_compnt_mngr->debug("case 2 AB");
            double ABmod = std::pow(vec_AB[0], 2) + std::pow(vec_AB[1], 2) + std::pow(vec_AB[2], 2);
            v_modelOutline = v_modelOutline + (vec_AB / ABmod * AOxAB);

            Vector3D newCoordADB (0.0,0.0,0.0);
            Vector3D newCoordABC (0.0,0.0,0.0);
            if (isEdgeCase(OA, vec_AB, vec_AD, normalADB, newCoordADB))
            {
                v_modelOutline = newCoordADB;
            }
            else if (isEdgeCase(OA, vec_AB, vec_AC, normalABC, newCoordABC))
            {
                v_modelOutline = newCoordABC;
            }
            else
            {
                // do nothing
            }
        }
        else if (AOxAC > 0) // case3 ABC面
        {
            log_compnt_mngr->debug("case 3 ABC");
            double dir = normalABC.dot(v_modelOutline);
            v_modelOutline[0] = normalABC[0] * dir;
            v_modelOutline[1] = normalABC[1] * dir;
            v_modelOutline[2] = normalABC[2] * dir;
            // 是否出现异常情况：最近点不在面上
            Vector3D AP = AO + v_modelOutline;
            Vector3D newCoord (0.0,0.0,0.0);
            if (isExceptionCase(AP, OA, vec_AB, vec_AC, newCoord))
            {
                v_modelOutline = newCoord;
            }
        }
        else if (AOxAD > 0) // case3 ADB面
        {
            log_compnt_mngr->debug("case 3 ADB");
            double dir = normalADB.dot(v_modelOutline);
            v_modelOutline[0] = normalADB[0] * dir;
            v_modelOutline[1] = normalADB[1] * dir;
            v_modelOutline[2] = normalADB[2] * dir;
            // 是否出现异常情况：最近点不在面上
            Vector3D AP = AO + v_modelOutline;
            Vector3D newCoord (0.0,0.0,0.0);
            if (isExceptionCase(AP, OA, vec_AD, vec_AB, newCoord))
            {
                v_modelOutline = newCoord;
            }
        }
    }
    else if (AOxAC > 0) // case2 & 3
    {
        if (AOxAB <= 0 && AOxAD <= 0) // case2 AC棱
        {
            log_compnt_mngr->debug("case 2 AC");
            double ACmod = std::pow(vec_AC[0], 2) + std::pow(vec_AC[1], 2) + std::pow(vec_AC[2], 2);
            v_modelOutline = v_modelOutline + (vec_AC / ACmod * AOxAC);

            Vector3D newCoordACD (0.0,0.0,0.0);
            Vector3D newCoordABC (0.0,0.0,0.0);
            if (isEdgeCase(OA, vec_AC, vec_AD, normalACD, newCoordACD))
            {
                v_modelOutline = newCoordACD;
            }
            else if (isEdgeCase(OA, vec_AB, vec_AC, normalABC, newCoordABC))
            {
                v_modelOutline = newCoordABC;
            }
            else
            {
                // do nothing
            }
        }
        else if (AOxAD > 0) // case3 ACD面
        {
            log_compnt_mngr->debug("case 3 ACD");
            double dir = normalACD.dot(v_modelOutline);
            v_modelOutline[0] = normalACD[0] * dir;
            v_modelOutline[1] = normalACD[1] * dir;
            v_modelOutline[2] = normalACD[2] * dir;

            // 是否出现异常情况：最近点不在面上
            Vector3D AP = AO + v_modelOutline;
            Vector3D newCoord (0.0,0.0,0.0);
            if (isExceptionCase(AP, OA, vec_AC, vec_AD, newCoord))
            {
                v_modelOutline = newCoord;
            }
        }
    }
    else if (AOxAD > 0) // case2 & 3
    {
        if (AOxAB <= 0 && AOxAC <= 0) // case2 AD棱
        {
            log_compnt_mngr->debug("case 2 AD");
            double ADmod = std::pow(vec_AD[0], 2) + std::pow(vec_AD[1], 2) + std::pow(vec_AD[2], 2);
            v_modelOutline = v_modelOutline + (vec_AD / ADmod * AOxAD);

            Vector3D newCoordACD (0.0,0.0,0.0);
            Vector3D newCoordADB (0.0,0.0,0.0);
            if (isEdgeCase(OA, vec_AC, vec_AD, normalACD, newCoordACD))
            {
                v_modelOutline = newCoordACD;
            }
            else if (isEdgeCase(OA, vec_AD, vec_AB, normalADB, newCoordADB))
            {
                v_modelOutline = newCoordADB;
            }
            else
            {
                // do nothing
            }
        }
    }
    else // 异常
    {
        log_compnt_mngr->warn( "warning!");
        // do nothing
    }
    log_compnt_mngr->debug("NearestCoord = ({},{},{})", v_modelOutline[0], v_modelOutline[1], v_modelOutline[2]);

    /* 最近点遮挡情况：
            case 1 面上最近点被遮挡
            case 2 棱边上最近点被遮挡
            case 3 顶点最近点被遮挡
            case 4 异常
    */
    float targetHorRad = atan2(v_modelOutline[1],v_modelOutline[0]); // 最近点水平方向弧度值
    float range = std::sqrt(std::pow(v_modelOutline[0],2) + std::pow(v_modelOutline[1],2));
    float targetVerRad = atan(v_modelOutline[2]/range); // 最近点垂直方向弧度值
    log_compnt_mngr->debug("targetHorRad {}, targetVerRad {}", targetHorRad,targetVerRad);
    // 获取当前ID被遮挡的弧度
    if (objOcclusionScale.find(id) != objOcclusionScale.end())
    {
        float occlusionR = std::get<3>(objOcclusionScale[id]);
        float occlusionL = std::get<4>(objOcclusionScale[id]);
        float occlusionB = std::get<5>(objOcclusionScale[id]);
        float occlusionU = std::get<6>(objOcclusionScale[id]);
        log_compnt_mngr->debug("occlusionR {}, occlusionL {}, occlusionB {}, occlusionU {}", occlusionR,occlusionL,occlusionB,occlusionU);
        // 第一个物体无需修改最近点情况
        if (occlusionR == 0.0 && occlusionL == 0.0 && occlusionB == 0.0 && occlusionU == 0.0)
        {
            v_modelOutlineR = v_modelOutline;
            v_modelOutlineL = v_modelOutline;
            log_compnt_mngr->debug("No occlusion");
            log_compnt_mngr->info("Sensor::calcNearestCoord end.");
            return;
        }
        // 最近点是否被遮挡
        if (hasOcclusion(id, targetHorRad, targetVerRad, occlusionR, occlusionL, occlusionB, occlusionU))
        {
            Vector3D ABACRightVal (0.0,0.0,0.0);
            Vector3D ABACLeftVal (0.0,0.0,0.0);
            Vector3D ADABRightVal (0.0,0.0,0.0);
            Vector3D ADABLeftVal (0.0,0.0,0.0);
            Vector3D ACADRightVal (0.0,0.0,0.0);
            Vector3D ACADLeftVal (0.0,0.0,0.0);
            // 计算三个相邻面的点
            calcCoordWithOcclusion(id, OA, normalABC, occlusionR, occlusionL, v_modelOutline, ABACRightVal, ABACLeftVal);
            calcCoordWithOcclusion(id, OA, normalADB, occlusionR, occlusionL, v_modelOutline, ADABRightVal, ADABLeftVal);
            calcCoordWithOcclusion(id, OA, normalACD, occlusionR, occlusionL, v_modelOutline, ACADRightVal, ACADLeftVal);
            double len_ABACRightVal = pow(ABACRightVal[0], 2) + pow(ABACRightVal[1], 2) + pow(ABACRightVal[2], 2);
            double len_ADABRightVal = pow(ADABRightVal[0], 2) + pow(ADABRightVal[1], 2) + pow(ADABRightVal[2], 2);
            double len_ACADRightVal = pow(ACADRightVal[0], 2) + pow(ACADRightVal[1], 2) + pow(ACADRightVal[2], 2);
            double len_ABACLeftVal = pow(ABACLeftVal[0], 2) + pow(ABACLeftVal[1], 2) + pow(ABACLeftVal[2], 2);
            double len_ADABLeftVal = pow(ADABLeftVal[0], 2) + pow(ADABLeftVal[1], 2) + pow(ADABLeftVal[2], 2);
            double len_ACADLeftVal = pow(ACADLeftVal[0], 2) + pow(ACADLeftVal[1], 2) + pow(ACADLeftVal[2], 2);
            log_compnt_mngr->debug("len_ABACRightVal = {}, len_ADABRightVal = {}, len_ACADRightVal = {}", len_ABACRightVal, len_ADABRightVal, len_ACADRightVal);
            log_compnt_mngr->debug("len_ABACLeftVal = {}, len_ADABLeftVal = {}, len_ACADLeftVal = {}", len_ABACLeftVal, len_ADABLeftVal, len_ACADLeftVal);

            // 分别在右侧和左侧找到距离最远的向量
            // 右侧
            double max = len_ABACRightVal;
            v_modelOutlineR = ABACRightVal;
            if ( max < len_ADABRightVal)
            {
                max = len_ADABRightVal;
                v_modelOutlineR = ADABRightVal;
            }
            if ( max < len_ACADRightVal)
            {
                max = len_ACADRightVal;
                v_modelOutlineR = ACADRightVal;
            }
            // 左侧
            max = len_ABACLeftVal;
            v_modelOutlineL = ABACLeftVal;
            if ( max < len_ADABLeftVal)
            {
                max = len_ADABLeftVal;
                v_modelOutlineL = ADABLeftVal;
            }
            if ( max < len_ACADLeftVal)
            {
                max = len_ACADLeftVal;
                v_modelOutlineL = ACADLeftVal;
            }
        }
        else // 异常
        {
            v_modelOutlineR = v_modelOutline;
            v_modelOutlineL = v_modelOutline;
        }
        // 水平方向完全遮挡
        if (v_modelOutlineR.length() == 0 && v_modelOutlineL.length() == 0)
        {
            v_modelOutlineR[0] = v_modelOutline[0];
            v_modelOutlineR[1] = v_modelOutline[1];
            v_modelOutlineR[2] = tan(occlusionU) * sqrt(pow(v_modelOutline[0], 2) + pow(v_modelOutline[1], 1));
        }
    }
    else // 异常
    {
        v_modelOutlineR = v_modelOutline;
        v_modelOutlineL = v_modelOutline;
        log_compnt_mngr->warn("Occlusion ID {} not found", id);
        // do nothing
    }
    log_compnt_mngr->info("Sensor::calcNearestCoord end.");
}

// 计算最近点
void Sensor::calcNearestInfo(int id, std::map<int, std::tuple<Vector3D,Vector3D,Vector3D,Vector3D>> m_outline, Vector3D sensorCoord, double yaw, double pitch, double roll, Vector3D& v_modelOutlineR, Vector3D& v_modelOutlineL)
{
    log_compnt_mngr->info("Sensor::calcNearestInfo start.");
    Vector3D v_modelOutline (0.0,0.0,0.0); // 相对距离最近的点
    if (m_outline.size() == 0)
    {
        log_compnt_mngr->error("Outline map is empty.");
        return ;
    }
    Vector3D v_adjacentPoint1(0.0,0.0,0.0); // 轮廓距离传感器最近点毗邻点1
    Vector3D v_adjacentPoint2(0.0,0.0,0.0); // 轮廓距离传感器最近点毗邻点2
    Vector3D v_adjacentPoint3(0.0,0.0,0.0); // 轮廓距离传感器最近点毗邻点3
    double nearestPoint = DBL_MAX;
    // 返回最近点坐标
    for (auto outlines : m_outline)
    {
        Vector3D outline = std::get<0>(outlines.second); // 取出当前轮廓点
        VectorRotated(yaw, pitch, roll, outline);
        outline = sensorCoord + outline; // 轮廓点在传感器坐标系下的坐标

        double tmp_nearestPoint = outline.length();
        // 找到相对距离最近的点
        if (tmp_nearestPoint < nearestPoint)
        {
            v_modelOutline = outline;
            nearestPoint = v_modelOutline.length();
        }
        else
        {
            log_compnt_mngr->warn("not find nearest point");
            continue;
        }

        // 找到相对距离最近的点毗邻的三个点
        v_adjacentPoint1 = std::get<1>(outlines.second); // 取出毗邻轮廓点1
        v_adjacentPoint2 = std::get<2>(outlines.second); // 取出毗邻轮廓点2
        v_adjacentPoint3 = std::get<3>(outlines.second); // 取出毗邻轮廓点3
    }
    log_compnt_mngr->debug("nearest Vertex = ({},{},{})", v_modelOutline[0], v_modelOutline[1], v_modelOutline[2]);

    VectorRotated(yaw, pitch, roll, v_adjacentPoint1);
    v_adjacentPoint1 = sensorCoord + v_adjacentPoint1; // 轮廓点在传感器坐标系下的坐标
    VectorRotated(yaw, pitch, roll, v_adjacentPoint2);
    v_adjacentPoint2 = sensorCoord + v_adjacentPoint2; // 轮廓点在传感器坐标系下的坐标
    VectorRotated(yaw, pitch, roll, v_adjacentPoint3);
    v_adjacentPoint3 = sensorCoord + v_adjacentPoint3; // 轮廓点在传感器坐标系下的坐标

    // 计算毗邻向量的点乘
    Vector3D AB (v_adjacentPoint1[0] - v_modelOutline[0], v_adjacentPoint1[1] - v_modelOutline[1], v_adjacentPoint1[2] - v_modelOutline[2]);
    Vector3D AC (v_adjacentPoint2[0] - v_modelOutline[0], v_adjacentPoint2[1] - v_modelOutline[1], v_adjacentPoint2[2] - v_modelOutline[2]);
    Vector3D AD (v_adjacentPoint3[0] - v_modelOutline[0], v_adjacentPoint3[1] - v_modelOutline[1], v_adjacentPoint3[2] - v_modelOutline[2]);

    // 计算最近点坐标
    calcNearestCoord(id, AB, AC, AD, v_modelOutline, v_modelOutlineR, v_modelOutlineL);
    log_compnt_mngr->debug("v_modelOutlineR = ({},{},{})", v_modelOutlineR[0], v_modelOutlineR[1], v_modelOutlineR[2]);
    log_compnt_mngr->debug("v_modelOutlineL = ({},{},{})", v_modelOutlineL[0], v_modelOutlineL[1], v_modelOutlineL[2]);
    log_compnt_mngr->info("Sensor::calcNearestInfo end.");
}


#ifdef ENABLE_YIQILOG //传感器最近点方案优化 pjd 2023.09.15
/**
 * @description: 获取传感器以z为0的平面与包围盒(传感器坐标系)的交点
 * @param {vector<Vector3D>} _vehBBoxVertexVectors  环境车的包围盒
 * @param {vector<Vector3D>} &intersectionPoints	传感器以z为0的平面与包围盒(传感器坐标系)的交点
 * @return {*} 传感器以z为0的平面与包围盒(传感器坐标系)的交点
 */
bool Sensor::Line_intersection(std::vector<Vector3D> _vehBBoxVertexVectors, std::vector<Vector3D> &intersectionPoints)
{
    log_compnt_mngr->info("Sensor::Line_intersection start.");
    if (egoState == nullptr)
    {
        log_compnt_mngr->error("egoState is nullptr");
        return false;
    }
    //主车世界坐标
    Transform egoTransform;
    egoTransform.update(egoState->sObjectState.sPos.u8X, egoState->sObjectState.sPos.u8Y, egoState->sObjectState.sPos.u8Z);

    Vector3D relativeCoord(0.0, 0.0, 0.0);
    Vector3D sensorRela(0.0, 0.0, 0.0);
    std::vector<Vector3D> vehBBoxVertexVector;
    Transform vehvector;

    for (unsigned int i = 0; i < _vehBBoxVertexVectors.size(); i++)
    {
        vehvector.v() = _vehBBoxVertexVectors[i];
        CoordTransform(egoState->sObjectState.sPos.u4H, egoState->sObjectState.sPos.u4P, egoState->sObjectState.sPos.u4R, egoTransform, vehvector, getAssemblePositionX(),getAssemblePositionY(), getAssemblePositionZ(), getHeading(), getPitch(), getRoll(), relativeCoord, sensorRela);
        vehBBoxVertexVector.push_back(sensorRela);
    }

    //0 左下前，1 左下后，2 右下前，3 右下后，4 左上前，5 左上后，6 右上前，7 右上后
    std::map<int, std::tuple<Vector3D,Vector3D>> m_Outline;  //包围盒的12条边
    //包围盒顶部四边
    m_Outline[0] = make_tuple(vehBBoxVertexVector[4],vehBBoxVertexVector[5]); // 4 左上前，5 左上后
    m_Outline[1] = make_tuple(vehBBoxVertexVector[5],vehBBoxVertexVector[7]); // 5 左上后，7 右上后
    m_Outline[2] = make_tuple(vehBBoxVertexVector[7],vehBBoxVertexVector[6]); // 6 右上前，7 右上后
    m_Outline[3] = make_tuple(vehBBoxVertexVector[6],vehBBoxVertexVector[4]); // 4 左上前，6 右上前

    //包围盒四条棱
    m_Outline[4] = make_tuple(vehBBoxVertexVector[4],vehBBoxVertexVector[0]); // 0 左下前, 4 左上前
    m_Outline[5] = make_tuple(vehBBoxVertexVector[5],vehBBoxVertexVector[1]); // 1 左下后, 5 左上后
    m_Outline[6] = make_tuple(vehBBoxVertexVector[7],vehBBoxVertexVector[3]); // 3 右下后，7 右上后
    m_Outline[7] = make_tuple(vehBBoxVertexVector[6],vehBBoxVertexVector[2]); // 2 右下前，6 右上前

    //包围盒底部四边
    m_Outline[8]  = make_tuple(vehBBoxVertexVector[0],vehBBoxVertexVector[1]); // 0 左下前, 1 左下后
    m_Outline[9]  = make_tuple(vehBBoxVertexVector[1],vehBBoxVertexVector[3]); // 1 左下后，3 右下后
    m_Outline[10] = make_tuple(vehBBoxVertexVector[3],vehBBoxVertexVector[2]); // 2 右下前，3 右下后
    m_Outline[11] = make_tuple(vehBBoxVertexVector[2],vehBBoxVertexVector[0]); // 0 左下前, 2 右下前

    //获取传感器以z为0的平面与包围盒(传感器坐标系)的交点
    for(unsigned int i = 0; i < m_Outline.size(); ++i)
    {
        const auto& point1 = std::get<0>(m_Outline[i]);
        const auto& point2 = std::get<1>(m_Outline[i]);

        if ((point1.z() >= 0 && point2.z() <= 0) || (point1.z() <= 0 && point2.z() >= 0))
            {
                // 计算当前边与传感器z为0的平面的交点
                if (point2.z() != point1.z() )
                {
                    double t = -point1.z() / (point2.z() - point1.z());
                    double x = point1.x() + t * (point2.x() - point1.x());
                    double y = point1.y() + t * (point2.y() - point1.y());

                    intersectionPoints.push_back({x, y, 0.0});
                }
            }
    }
    log_compnt_mngr->info("Sensor::Line_intersection end.");
    return true;
}

/**
 * @description: 计算当前ID被遮挡的弧度的直线和传感器以z为0的平面与包围盒(传感器坐标系)的相邻两交点构成直线交点
 * @param {double} Occlusion_angle 当前ID被遮挡的弧度
 * @param {Vector3D} point1 传感器以z为0的平面与包围盒(传感器坐标系)的交点
 * @param {Vector3D} point2	传感器以z为0的平面与包围盒(传感器坐标系)的交点中的与point1相邻的点
 * @return {*}两直线的交点
 */
Vector3D Sensor::GetSegment(double Occlusion_angle, Vector3D point1, Vector3D point2)
{
    log_compnt_mngr->info("Sensor::GetSegment start.");
    Vector3D _nearest_point;

    // 计算交点的位置
    //线段方程：A * X + B * Y + C =0  ,传感器被遮挡的弧度构成方程上的任意点(R * cos(Occlusion_angle),R * sin(Occlusion_angle))
    double A = point2.y() - point1.y();
    double B = point1.x() - point2.x();
    double C = point2.x() * point1.y() - point1.x() * point2.y();
    if ( A * cos(Occlusion_angle) + B * sin(Occlusion_angle) == 0)
    {
        point1.length() > point2.length() ? _nearest_point = point2 : _nearest_point = point1;
    }
    else
    {
        double R = -C /(A * cos(Occlusion_angle) + B * sin(Occlusion_angle));
        _nearest_point.x() = R * cos(Occlusion_angle);
        _nearest_point.y() = R * sin(Occlusion_angle);
        _nearest_point.z() = 0;
    }

    log_compnt_mngr->info("Sensor::GetSegment end.");
    return _nearest_point;
}

/**
 * @description: 计算传感器位置到包围盒交点中相邻的两边构成的线段的最近距离和最近点
 * @param {Vector3D} sensor_point 传感器坐标(传感器坐标系下)
 * @param {Vector3D} point1 传感器以z为0的平面与包围盒(传感器坐标系)的交点
 * @param {Vector3D} point2	传感器以z为0的平面与包围盒(传感器坐标系)的交点中的与point1相邻的点
 * @param {Vector3D} &point 传感器位置到包围盒交点中相邻的两边构成的线段的最近距离的最近点
 * @return {*} 传感器位置到包围盒交点中相邻的两边构成的线段的最近距离
 */
double Sensor::distanceToSegment(Vector3D sensor_point, Vector3D point1, Vector3D point2,Vector3D &point)
{
    double dx = point2.x() - point1.x();

    double dy = point2.y() - point1.y();

    double t = ((sensor_point.x() - point1.x()) * dx + (sensor_point.y() - point1.y()) * dy) / (dx * dx + dy * dy);
    log_compnt_mngr->debug("Sensor::distanceToSegment t ={}.",t);

    if (t < 0) //位于交点所构成的线段上方
    {
        point = { point1.x() - sensor_point.x() , point1.y() - sensor_point.y(),0};
        double distance = sqrt((sensor_point.x() - point1.x()) * (sensor_point.x() - point1.x()) + (sensor_point.y() - point1.y()) * (sensor_point.y() - point1.y()));
        return distance;

    }
    else if (t > 1) //位于交点所构成的线段下方
    {
        point = { point2.x() - sensor_point.x() , point2.y() - sensor_point.y(),0};
        double distance =sqrt((sensor_point.x() - point2.x()) * (sensor_point.x() - point2.x()) + (sensor_point.y() - point2.y()) * (sensor_point.y() - point2.y()));
        return distance;
    }
    else //位于交点所构成的线段上
    {
        point = { point1.x() + t * (point2.x() - point1.x()), point1.y() + t * (point2.y() - point1.y()) , 0};
        double distance = sqrt((sensor_point.x() - point.x()) * (sensor_point.x() - point.x()) + (sensor_point.y() - point.y()) * (sensor_point.y() - point.y()));
        return distance;
    }

}

/**
 * @description: 获取最近点和最近距离
 * @param {int} id 环境车的id
 * @param {vector<Vector3D>} &_pointVector 传感器以z为0的平面与包围盒(传感器坐标系)的所有交点
 * @param {Vector3D} &_nearest_point 传感器位置到 z为0的平面与包围盒(传感器坐标系)交点所在构成的多边形的最近点
 * @param {double} &mindistance	传感器位置到 z为0的平面与包围盒(传感器坐标系)交点所在构成的多边形的最近距离
 * @return {*}
 */
void Sensor::GetNearest_Point(int id ,std::vector<Vector3D> &_pointVector,Vector3D &_nearest_point,double &mindistance)
{
    log_compnt_mngr->info("Sensor::GetNearest_Point start.");
    mindistance = std::numeric_limits<double>::max();
    Vector3D Sensor_point(0.0,0.0,0.0); //传感器坐标(传感器坐标系下)
    Vector3D point(0.0,0.0,0.0); 		//最近点

    if (_pointVector.size() == 0) //若无交点则返回NAN
    {
        _nearest_point = NAN;
        mindistance = NAN;
        log_compnt_mngr->debug("Sensor::GetNearest_Point No intersections,return nan.");
        log_compnt_mngr->info("Sensor::GetNearest_Point end.");
        return ;
    }

    for (unsigned int i = 0; i < _pointVector.size(); i++)
    {
        Vector3D p1 = _pointVector[i];
        Vector3D p2 = _pointVector[(i + 1) % _pointVector.size()];

        double distance = distanceToSegment(Sensor_point,p1,p2,point);
        if (distance < mindistance)
        {
            mindistance = distance;
            _nearest_point = point;
        }
    }

    if (std::isnan(getOcclusion_point(id, _pointVector,  _nearest_point).length()))
    {
        mindistance = NAN ;
    }
    else
    {
        mindistance = _nearest_point.length();
    }
    log_compnt_mngr->info("Sensor::GetNearest_Point end.");

}

/**
 * @description: 计算遮挡后的最近点
 * @param {double} Occlusion_angle 当前ID被遮挡的弧度
 * @param {vector<Vector3D>} &_pointVector 传感器以z为0的平面与包围盒(传感器坐标系)的所有交点
 * @param {Vector3D} &_nearest_point 传感器位置在有遮挡情况下到z为0的平面与包围盒(传感器坐标系)交点所在构成的多边形的最近点
 * @return {*}
 */
void Sensor::GetNearest_boolPoint(double Occlusion_angle,std::vector<Vector3D> &_pointVector,Vector3D &_nearest_point)
{
    log_compnt_mngr->info("Sensor::GetNearest_boolPoint start.");
    double mindistance = std::numeric_limits<double>::max();
    if (_pointVector.size() == 0)
    {
        //若无交点则返回NAN
        _nearest_point = NAN;
        mindistance = NAN;
        log_compnt_mngr->debug("Sensor::GetNearest_boolPoint No intersections,return nan.");
        log_compnt_mngr->info("Sensor::GetNearest_boolPoint end.");
        return ;
    }

    for (unsigned int i = 0; i < _pointVector.size(); i++)
    {
        Vector3D p1 = _pointVector[i];
        Vector3D p2 = _pointVector[(i + 1) % _pointVector.size()];
        Vector3D point = GetSegment(Occlusion_angle,p1,p2);
        Vector3D AP = {point[0]-p1[0],point[1]-p1[1],point[2]-p1[2]};
        Vector3D BP = {point[0]-p2[0],point[1]-p2[1],point[2]-p2[2]};
        if (AP.dot(BP) > 0) // 交点在外
        {
            log_compnt_mngr->warn("Sensor::GetNearest_boolPoint intersection point is outside the segment.");
            continue;
        }
        else
        {
            double distance = point.length();
            if (distance < mindistance)
            {
                mindistance = distance;
                _nearest_point = point;
            }
        }
    }
    log_compnt_mngr->info("Sensor::GetNearest_boolPoint end.");
}

/**
 * @description: 获取遮挡后的最近点
 * @param {int} id 环境车的id
 * @param {vector<Vector3D>} &_pointVector  传感器以z为0的平面与包围盒(传感器坐标系)的所有交点
 * @param {Vector3D} &v_modelOutline 传感器位置在有遮挡情况下到z为0的平面与包围盒(传感器坐标系)交点所在构成的多边形的最近点
 * @return {*} 遮挡后的最近点
 */
Vector3D Sensor::getOcclusion_point(int id,  std::vector<Vector3D> &_pointVector,Vector3D &v_modelOutline)
{
    float targetHorRad = atan2(v_modelOutline[1],v_modelOutline[0]); 							// 最近点水平方向弧度值
    float range = std::sqrt(std::pow(v_modelOutline[0],2) + std::pow(v_modelOutline[1],2));
    float targetVerRad = atan(v_modelOutline[2]/range); 										// 最近点垂直方向弧度值

    // 获取当前ID被遮挡的弧度
    if (objOcclusionScale.find(id) != objOcclusionScale.end())
    {
        float occlusionR = std::get<3>(objOcclusionScale[id]);
        float occlusionL = std::get<4>(objOcclusionScale[id]);
        float occlusionB = std::get<5>(objOcclusionScale[id]);
        float occlusionU = std::get<6>(objOcclusionScale[id]);

        // 第一个物体无需修改最近点情况
        if (occlusionR == 0.0 && occlusionL == 0.0 && occlusionB == 0.0 && occlusionU == 0.0)
        {
            return v_modelOutline;
        }
        // 最近点是否被遮挡
        if (hasOcclusion(id, targetHorRad, targetVerRad, occlusionR, occlusionL, occlusionB, occlusionU))
        {
            Vector3D CRightVal (0.0,0.0,0.0);
            Vector3D CLeftVal (0.0,0.0,0.0);

            // 计算遮挡后的左右两侧的最近点
            calcCoord(id,_pointVector, occlusionR, occlusionL, CRightVal, CLeftVal);

            double len_CRightVal = pow(CRightVal[0], 2) + pow(CRightVal[1], 2) + pow(CRightVal[2], 2);
            double len_CLeftVal = pow(CLeftVal[0], 2) + pow(CLeftVal[1], 2) + pow(CLeftVal[2], 2);

            log_compnt_mngr->debug("CRightVal length = {}, CLeftVal length = {}", len_CRightVal.length(), len_CLeftVal.length());
            if( CRightVal.length() != 0 && CLeftVal.length() != 0 ) 	//左右两侧有最近点
            {
                CRightVal.length() > CLeftVal.length() ? v_modelOutline = CLeftVal: v_modelOutline = CRightVal;
                return v_modelOutline;
            }
            else if ( CLeftVal.length() == 0 && CRightVal.length() != 0) 	//右侧有最近点，左侧无
            {
                v_modelOutline = CRightVal;
                return v_modelOutline;
            }
            else if ( CRightVal.length() == 0  &&  CLeftVal.length() != 0)   // 左侧有最近点，右侧无
            {
                v_modelOutline = CLeftVal;
                return v_modelOutline;
            }
            else // 水平方向完全遮挡
            {
                v_modelOutline = NAN;
                return v_modelOutline;
            }
        }
        else //无遮挡
        {
            return v_modelOutline;
        }
    }
    else // 异常
    {
        log_compnt_mngr->error("Occlusion ID {} not found",id);
        // do nothing
        return v_modelOutline;
    }
    return v_modelOutline;
}

/**
 * @description: 计算遮挡后的最近点
 * @param {int} id 							环境车的id
 * @param {vector<Vector3D>} &_pointVector 	传感器以z为0的平面与包围盒(传感器坐标系)的所有交点
 * @param {float} occlusionR 				当前ID被遮挡的右侧弧度
 * @param {float} occlusionL 				当前ID被遮挡的左侧弧度
 * @param {Vector3D} &v_modelR				左侧最近点
 * @param {Vector3D} &v_modelL				左侧最近点
 * @return {*}
 */
void Sensor::calcCoord(int id, std::vector<Vector3D> &_pointVector, float occlusionR, float occlusionL, Vector3D &v_modelR, Vector3D &v_modelL)
{
    log_compnt_mngr->info("Sensor::calcCoord start.");
    if (agentFourPointRadMap.find(id) != agentFourPointRadMap.end())
    {
        float targetR = std::get<0>(agentFourPointRadMap[id]);
        float targetL = std::get<1>(agentFourPointRadMap[id]);

        if (occlusionR > targetR && occlusionR < targetL)
        {
            GetNearest_boolPoint(occlusionR,_pointVector,v_modelR);
        }

        if (occlusionL < targetL && occlusionL > targetR)
        {
            GetNearest_boolPoint(occlusionL,_pointVector,v_modelL);
        }
    }
    log_compnt_mngr->info("Sensor::calcCoord end.");
}
#endif

bool Sensor::getObjectList(std::vector<S_SP_SENSOR_DETECTION_INFO> &objects)
{
    log_compnt_mngr->info("Sensor::getObjectList start.");

    if (_description.enable == false)
    {
        log_compnt_mngr->error("Sensor::getObjectList enable=false.");
        return false;
    }

    if (egoState == nullptr)
    {
        log_compnt_mngr->error("Sensor::getObjectList Error1: egoState=nullptr.");
        return false;
    }

    //主车世界坐标
    Transform vehicleTransform;
    vehicleTransform.update(egoState->sObjectState.sPos.u8X, egoState->sObjectState.sPos.u8Y, egoState->sObjectState.sPos.u8Z);

    //[帧率控制数据传输架构整改]TODO: 所有用到sObjectState.sPos.u4H/P/R
    // 的地方可能会造成传感器移植到CM后输出结果有误差，
    // 因为之前用的姿态角为double类型
    double yaw = egoState->sObjectState.sPos.u4H;
    double pitch = egoState->sObjectState.sPos.u4P;
    double roll = egoState->sObjectState.sPos.u4R;
    double sensorRelative_x = getAssemblePositionX(); // 传感器在主车坐标系下的x位置
    double sensorRelative_y = getAssemblePositionY(); // 传感器在主车坐标系下的y位置
    double sensorRelative_z = getAssemblePositionZ(); // 传感器在主车坐标系下的z位置

    double detectedPct = 1.0; 					// 默认检测概率为1 - [存在可能性]
    float rcs = 6.5; 							// 目标雷达散射截面积 - [存在可能性]
    float occlusionScale = 0; 					// 对应物体ID的遮挡比例 - [存在可能性]
    Vector3D sphereCoord(0.0,0.0,0.0); 			// 球坐标系 - [误差模型]
    Vector3D randErrSensorCoord(0.0,0.0,0.0); 	// 混合模型误差坐标 - [误差模型]
    Vector3D gaussRandErr(0.0,0.0,0.0); 		// 混合模型高斯误差 - [误差模型]
    float insensitiveRadius = 1; 				// 目标不敏感半径范围 - [分类正确可能性]

    // 获取欧式距离最近点信息
    std::map<int, Vector3D> nearestObjRelCoord; // 目标欧氏距离最近物体 - [分类正确可能性]

    //车辆
    std::set<int> vehicle_ids;
    //获取需要输出车辆ids
    if (getObjectList(SENSOR_OBJECT_DETECTION_TYPE_VEHICLE, vehicle_ids) == true)
    {
        //遍历所有车辆
        for (auto vehIt = vehicleList.begin(); vehIt != vehicleList.end(); ++vehIt)
        {
            S_SP_MIL_OBJECT_STATE *objectState = *vehIt;
            if (objectState == nullptr)
            {
                log_compnt_mngr->warn("vehicle objectState=nullptr.");
                continue;
            }

            // 目标物ID
            uint32_t objectId = objectState->sObjectState.u4Id;

            //如果不在识别车辆列表中，则忽略
            if (vehicle_ids.find(static_cast<int>(objectId)) == vehicle_ids.end())
            {
                log_compnt_mngr->warn("vehicle objectId={} not in vehicle_ids.", objectId);
                continue;
            }

            Transform objectTransform;
            objectTransform.update(objectState->sObjectState.sPos.u8X, objectState->sObjectState.sPos.u8Y, objectState->sObjectState.sPos.u8Z);

            // 世界坐标系转传感器坐标系
            Vector3D relativeCoord(0, 0, 0);
            Vector3D sensorRela(0, 0, 0);
            CoordTransform(yaw, pitch, roll, vehicleTransform, objectTransform, getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(), getHeading(), getPitch(), getRoll(), relativeCoord, sensorRela);

            double sensor_distance = sqrt(std::pow(sensorRela[0], 2.0) + std::pow(sensorRela[1], 2.0)); // 计算目标物体与传感器的距离

            S_SP_SENSOR_DETECTION_INFO object = {0};
            object.u1Type = objectState->sObjectState.u1Type; //类型@D_SP_OBJECT_TYPE
            object.u2Flags = D_SP_SENSOR_OBJECT_FLAG_NONE;
            object.u4Id = objectId;
            object.u4SensorId = 0; //TODO

            // [一汽]混合传感器 [Modify] LS 23.04.17
            if (_description.modelType == SENSOR_MODEL_IDEAL)
            {
                // 对于理想传感器，这两项值均为100%
                object.u8TypeIdProb = 1.0;
            }
            else if (_description.modelType == SENSOR_MODEL_PROBABLY)
            {
                if(_description.type == SENSOR_TYPE_CAMERA)	// 对于相机
                {
                    object.u8TypeIdProb = 1.0;
                    sensorRela = calDistCoord(sensorRela);
                }
                else	// 对于其他传感器
                {
                    /* 转球坐标 */
                    getSphereCoord(sensorRela[0], sensorRela[1], sensorRela[2], sphereCoord);

                    double rangeGaussRand = getGaussRand(getRangeMSENoise()) + getEnvNoise(sensor_distance);
                    double azimuthGaussRand = getGaussRand(getAzimuthMSENoise()) * SENSOR_OBJECT_DETECTION_PI / 180.0;    // 水平角
                    double elevationGaussRand = getGaussRand(getElevationMSENoise()) * SENSOR_OBJECT_DETECTION_PI / 180.0;

                    // 增加维度高斯误差后转笛卡尔坐标系
                    getCartesianCoord(sphereCoord[0] + rangeGaussRand, sphereCoord[1] + azimuthGaussRand, sphereCoord[2] + elevationGaussRand, randErrSensorCoord);
                    log_compnt_mngr->trace("rangeGaussRand, azimuthGaussRand elevationGaussRand {},{},{}", rangeGaussRand,azimuthGaussRand,elevationGaussRand);
                    gaussRandErr = randErrSensorCoord - sensorRela;
                    sensorRela = randErrSensorCoord;
                }
            }

            log_compnt_mngr->trace("GaussRandSensorX, GaussRandSensorY GaussRandSensorZ {},{},{}", sensorRela[0],sensorRela[1],sensorRela[2]);
            object.sExtraInfo.sDynamicObj.sSensorPos.u8X =  static_cast<float>(sensorRela[0]);
            object.sExtraInfo.sDynamicObj.sSensorPos.u8Y =  static_cast<float>(sensorRela[1]);
            object.sExtraInfo.sDynamicObj.sSensorPos.u8Z =  static_cast<float>(sensorRela[2]);

            sensor_distance = sqrt(std::pow(object.sExtraInfo.sDynamicObj.sSensorPos.u8X, 2.0) + std::pow(object.sExtraInfo.sDynamicObj.sSensorPos.u8Y, 2.0));

            // 当高级设置开关打开时，计算置信度
            if (isEnableAdvSettings() && (_description.type != SENSOR_TYPE_CAMERA))
            {
                // 计算天气影响因子
                double weatherPct = calcWeatherNoise(_description.type, sensor_distance);

                // bug7385 置信度增加遮挡比例的影响 LS 22.06.17
                if (objOcclusionScale.find(static_cast<int>(objectId)) != objOcclusionScale.end())
                {
                    occlusionScale = std::get<0>(objOcclusionScale[static_cast<int>(objectId)]);
                }
                // 计算检测概率getDetectedPR（距离，毫米波雷达RCS，天气因子）
                detectedPct = getDetectedPR(sensor_distance, rcs, weatherPct, occlusionScale);

                // 目标分类正确可能性
                if (nearestObjRelCoord.size() == 0)
                {
                    getNearestObjRelCoordToTarget(insensitiveRadius,nearestObjRelCoord);
                }
                log_compnt_mngr->trace("nearestObjRelCoord.size {}", nearestObjRelCoord.size());
                object.u8TypeIdProb = getClassificationAccuracy(static_cast<int>(objectId), nearestObjRelCoord);

                // 混合传感器 需求对应变更（只有毫米波雷达出现虚警目标）LS 23.05.18
                if (_description.type == SENSOR_TYPE_RADAR)
                {
                    // 相对主车的姿态角
                    Vector3D relOri(objectState->sObjectState.sPos.u4H - yaw - getHeading(), objectState->sObjectState.sPos.u4P - pitch - getPitch(), objectState->sObjectState.sPos.u4R - roll - getRoll());
                    // 有效性标识
                    getValidityLabel(static_cast<int>(objectId), sensorRela, relOri);
                }
            }
            log_compnt_mngr->debug("detectedPct =  occlusionScale = {},{}", detectedPct, occlusionScale);
            // 有效性标识
            object.u1ValidFlag = 1.0;
            // 目标存在的可能性
            object.u8ExistProb = detectedPct; // 概率传感器置信度

            /* Tang Zentao10745 添加航向角、相对速度、相对加速度字段[start]*/
            // [一汽]混合传感器 [Modify] LS 23.04.17
            auto vehVelToEgo = getRelaVelToEgo(objectState);
            auto vehAccToEgo = getRelaAccToEgo(objectState);

            object.sExtraInfo.sDynamicObj.u8HeadingAngle = objectState->sObjectState.sPos.u4H;
            object.sExtraInfo.sDynamicObj.u8RelativeVelx = vehVelToEgo.x();
            object.sExtraInfo.sDynamicObj.u8RelativeVely = vehVelToEgo.y();
            object.sExtraInfo.sDynamicObj.u8RelativeAccx = vehAccToEgo.x();
            object.sExtraInfo.sDynamicObj.u8RelativeAccy = vehAccToEgo.y();
            /* Tang Zentao10745 添加航向角、相对速度、相对加速度字段[end]*/

            object.sExtraInfo.sDynamicObj.u8Dist = sqrt(std::pow(object.sExtraInfo.sDynamicObj.sSensorPos.u8X, 2.0) + std::pow(object.sExtraInfo.sDynamicObj.sSensorPos.u8Y, 2.0));

#ifdef ENABLE_YIQILOG  //传感器最近点方案优化 pjd 2023.09.15
            Vector3D vSensorCoorTrans(0.0,0.0,0.0);  										//最近点
            double Closest_distance = 0.0;	    											//最近距离
            std::vector<Vector3D> m_Outline; 												//传感器以z为0的平面与环境车的包围盒(传感器坐标系)的交点
            std::vector<Vector3D> vehBBoxVertexVectors; 	//获取环境车包围盒八个顶点信息
            computeVehicleObstacleBBox(objectState, vehBBoxVertexVectors);
            if (Line_intersection(vehBBoxVertexVectors,m_Outline))
            {
                GetNearest_Point(static_cast<int>(objectId),m_Outline,vSensorCoorTrans,Closest_distance);
            }
#else
            Vector3D vEgoCoorTrans(0,0,0);
            Vector3D vSensorCoorTrans(0,0,0);

            std::map<int, std::tuple<Vector3D,Vector3D,Vector3D,Vector3D>> m_upperOutline; // 目标上部特征点
            std::map<int, std::tuple<Vector3D,Vector3D,Vector3D,Vector3D>> m_bottomOutline; // 目标下部特征点

            Vector3D paramScale(objectState->au8ParamScale[0], objectState->au8ParamScale[1], objectState->au8ParamScale[2]); //车模长宽高缩放比
            if (fillTwoPartOutlineMap(objectState->au1ModelName, m_upperOutline, m_bottomOutline, paramScale))
            {
                // 获取目标物体的特征点最近点
                Vector3D v_upOutlineR(DBL_MAX,DBL_MAX,DBL_MAX); 	// 轮廓距离传感器右侧最近点
                Vector3D v_upOutlineL(DBL_MAX,DBL_MAX,DBL_MAX); 	// 轮廓距离传感器左侧最近点
                Vector3D v_btmOutlineR(DBL_MAX,DBL_MAX,DBL_MAX); 	// 轮廓距离传感器右侧最近点
                Vector3D v_btmOutlineL(DBL_MAX,DBL_MAX,DBL_MAX); 	// 轮廓距离传感器左侧最近点
                double agentYaw 	= objectState->sObjectState.sPos.u4H - yaw - getHeading(); 			// 目标相对传感器的航向角
                double agentPitch 	= objectState->sObjectState.sPos.u4P - pitch - getPitch(); 	// 目标相对传感器的俯仰角
                double agentRoll	= objectState->sObjectState.sPos.u4R - roll - getRoll();	 	// 目标相对传感器的横滚角

                double nearDistUp = 0.0;
                double nearDistBtm = 0.0;
                // 计算模型上半部分遮挡后最近点
                calcNearestInfo(static_cast<int>(objectId), m_upperOutline, sensorRela, agentYaw, agentPitch, agentRoll, v_upOutlineR, v_upOutlineL); // 模型上部分最近点
                // 计算模型下半部分遮挡后最近点
                calcNearestInfo(static_cast<int>(objectId), m_bottomOutline, sensorRela, agentYaw, agentPitch, agentRoll, v_btmOutlineR, v_btmOutlineL); // 模型下部分最近点

                double d_upOutlineR = v_upOutlineR.length();
                double d_upOutlineL = v_upOutlineL.length();
                double d_btmOutlineR = v_btmOutlineR.length();
                double d_btmOutlineL = v_btmOutlineL.length();
                log_compnt_mngr->debug("d_upOutlineR = {},  d_upOutlineL = {}, d_btmOutlineR = {},  d_btmOutlineL = {}", d_upOutlineR, d_upOutlineL, d_btmOutlineR, d_btmOutlineL);
                // 取四个值的最小值
                double min = numeric_limits<double>::max();
                if (d_upOutlineR != 0)
                {
                    min = d_upOutlineR;
                    object.sExtraInfo.sDynamicObj.u8NearestDist = d_upOutlineR;
                    vSensorCoorTrans = v_upOutlineR;
                }
                if (min > d_upOutlineL && d_upOutlineL != 0)
                {
                    min = d_upOutlineL;
                    object.sExtraInfo.sDynamicObj.u8NearestDist = d_upOutlineL;
                    vSensorCoorTrans = v_upOutlineL;
                }
                if (min > d_btmOutlineR && d_btmOutlineR != 0)
                {
                    min = d_btmOutlineR;
                    object.sExtraInfo.sDynamicObj.u8NearestDist = d_btmOutlineR;
                    vSensorCoorTrans = v_btmOutlineR;
                }
                if (min > d_btmOutlineL && d_btmOutlineL != 0)
                {
                    min = d_btmOutlineL;
                    object.sExtraInfo.sDynamicObj.u8NearestDist = d_btmOutlineL;
                    vSensorCoorTrans = v_btmOutlineL;
                }

                log_compnt_mngr->debug("NearestSensorPos = ({},{},{})", vSensorCoorTrans[0],vSensorCoorTrans[1],vSensorCoorTrans[2]);
            }
            else
            {
                object.sExtraInfo.sDynamicObj.u8NearestDist = getObjNearestCoordInfo(objectState, sensorRelative_x, sensorRelative_y, sensorRelative_z, getHeading(), getPitch(), getRoll(), vEgoCoorTrans, vSensorCoorTrans);
            }
#endif

            if(_description.type == SENSOR_TYPE_CAMERA)
            {
#ifdef ENABLE_YIQILOG  //传感器最近点方案优化 pjd 2023.09.15
                Vector3D vSensorCoorTran = calDistCoord(vSensorCoorTrans);
                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8X =  vSensorCoorTran[0];
                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8Y =  vSensorCoorTran[1];
                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8Z =  vSensorCoorTrans[2];
                object.sExtraInfo.sDynamicObj.u8NearestDist = sqrt(vSensorCoorTran[0] * vSensorCoorTran[0] + vSensorCoorTran[1] * vSensorCoorTran[1] + vSensorCoorTrans[2] * vSensorCoorTrans[2]);
#else
                vSensorCoorTrans = calDistCoord(vSensorCoorTrans);
                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8X =  vSensorCoorTrans[0];
                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8Y =  vSensorCoorTrans[1];
                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8Z =  vSensorCoorTrans[2];
#endif
            }
            else
            {
#ifdef ENABLE_YIQILOG   //传感器最近点方案优化 pjd 2023.09.15
                vSensorCoorTrans = {vSensorCoorTrans[0] + gaussRandErr[0],vSensorCoorTrans[1] + gaussRandErr[1],vSensorCoorTrans[2]};
                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8X = vSensorCoorTrans[0];
                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8Y = vSensorCoorTrans[1];
                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8Z = vSensorCoorTrans[2];
                object.sExtraInfo.sDynamicObj.u8NearestDist = vSensorCoorTrans.length();
#else
                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8X =  vSensorCoorTrans[0] + gaussRandErr[0];
                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8Y =  vSensorCoorTrans[1] + gaussRandErr[1];
                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8Z =  vSensorCoorTrans[2] + gaussRandErr[2];
#endif
            }

            object.u1Occlusion = 0;

            //目标物的速度
            double objectVelo = sqrt(pow(objectState->sObjectState.sSpeed.u8X, 2) + pow(objectState->sObjectState.sSpeed.u8Y, 2));
            if (fabs(objectVelo) > 1e-3)
            {
                object.sExtraInfo.sDynamicObj.u1MovingSt = 1;
            }
            else
            {
                object.sExtraInfo.sDynamicObj.u1MovingSt = 0;
            }

#ifdef ENABLE_YIQILOG /* pjd 2023.12.06 [bug22151] [一汽X轴和Y轴两侧最近点采用包围围盒（8顶点，包含后视镜）计算；非一汽采用车辆模型轮廓（16点，不包含后视镜）] [CHG] [START] */
            auto xyNearPos = getXYNearestPois(objectState, gaussRandErr);
            if(_description.type == SENSOR_TYPE_CAMERA)
            {
                xyNearPos[0] = calDistCoord(xyNearPos[0]);
                xyNearPos[1] = calDistCoord(xyNearPos[1]);
                xyNearPos[2] = calDistCoord(xyNearPos[2]);
                xyNearPos[3] = calDistCoord(xyNearPos[3]);
            }
            else
            {
                /* do nothing */
            }
			for(auto &it : xyNearPos)
			{
				log_compnt_mngr->trace("Sensor::getObjectListIdeal XYNearest Pos=({},{},{})",it.x(), it.y(), it.z());
			}
            float nan = sqrt(-1);
#else
            auto xyNearPos = getXYNearestPois(objectState, gaussRandErr, sensorRela);
            if(_description.type == SENSOR_TYPE_CAMERA)
            {
                xyNearPos[0] = calDistCoord(xyNearPos[0]);
                xyNearPos[1] = calDistCoord(xyNearPos[1]);
                xyNearPos[2] = calDistCoord(xyNearPos[2]);
                xyNearPos[3] = calDistCoord(xyNearPos[3]);
            }
            else
            {
                /* do nothing */
            }
           for(auto &it : xyNearPos)
           {
                log_compnt_mngr->trace("Sensor::getObjectListIdeal XYNearest Pos=({},{},{})",it.x(), it.y(), it.z());
           }
            float nan = sqrt(-1);
#endif /* pjd 2023.12.06 [bug22151] [一汽X轴和Y轴两侧最近点采用包围围盒（8顶点，包含后视镜）计算；非一汽采用车辆模型轮廓（16点，不包含后视镜）] [CHG] [END] */

            /* CJW 2023.01.04 理想传感器输出优化[start]*/
            object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u8X = xyNearPos[0].x();
            object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u8Y = xyNearPos[0].y();
            object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u8Z = 0.0;
            object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u4H = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u4P = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u4R = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u8X = xyNearPos[1].x();
            object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u8Y = xyNearPos[1].y();
            object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u8Z = 0.0;
            object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u4H = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u4P = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u4R = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u8X = xyNearPos[2].x();
            object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u8Y = xyNearPos[2].y();
            object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u8Z = 0.0;
            object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u4H = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u4P = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u4R = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u8X = xyNearPos[3].x();
            object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u8Y = xyNearPos[3].y();
            object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u8Z = 0.0;
            object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u4H = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u4P = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u4R = nan;
            /* CJW 2023.01.04 理想传感器输出优化[end]*/

            //理想和混合传感器输出BoundingBox
            for (int _pointi =0; _pointi < BOUNDINGBOX_VERTEX_NUM; _pointi++)
            {
                S_SP_POINT3D tmpPoint = {nan, nan, nan};
                object.sExtraInfo.sDynamicObj.sBoundingBox[_pointi] = tmpPoint;
            }

            if (_description.modelType == SENSOR_MODEL_IDEAL || _description.modelType == SENSOR_MODEL_PROBABLY)
            {
                if(_description.boundingBoxEnable)
                {
                    std::vector<Vector3D> vehBBoxVertexVector;
                    computeVehicleObstacleBBox(objectState, vehBBoxVertexVector); //获取环境车包围盒八个顶点信息

                    if (vehBBoxVertexVector.size()==BOUNDINGBOX_VERTEX_NUM)
                    {
                        for (int _pointi =0; _pointi < vehBBoxVertexVector.size(); _pointi++)
                        {
                            S_SP_POINT3D tmpPoint = {vehBBoxVertexVector[_pointi].x(), vehBBoxVertexVector[_pointi].y(), vehBBoxVertexVector[_pointi].z()};
                            object.sExtraInfo.sDynamicObj.sBoundingBox[_pointi] = tmpPoint;
                        }
                    }
                }
            }
            objects.push_back(object);
        }
        if (getInvalidObjectInfo(ghostInfo) == true)
        {
            //遍历所有虚警目标
            for (int i = 0; i < ghostInfo.size(); i++)
            {
                S_SP_SENSOR_DETECTION_INFO object = {0};
                object.u1Type = std::get<1>(ghostInfo[i]);
                object.u2Flags = D_SP_SENSOR_OBJECT_FLAG_NONE;
                object.u4Id = -std::get<0>(ghostInfo[i]);
                object.u4SensorId = 0; //TODO

                /* 增加交通灯类型 */
                if (object.u1Type == D_SP_OBJECT_TYPE_TRAFFIC_LIGHT || object.u1Type == D_SP_OBJECT_TYPE_TRAFFIC_SIGN)
                {
                    object.sExtraInfo.sSignObj.sSensorPos.u8X =  static_cast<float>(std::get<2>(ghostInfo[i]).x());
                    object.sExtraInfo.sSignObj.sSensorPos.u8Y =  static_cast<float>(std::get<2>(ghostInfo[i]).y());
                    object.sExtraInfo.sSignObj.sSensorPos.u8Z =  static_cast<float>(std::get<2>(ghostInfo[i]).z());
                    object.sExtraInfo.sSignObj.u8Dist = sqrt(std::pow(object.sExtraInfo.sSignObj.sSensorPos.u8X, 2.0) + std::pow(object.sExtraInfo.sSignObj.sSensorPos.u8Y, 2.0));
                }
                else
                {
                    object.sExtraInfo.sDynamicObj.sSensorPos.u8X =  static_cast<float>(std::get<2>(ghostInfo[i]).x());
                    object.sExtraInfo.sDynamicObj.sSensorPos.u8Y =  static_cast<float>(std::get<2>(ghostInfo[i]).y());
                    object.sExtraInfo.sDynamicObj.sSensorPos.u8Z =  static_cast<float>(std::get<2>(ghostInfo[i]).z());
                    object.sExtraInfo.sDynamicObj.u8Dist = sqrt(std::pow(object.sExtraInfo.sDynamicObj.sSensorPos.u8X, 2.0) + std::pow(object.sExtraInfo.sDynamicObj.sSensorPos.u8Y, 2.0));
                }

                // 目标存在的可能性
                object.u8ExistProb = 0;
                // 目标分类正确的可能性
                object.u8TypeIdProb = 0;
                // 有效性标识
                object.u1ValidFlag = 0;

                if (object.u1Type != SENSOR_OBJECT_DETECTION_TYPE_OBSTACLE)
                    objects.push_back(object);
            }
        }
    }

    //行人
    std::set<int> pedestrain_ids;
    rcs = 0.9; //对于行人雷达散射截面积
    insensitiveRadius = 0.1;
    //获取需要输出行人ids
    if (getObjectList(SENSOR_OBJECT_DETECTION_TYPE_PEDESTRIAN, pedestrain_ids) == true)
    {
        //遍历所有行人
        for (auto pedIt = pedestrianList.begin(); pedIt != pedestrianList.end(); ++pedIt)
        {
            S_SP_MIL_OBJECT_STATE *objectState = *pedIt;
            if (objectState == nullptr)
            {
                log_compnt_mngr->warn("pedestrain objectState=nullptr.");
                continue;
            }

            // 目标物ID
            uint32_t objectId = objectState->sObjectState.u4Id;

            //如果不在识别行人列表中，则忽略
            if (pedestrain_ids.find(static_cast<int>(objectId)) == pedestrain_ids.end())
            {
                log_compnt_mngr->warn("pedestrain objectId={} not in pedestrain_ids.", objectId);
                continue;
            }

            Transform objectTransform;
            objectTransform.update(objectState->sObjectState.sPos.u8X, objectState->sObjectState.sPos.u8Y, objectState->sObjectState.sPos.u8Z);

            // 世界坐标系转传感器坐标系
            Vector3D relativeCoord(0, 0, 0);
            Vector3D sensorRela(0, 0, 0);
            CoordTransform(yaw, pitch, roll, vehicleTransform, objectTransform, getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(), getHeading(), getPitch(), getRoll(), relativeCoord, sensorRela);

            double sensor_distance = sqrt(std::pow(sensorRela[0], 2.0) + std::pow(sensorRela[1], 2.0)); // 计算目标物体与传感器的距离

            S_SP_SENSOR_DETECTION_INFO object = {0};

            object.u1Type = objectState->sObjectState.u1Type; //类型@D_SP_OBJECT_TYPE
            // 对于理想传感器，存在可能性和分类正确性均为100%
            object.u2Flags = D_SP_SENSOR_OBJECT_FLAG_NONE;
            object.u4Id = objectId;
            object.u4SensorId = 0; //TODO

            // [一汽]混合传感器 [Modify] LS 23.04.17
            if (_description.modelType == SENSOR_MODEL_IDEAL)
            {
                // 对于理想传感器，这两项值均为100%
                object.u8TypeIdProb = 1.0;
            }
            else if (_description.modelType == SENSOR_MODEL_PROBABLY)
            {
                if (_description.type == SENSOR_TYPE_CAMERA)	// 相机的畸变计算
                {
                    object.u8TypeIdProb = 1.0;
                    sensorRela = calDistCoord(sensorRela);
                }
                else	// 其他传感器的畸变计算
                {
                    /* 转球坐标 */
                    getSphereCoord(sensorRela[0], sensorRela[1], sensorRela[2], sphereCoord);
                    // 增加维度高斯误差
                    double rangeGaussRand = getGaussRand(getRangeMSENoise()) + getEnvNoise(sensor_distance);
                    double azimuthGaussRand = getGaussRand(getAzimuthMSENoise()) * SENSOR_OBJECT_DETECTION_PI / 180.0;  // 水平角
                    double elevationGaussRand = getGaussRand(getElevationMSENoise()) * SENSOR_OBJECT_DETECTION_PI / 180.0;
                    // 增加维度高斯误差后转笛卡尔坐标系
                    getCartesianCoord(sphereCoord[0] + rangeGaussRand, sphereCoord[1] + azimuthGaussRand, sphereCoord[2] + elevationGaussRand, randErrSensorCoord);
                    log_compnt_mngr->trace("rangeGaussRand, azimuthGaussRand elevationGaussRand {},{},{}", rangeGaussRand,azimuthGaussRand,elevationGaussRand);
                    gaussRandErr = randErrSensorCoord - sensorRela;
                    sensorRela = randErrSensorCoord;
                }
            }
            log_compnt_mngr->debug("GaussRandSensorX, GaussRandSensorY GaussRandSensorZ {},{},{}", sensorRela[0],sensorRela[1],sensorRela[2]);

            object.sExtraInfo.sDynamicObj.sSensorPos.u8X =  static_cast<float>(sensorRela[0]);
            object.sExtraInfo.sDynamicObj.sSensorPos.u8Y =  static_cast<float>(sensorRela[1]);
            object.sExtraInfo.sDynamicObj.sSensorPos.u8Z =  static_cast<float>(sensorRela[2]);

            sensor_distance = sqrt(std::pow(object.sExtraInfo.sDynamicObj.sSensorPos.u8X, 2.0) + std::pow(object.sExtraInfo.sDynamicObj.sSensorPos.u8Y, 2.0));

            // 当高级设置开关打开时，计算置信度
            /*  相机不计算目标存在可能性等参数 */
            if (isEnableAdvSettings() && (_description.type != SENSOR_TYPE_CAMERA))
            {
                // bug15947 [Mod] 代码规范：计算天气影响因子 LS 23.05.15
                double weatherPct = calcWeatherNoise(_description.type, sensor_distance);

                // bug7385 置信度增加遮挡比例的影响 LS 22.06.17
                if (objOcclusionScale.find(static_cast<int>(objectId)) != objOcclusionScale.end())
                {
                    // [一汽需求] 遮挡增加Z维度 23.06.21 LS <id,<occlusionScale, horOcclusionScale, verOcclusionScale, occupiedRightRad, occupiedLeftRad, occupiedBottomRad, occupiedUpperRad>>
                    occlusionScale = std::get<0>(objOcclusionScale[static_cast<int>(objectId)]);
                }
                // 计算检测概率getDetectedPR（距离，毫米波雷达RCS，天气因子）
                detectedPct = getDetectedPR(sensor_distance, rcs, weatherPct, occlusionScale);

                // 目标分类正确可能性
                if (nearestObjRelCoord.size() == 0)
                {
                    getNearestObjRelCoordToTarget(insensitiveRadius, nearestObjRelCoord);
                }
                object.u8TypeIdProb = getClassificationAccuracy(static_cast<int>(objectId), nearestObjRelCoord);
            }
            // 有效性标识
            object.u1ValidFlag = 1.0;
            // 目标存在的可能性
            object.u8ExistProb = detectedPct; // 概率传感器置信度
            log_compnt_mngr->debug("detectedPct =  occlusionScale = {},{}", detectedPct, occlusionScale);

            object.sExtraInfo.sDynamicObj.u8Dist = sqrt(std::pow(object.sExtraInfo.sDynamicObj.sSensorPos.u8X, 2.0) + std::pow(object.sExtraInfo.sDynamicObj.sSensorPos.u8Y, 2.0));

            Vector3D vEgoCoorTrans(0,0,0);
            Vector3D vSensorCoorTrans(0,0,0);
            object.sExtraInfo.sDynamicObj.u8NearestDist = getObjNearestCoordInfo(objectState, sensorRelative_x, sensorRelative_y, sensorRelative_z, getHeading(), getPitch(), getRoll(), vEgoCoorTrans, vSensorCoorTrans);

            /* 为相机最近点的输出增加畸变和噪声*/
            if(_description.type == SENSOR_TYPE_CAMERA)
            {
                vSensorCoorTrans = calDistCoord(vSensorCoorTrans);

                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8X =  static_cast<float>(vSensorCoorTrans[0]);
                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8Y =  static_cast<float>(vSensorCoorTrans[1]);
                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8Z =  static_cast<float>(vSensorCoorTrans[2]);
            }
            else
            {
            
                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8X =  static_cast<float>(vSensorCoorTrans[0] + gaussRandErr[0]);
                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8Y =  static_cast<float>(vSensorCoorTrans[1] + gaussRandErr[1]);
                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8Z =  static_cast<float>(vSensorCoorTrans[2] + gaussRandErr[2]);
            }
            object.u1Occlusion = 0;

            /* Tang Zentao10745 添加航向角、相对速度、相对加速度字段[start]*/
            auto pedVelToEgo = getRelaVelToEgo(objectState);
            auto pedAccToEgo = getRelaAccToEgo(objectState);
            object.sExtraInfo.sDynamicObj.u8HeadingAngle = objectState->sObjectState.sPos.u4H;
            object.sExtraInfo.sDynamicObj.u8RelativeVelx = pedVelToEgo.x();
            object.sExtraInfo.sDynamicObj.u8RelativeVely = pedVelToEgo.y();
            object.sExtraInfo.sDynamicObj.u8RelativeAccx = pedAccToEgo.x();
            object.sExtraInfo.sDynamicObj.u8RelativeAccy = pedAccToEgo.y();
            /* Tang Zentao10745 添加航向角、相对速度、相对加速度字段[end]*/

            /* Tang 2022.10.31 一汽一阶段，拆分传感器[start]*/
            //目标物的速度
            double objectVelo = sqrt(pow(objectState->sObjectState.sSpeed.u8X, 2) + pow(objectState->sObjectState.sSpeed.u8Y, 2));
            if (fabs(objectVelo) > 1e-3)	// 速度小于0.001视为不在移动
            {
                object.sExtraInfo.sDynamicObj.u1MovingSt = 1;
            }
            else
            {
                object.sExtraInfo.sDynamicObj.u1MovingSt = 0;
            }

            std::vector<Vector3D> xyNearPos = getXYNearestPois(objectState, gaussRandErr);

            float nan = sqrt(-1);

            /* Tang 2023.5.17 zentao15936 为相机最近点的输出增加畸变和噪声[start] */
            if(_description.type == SENSOR_TYPE_CAMERA)
            {
                xyNearPos[0] = calDistCoord(xyNearPos[0]);
                xyNearPos[1] = calDistCoord(xyNearPos[1]);
                xyNearPos[2] = calDistCoord(xyNearPos[2]);
                xyNearPos[3] = calDistCoord(xyNearPos[3]);
            }
            else
            {
                /* do nothing */
            }
            /* Tang 2023.5.17 zentao15936 为相机最近点的输出增加畸变和噪声[end] */

            object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u8X = xyNearPos[0].x();
            object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u8Y = xyNearPos[0].y();
            object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u8Z = 0.0;
            object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u4H = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u4P = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u4R = nan;

            object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u8X = xyNearPos[1].x();
            object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u8Y = xyNearPos[1].y();
            object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u8Z = 0.0;
            object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u4H = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u4P = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u4R = nan;

            object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u8X = xyNearPos[2].x();
            object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u8Y = xyNearPos[2].y();
            object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u8Z = 0.0;
            object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u4H = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u4P = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u4R = nan;

            object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u8X = xyNearPos[3].x();
            object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u8Y = xyNearPos[3].y();
            object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u8Z = 0.0;
            object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u4H = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u4P = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u4R = nan;
            /* Tang 2022.10.31 一汽一阶段，拆分传感器[end]*/

            //理想和混合传感器输出BoundingBox
            for (int _pointi =0; _pointi < BOUNDINGBOX_VERTEX_NUM; _pointi++)
            {
                S_SP_POINT3D tmpPoint = {nan, nan, nan};
                object.sExtraInfo.sDynamicObj.sBoundingBox[_pointi] = tmpPoint;
            }

            if (_description.modelType == SENSOR_MODEL_IDEAL || _description.modelType == SENSOR_MODEL_PROBABLY)
            {
                if(_description.boundingBoxEnable)
                {
                    std::vector<Vector3D> pedBBoxVertexVector;
                    computePedestrianBBox(objectState, pedBBoxVertexVector); //获取行人包围盒八个顶点信息
                    if (pedBBoxVertexVector.size()==BOUNDINGBOX_VERTEX_NUM)
                    {
                        for (int _pointi =0; _pointi < pedBBoxVertexVector.size(); _pointi++)
                        {
                            S_SP_POINT3D tmpPoint = {pedBBoxVertexVector[_pointi].x(), pedBBoxVertexVector[_pointi].y(), pedBBoxVertexVector[_pointi].z()};
                            object.sExtraInfo.sDynamicObj.sBoundingBox[_pointi] = tmpPoint;
                        }
                    }
                }
            }
            objects.push_back(object);
        }
    }

    //障碍物
    std::set<int> obstacle_ids;
    rcs = 0.3; //对于障碍物雷达散射截面积
    insensitiveRadius = 0.05;
    //获取需要输出障碍物IDs
    if (getObjectList(SENSOR_OBJECT_DETECTION_TYPE_OBSTACLE, obstacle_ids) == true)
    {
        //遍历所有障碍物
        for (auto miscIt = obstacleList.begin(); miscIt != obstacleList.end(); ++miscIt)
        {
            S_SP_MIL_OBJECT_STATE *objectState = *miscIt;
            if (objectState == nullptr)
            {
                log_compnt_mngr->warn("obstacle objectState=nullptr.");
                continue;
            }

            // 目标物ID
            uint32_t objectId = objectState->sObjectState.u4Id;

            //如果不在识别障碍物列表中，则忽略
            if (obstacle_ids.find(static_cast<int>(objectId)) == obstacle_ids.end())
            {
                log_compnt_mngr->warn("obstacle objectId={} not in obstacle_ids.", objectId);
                continue;
            }

            // bug 34020 标线和停车位，仅相机输出且相机勾选道路标志
            if (objectState->sObjectState.u1Type == D_SP_OBJECT_TYPE_GRATICULE || objectState->sObjectState.u1Type == D_SP_OBJECT_TYPE_PARKING_SPACE)
            {
                log_compnt_mngr->warn("obstacle objectId={} type is graticule or parking space.", objectId);
                continue;
            }

            Transform objectTransform;
            objectTransform.update(objectState->sObjectState.sPos.u8X, objectState->sObjectState.sPos.u8Y, objectState->sObjectState.sPos.u8Z);

            // 世界坐标系转传感器坐标系
            Vector3D relativeCoord(0, 0, 0);
            Vector3D sensorRela(0, 0, 0);
            CoordTransform(yaw, pitch, roll, vehicleTransform, objectTransform, getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(), getHeading(), getPitch(), getRoll(), relativeCoord, sensorRela);

            double sensor_distance = sqrt(std::pow(sensorRela[0], 2.0) + std::pow(sensorRela[1], 2.0)); // 计算目标物体与传感器的距离

            S_SP_SENSOR_DETECTION_INFO object = {0};

            object.u1Type = objectState->sObjectState.u1Type; //类型@D_SP_OBJECT_TYPE
            object.u2Flags = D_SP_SENSOR_OBJECT_FLAG_NONE;
            object.u4Id = objectId;
            object.u4SensorId = 0; //TODO

            // [一汽]混合传感器 [Modify] LS 23.04.17
            if (_description.modelType == SENSOR_MODEL_IDEAL)
            {
                // 对于理想传感器，这两项值均为100%
                object.u8TypeIdProb = 1.0;
            }
            else if (_description.modelType == SENSOR_MODEL_PROBABLY)
            {
                if (_description.type == SENSOR_TYPE_CAMERA)	// 相机的畸变
                {
                    object.u8TypeIdProb = 1.0;
                    sensorRela = calDistCoord(sensorRela);
                }
                else	// 其他传感器的畸变计算
                {
                    /* 转球坐标 */
                    getSphereCoord(sensorRela[0], sensorRela[1], sensorRela[2], sphereCoord);
                    // 增加维度高斯误差
                    double rangeGaussRand = getGaussRand(getRangeMSENoise()) + getEnvNoise(sensor_distance);
                    double azimuthGaussRand = getGaussRand(getAzimuthMSENoise()) * SENSOR_OBJECT_DETECTION_PI / 180.0;  // 水平角
                    double elevationGaussRand = getGaussRand(getElevationMSENoise()) * SENSOR_OBJECT_DETECTION_PI / 180.0;
                    // 增加维度高斯误差后转笛卡尔坐标系
                    getCartesianCoord(sphereCoord[0] + rangeGaussRand, sphereCoord[1] + azimuthGaussRand, sphereCoord[2] + elevationGaussRand, randErrSensorCoord);
                    log_compnt_mngr->trace("rangeGaussRand, azimuthGaussRand elevationGaussRand {},{},{}", rangeGaussRand,azimuthGaussRand,elevationGaussRand);
                    gaussRandErr = randErrSensorCoord - sensorRela;
                    sensorRela = randErrSensorCoord;
                }
            }

            log_compnt_mngr->debug("GaussRandSensorX, GaussRandSensorY GaussRandSensorZ {},{},{}", sensorRela[0],sensorRela[1],sensorRela[2]);

            object.sExtraInfo.sDynamicObj.sSensorPos.u8X =  static_cast<float>(sensorRela[0]);
            object.sExtraInfo.sDynamicObj.sSensorPos.u8Y =  static_cast<float>(sensorRela[1]);
            object.sExtraInfo.sDynamicObj.sSensorPos.u8Z =  static_cast<float>(sensorRela[2]);

            sensor_distance = sqrt(std::pow(object.sExtraInfo.sDynamicObj.sSensorPos.u8X, 2.0) + std::pow(object.sExtraInfo.sDynamicObj.sSensorPos.u8Y, 2.0));


            // 当高级设置开关打开时，计算置信度
            /* Tang 20230512 相机不计算目标存在可能性等参数*/
            if (isEnableAdvSettings() && (_description.type != SENSOR_TYPE_CAMERA))
            {
                // bug15947 [Mod] 代码规范：计算天气影响因子 LS 23.05.15
                double weatherPct = calcWeatherNoise(_description.type, sensor_distance);

                // bug7385 置信度增加遮挡比例的影响 LS 22.06.17
                if (objOcclusionScale.find(static_cast<int>(objectId)) != objOcclusionScale.end())
                {
                    occlusionScale = std::get<0>(objOcclusionScale[static_cast<int>(objectId)]);
                }
                // 计算检测概率getDetectedPR（距离，毫米波雷达RCS，天气因子）
                detectedPct = getDetectedPR(sensor_distance, rcs, weatherPct, occlusionScale);

                // 目标分类正确可能性
                if (nearestObjRelCoord.size() == 0)
                {
                    getNearestObjRelCoordToTarget(insensitiveRadius, nearestObjRelCoord);
                }
                object.u8TypeIdProb = getClassificationAccuracy(static_cast<int>(objectId), nearestObjRelCoord);
            }
            // 有效性标识
            object.u1ValidFlag = 1.0;
            // 目标存在的可能性
            object.u8ExistProb = detectedPct; // 概率传感器置信度
            log_compnt_mngr->debug("detectedPct =  occlusionScale = {},{}", detectedPct, occlusionScale);

            object.sExtraInfo.sDynamicObj.u8Dist = sqrt(std::pow(object.sExtraInfo.sDynamicObj.sSensorPos.u8X, 2.0) + std::pow(object.sExtraInfo.sDynamicObj.sSensorPos.u8Y, 2.0));

            Vector3D vEgoCoorTrans(0,0,0);
            Vector3D vSensorCoorTrans(0,0,0);
            object.sExtraInfo.sDynamicObj.u8NearestDist = getObjNearestCoordInfo(objectState, sensorRelative_x, sensorRelative_y, sensorRelative_z, getHeading(), getPitch(), getRoll(), vEgoCoorTrans, vSensorCoorTrans);

            /* Tang 2023.5.17 zentao15936 为相机最近点的输出增加畸变和噪声 */
            if(_description.type == SENSOR_TYPE_CAMERA)
            {
                vSensorCoorTrans = calDistCoord(vSensorCoorTrans);

                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8X =  static_cast<float>(vSensorCoorTrans[0]);
                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8Y =  static_cast<float>(vSensorCoorTrans[1]);
                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8Z =  static_cast<float>(vSensorCoorTrans[2]);
            }
            else
            {
                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8X =  static_cast<float>(vSensorCoorTrans[0] + gaussRandErr[0]);
                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8Y =  static_cast<float>(vSensorCoorTrans[1] + gaussRandErr[1]);
                object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8Z =  static_cast<float>(vSensorCoorTrans[2] + gaussRandErr[2]);
            }
            /* Tang Zentao10745 添加航向角、相对速度、相对加速度字段[start]*/
            object.sExtraInfo.sDynamicObj.u8HeadingAngle = objectState->sObjectState.sPos.u4H;

            auto relaVelToEgo = getRelaVelToEgo(objectState);
            auto relaAccToEgo = getRelaAccToEgo(objectState);
            object.sExtraInfo.sDynamicObj.u8RelativeVelx = relaVelToEgo.x();
            object.sExtraInfo.sDynamicObj.u8RelativeVely = relaVelToEgo.y();
            object.sExtraInfo.sDynamicObj.u8RelativeAccx = relaAccToEgo.x();
            object.sExtraInfo.sDynamicObj.u8RelativeAccy = relaAccToEgo.y();
            /* Tang Zentao10745 添加航向角、相对速度、相对加速度字段[end]*/

            object.u1Occlusion = 0;

            /* Tang 2022.10.31 一汽一阶段，拆分传感器[start]*/
            auto xyNearPos = getXYNearestPois(objectState, gaussRandErr);

            float nan = sqrt(-1);

            /* Tang 2023.5.17 zentao15936 为相机最近点的输出增加畸变和噪声[start] */
            if(_description.type == SENSOR_TYPE_CAMERA)
            {
                xyNearPos[0] = calDistCoord(xyNearPos[0]);
                xyNearPos[1] = calDistCoord(xyNearPos[1]);
                xyNearPos[2] = calDistCoord(xyNearPos[2]);
                xyNearPos[3] = calDistCoord(xyNearPos[3]);
            }
            else
            {
                /* do nothing */
            }
            /* Tang 2023.5.17 zentao15936 为相机最近点的输出增加畸变和噪声[end] */

            object.sExtraInfo.sDynamicObj.u1MovingSt = 0;
            object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u8X = xyNearPos[0].x();
            object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u8Y = xyNearPos[0].y();
            object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u8Z = 0.0;
            object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u4H = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u4P = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u4R = nan;

            object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u8X = xyNearPos[1].x();
            object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u8Y = xyNearPos[1].y();
            object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u8Z = 0.0;
            object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u4H = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u4P = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u4R = nan;

            object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u8X = xyNearPos[2].x();
            object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u8Y = xyNearPos[2].y();
            object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u8Z = 0.0;
            object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u4H = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u4P = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u4R = nan;

            object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u8X = xyNearPos[3].x();
            object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u8Y = xyNearPos[3].y();
            object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u8Z = 0.0;
            object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u4H = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u4P = nan;
            object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u4R = nan;
            /* Tang 2022.10.31 一汽一阶段，拆分传感器[end]*/

            //理想和混合传感器输出BoundingBox
            for (int _pointi =0; _pointi < BOUNDINGBOX_VERTEX_NUM; _pointi++)
            {
                S_SP_POINT3D tmpPoint = {nan, nan, nan};
                object.sExtraInfo.sDynamicObj.sBoundingBox[_pointi] = tmpPoint;
            }

            if (_description.modelType == SENSOR_MODEL_IDEAL || _description.modelType == SENSOR_MODEL_PROBABLY)
            {
                if(_description.boundingBoxEnable)
                {
                    std::vector<Vector3D> miscBBoxVertexVector;
                    computeVehicleObstacleBBox(objectState, miscBBoxVertexVector); //获取障碍物包围盒八个顶点信息
                    if (miscBBoxVertexVector.size()==BOUNDINGBOX_VERTEX_NUM)
                    {
                        for (int _pointi =0; _pointi < miscBBoxVertexVector.size(); _pointi++)
                        {
                            S_SP_POINT3D tmpPoint = {miscBBoxVertexVector[_pointi].x(), miscBBoxVertexVector[_pointi].y(), miscBBoxVertexVector[_pointi].z()};
                            object.sExtraInfo.sDynamicObj.sBoundingBox[_pointi] = tmpPoint;
                        }
                    }
                }
            }
            objects.push_back(object);
        }
    }

    //交通灯
    std::set<int> traficlight_ids;
    if (getObjectList(SENSOR_OBJECT_DETECTION_TYPE_TRFICLIGHT, traficlight_ids) == true)
    {
        //遍历所有交通灯
        for (auto itor = trafficLightList.begin(); itor != trafficLightList.end(); ++itor)
        {
            S_SP_TRAFFIC_LIGHT *trafficLight = *itor;
            if (trafficLight == nullptr)
            {
                log_compnt_mngr->warn("traficlight traficlight=nullptr.");
                continue;
            }

            // 目标物ID
            int32_t objectId = trafficLight->u4Id;

            //如果不在识别交通灯列表中，则忽略
            if (traficlight_ids.find(objectId) == traficlight_ids.end())
            {
                log_compnt_mngr->warn("traficlight objectId={} not in traficlight_ids.", objectId);
                continue;
            }

            RoadSignal *roadSignal = RoadSystem::Instance()->getSignal(std::to_string(objectId));
            if (roadSignal == nullptr)
            {
                log_compnt_mngr->warn("roadSignal roadSignal=nullptr.");
                continue;
            }

            const Transform &objectTransform = roadSignal->getTransform();

            S_SP_SENSOR_DETECTION_INFO object = {0};
            /* zentao16118 增加交通灯类型[start] */
            object.u1Type = D_SP_OBJECT_TYPE_TRAFFIC_LIGHT;
            /* zentao16118 增加交通灯类型[end] */
            object.u2Flags = D_SP_SENSOR_OBJECT_FLAG_NONE;
            object.u4Id = static_cast<uint32_t>(objectId);
            object.u4SensorId = 0; //TODO
            //计算相对坐标，即转换为主车坐标系，x轴向前，y轴向左

            // 世界坐标系转传感器坐标系
            Vector3D relativeCoord(0, 0, 0);
            Vector3D sensorRela(0, 0, 0);
            CoordTransform(yaw, pitch, roll, vehicleTransform, objectTransform, getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(), getHeading(), getPitch(), getRoll(), relativeCoord, sensorRela);

            // [一汽]混合传感器 [Add] LS 23.04.07
            double sensor_distance = sqrt(std::pow(sensorRela[0], 2.0) + std::pow(sensorRela[1], 2.0)); // 计算目标物体与传感器的距离

            if (_description.modelType == SENSOR_MODEL_IDEAL)
            {
                // 对于理想传感器，这两项值均为100%
                object.u8TypeIdProb = 1.0;
            }
            else if (_description.modelType == SENSOR_MODEL_PROBABLY)
            {
                if (_description.type == SENSOR_TYPE_CAMERA)	// 相机的畸变
                {
                    object.u8TypeIdProb = 1.0;
                    sensorRela = calDistCoord(sensorRela);
                }
                else	// 其他传感器的畸变计算
                {
                    /* 转球坐标 */
                    getSphereCoord(sensorRela[0], sensorRela[1], sensorRela[2], sphereCoord);
                    // 增加维度高斯误差
                    double rangeGaussRand = getGaussRand(getRangeMSENoise()) + getEnvNoise(sensor_distance);
                    double azimuthGaussRand = getGaussRand(getAzimuthMSENoise()) * SENSOR_OBJECT_DETECTION_PI / 180.0;   // 水平角
                    double elevationGaussRand = getGaussRand(getElevationMSENoise()) * SENSOR_OBJECT_DETECTION_PI / 180.0;
                    // 增加维度高斯误差后转笛卡尔坐标系
                    getCartesianCoord(sphereCoord[0] + rangeGaussRand, sphereCoord[1] + azimuthGaussRand, sphereCoord[2] + elevationGaussRand, randErrSensorCoord);
                    log_compnt_mngr->trace("rangeGaussRand, azimuthGaussRand elevationGaussRand {},{},{}", rangeGaussRand,azimuthGaussRand,elevationGaussRand);
                    gaussRandErr = randErrSensorCoord - sensorRela;
                    sensorRela = randErrSensorCoord;
                }
            }

            log_compnt_mngr->debug("GaussRandSensorX, GaussRandSensorY GaussRandSensorZ {},{},{}", sensorRela[0],sensorRela[1],sensorRela[2]);

            object.sExtraInfo.sSignObj.sSensorPos.u8X =  static_cast<float>(sensorRela[0]);
            object.sExtraInfo.sSignObj.sSensorPos.u8Y =  static_cast<float>(sensorRela[1]);
            object.sExtraInfo.sSignObj.sSensorPos.u8Z =  static_cast<float>(sensorRela[2]);

            sensor_distance = sqrt(std::pow(object.sExtraInfo.sSignObj.sSensorPos.u8X, 2.0) + std::pow(object.sExtraInfo.sSignObj.sSensorPos.u8Y, 2.0));

            if ( sensor_distance > getRange() || sensor_distance < getMinimumDetectRange() )
            {
                log_compnt_mngr->warn("traficlight object id {} distance not in scope, continue.",objectId);
                continue;
            }

            /* Tang 20230512 相机不计算目标存在可能性等参数 */
            if ((_description.modelType == SENSOR_MODEL_PROBABLY) && (_description.type != SENSOR_TYPE_CAMERA))
            {
                // 目标分类正确可能性
                if (nearestObjRelCoord.size() == 0)
                {
                    getNearestObjRelCoordToTarget(/*insensitiveRadius=*/0, nearestObjRelCoord);
                }
                object.u8TypeIdProb = getClassificationAccuracy(objectId, nearestObjRelCoord);
            }
            // 有效性标识
            object.u1ValidFlag = 1.0;
            // 目标存在的可能性
            object.u8ExistProb = detectedPct; // 概率传感器置信度

            object.sExtraInfo.sSignObj.u8Dist = sqrt(std::pow(object.sExtraInfo.sSignObj.sSensorPos.u8X, 2.0) + std::pow(object.sExtraInfo.sSignObj.sSensorPos.u8Y, 2.0));

            object.sExtraInfo.sSignObj.u8NearestDist = object.sExtraInfo.sSignObj.u8Dist;

            object.sExtraInfo.sSignObj.sNearestSensorPos.u8X =  static_cast<float>(sensorRela[0] + gaussRandErr[0]);
            object.sExtraInfo.sSignObj.sNearestSensorPos.u8Y =  static_cast<float>(sensorRela[1] + gaussRandErr[1]);
            object.sExtraInfo.sSignObj.sNearestSensorPos.u8Z =  static_cast<float>(sensorRela[2] + gaussRandErr[2]);

            object.u1Occlusion = 0;
            object.sExtraInfo.sSignObj.u1TrafficSignType = TRAFFIC_SIGN_TRAFFICLIGH;
            object.sExtraInfo.sSignObj.u8TrafficSignDistX = object.sExtraInfo.sSignObj.sNearestSensorPos.u8X;
            object.sExtraInfo.sSignObj.u8TrafficSignDistY = object.sExtraInfo.sSignObj.sNearestSensorPos.u8Y;

            objects.push_back(object);
        }
    }

    //交通标识
    std::set<int> traficsign_ids;
    if (getObjectList(SENSOR_OBJECT_DETECTION_TYPE_TRAFICSIGN, traficsign_ids) == true)
    {
        //遍历所有交通标识
        for (auto itor = trafficSignList.begin(); itor != trafficSignList.end(); ++itor)
        {
            S_SP_TRAFFIC_SIGN *trafficSign = *itor;
            if (trafficSign == nullptr)
            {
                log_compnt_mngr->warn("trafficSign trafficSign=nullptr.");
                continue;
            }

            // 目标物ID
            uint32_t objectId = trafficSign->u4TrafficSignId;

            if (trafficSign_infos.find(std::pair<std::string, int>(trafficSign->au1Type, static_cast<int>(objectId))) == trafficSign_infos.end())
            {
                log_compnt_mngr->warn("trafficSign objectId={} not in traficsign_ids.", objectId);
                continue;
            }

            Transform objectTransform;
            objectTransform.update(trafficSign->sPos.u8X, trafficSign->sPos.u8Y, trafficSign->sPos.u8Z);

            S_SP_SENSOR_DETECTION_INFO object = {0};
            object.u1Type = D_SP_OBJECT_TYPE_TRAFFIC_SIGN;
            object.u2Flags = D_SP_SENSOR_OBJECT_FLAG_NONE;
            object.u4Id = objectId;
            object.u4SensorId = 0; //TODO
            //计算相对坐标，即转换为主车坐标系，x轴向前，y轴向左

            // 世界坐标系转传感器坐标系
            Vector3D relativeCoord(0, 0, 0);
            Vector3D sensorRela(0, 0, 0);
            CoordTransform(yaw, pitch, roll, vehicleTransform, objectTransform, getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(), getHeading(), getPitch(), getRoll(), relativeCoord, sensorRela);

            // [一汽]混合传感器 [Add] LS 23.04.07
            double sensor_distance = sqrt(std::pow(sensorRela[0], 2.0) + std::pow(sensorRela[1], 2.0)); // 计算目标物体与传感器的距离

            if (_description.modelType == SENSOR_MODEL_IDEAL)
            {
                // 对于理想传感器，这两项值均为100%
                object.u8TypeIdProb = 1.0;
            }
            else if (_description.modelType == SENSOR_MODEL_PROBABLY)
            {
                if (_description.type == SENSOR_TYPE_CAMERA)	// 相机的畸变
                {
                    object.u8TypeIdProb = 1.0;
                    sensorRela = calDistCoord(sensorRela);
                }
                else	// 其他传感器的畸变计算
                {
                    /* 转球坐标 */
                    getSphereCoord(sensorRela[0], sensorRela[1], sensorRela[2], sphereCoord);
                    // 增加维度高斯误差
                    double rangeGaussRand = getGaussRand(getRangeMSENoise()) + getEnvNoise(sensor_distance);
                    double azimuthGaussRand = getGaussRand(getAzimuthMSENoise()) * SENSOR_OBJECT_DETECTION_PI / 180.0; // 水平角
                    double elevationGaussRand = getGaussRand(getElevationMSENoise()) * SENSOR_OBJECT_DETECTION_PI / 180.0;
                    // 增加维度高斯误差后转笛卡尔坐标系
                    getCartesianCoord(sphereCoord[0] + rangeGaussRand, sphereCoord[1] + azimuthGaussRand, sphereCoord[2] + elevationGaussRand, randErrSensorCoord);
                    log_compnt_mngr->trace("rangeGaussRand, azimuthGaussRand elevationGaussRand {},{},{}", rangeGaussRand,azimuthGaussRand,elevationGaussRand);
                    gaussRandErr = randErrSensorCoord - sensorRela;
                    sensorRela = randErrSensorCoord;
                }
            }

            log_compnt_mngr->debug("GaussRandSensorX, GaussRandSensorY GaussRandSensorZ {},{},{}", sensorRela[0],sensorRela[1],sensorRela[2]);

            object.sExtraInfo.sSignObj.sSensorPos.u8X =  static_cast<float>(sensorRela[0]);
            object.sExtraInfo.sSignObj.sSensorPos.u8Y =  static_cast<float>(sensorRela[1]);
            object.sExtraInfo.sSignObj.sSensorPos.u8Z =  static_cast<float>(sensorRela[2]);

            sensor_distance = sqrt(std::pow(object.sExtraInfo.sSignObj.sSensorPos.u8X, 2.0) + std::pow(object.sExtraInfo.sSignObj.sSensorPos.u8Y, 2.0));

            if ( sensor_distance > getRange() || sensor_distance < getMinimumDetectRange() )
            {
                log_compnt_mngr->warn("trafficSign object id {} distance not in scope, continue.",objectId);
                continue;
            }

            /* Tang 20230512 相机不计算目标存在可能性等参数*/
            if ((_description.modelType == SENSOR_MODEL_PROBABLY) && (_description.type != SENSOR_TYPE_CAMERA))
            {
                // 目标分类正确可能性
                if (nearestObjRelCoord.size() == 0)
                {
                    getNearestObjRelCoordToTarget(/*insensitiveRadius=*/0, nearestObjRelCoord);
                }
                object.u8TypeIdProb = getClassificationAccuracy(static_cast<int>(objectId), nearestObjRelCoord);
            }
            // 有效性标识
            object.u1ValidFlag = 1.0;
            // 目标存在的可能性
            object.u8ExistProb = detectedPct; // 概率传感器置信度

            object.sExtraInfo.sSignObj.u8Dist = sqrt(std::pow(object.sExtraInfo.sSignObj.sSensorPos.u8X, 2.0) + std::pow(object.sExtraInfo.sSignObj.sSensorPos.u8Y, 2.0));

            object.sExtraInfo.sSignObj.u8NearestDist = object.sExtraInfo.sSignObj.u8Dist;

            object.sExtraInfo.sSignObj.sNearestSensorPos.u8X =  static_cast<float>(sensorRela[0]);
            object.sExtraInfo.sSignObj.sNearestSensorPos.u8Y =  static_cast<float>(sensorRela[1]);
            object.sExtraInfo.sSignObj.sNearestSensorPos.u8Z =  static_cast<float>(sensorRela[2]);

            object.u1Occlusion = 0;
            object.sExtraInfo.sSignObj.u1TrafficSignType = TRAFFIC_SIGN_NONE;
            object.sExtraInfo.sSignObj.u8TrafficSignDistX = object.sExtraInfo.sSignObj.sNearestSensorPos.u8X;
            object.sExtraInfo.sSignObj.u8TrafficSignDistY = object.sExtraInfo.sSignObj.sNearestSensorPos.u8Y;

            objects.push_back(object);
        }
    }

    /* Tang zentao15985 只有相机才输出车道线信息 */
    if(_description.type == SENSOR_TYPE_CAMERA || _description.type == SENSOR_TYPE_BASIC_SENSOR)
	{
        //道路信息，车道信息
		std::vector<S_SP_LANE_INFO> laneInfos;
	    if (getLaneInfo(laneInfos) == true)
		{
			//遍历所有道路信息，车道信息
			Road *road = RoadSystem::Instance()->getRoad(std::to_string(egoState->sObjectState.u8RoadId));
			LaneSection *laneSection = road->getLaneSection(egoState->sObjectState.u4RoadS);
			std::map<int, Lane *> laneMap = laneSection->getLaneMap();
			int laneId = egoState->sObjectState.u1LaneId;

			S_SP_SENSOR_DETECTION_INFO object = {0};
			// 对于理想传感器，存在可能性和分类正确性均为100%
			object.u8ExistProb = 1.0;
			object.u8TypeIdProb = 1.0;
			object.u1ValidFlag = 1.0;
			object.u1Type = D_SP_OBJECT_TYPE_LANE;
			object.u2Flags = D_SP_SENSOR_OBJECT_FLAG_NONE;
			object.u4Id = laneId;
			object.u4SensorId = 0; //TODO
			object.u1Occlusion = 0;
			// 计算车道线信息
			std::vector<double> RoadMarkCoeff;
			RoadMarkCoeff = getRoadMarkCoeff(laneId);
			for(int i=0 ; i<4 ; ++i){
				object.sExtraInfo.sMarkObj.au8RoadMarkCoeffL[i] = RoadMarkCoeff[i];
			}
			for(int i=4 ; i<8 ; ++i){
				object.sExtraInfo.sMarkObj.au8RoadMarkCoeffR[i-4] = RoadMarkCoeff[i];
			}
			objects.push_back(object);

			int direction = 1; //方向
			if (egoState->sObjectState.u1Dir == D_SP_ROAD_DIR_FLAG_DIR_FWD)
			{
				direction = 1;
			}
			else if (egoState->sObjectState.u1Dir == D_SP_ROAD_DIR_FLAG_DIR_REAR)
			{
				direction = -1;
			}

			/* 左车道 */
			int leftLaneId = laneId + direction; //左车道Id
			if(leftLaneId == 0){leftLaneId += direction;}
			object.u4Id = leftLaneId;
			// 计算车道线信息
			std::vector<double> LeftRoadMarkCoeff;
			//检查左右车道是否存在
			if (laneMap.find(leftLaneId) != laneMap.end()){
				LeftRoadMarkCoeff = getRoadMarkCoeff(leftLaneId);
			}
			else{
				LeftRoadMarkCoeff.resize(8, sqrt(-1));
			}
			for(int i=0 ; i<4 ; ++i){
				object.sExtraInfo.sMarkObj.au8RoadMarkCoeffL[i] = LeftRoadMarkCoeff[i];
			}
			for(int i=4 ; i<8 ; ++i){
				object.sExtraInfo.sMarkObj.au8RoadMarkCoeffR[i-4] = LeftRoadMarkCoeff[i];
			}
			objects.push_back(object);

			/* 右车道 */
			int rightLaneId = laneId - direction; //右车道Id
			if(rightLaneId == 0){rightLaneId -= direction;}
			object.u4Id = rightLaneId;
			// 计算车道线信息
			std::vector<double> RightRoadMarkCoeff;
			//检查左右车道是否存在
			if (laneMap.find(rightLaneId) != laneMap.end()){
				RightRoadMarkCoeff = getRoadMarkCoeff(rightLaneId);
			}
			else{
				RightRoadMarkCoeff.resize(8, sqrt(-1));
			}
			for(int i=0 ; i<4 ; ++i)
			{object.sExtraInfo.sMarkObj.au8RoadMarkCoeffL[i] = RightRoadMarkCoeff[i];}
			for(int i=4 ; i<8 ; ++i)
			{object.sExtraInfo.sMarkObj.au8RoadMarkCoeffR[i-4] = RightRoadMarkCoeff[i];}
			objects.push_back(object);
		}

        //道路标志，车道线
        std::vector<S_SP_MIL_ROADMARK> road_marks;
        if (getRoadMark(road_marks) == true)
        {
            //遍历所有道路标志，车道线
            std::string roadId = std::to_string(egoState->sObjectState.u8RoadId);
            Road *road = RoadSystem::Instance()->getRoad(roadId);
            if (road != nullptr)
            {
                //[帧率控制数据传输架构整改]TODO: 此处可能会造成传感器移植到CM后输出结果有误差，因为之前用的s坐标为double类型
                LaneSection *laneSection = road->getLaneSection(egoState->sObjectState.u4RoadS);
                if (laneSection != nullptr)
                {
                    std::map<int, Lane *> laneMap = laneSection->getLaneMap();

                    //填充roadMark元素
                    for (std::map<int, Lane *>::iterator laneMapIt = laneMap.begin(); laneMapIt != laneMap.end(); laneMapIt++)
                    {
                        Lane *lane = laneMapIt->second;
                        std::map<double, RoadMark *> roadMarkMap = lane->getRoadMarkMap();

                        for (std::map<double, RoadMark *>::iterator roadMarkMapIt = roadMarkMap.begin(); roadMarkMapIt != roadMarkMap.end(); roadMarkMapIt++)
                        {
                            S_SP_SENSOR_DETECTION_INFO object = {0};
                            object.u8ExistProb = 1;
                            object.u8TypeIdProb = 1;
                            object.u1ValidFlag = 1;
                            object.u1Type = D_SP_OBJECT_TYPE_ROADMARK;
                            object.u2Flags = D_SP_SENSOR_OBJECT_FLAG_NONE;
                            // bug16421 pjd 2023.08.25
                            object.u4Id = laneMapIt->first;
                            object.u4SensorId = 0; //TODO
                            object.u1Occlusion = 0;

                            objects.push_back(object);
                        }
                    }
                }
            }
        }

        std::set<int> roadmark_ids; //道路标志ids集合
        if (getObjectList(SENSOR_OBJECT_DETECTION_TYPE_ROADMARK, roadmark_ids) == true)
        {
            //遍历所有障碍物
            for (auto miscIt = obstacleList.begin(); miscIt != obstacleList.end(); ++miscIt)
            {
                S_SP_MIL_OBJECT_STATE *objectState = *miscIt;
                if (objectState == nullptr)
                {
                    log_compnt_mngr->warn("obstacle 2  objectState=nullptr.");
                    continue;
                }

                // 目标物ID
                uint32_t objectId = objectState->sObjectState.u4Id;

                //如果不在识别障碍物列表中，则忽略
                if (roadmark_ids.find(static_cast<int>(objectId)) == roadmark_ids.end())
                {
                    log_compnt_mngr->warn("obstacle objectId={} not in roadmark_ids.", objectId);
                    continue;
                }

                // bug 34020 标线和停车位，仅相机输出且相机勾选道路标志
                if (objectState->sObjectState.u1Type != D_SP_OBJECT_TYPE_GRATICULE && objectState->sObjectState.u1Type != D_SP_OBJECT_TYPE_PARKING_SPACE)
                {
                    log_compnt_mngr->warn("obstacle objectId={} type = {} not in identify.", objectId, objectState->sObjectState.u1Type);
                    continue;
                }

                Transform objectTransform;
                objectTransform.update(objectState->sObjectState.sPos.u8X, objectState->sObjectState.sPos.u8Y, objectState->sObjectState.sPos.u8Z);

                // 世界坐标系转传感器坐标系
                Vector3D relativeCoord(0, 0, 0);
                Vector3D sensorRela(0, 0, 0);
                CoordTransform(yaw, pitch, roll, vehicleTransform, objectTransform, getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(), getHeading(), getPitch(), getRoll(), relativeCoord, sensorRela);

                double sensor_distance = sqrt(std::pow(sensorRela[0], 2.0) + std::pow(sensorRela[1], 2.0)); // 计算目标物体与传感器的距离

                S_SP_SENSOR_DETECTION_INFO object = {0};
                object.u1Type = objectState->sObjectState.u1Type;
                object.u2Flags = D_SP_SENSOR_OBJECT_FLAG_NONE;
                object.u4Id = objectId;
                object.u4SensorId = 0; //TODO

                if (_description.modelType == SENSOR_MODEL_IDEAL)
                {
                    // 对于理想传感器，这两项值均为100%
                    object.u8TypeIdProb = 1.0;
                }
                else if (_description.modelType == SENSOR_MODEL_PROBABLY)
                {
                    if (_description.type == SENSOR_TYPE_CAMERA)	// 相机的畸变
                    {
                        object.u8TypeIdProb = 1.0;
                        sensorRela = calDistCoord(sensorRela);
                    }
                    else	// 其他传感器的畸变计算
                    {
                        /* 转球坐标 */
                        getSphereCoord(sensorRela[0], sensorRela[1], sensorRela[2], sphereCoord);
                        // 增加维度高斯误差
                        double rangeGaussRand = getGaussRand(getRangeMSENoise()) + getEnvNoise(sensor_distance);
                        double azimuthGaussRand = getGaussRand(getAzimuthMSENoise()) * SENSOR_OBJECT_DETECTION_PI / 180.0;  // 水平角
                        double elevationGaussRand = getGaussRand(getElevationMSENoise()) * SENSOR_OBJECT_DETECTION_PI / 180.0;
                        // 增加维度高斯误差后转笛卡尔坐标系
                        getCartesianCoord(sphereCoord[0] + rangeGaussRand, sphereCoord[1] + azimuthGaussRand, sphereCoord[2] + elevationGaussRand, randErrSensorCoord);
                        log_compnt_mngr->trace("rangeGaussRand, azimuthGaussRand elevationGaussRand {},{},{}", rangeGaussRand,azimuthGaussRand,elevationGaussRand);
                        gaussRandErr = randErrSensorCoord - sensorRela;
                        sensorRela = randErrSensorCoord;
                    }
                }

                log_compnt_mngr->debug("GaussRandSensorX, GaussRandSensorY GaussRandSensorZ {},{},{}", sensorRela[0],sensorRela[1],sensorRela[2]);

                object.sExtraInfo.sDynamicObj.sSensorPos.u8X =  static_cast<float>(sensorRela[0]);
                object.sExtraInfo.sDynamicObj.sSensorPos.u8Y =  static_cast<float>(sensorRela[1]);
                object.sExtraInfo.sDynamicObj.sSensorPos.u8Z =  static_cast<float>(sensorRela[2]);

                sensor_distance = sqrt(std::pow(object.sExtraInfo.sDynamicObj.sSensorPos.u8X, 2.0) + std::pow(object.sExtraInfo.sDynamicObj.sSensorPos.u8Y, 2.0));

                // 当高级设置开关打开时，计算置信度
                if (isEnableAdvSettings() && (_description.type != SENSOR_TYPE_CAMERA))
                {

                    double weatherPct = calcWeatherNoise(_description.type, sensor_distance);
                    // bug7385 置信度增加遮挡比例的影响 LS 22.06.17
                    if (objOcclusionScale.find(static_cast<int>(objectId)) != objOcclusionScale.end())
                    {
                        occlusionScale = std::get<0>(objOcclusionScale[static_cast<int>(objectId)]);
                    }
                    // 计算检测概率getDetectedPR（距离，毫米波雷达RCS，天气因子）
                    detectedPct = getDetectedPR(sensor_distance, rcs, weatherPct, occlusionScale);

                    // 目标分类正确可能性
                    if (nearestObjRelCoord.size() == 0)
                    {
                        getNearestObjRelCoordToTarget(insensitiveRadius, nearestObjRelCoord);
                    }
                    object.u8TypeIdProb = getClassificationAccuracy(static_cast<int>(objectId), nearestObjRelCoord);
                }
                // 有效性标识
                object.u1ValidFlag = 1.0;
                // 目标存在的可能性
                object.u8ExistProb = detectedPct; // 概率传感器置信度
                log_compnt_mngr->debug("detectedPct =  occlusionScale = {},{}", detectedPct, occlusionScale);

                object.sExtraInfo.sDynamicObj.u8Dist = sqrt(std::pow(object.sExtraInfo.sDynamicObj.sSensorPos.u8X, 2.0) + std::pow(object.sExtraInfo.sDynamicObj.sSensorPos.u8Y, 2.0));

                Vector3D vEgoCoorTrans(0,0,0);
                Vector3D vSensorCoorTrans(0,0,0);
                object.sExtraInfo.sDynamicObj.u8NearestDist = getObjNearestCoordInfo(objectState, sensorRelative_x, sensorRelative_y, sensorRelative_z, getHeading(), getPitch(), getRoll(), vEgoCoorTrans, vSensorCoorTrans);
                if(_description.type == SENSOR_TYPE_CAMERA)
                {
                    vSensorCoorTrans = calDistCoord(vSensorCoorTrans);
                    object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8X =  static_cast<float>(vSensorCoorTrans[0]);
                    object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8Y =  static_cast<float>(vSensorCoorTrans[1]);
                    object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8Z =  static_cast<float>(vSensorCoorTrans[2]);
                }
                else
                {
                    object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8X =  static_cast<float>(vSensorCoorTrans[0] + gaussRandErr[0]);
                    object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8Y =  static_cast<float>(vSensorCoorTrans[1] + gaussRandErr[1]);
                    object.sExtraInfo.sDynamicObj.sNearestSensorPos.u8Z =  static_cast<float>(vSensorCoorTrans[2] + gaussRandErr[2]);
                }

                /* Tang Zentao10745 添加航向角、相对速度、相对加速度字段[start]*/
                object.sExtraInfo.sDynamicObj.u8HeadingAngle = objectState->sObjectState.sPos.u4H;

                auto relaVelToEgo = getRelaVelToEgo(objectState);
                auto relaAccToEgo = getRelaAccToEgo(objectState);
                object.sExtraInfo.sDynamicObj.u8RelativeVelx = relaVelToEgo.x();
                object.sExtraInfo.sDynamicObj.u8RelativeVely = relaVelToEgo.y();
                object.sExtraInfo.sDynamicObj.u8RelativeAccx = relaAccToEgo.x();
                object.sExtraInfo.sDynamicObj.u8RelativeAccy = relaAccToEgo.y();
               /* Tang Zentao10745 添加航向角、相对速度、相对加速度字段[end]*/

                object.u1Occlusion = 0;
                auto xyNearPos = getXYNearestPois(objectState, gaussRandErr);
                float nan = sqrt(-1);
                if(_description.type == SENSOR_TYPE_CAMERA)
                {
                    xyNearPos[0] = calDistCoord(xyNearPos[0]);
                    xyNearPos[1] = calDistCoord(xyNearPos[1]);
                    xyNearPos[2] = calDistCoord(xyNearPos[2]);
                    xyNearPos[3] = calDistCoord(xyNearPos[3]);
                }
                else
                {
                    /* do nothing */
                }
                object.sExtraInfo.sDynamicObj.u1MovingSt = 0;
                object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u8X = xyNearPos[0].x();
                object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u8Y = xyNearPos[0].y();
                object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u8Z = 0.0;
                object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u4H = nan;
                object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u4P = nan;
                object.sExtraInfo.sDynamicObj.sSenNearestPosXL.u4R = nan;
                object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u8X = xyNearPos[1].x();
                object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u8Y = xyNearPos[1].y();
                object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u8Z = 0.0;
                object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u4H = nan;
                object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u4P = nan;
                object.sExtraInfo.sDynamicObj.sSenNearestPosXR.u4R = nan;
                object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u8X = xyNearPos[2].x();
                object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u8Y = xyNearPos[2].y();
                object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u8Z = 0.0;
                object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u4H = nan;
                object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u4P = nan;
                object.sExtraInfo.sDynamicObj.sSenNearestPosYL.u4R = nan;
                object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u8X = xyNearPos[3].x();
                object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u8Y = xyNearPos[3].y();
                object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u8Z = 0.0;
                object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u4H = nan;
                object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u4P = nan;
                object.sExtraInfo.sDynamicObj.sSenNearestPosYR.u4R = nan;

                for (int _pointi =0; _pointi < 8; _pointi++)
                {
                    S_SP_POINT3D tmpPoint = {nan, nan, nan};
                    object.sExtraInfo.sDynamicObj.sBoundingBox[_pointi] = tmpPoint;
                }
                if (_description.modelType == SENSOR_MODEL_IDEAL || _description.modelType == SENSOR_MODEL_PROBABLY)
                {
                    if(_description.boundingBoxEnable)
                    {
                        std::vector<Vector3D> miscBBoxVertexVector;
                        computeVehicleObstacleBBox(objectState, miscBBoxVertexVector); //获取障碍物包围盒八个顶点信息
                        if (miscBBoxVertexVector.size()==8)
                        {
                            for (int _pointi =0; _pointi < miscBBoxVertexVector.size(); _pointi++)
                            {
                                S_SP_POINT3D tmpPoint = {miscBBoxVertexVector[_pointi].x(), miscBBoxVertexVector[_pointi].y(), miscBBoxVertexVector[_pointi].z()};
                                object.sExtraInfo.sDynamicObj.sBoundingBox[_pointi] = tmpPoint;
                            }
                        }
                    }
                }
                objects.push_back(object);
            }
        }
    }

    log_compnt_mngr->info("Sensor::getObjectList end.");
    return true;
}

/**
 * @description: 判断物体是否在传感器的探测范围
 * @param {S_SP_MIL_EGO_STATE} *egoState  主车参数指针
 * @param {Transform} _egoTransform 主车参考点（后轴中心）
 * @param {vector<Vector3D>} &_pointVector 物体的包围盒
 * @param {float} hfov  传感器的hfov 单位 弧度制
 * @param {float} vfov  传感器的vfov 单位 弧度制
 * @param {double} _sensorX   传感器安装在主车上相对于参考点的x坐标
 * @param {double} _sensorY   传感器安装在主车上相对于参考点的y坐标
 * @param {double} _sensorZ   传感器安装在主车上相对于参考点的y坐标
 * @param {float} _sensorRange  传感器传感器的最大探测范围
 * @param {float} _sensorMinRange 传感器传感器的最小探测范围
 * @param {double} _sensorYaw     传感器传感器的航向角
 * @param {double} _sensorPitch   传感器传感器的俯仰角
 * @param {double} _sensorRoll    传感器传感器的横滚角
 * @return {*} 物体在探测范围内，返回true，否则false
 */
bool Sensor::InCircleandInSector(S_SP_MIL_EGO_STATE *egoState, Transform _egoTransform, const std::vector<Vector3D> &_pointVector, float hfov, float vfov, double _sensorX, double _sensorY, double _sensorZ, float _sensorRange, float _sensorMinRange, double _sensorYaw, double _sensorPitch, double _sensorRoll)
{
    log_compnt_mngr->info("Sensor::InCircleandInSector start.");
    bool bInCircleandInSector = false;
    // 世界坐标系转传感器坐标系
    Vector3D relativeCoord(0, 0, 0);
    Vector3D sensorRela(0, 0, 0);
    Transform agent_transform;

    for (unsigned int i = 0; i < _pointVector.size(); i++)
    {
        agent_transform.v() = _pointVector[i];
        CoordTransform(egoState->sObjectState.sPos.u4H, egoState->sObjectState.sPos.u4P, egoState->sObjectState.sPos.u4R, _egoTransform, agent_transform, _sensorX, _sensorY, _sensorZ, _sensorYaw, _sensorPitch, _sensorRoll, relativeCoord, sensorRela);
        if ((InCircle(egoState, _egoTransform, _pointVector[i],  _sensorX, _sensorY, _sensorZ, _sensorRange, _sensorMinRange))
            && (InSector(hfov, vfov, sensorRela[0], sensorRela[1], sensorRela[2])))
        {
            bInCircleandInSector = true;
            break;
        }
    }

    log_compnt_mngr->info("Sensor::InCircleandInSector end.");
    return bInCircleandInSector;
}

//传感器筛选代码，判断物体是否在圆内
bool Sensor::InCircle(S_SP_MIL_EGO_STATE *egoState, Transform _egoTransform, Vector3D _agentTransform, double _sensorX, double _sensorY, double _sensorZ, float _sensorRange, float _sensorMinRange)
{
    bool bIsInCircle = false;
    // 主车坐标系转世界坐标系
    Vector3D vSensorWorld( _sensorX, _sensorY, _sensorZ ); //得到传感器坐标
    VectorRotated(egoState->sObjectState.sPos.u4H, egoState->sObjectState.sPos.u4P, egoState->sObjectState.sPos.u4R, vSensorWorld);

    // 是否在最近探测距离和最远探测距离之间
    if ((std::pow((_egoTransform.v().x() + vSensorWorld[0]) - _agentTransform.x(), 2.0)
        + std::pow((_egoTransform.v().y() + vSensorWorld[1]) - _agentTransform.y(), 2.0)
        + std::pow((_egoTransform.v().z() + vSensorWorld[2]) - _agentTransform.z(), 2.0)
        < std::pow(_sensorRange, 2.0)) &&
        (std::pow((_egoTransform.v().x() + vSensorWorld[0]) - _agentTransform.x(), 2.0)
        + std::pow((_egoTransform.v().y() + vSensorWorld[1]) - _agentTransform.y(), 2.0)
        + std::pow((_egoTransform.v().z() + vSensorWorld[2]) - _agentTransform.z(), 2.0)
        > std::pow(_sensorMinRange, 2.0)))
    {
        bIsInCircle = true;
    }

    return bIsInCircle;
}


//判断包围盒中间截面的四条边和传感器x轴是否有交点
bool Sensor::IsIntersection(S_SP_MIL_EGO_STATE *egoState, Transform _egoTransform, const std::vector<Vector3D> &_pointVector, double _sensorX, double _sensorY, double _sensorZ,  double _sensorYaw, double _sensorPitch, double _sensorRoll,float _sensorRange, float _sensorMinRange)
{
    log_compnt_mngr->info("Sensor::IsIntersection start.");
    std::vector<Vector3D> _tempVector;
    bool bIsInCircle = false;
    //求包围盒中间面的四个点坐标；0 左前，1 左后，2 右前，3 右后
    Vector3D relativeCoord(0, 0, 0);
    Vector3D sensorRela(0, 0, 0);
    Vector3D sensoa(0, 0, 0);
    Transform vehvector;
    double intersection;
    for (unsigned int i = 0; i < 4; i++)
    {
        sensoa=_pointVector[i];
        sensoa.z() = (_pointVector[i+4].z() - _pointVector[i].z())/2;
        vehvector.v()=sensoa;
        // 世界坐标系转传感器坐标系
        CoordTransform(egoState->sObjectState.sPos.u4H, egoState->sObjectState.sPos.u4P, egoState->sObjectState.sPos.u4R, _egoTransform,vehvector, _sensorX,_sensorY, _sensorZ,_sensorYaw, _sensorPitch, _sensorRoll, relativeCoord, sensorRela);
        _tempVector.push_back(sensorRela);
    }
    //求四条边和x轴是否有交点，顺序为(左前，左后)，(左后，右前)，(右前，右后)，(右后，左前)
    for (unsigned int i =0; i < 4; i++)
    {

        Vector3D p1 = _tempVector[i];
        Vector3D p2 = _tempVector[(i + 1) % _tempVector.size()];
        Vector2D v(1,0);
        //p1和p2分别与法向量v叉乘，判断两点是否在同侧
        if ((v.x()*p1.y() - v.y()*p1.x()) * (v.x()*p2.y() - v.y()*p2.x()) < 0)
        {
            //直线没有斜率的情况
            if (p1.x() == p2.x() && p1.x() >  _sensorMinRange && p1.x() <  _sensorRange)
            {
                bIsInCircle = true;
                break;
            }
            else //判断直线有斜率的情况下是否与x轴有交点
            {
                intersection = p1.x() - (p2.x() - p1.x())/(p2.y()-p1.y())*p1.y();
                if (p1.y() != p2.y() && intersection > _sensorMinRange && intersection < _sensorRange)
                {
                    bIsInCircle = true;
                    break;
                }
            }
        }
    }
    log_compnt_mngr->info("Sensor::IsIntersection end.");
    return bIsInCircle;
}

//传感器筛选代码，判断物体是否在扇形内
bool Sensor::InSector(float _hfov, float _vfov, double _x2, double _y2, double _z2)
{

    log_compnt_mngr->debug("x2 , y2 z2 {},{},{}", _x2,_y2,_z2);
    log_compnt_mngr->debug("hfov , vfov {},{}", _hfov,_vfov);

    // 水平方向上的方位
    float m_fHAtan = std::atan2(_y2, _x2);
    float m_fHAngle = std::abs((m_fHAtan));

    if (m_fHAngle > SENSOR_OBJECT_DETECTION_PI)
        m_fHAngle = 2 * SENSOR_OBJECT_DETECTION_PI - m_fHAngle;

    // 垂直方向上的方位
    float m_fVAtan = std::atan2(_z2, sqrt(std::pow(_x2, 2.0) + std::pow(_y2, 2.0)));
    float m_fVAngle = std::abs((m_fVAtan));
    if (m_fVAngle > SENSOR_OBJECT_DETECTION_PI)
        m_fVAngle = 2 * SENSOR_OBJECT_DETECTION_PI - m_fVAngle;

    log_compnt_mngr->debug("m_fHAngle , m_fVAngle  {},{}.", m_fHAngle,m_fVAngle);
    log_compnt_mngr->debug("std::abs(_hfov / 2) , std::abs(_vfov / 2) {},{}.", std::abs(_hfov / 2),std::abs(_vfov / 2));

    // 判断物体水平方位是否在hFov范围内, 垂直方位是否在vFov范围内
    if ( m_fHAngle <= std::abs(_hfov / 2) && m_fVAngle <= std::abs(_vfov / 2))
    {
        return true;
    }
    else
    {
        return false;
    }
}

//将弧度角值域调整到[0, 2PI)
double Sensor::normalizeRad(double angle)
{
    //值域调整到(-2PI, 2PI)
    double output = fmod(angle, 2 * M_PI);

    //值域调整到[0, 2PI)
    if (output < 0)
    {
        output += 2 * M_PI;
    }

    return output;
}

//将弧度角值域调整到[-PI, PI)
double Sensor::normalizeRad2(double angle)
{
    //值域调整到[0, 2PI)
    double output = normalizeRad(angle);

    //值域调整到[-PI, PI)
    if (output >= M_PI) //范围[M_PI, 2PI)
    {
        output -= 2 * M_PI;
    }

    return output;
}

//判断物体是否被遮挡
bool Sensor::IsMaskedView(int count, int id, float theta)
{
    // ID对应到map的ID，获取θmin,θmax，得到弧度
    log_compnt_mngr->debug("id: theta:{},{} ", id,theta);
    occupiedRadVec.reserve(2 * agentFourPointRadMap.size());
    std::map<int, std::tuple<float, float, float, float>>::iterator iter = agentFourPointRadMap.find(id);

    // 如果没有找到对应ID
    if (iter == agentFourPointRadMap.end())
    {
        log_compnt_mngr->warn("ID {} not found",id);
        return true;
    }
    else
    {

        auto agentRad = iter->second;				// map里的最大弧度
        float thetaMin = std::get<0>(agentRad); 	// 当前ID的水平方向最小theta
        float thetaMax = std::get<1>(agentRad); 	// 当前ID的水平方向最大theta
        float thetaMinZDim = std::get<2>(agentRad); // 当前ID的垂直方向最小theta
        float thetaMaxZDim = std::get<3>(agentRad); // 当前ID的垂直方向最大theta
        log_compnt_mngr->debug("id: thetaMin: thetaMax: thetaMinZDim: thetaMaxZDim:{},{},{},{},{}", id,thetaMin,thetaMax,thetaMinZDim,thetaMaxZDim);

        //遮挡增加Z维度 23.06.21 LS <id,<occlusionScale, horOcclusionScale, verOcclusionScale, occupiedRightRad, occupiedLeftRad, occupiedBottomRad, occupiedUpperRad>>
        objOcclusionScale[id] = make_tuple(0.0,0.0,0.0,0.0,0.0,0.0,0.0); // bug7385 置信度增加遮挡比例的影响 LS 22.06.17
        // 当查找第一个id时，直接取得第一个目标物体的最大最小夹角
        // bug6659 LS 22.04.20 当目标为最近的物体且不在主车后方时，直接输出
        if (count == 0 && thetaMax - thetaMin < M_PI )
        {
            occupiedRadVec.push_back(std::make_tuple(thetaMin,thetaMax,thetaMinZDim,thetaMaxZDim));
            return false;
        }
        else
        {
            //bug5940 更新主车后方的遮挡判断 22.04.14 LS
            float negMax = 0;
            float negMin = 0;
            // 	当最大弧度-最小弧度大于PI，即弧度夹角大于PI
            if ( thetaMax - thetaMin > M_PI)
            {
                float tempAbs = std::abs((2 * SENSOR_OBJECT_DETECTION_PI - thetaMax + thetaMin)); // 计算后方车辆的夹角
                // 为负值的弧度延长到0-2pi之间，更新最小弧度和最大弧度
                thetaMin = thetaMax;
                thetaMax = thetaMin +  tempAbs;

                // 更新负方向的最小和最大弧度
                negMax = -(2 * SENSOR_OBJECT_DETECTION_PI - thetaMax);
                negMin = -(2 * SENSOR_OBJECT_DETECTION_PI - thetaMax + (thetaMax - thetaMin));
                log_compnt_mngr->debug("id behind of ego {}", id);
            }
            // 修改被占用的弧度的vec
            for(int i = 0; i < occupiedRadVec.size(); i++)
            {
                log_compnt_mngr->debug("occupiedRadVec[i]=> {} {} {} {}", std::get<0>(occupiedRadVec[i]), std::get<1>(occupiedRadVec[i]), std::get<2>(occupiedRadVec[i]), std::get<3>(occupiedRadVec[i]));
                float maskedPct = 0.0; 		// 遮挡比例
                float maskedPctXDim = 0.0; 	// 水平遮挡比例
                float result = 0.0; 		// 水平未遮挡比例
                float maskedPctZDim = 0.0;	// 垂直遮挡比例
                float resultZDim = 0.0; 	// 垂直未遮挡比例

                // 当前物体左右两边弧度之差不为0时，计算遮挡比例
                if ((thetaMax - thetaMin) != 0)
                {
                    // 如果当前物体的左边弧度与前面物体的左边弧度之差小于0，则取两者之差为0；如果前面物体的右边弧度与当前物体的右边弧度之差小于0，则取两者之差为0
                    result = ( relu0(thetaMax, std::get<1>(occupiedRadVec[i])) + relu0(std::get<0>(occupiedRadVec[i]), thetaMin) ) / (thetaMax - thetaMin);
                    maskedPctXDim = 1- relu1(result, 0); // 遮挡比例取值范围为[0, 1-result]
                }

                if ((thetaMaxZDim - thetaMinZDim) != 0)
                {
                    // 如果当前物体的左边弧度与前面物体的左边弧度之差小于0，则取两者之差为0；如果前面物体的右边弧度与当前物体的右边弧度之差小于0，则取两者之差为0
                    resultZDim = ( relu0(thetaMaxZDim, std::get<3>(occupiedRadVec[i])) + relu0(std::get<2>(occupiedRadVec[i]), thetaMinZDim) ) / (thetaMaxZDim - thetaMinZDim);
                    maskedPctZDim = 1- relu1(resultZDim, 0); // 遮挡比例取值范围为[0, 1-result]
                }

                maskedPct = maskedPctXDim *  maskedPctZDim;
                objOcclusionScale[id] = make_tuple(maskedPct,maskedPctXDim,maskedPctZDim,std::get<0>(occupiedRadVec[i]),std::get<1>(occupiedRadVec[i]),std::get<2>(occupiedRadVec[i]),std::get<3>(occupiedRadVec[i]));
                log_compnt_mngr->debug("mask percentage {} objOcclusionScale.size:{}", maskedPct, objOcclusionScale.size());

                // 目标在左侧且有遮挡关系
                if (thetaMax > std::get<1>(occupiedRadVec[i]) && thetaMin <= std::get<1>(occupiedRadVec[i]) && thetaMin >= std::get<0>(occupiedRadVec[i])) //  更新判断遮挡情况 22.03.15 LS
                {
                    log_compnt_mngr->debug("target is on the left side {}", id);
                    std::get<1>(occupiedRadVec[i]) = thetaMax; // 更新占用弧度的最大边
                    //bug5940 更新主车后方的遮挡判断 22.04.14 LS： 目标在主车后方时，更新正方向和负方向的弧度值
                    if ( thetaMax > M_PI)
                    {
                        occupiedRadVec.push_back(std::make_tuple(thetaMin, thetaMax, thetaMinZDim, thetaMaxZDim));
                        occupiedRadVec.push_back(std::make_tuple(negMin, negMax, thetaMinZDim, thetaMaxZDim));
                        log_compnt_mngr->debug("target behind of ego {}", id);
                    }
                    // bug18580 当z轴无遮挡情况时，输出目标 LS 23.07.24
                    if (maskedPctZDim != 1)
                    {
                        log_compnt_mngr->debug("id Z dimension in the view.");
                        std::get<3>(occupiedRadVec[i]) = thetaMaxZDim; // 更新Z轴方向占用弧度的最大边
                        return false;
                    }

                    // 遮挡比例大于90%时，判断被遮挡，不输出；反之输出
                    if (maskedPct > 0.9)
                    {
                        log_compnt_mngr->debug("Target is obscured by more than 90 pct.");
                        return true;
                    }
                    else
                    {
                        log_compnt_mngr->debug("Target is obscured by less than 10 pct.");
                        return false;
                    }
                }
                // 目标在右侧且有遮挡关系
                if (thetaMin < std::get<0>(occupiedRadVec[i]) && thetaMax >= std::get<0>(occupiedRadVec[i]) && thetaMax <= std::get<1>(occupiedRadVec[i]))
                {
                    log_compnt_mngr->debug("id is on the right side {}", id);
                    std::get<0>(occupiedRadVec[i]) = thetaMin; // 更新占用弧度的最小边
                    //bug5940 更新主车后方的遮挡判断 22.04.14 LS： 目标在主车后方时，更新正方向和负方向的弧度值
                    if ( thetaMax > M_PI)
                    {
                        occupiedRadVec.push_back(std::make_tuple(thetaMin, thetaMax, thetaMinZDim, thetaMaxZDim));
                        occupiedRadVec.push_back(std::make_tuple(negMin, negMax, thetaMinZDim, thetaMaxZDim));
                        log_compnt_mngr->debug("id behind of ego {}", id);
                    }
                    // bug18580 当z轴无遮挡情况时，输出目标
                    if (maskedPctZDim != 1)
                    {
                        log_compnt_mngr->debug("id Z dimension in the view.");
                        std::get<3>(occupiedRadVec[i]) = thetaMaxZDim; // 更新Z轴方向占用弧度的最大边
                        return false;
                    }

                    // 遮挡比例大于90%时，判断被遮挡，不输出；反之输出
                    if (maskedPct > 0.9)
                    {
                        log_compnt_mngr->debug("Target is obscured by more than 90 pct.");
                        return true;
                    }
                    else
                    {
                        log_compnt_mngr->debug("Target is obscured by less than 10 pct.");
                        return false;
                    }
                }
                // 中心全部遮挡
                if (thetaMax > std::get<1>(occupiedRadVec[i]) && thetaMin < std::get<0>(occupiedRadVec[i]))
                {
                    std::get<1>(occupiedRadVec[i]) = thetaMax;
                    std::get<0>(occupiedRadVec[i]) = thetaMin;
                    // bug18580 当z轴无遮挡情况时，输出目标
                    if (maskedPctZDim != 1)
                    {
                        log_compnt_mngr->debug("id Z dimension in the view.");
                        std::get<3>(occupiedRadVec[i]) = thetaMaxZDim; // 更新Z轴方向占用弧度的最大边
                    }

                    log_compnt_mngr->debug("Target centre is obscured.");
                    return false;
                }
                // 水平维度全部遮挡
                if (thetaMax <= std::get<1>(occupiedRadVec[i]) && thetaMin >= std::get<0>(occupiedRadVec[i]))
                {
                    // 遮挡比例为100%（z轴方向也被遮挡）
                    if(maskedPct == 1)
                    {
                        log_compnt_mngr->debug("Target is obscured.");
                        //bug5940 更新主车后方的遮挡判断 22.04.14 LS
                        // 目标在主车后方时，更新正方向和负方向的弧度值
                        if ( thetaMax > M_PI)
                        {
                            occupiedRadVec.push_back(std::make_tuple(thetaMin, thetaMax, thetaMinZDim, thetaMaxZDim));
                            occupiedRadVec.push_back(std::make_tuple(negMin, negMax, thetaMinZDim, thetaMaxZDim));
                            log_compnt_mngr->debug("id behind of ego {}", id);
                        }
                        return true;
                    }
                    else // 遮挡比例小于100%（z轴方向未被遮挡）
                    {
                        log_compnt_mngr->debug("Target is obscured, except Z dimension.");
                        std::get<3>(occupiedRadVec[i]) = thetaMaxZDim; // 更新Z轴方向占用弧度的最大边
                        if ( thetaMax > M_PI)
                        {
                            occupiedRadVec.push_back(std::make_tuple(thetaMin, thetaMax, thetaMinZDim, thetaMaxZDim));
                            occupiedRadVec.push_back(std::make_tuple(negMin, negMax, thetaMinZDim, thetaMaxZDim));
                            log_compnt_mngr->debug("id behind of ego {}", id);
                        }
                        return false;
                    }
                }
            }
            // 当目标为第一个物体，且在主车后方时
            //bug5940 更新主车后方的遮挡判断  22.04.14 LS： 目标在主车后方时，更新正方向和负方向的弧度值
            if ( thetaMax > M_PI)
            {
                occupiedRadVec.push_back(std::make_tuple(negMin, negMax, thetaMinZDim, thetaMaxZDim));
                log_compnt_mngr->debug("id behind of ego {}", id);
            }
            // 所有目标占用的弧度
            occupiedRadVec.push_back(std::make_tuple(thetaMin,thetaMax, thetaMinZDim, thetaMaxZDim));
            log_compnt_mngr->debug("id in the view {}", id);
            return false;
        }
    }
}

//获取范围内物体质点的相对坐标并按照x^2+y^2距离远近排序
std::vector<tuple<int,int, float,float,float>> Sensor::getSortedAgentCoord(std::vector<tuple<int,int,float,float,float>> &unsortedVec)
{
    using tupl_i = std::tuple<int,int,float,float,float>;
    std::sort(unsortedVec.begin(),unsortedVec.end(),
            [](const tupl_i& tpl1,const tupl_i& tpl2){
                // sort属于弱序关系，如果两者相等，必须return false
                return (std::pow(std::get<2>(tpl1), 2.0) + std::pow(std::get<3>(tpl1), 2.0)) < (std::pow(std::get<2>(tpl2), 2.0) + std::pow(std::get<3>(tpl2), 2.0));
            }
    );
    return unsortedVec;
}

//获取目标物体的最大弧度和最小弧度
std::map<int, std::tuple<float, float, float, float>>  Sensor::getAgentFourPointMap()
{
    return agentFourPointRadMap;
}

//判定是否是物体识别类型
bool Sensor::isDetectionObject(int objectType)
{
    return true;
}

//取得道路信息，车道信息
bool Sensor::getLaneInfo(std::vector<S_SP_LANE_INFO> &laneInfos)
{
    log_compnt_mngr->info("Sensor::getLaneInfo start.");
    if (_description.enable == false)
    {
        log_compnt_mngr->error("description.enable is false,return.");
        return false;
    }

    if (isDetectionObject(SENSOR_OBJECT_DETECTION_TYPE_LANEINFO) == false)
    {
        log_compnt_mngr->error("object is not detection,return.");
        return false;
    }

    if (egoState == nullptr)
    {
        log_compnt_mngr->error( "egoState=nullptr.");
        return false;
    }

    Road *road = RoadSystem::Instance()->getRoad(std::to_string(egoState->sObjectState.u8RoadId));
    if (road == nullptr)
    {
        log_compnt_mngr->error( "road=nullptr.");
        return false;
    }

    LaneSection *laneSection = road->getLaneSection(egoState->sObjectState.u4RoadS);
    if (laneSection == nullptr)
    {
        log_compnt_mngr->error( "laneSection=nullptr.");
        return false;
    }

    std::map<int, Lane *> laneMap = laneSection->getLaneMap();

    // Sue 2021.1.8 roadId字段从uint32_t改为uint64_t
    uint64_t roadId = static_cast<uint64_t>(TrafficSimulation::stoul(road->getId()));

    uint32_t playerId = egoState->sObjectState.u4Id;
    int direction = 1; //方向
    if (egoState->sObjectState.u1Dir == D_SP_ROAD_DIR_FLAG_DIR_FWD)
    {
        direction = 1;
    }
    else if (egoState->sObjectState.u1Dir == D_SP_ROAD_DIR_FLAG_DIR_REAR)
    {
        direction = -1;
    }

    //填充lane元素
    for (std::map<int, Lane *>::iterator laneMapIt = laneMap.begin(); laneMapIt != laneMap.end(); laneMapIt++)
    {
        S_SP_LANE_INFO lane_info = {0};

        int laneId = laneMapIt->first;
        Lane *lane = laneMapIt->second;
        int leftLaneId = laneId + direction; //左右车道Id
        int rightLaneId = laneId - direction;

        //检查左右车道是否存在
        if (laneMap.find(leftLaneId) == laneMap.end())
        {
            leftLaneId = Lane::NOLANE;
        }
        if (laneMap.find(rightLaneId) == laneMap.end())
        {
            rightLaneId = Lane::NOLANE;
        }

        // Sue 2021.1.8 roadId字段从uint32_t改为uint64_t
        lane_info.u8RoadId = roadId;
        lane_info.u1Id = static_cast<int8_t>(laneId);
        lane_info.u1NeighborMask = D_SP_LANE_EXISTS_OWN; //自有车道存在
        if (leftLaneId != Lane::NOLANE)
        {
            lane_info.u1NeighborMask = lane_info.u1NeighborMask | D_SP_LANE_EXISTS_LEFT; //左侧车道存在
        }
        if (rightLaneId != Lane::NOLANE)
        {
            lane_info.u1NeighborMask = lane_info.u1NeighborMask | D_SP_LANE_EXISTS_RIGHT; //右侧车道存在
        }

        //左侧车道Id
        if (leftLaneId != Lane::NOLANE)
        {
            lane_info.u1LeftLaneId = static_cast<int8_t>(leftLaneId);
        }
        else
        {
            lane_info.u1LeftLaneId = 127; //不存在就最大值
        }

        //右侧车道Id
        if (rightLaneId != Lane::NOLANE)
        {
            lane_info.u1RightLaneId = static_cast<int8_t>(rightLaneId);
        }
        else
        {
            lane_info.u1RightLaneId = 127; //不存在就最大值
        }

        lane_info.u2Type = lane->getLaneType();	  //车道类型
        lane_info.u4Width = static_cast<float>(fabs(laneSection->getLaneWidth(egoState->sObjectState.u4RoadS, laneId)));
        lane_info.u4ObjectId = playerId; //主车ID为1

        laneInfos.push_back(lane_info);
    }

    Road *nextRaod = nullptr;//下条路
    int nextRoadDir = 0;//在下条路上的行驶方向
    getNextRoadByLightMask(nextRaod, nextRoadDir);

    if (nextRaod != nullptr)
    {
        LaneSection *laneSection2 = nullptr;

        if (nextRoadDir == 1)
        {
            laneSection2 = nextRaod->getLaneSection(0.0);
        }
        else if (nextRoadDir == -1)
        {
            laneSection2 = nextRaod->getLaneSection(nextRaod->getLength());
        }
        else
        {
            //nothing to do
        }

        if (laneSection2 != nullptr)
        {
            std::map<int, Lane *> laneMap = laneSection2->getLaneMap();
            uint64_t nextRoadId = static_cast<uint64_t>(TrafficSimulation::stoul(nextRaod->getId()));

            //填充lane元素
            for (std::map<int, Lane *>::iterator laneMapIt = laneMap.begin(); laneMapIt != laneMap.end(); laneMapIt++)
            {
                S_SP_LANE_INFO lane_info = {0};

                int laneId = laneMapIt->first;
                Lane *lane = laneMapIt->second;
                int leftLaneId = laneId + nextRoadDir; //左右车道Id
                int rightLaneId = laneId - nextRoadDir;

                //检查左右车道是否存在
                if (laneMap.find(leftLaneId) == laneMap.end())
                {
                    leftLaneId = Lane::NOLANE;
                }
                if (laneMap.find(rightLaneId) == laneMap.end())
                {
                    rightLaneId = Lane::NOLANE;
                }

                lane_info.u8RoadId = nextRoadId;
                lane_info.u1Id = static_cast<int8_t>(laneId);
                lane_info.u1NeighborMask = D_SP_LANE_EXISTS_OWN; //自有车道存在
                if (leftLaneId != Lane::NOLANE)
                {
                    lane_info.u1NeighborMask = lane_info.u1NeighborMask | D_SP_LANE_EXISTS_LEFT; //左侧车道存在
                }
                if (rightLaneId != Lane::NOLANE)
                {
                    lane_info.u1NeighborMask = lane_info.u1NeighborMask | D_SP_LANE_EXISTS_RIGHT; //右侧车道存在
                }

                //左侧车道Id
                if (leftLaneId != Lane::NOLANE)
                {
                    lane_info.u1LeftLaneId = static_cast<int8_t>(leftLaneId);
                }
                else
                {
                    lane_info.u1LeftLaneId = 127; //不存在就最大值
                }

                //右侧车道Id
                if (rightLaneId != Lane::NOLANE)
                {
                    lane_info.u1RightLaneId = static_cast<int8_t>(rightLaneId);
                }
                else
                {
                    lane_info.u1RightLaneId = 127; //不存在就最大值
                }

                float u4Width = 0.0;
                if (nextRoadDir == 1)//下条路S轴正方向
                {
                    u4Width = static_cast<float>(fabs(laneSection2->getLaneWidth(0.0, laneId)));
                }
                else if (nextRoadDir == -1)//下条路S轴负方向
                {
                    u4Width = static_cast<float>(fabs(laneSection2->getLaneWidth(nextRaod->getLength(), laneId)));
                }
                else
                {
                    //nothing to do
                }

                lane_info.u2Type = lane->getLaneType();	  //车道类型
                lane_info.u4Width = u4Width;
                lane_info.u4ObjectId = playerId; //主车ID为1

                laneInfos.push_back(lane_info);
            }
        }
    }

    log_compnt_mngr->info("Sensor::getLaneInfo end.");
    return true;
}

//取得道路标志，车道线
bool Sensor::getRoadMark(std::vector<S_SP_MIL_ROADMARK> &roadMarks)
{
    log_compnt_mngr->info("Sensor::getRoadMark start.");
    if (_description.enable == false)
    {
        log_compnt_mngr->error("description.enable is false , return.");
        return false;
    }

    if (isDetectionObject(SENSOR_OBJECT_DETECTION_TYPE_ROADMARK) == false)
    {
        log_compnt_mngr->error("object is not detection,return.");
        return false;
    }

    if (egoState == nullptr)
    {
        log_compnt_mngr->error( "egoState=nullptr.");
        return false;
    }

    Road *road = RoadSystem::Instance()->getRoad(std::to_string(egoState->sObjectState.u8RoadId));
    if (road == nullptr)
    {
        log_compnt_mngr->error( "road=nullptr.");
        return false;
    }

    LaneSection *laneSection = road->getLaneSection(egoState->sObjectState.u4RoadS);
    if (laneSection == nullptr)
    {
        log_compnt_mngr->error( "laneSection=nullptr.");
        return false;
    }

    std::map<int, Lane *> laneMap = laneSection->getLaneMap();
    uint64_t roadId = static_cast<uint64_t>(TrafficSimulation::stoul(road->getId()));

    //填充roadMark元素
    for (std::map<int, Lane *>::iterator laneMapIt = laneMap.begin(); laneMapIt != laneMap.end(); laneMapIt++)
    {
        int laneId = laneMapIt->first;
        Lane *lane = laneMapIt->second;
        std::map<double, RoadMark *> roadMarkMap = lane->getRoadMarkMap();

        for (std::map<double, RoadMark *>::iterator roadMarkMapIt = roadMarkMap.begin(); roadMarkMapIt != roadMarkMap.end(); roadMarkMapIt++)
        {
            S_SP_MIL_ROADMARK road_mark = {0};

            RoadMark *roadMark = roadMarkMapIt->second;
            double roadMarkStart = roadMark->getStart();

            //填充数据
            road_mark.u8RoadId = roadId;
            road_mark.u4StartDx = roadMarkStart;
            road_mark.u4ViewRangeStart = 0.0;
            road_mark.u4ViewRangeEnd = 100.0;
            road_mark.u4MeasuredVREnd = 100.0;
            road_mark.u1LaneId = static_cast<int8_t>(laneId);
            road_mark.u1Quality = 3; //0 - 1: invalid 2: predicated 3: real

            RoadMark::RoadMarkType const roadMarkType = roadMark->getType();
            if (roadMarkType == RoadMark::TYPE_NONE)
            {
                road_mark.u1Type = D_SP_ROADMARK_TYPE_UNDECIDED;
            }
            else if (roadMarkType == RoadMark::TYPE_SOLID)
            {
                road_mark.u1Type = D_SP_ROADMARK_TYPE_SOLID;
            }
            else if (roadMarkType == RoadMark::TYPE_BROKEN)
            {
                road_mark.u1Type = D_SP_ROADMARK_TYPE_BROKEN;
            }
            else if (roadMarkType == RoadMark::TYPE_BROKENSOLID)
            {
                road_mark.u1Type = D_SP_ROADMARK_TYPE_BROKEN_SOLID;
            }
            else if (roadMarkType == RoadMark::TYPE_SOLIDBROKEN)
            {
                road_mark.u1Type = D_SP_ROADMARK_TYPE_SOLID_BROKEN;
            }
            else if (roadMarkType == RoadMark::TYPE_SOLIDSOLID)
            {
                road_mark.u1Type = D_SP_ROADMARK_TYPE_SOLID_SOLID;
            }
            else
            {
                road_mark.u1Type = D_SP_ROADMARK_TYPE_UNDECIDED;
            }

            RoadMark::RoadMarkColor const roadMarkColor = roadMark->getColor();
            if (roadMarkColor == RoadMark::COLOR_STANDARD)
            {
                road_mark.u1Color = D_SP_ROADMARK_COLOR_WHITE;
            }
            else if (roadMarkColor == RoadMark::COLOR_YELLOW)
            {
                road_mark.u1Color = D_SP_ROADMARK_COLOR_YELLOW;
            }
            else if (roadMarkColor == RoadMark::COLOR_BLUE)
            {
                road_mark.u1Color = D_SP_ROADMARK_COLOR_BLUE;
            }
            else
            {
                road_mark.u1Color = D_SP_ROADMARK_COLOR_UNDECIDED;
            }

            road_mark.u1LeftCrossing = 0;
            road_mark.u1RightCrossing = 0;

            double egoOffset = egoState->sObjectState.u4LaneOffset;
            double lanewidth = static_cast<float>(fabs(laneSection->getLaneWidth(egoState->sObjectState.u4RoadS, laneId)));

            auto roadMarkInfo = laneSection->getLaneRoadMark(egoState->sObjectState.u4RoadS, laneId);
            road_mark.u4Width = roadMarkInfo->getWidth();

            if (laneId == egoState->sObjectState.u1LaneId)
            {
                if (egoOffset > 0) //向左偏移
                {
                    if ((lanewidth / 2) - egoOffset <= 0.7)
                    {
                        road_mark.u1LeftCrossing = 1;
                    }
                }
                else //向右偏移
                {
                    if ((lanewidth / 2) - fabs(egoOffset) <= 0.7)
                    {
                        road_mark.u1RightCrossing = 1;
                    }
                }
            }

            // 车道线拟合（俯视图）
            Transform egoTransform; //获取主车世界坐标
            egoTransform.update(egoState->sObjectState.sPos.u8X, egoState->sObjectState.sPos.u8Y, egoState->sObjectState.sPos.u8Z);

            double egoX = egoTransform.v().x();
            double egoY = egoTransform.v().y();
            double egoZ = egoTransform.v().z();
            double yaw = egoState->sObjectState.sPos.u4H; //获取世界坐标系下的航向角
            double cameraX = egoX + 1.5 * cos(yaw);
            double cameraY = egoY + 1.5 * sin(yaw);
            egoTransform.update(cameraX, cameraY, egoZ);

            float s = egoState->sObjectState.u4RoadS;
            int direction = 1; //方向
            if (egoState->sObjectState.u1Dir == D_SP_ROAD_DIR_FLAG_DIR_FWD)
            {
                direction = 1;
            }
            else if (egoState->sObjectState.u1Dir == D_SP_ROAD_DIR_FLAG_DIR_REAR)
            {
                direction = -1;
            }

            double roadLength = road->getLength();
            double sampleS = s; //采样点s坐标
            //获取左侧右侧车道线的一系列坐标点（世界坐标）
            std::list<Vector3D> pointList; //车道线的点
            for (int i = 0; i < 50; i++) //取50个点
            {
                RoadPoint pointIn(0, 0, 0, 0, 0, 0); //内侧车道线的世界坐标
                RoadPoint pointOut(0, 0, 0, 0, 0, 0); //外侧车道线的世界坐标
                double disIn = 0;
                double disOut = 0;
                //获取内侧与外侧车道线的世界坐标和t坐标
                road->getLaneRoadPoints(sampleS, laneId, pointIn, pointOut, disIn, disOut);
                Vector3D pointOutVector3D(pointOut.x(), pointOut.y(), pointOut.z()); //只用外侧车道线上的点
                pointList.push_back(pointOutVector3D);

                //更新sampleS，即下一个采样点s坐标
                double samplesBackup = sampleS ;  //记录上一个点的位置
                bool endCheck = (((fabs(0.0 - sampleS) < 0.01) && direction < 0 ) || ((fabs(roadLength - sampleS ) < 0.01)  && direction > 0 )) ? true : false ; //判断是否车辆位于道路终点且车辆是正向行驶

                sampleS += 2.0 * direction; //间隔2米
                if (sampleS < 0 || sampleS > roadLength)
                {
                    unsigned int nodeCount = 5; //3项式最少需要5个点才能产生拟合（有常数项）
                    if (pointList.size() < nodeCount ) //检查采样数量是否满足拟合条件
                    {
                        double peakVal = 0.0;
                        if (endCheck) //判断是否车辆位于道路终点
                        {
                            if (roadLength < 2) //判断道路长度是否小于2
                            {
                                peakVal = (direction > 0) ? 0.0 : roadLength ; //车辆位于道路终点时 设置为道路的另一端
                            }
                            else
                            {
                                peakVal = (direction > 0) ? roadLength - 2.0 : 2.0; //如果道路长度，道路终点设置为当前方向尽头位置-2m；
                            }
                        }
                        else
                        {
                            peakVal = (direction > 0) ? roadLength : 0.0 ; //根据方向判断道路的尽头是起点 还是 终点
                        }

                        int paddingSamples = nodeCount - pointList.size(); //判断需要补充的采样点数量
                        double interval = (peakVal - samplesBackup ) / (double)paddingSamples ; //根据道路尽头和上一个采样点计算采样间隔
                        for (int i = 0 ; i < paddingSamples ; i++) //以为末采样点与尽头之间的均间距提供满5个采样点
                        {
                            samplesBackup += interval;
                            if (samplesBackup > roadLength) //如果采样点超过道路长度
                            {
                                samplesBackup = roadLength ;
                            }
                            else if (samplesBackup < 0.0) //如果采样点小于0
                            {
                                samplesBackup = 0.0 ;
                            }

                            road->getLaneRoadPoints(samplesBackup, laneId, pointIn, pointOut, disIn, disOut);
                            Vector3D pointOutendVector3D(pointOut.x(), pointOut.y(), pointOut.z()); //只用外侧车道线上的点
                            pointList.push_back(pointOutendVector3D);
                        }
                    }
                    break;
                }
            }

            //世界坐标点转为相对主车坐标系
            std::list<Vector3D> relativePointList; //车道线的点
            for (const auto &i : pointList)
            {
                //计算相对坐标，即转换为主车坐标系，x轴向前，y轴向左
                Vector3D relativeCoord(0, 0, 0);
                egoTransform.relativeCoordinate(yaw, i, relativeCoord);
                relativePointList.push_back(relativeCoord);
            }

            std::vector<double> xVec;
            std::vector<double> yVec;
            int order = 3; //3次多项式
            //拟合车道线
            for (const auto &i : relativePointList)
            {
                xVec.push_back(i.x());
            }
            for (const auto &i : relativePointList)
            {
                yVec.push_back(i.y() * (-1)); // 乘以 -1 是因为一汽坐标系y轴和仿真引擎主车坐标系相反
            }
            std::vector<double> coeffs = PolynomialRegression::fit(xVec, yVec, order); //曲线拟合，返回值为a0~a3
            road_mark.u4C0 = coeffs[0];
            road_mark.u4C1 = coeffs[1];
            road_mark.u4C2 = coeffs[2];
            road_mark.u4C3 = coeffs[3];

            if (fabs(road->getCurvature(egoState->sObjectState.u4RoadS)) > 0.3)
            {
                road_mark.u2LaneChange = 1;
            }
            else
            {
                road_mark.u2LaneChange = 0;
            }

            roadMarks.push_back(road_mark);
        }
    }

    Road *nextRaod = nullptr;//下条路
    int nextRoadDir = 0;//在下条路上的行驶方向

    getNextRoadByLightMask(nextRaod, nextRoadDir);

    if (nextRaod != nullptr)
    {
        LaneSection *laneSection2 = nullptr;

        if (nextRoadDir == 1)//道路S轴正方向
        {
            laneSection2 = nextRaod->getLaneSection(0.0);
        }
        else if (nextRoadDir == -1)//道路S轴负方向
        {
            laneSection2 = nextRaod->getLaneSection(nextRaod->getLength());
        }
        else
        {
            //nothing to do
        }

        if (laneSection2 != nullptr)
        {
            std::map<int, Lane *> laneMap = laneSection2->getLaneMap();
            uint64_t nextRoadId = static_cast<uint64_t>(TrafficSimulation::stoul(nextRaod->getId()));

            //填充roadMark元素
            for (std::map<int, Lane *>::iterator laneMapIt = laneMap.begin(); laneMapIt != laneMap.end(); laneMapIt++)
            {
                int laneId = laneMapIt->first;
                Lane *lane = laneMapIt->second;
                std::map<double, RoadMark *> roadMarkMap = lane->getRoadMarkMap();

                for (std::map<double, RoadMark *>::iterator roadMarkMapIt = roadMarkMap.begin(); roadMarkMapIt != roadMarkMap.end(); roadMarkMapIt++)
                {
                    S_SP_MIL_ROADMARK road_mark = {0};

                    RoadMark *roadMark = roadMarkMapIt->second;
                    double roadMarkStart = roadMark->getStart();

                    //填充数据
                    road_mark.u8RoadId = nextRoadId;
                    road_mark.u4StartDx = roadMarkStart;
                    road_mark.u4ViewRangeStart = 0.0;
                    road_mark.u4ViewRangeEnd = 100.0;
                    road_mark.u4MeasuredVREnd = 100.0;
                    road_mark.u1LaneId = static_cast<int8_t>(laneId);
                    road_mark.u1Quality = 3; //0 - 1: invalid 2: predicated 3: real

                    RoadMark::RoadMarkType const roadMarkType = roadMark->getType();
                    if (roadMarkType == RoadMark::TYPE_NONE)
                    {
                        road_mark.u1Type = D_SP_ROADMARK_TYPE_UNDECIDED;
                    }
                    else if (roadMarkType == RoadMark::TYPE_SOLID)
                    {
                        road_mark.u1Type = D_SP_ROADMARK_TYPE_SOLID;
                    }
                    else if (roadMarkType == RoadMark::TYPE_BROKEN)
                    {
                        road_mark.u1Type = D_SP_ROADMARK_TYPE_BROKEN;
                    }
                    else if (roadMarkType == RoadMark::TYPE_BROKENSOLID)
                    {
                        road_mark.u1Type = D_SP_ROADMARK_TYPE_BROKEN_SOLID;
                    }
                    else if (roadMarkType == RoadMark::TYPE_SOLIDBROKEN)
                    {
                        road_mark.u1Type = D_SP_ROADMARK_TYPE_SOLID_BROKEN;
                    }
                    else if (roadMarkType == RoadMark::TYPE_SOLIDSOLID)
                    {
                        road_mark.u1Type = D_SP_ROADMARK_TYPE_SOLID_SOLID;
                    }
                    else
                    {
                        road_mark.u1Type = D_SP_ROADMARK_TYPE_UNDECIDED;
                    }

                    RoadMark::RoadMarkColor const roadMarkColor = roadMark->getColor();
                    if (roadMarkColor == RoadMark::COLOR_STANDARD)
                    {
                        road_mark.u1Color = D_SP_ROADMARK_COLOR_WHITE;
                    }
                    else if (roadMarkColor == RoadMark::COLOR_YELLOW)
                    {
                        road_mark.u1Color = D_SP_ROADMARK_COLOR_YELLOW;
                    }
                    else if (roadMarkColor == RoadMark::COLOR_BLUE)
                    {
                        road_mark.u1Color = D_SP_ROADMARK_COLOR_BLUE;
                    }
                    else
                    {
                        road_mark.u1Color = D_SP_ROADMARK_COLOR_UNDECIDED;
                    }

                    road_mark.u1LeftCrossing = 0;
                    road_mark.u1RightCrossing = 0;

                    double lanewidth = 0.0;
                    float s = 0.0;
                    if (nextRoadDir == 1)//下条路s轴正方向
                    {
                        lanewidth = static_cast<float>(fabs(laneSection2->getLaneWidth(0.0, laneId)));
                        s = 0.0;
                    }
                    else if (nextRoadDir == -1)//下条路s轴负方向
                    {
                        lanewidth = static_cast<float>(fabs(laneSection2->getLaneWidth(nextRaod->getLength(), laneId)));
                        s = nextRaod->getLength();
                    }
                    else
                    {
                        //nothing to do
                    }

                    road_mark.u4Width = lanewidth;

                    Transform egoTransform; //获取主车世界坐标
                    egoTransform.update(egoState->sObjectState.sPos.u8X, egoState->sObjectState.sPos.u8Y, egoState->sObjectState.sPos.u8Z);

                    double egoX = egoTransform.v().x();
                    double egoY = egoTransform.v().y();
                    double egoZ = egoTransform.v().z();

                    double yaw = egoState->sObjectState.sPos.u4H; //获取世界坐标系下的航向角
                    double cameraX = egoX + 1.5 * cos(yaw);
                    double cameraY = egoY + 1.5 * sin(yaw);
                    egoTransform.update(cameraX, cameraY, egoZ);
                    double roadLength = nextRaod->getLength();
                    double sampleS = s; //采样点s坐标
                    //获取左侧右侧车道线的一系列坐标点（世界坐标）
                    std::list<Vector3D> pointList; //车道线的点
                    for (int i = 0; i < 50; i++) //取50个点
                    {
                        RoadPoint pointIn(0, 0, 0, 0, 0, 0); //内侧车道线的世界坐标
                        RoadPoint pointOut(0, 0, 0, 0, 0, 0); //外侧车道线的世界坐标
                        double disIn = 0;
                        double disOut = 0;
                        //获取内侧与外侧车道线的世界坐标和t坐标
                        nextRaod->getLaneRoadPoints(sampleS, laneId, pointIn, pointOut, disIn, disOut);
                        Vector3D pointOutVector3D(pointOut.x(), pointOut.y(), pointOut.z()); //只用外侧车道线上的点
                        pointList.push_back(pointOutVector3D);

                        //更新sampleS，即下一个采样点s坐标
                        double samplesBackup = sampleS ;  //记录上一个点的位置
                        bool endCheck = (((fabs(0.0 - sampleS) < 0.01) && nextRoadDir < 0 ) || ((fabs(roadLength - sampleS ) < 0.01)  && nextRoadDir > 0 )) ? true : false ; //判断是否车辆位于道路终点且车辆是正向行驶
                        sampleS += 2.0 * nextRoadDir; //间隔2米
                        if (sampleS < 0 || sampleS > roadLength)
                        {
                            unsigned int nodeCount = 5; //3项式最少需要5个点才能产生拟合（有常数项）
                            if (pointList.size() < nodeCount ) //检查采样数量是否满足拟合条件
                            {
                                double peakVal = 0.0;
                                if (endCheck) //判断是否车辆位于道路终点
                                {
                                    if (roadLength < 2) //判断道路长度是否小于2
                                    {
                                        peakVal = (nextRoadDir > 0) ? 0.0 : roadLength ; //车辆位于道路终点时 设置为道路的另一端
                                    }
                                    else
                                    {
                                        peakVal = (nextRoadDir > 0) ? roadLength - 2.0 : 2.0; //如果道路长度，道路终点设置为当前方向尽头位置-2m；
                                    }
                                }
                                else
                                {
                                    peakVal = (nextRoadDir > 0) ? roadLength : 0.0 ; //根据方向判断道路的尽头是起点 还是 终点
                                }
                                int paddingSamples = nodeCount - pointList.size(); //判断需要补充的采样点数量
                                double interval = (peakVal - samplesBackup ) / (double)paddingSamples ; //根据道路尽头和上一个采样点计算采样间隔
                                for (int i = 0 ; i < paddingSamples ; i++) //以为末采样点与尽头之间的均间距提供满5个采样点
                                {
                                    samplesBackup += interval;
                                    if (samplesBackup > roadLength) //如果采样点超过道路长度
                                    {
                                        samplesBackup = roadLength ;
                                    }
                                    else if (samplesBackup < 0.0) //如果采样点小于0
                                    {
                                        samplesBackup = 0.0 ;
                                    }
                                    nextRaod->getLaneRoadPoints(samplesBackup, laneId, pointIn, pointOut, disIn, disOut);
                                    Vector3D pointOutendVector3D(pointOut.x(), pointOut.y(), pointOut.z()); //只用外侧车道线上的点
                                    pointList.push_back(pointOutendVector3D);
                                }
                            }
                            break;
                        }
                    }

                    //世界坐标点转为相对主车坐标系
                    std::list<Vector3D> relativePointList; //车道线的点
                    for (const auto &i : pointList)
                    {
                        //计算相对坐标，即转换为主车坐标系，x轴向前，y轴向左
                        Vector3D relativeCoord(0, 0, 0);
                        egoTransform.relativeCoordinate(yaw, i, relativeCoord);
                        relativePointList.push_back(relativeCoord);
                    }

                    std::vector<double> xVec;
                    std::vector<double> yVec;
                    int order = 3; //3次多项式
                    //拟合车道线
                    for (const auto &i : relativePointList)
                    {
                        xVec.push_back(i.x());
                    }
                    for (const auto &i : relativePointList)
                    {
                        yVec.push_back(i.y() * (-1)); // 乘以 -1 是因为一汽坐标系y轴和仿真引擎主车坐标系相反
                    }
                    std::vector<double> coeffs = PolynomialRegression::fit(xVec, yVec, order); //曲线拟合，返回值为a0~a3
                    road_mark.u4C0 = coeffs[0];
                    road_mark.u4C1 = coeffs[1];
                    road_mark.u4C2 = coeffs[2];
                    road_mark.u4C3 = coeffs[3];
                    road_mark.u2LaneChange = 0;

                    roadMarks.push_back(road_mark);
                }
            }
        }
    }

    // 车道线拟合调用优化
    setRoadMarkInfo(roadMarks);
    log_compnt_mngr->info("Sensor::getRoadMark end.");
    return true;
}

std::vector<S_SP_MIL_ROADMARK> Sensor::getRoadMarkInfo()
{
    return roadMarkInfo;
}

void Sensor::setRoadMarkInfo(std::vector<S_SP_MIL_ROADMARK> &roadMarks)
{
    roadMarkInfo = roadMarks;
}

int Sensor::getSensorFrameRate()
{
    return getFrameRate();
}

// 高斯误差函数
double Sensor::getGaussRand(double sigma)
{
    double x = 0.0;
    double Z = 0.0;

    double r1 = (double)rand() / RAND_MAX;
    double r2 = (double)rand() / RAND_MAX;
    Z = sqrt(-2 * log(r2)) * cos(2 * M_PI * r1);
    x = sqrt(sigma) * Z;
    return x;
}

// 球坐标系转笛卡尔坐标系
void Sensor::getCartesianCoord(double _range, double _azimuth, double _elevation, Vector3D &vCoordSys)
{
    vCoordSys[0] = _range * cos(_elevation) * cos(_azimuth);
    vCoordSys[1] = _range * cos(_elevation) * sin(_azimuth);
    vCoordSys[2] = _range * sin(_elevation);
}
// 笛卡尔坐标系转球坐标系
void Sensor::getSphereCoord(double _x, double _y, double _z, Vector3D &vCoordSys)
{
    vCoordSys[0] = sqrt(_x * _x + _y * _y + _z * _z);
    vCoordSys[1] = atan2(_y, _x); // 方位角
    vCoordSys[2] = asin(_z / vCoordSys[0]); // 高低角
}

Node* Sensor::build_kdtree(std::vector<tuple<int, int, Vector3D>>& points, int start, int end, int depth)
{
    if (start == end) {
        return nullptr;
    }
    int axis = depth % 3;
    std::nth_element(points.begin() + start, points.begin() + (start + end) / 2, points.begin() + end, [axis](const tuple<int, int, Vector3D>& a, const tuple<int, int, Vector3D>& b) {
        if (axis == 0) {
            return std::get<2>(a).x() < std::get<2>(b).x();
        } else if (axis == 1) {
            return std::get<2>(a).y() < std::get<2>(b).y();
        } else {
            return std::get<2>(a).z() < std::get<2>(b).z();
        }
    });
    Node* root = new Node(std::get<2>(points[(start + end) / 2]));
    root->left = build_kdtree(points, start, (start + end) / 2, depth + 1);
    root->right = build_kdtree(points, (start + end) / 2 + 1, end, depth + 1);
    return root;
}

// 寻找每个物体对应的最近点
void Sensor::kdtree_nearest_neighbor(Node* root, const Vector3D& target, Vector3D& nearest, double& minDist)
{
    log_compnt_mngr->info("Sensor::kdtree_nearest_neighbor start.");
    // 使用栈来实现dfs
    stack<Node*> s;
    // 计算两点之间的欧氏距离
    double dist = (root->point.x() - target.x()) * (root->point.x() - target.x()) +
                (root->point.y() - target.y()) * (root->point.y() - target.y()) +
                (root->point.z() - target.z()) * (root->point.z() - target.z());
    while (root || !s.empty())
    {
        while (root)
        {
            s.push(root);
            if (dist < minDist)
            {
                nearest = root->point;
                minDist = dist;
            }
            if (target.x() < root->point.x())
            {
                root = root->left;
            }
            else
            {
                root = root->right;
            }
        }
        root = s.top();
        s.pop();
        if (fabs(root->point.x() - target.x()) > minDist)
        {
            break;
        }
        if (dist < minDist)
        {
            nearest = root->point;
            minDist = dist;
        }
        if (target.x() < root->point.x())
        {
            root = root->right;
        }
        else
        {
            root = root->left;
        }
    }
    log_compnt_mngr->info("Sensor::kdtree_nearest_neighbor end.");
}

// 保存目标id和欧氏距离最短目标的相对距离，相对水平角，相对俯仰角
void Sensor::getNearestObjRelCoordToTarget(float insensitiveRadius, std::map<int, Vector3D> &nearestObjRelCoord)
{
    log_compnt_mngr->info("Sensor::getNearestObjRelCoordToTarget start.");
    float relRange = 0.0;		// 相对距离
    float relAzimuth = 0.0; 	// 相对水平角
    // float relElevation = 0.0; 	// 相对俯仰角
    float aziRad = 0.0; 		// 水平角不敏感半径

    // 找到每个点的最近邻点
    for (int i = 0; i < sortedObjs.size(); i++)
    {
        Vector3D target = std::get<2>(sortedObjs[i]); // 目标传感器坐标系下的坐标
        Vector3D nearestObj; // 欧氏距离最近物体传感器坐标系下的坐标
        double min_dist = std::numeric_limits<double>::max();

        for (int j = 0; j < sortedObjs.size(); j++)
        {
            Vector3D object = std::get<2>(sortedObjs[j]); // 对照物1传感器坐标系下的坐标
            if (std::get<0>(sortedObjs[j]) == std::get<0>(sortedObjs[i]))
            {
                continue;
            }

            // 计算两点之间的欧氏距离
            double dist = (object.x() - target.x()) * (object.x() - target.x()) +
                        (object.y() - target.y()) * (object.y() - target.y()) +
                        (object.z() - target.z()) * (object.z() - target.z());

            if (dist < min_dist)
            {
                nearestObj = object;
                min_dist = dist;
            }
            else
            {
                continue;
            }
        }

        log_compnt_mngr->debug("targetX, targetY targetZ {},{},{}",target.x(),target.y(),target.z());
        log_compnt_mngr->debug("nearest obj1X, obj1Y obj1Z {},{},{}", nearestObj.x(),nearestObj.y(),nearestObj.z());

        // 计算球坐标系坐标
        Vector3D targetSphereCoord(0.0,0.0,0.0);
        Vector3D ObjSphereCoord(0.0,0.0,0.0);
        getSphereCoord(target.x(),target.y(),target.z(), targetSphereCoord);
        getSphereCoord(nearestObj.x(),nearestObj.y(),nearestObj.z(), ObjSphereCoord);

        relRange = fabs((targetSphereCoord[0] - insensitiveRadius) - (ObjSphereCoord[0] - insensitiveRadius));
        aziRad = asin(insensitiveRadius/sqrt(target.x() * target.x() + target.y() * target.y())) + asin(insensitiveRadius/sqrt(nearestObj.x() * nearestObj.x() + nearestObj.y() * nearestObj.y()));
        relAzimuth = relu0(fabs(targetSphereCoord[1] - ObjSphereCoord[1]), aziRad);

        log_compnt_mngr->debug("relRange, relAzimuth {},{}",relRange,relAzimuth);
        nearestObjRelCoord[std::get<0>(sortedObjs[i])] = Vector3D(relRange, relAzimuth, 0);
    }
    log_compnt_mngr->info("Sensor::getNearestObjRelCoordToTarget end.");
}

// 目标分类正确的可能性
double Sensor::getClassificationAccuracy(int targetId, std::map<int, Vector3D>& nearestObjRelCoord)
{
    log_compnt_mngr->info("Sensor::getClassificationAccuracy start.");
    log_compnt_mngr->debug("targetId {}", targetId);

    // 除雷达外不输出目标分类正确的可能性
    if (getRangeMSENoise() == 0.0 && getAzimuthMSENoise() == 0.0)
    {
        log_compnt_mngr->debug("Sensor::getClassificationAccuracy return 1.");
        log_compnt_mngr->info("Sensor::getClassificationAccuracy end.");
        return 1;
    }

    double Pr = 0.0; // 距离维度上，不能被正确分类的概率
    double Pa = 0.0; // 水平角维度上，不能被正确分类的概率
    // double Pe = 0.0; // 俯仰角维度上，不能被正确分类的概率
    double classificationAccuracy = 0.0; // 正确分类的概率

    std::map<int, Vector3D>::iterator iter = nearestObjRelCoord.find(targetId);

    if (iter != nearestObjRelCoord.end() && nearestObjRelCoord.size() > 1)
    {
        float rangeDim = iter->second.x(); 	// 距离维度，最近物体的相对距离
        float azimuthDim = iter->second.y();	// 水平角维度，最近物体的相对水平角
        // float elevationDim = iter->second.z(); 	// 俯仰角维度，最近物体的相对俯仰角
        azimuthDim = azimuthDim / SENSOR_OBJECT_DETECTION_PI * 180.0; // 弧度转角度

        log_compnt_mngr->debug("rangeDim, azimuthDim {},{}",rangeDim,azimuthDim);
        Pr = 0.5 * erfc( (rangeDim + getRangeResolution()) / (sqrt(2 * getRangeMSENoise()) + 0.001) ) - 0.5 * erfc( (rangeDim - getRangeResolution()) / (sqrt(2 * getRangeMSENoise()) + 0.001) );
        Pa = 0.5 * erfc( (azimuthDim + getAzimuthResolution()) / (sqrt(2 * getAzimuthMSENoise()) + 0.001) ) - 0.5 * erfc( (azimuthDim - getAzimuthResolution()) / (sqrt(2 * getAzimuthMSENoise()) + 0.001) );
    }

    classificationAccuracy = 1 - fabs(Pr) * fabs(Pa);
    log_compnt_mngr->debug("classificationAccuracy {}", classificationAccuracy);

    log_compnt_mngr->info("Sensor::getClassificationAccuracy end.");
    return classificationAccuracy;
}

// 有效性标识 (传感器坐标系下的目标位置，目标的相对传感器姿态角)
void Sensor::getValidityLabel(int curID, Vector3D curPos, Vector3D curOri)
{
    log_compnt_mngr->info("Sensor::getValidityLabel start.");
    double invalidityX; // X轴方向有效性
    double invalidityY; // Y轴方向有效性
    double invalidityThreshold = 0.9994; // 有效性阈值: [Mod] bug16053 修改阈值 -> cos(2Deg) = 0.99939

    // 找到两个目标之间可能反射出的无效目标
    for (int i = 0; i < sortedObjs.size() ; i++)
    {
        if (curID == std::get<0>(sortedObjs[i]))
        {
            log_compnt_mngr->warn("curID {} == std::get<0>(sortedObjs[{}] {}", curID, i, std::get<0>(sortedObjs[i]));
            continue;
        }

        Vector3D object = std::get<2>(sortedObjs[i]); // 目标传感器坐标系下的坐标

        // 物体与目标的相对坐标向量
        Vector3D relCoord(object.x() - curPos.x(), object.y() - curPos.y(), object.z() - curPos.z());
        // 当相对距离大时，忽略反射
        if (pow(relCoord[0], 2) + pow(relCoord[1], 2) >= RADAR_FALSE_ALARM_RANGE)
        {
            log_compnt_mngr->warn("getValidityLabel distance >= RADAR_FALSE_ALARM_RANGE, continue.");
            continue;
        }

        // 物体与目标的相对坐标向量归一化
        double relCoordLen = sqrt( pow(relCoord.x(), 2) + pow(relCoord.y(), 2) + pow(relCoord.z(), 2) );
        relCoord[0] = relCoord.x() / relCoordLen;
        relCoord[1] = relCoord.y() / relCoordLen;
        relCoord[2] = relCoord.z() / relCoordLen;

        // 目标传感器坐标下向量归一化
        double curCoordLen = sqrt( pow(curPos.x(), 2) + pow(curPos.y(), 2) + pow(curPos.z(), 2) );
        curPos[0] = curPos.x() / curCoordLen;
        curPos[1] = curPos.y() / curCoordLen;
        curPos[2] = curPos.z() / curCoordLen;

        // 反射向量
        Vector3D invalidVec(relCoord[0] - curPos[0], relCoord[1] - curPos[1], relCoord[2] - curPos[2]);
        // 反射向量归一化
        double invalidVecLen = sqrt( pow(invalidVec.x(), 2) + pow(invalidVec.y(), 2) + pow(invalidVec.z(), 2) );
        invalidVec[0] = invalidVec.x() / invalidVecLen;
        invalidVec[1] = invalidVec.y() / invalidVecLen;
        invalidVec[2] = invalidVec.z() / invalidVecLen;

        Vector3D curOriX(1, 0, 0); // x轴基向量
        Vector3D curOriY(0, 1, 0); // y轴基向量
        // 旋转基向量
        VectorRotated(curOri[0], curOri[1], curOri[2], curOriX); // 旋转后的x轴基向量
        VectorRotated(curOri[0], curOri[1], curOri[2], curOriY); // 旋转后的y轴基向量

        invalidityX = fabs(invalidVec.x() * curOriX.x() + invalidVec.y() * curOriX.y() + invalidVec.z() * curOriX.z());
        invalidityY = fabs(invalidVec.x() * curOriY.x() + invalidVec.y() * curOriY.y() + invalidVec.z() * curOriY.z());
        log_compnt_mngr->debug("invalidityX, invalidityY {},{}",invalidityX,invalidityY);
        if (invalidityX > invalidityThreshold || invalidityY > invalidityThreshold)
        {
            // 反射向量
            invalidVec[0] = (relCoordLen + curCoordLen) * curPos[0];
            invalidVec[1] = (relCoordLen + curCoordLen) * curPos[1];
            invalidVec[2] = (relCoordLen + curCoordLen) * curPos[2];

            ghostInfo.push_back(make_tuple(std::get<0>(sortedObjs[i]), std::get<1>(sortedObjs[i]), invalidVec));
        }
    }
    log_compnt_mngr->info("Sensor::getValidityLabel end.");
}

// 计算无效目标的位置信息等
bool Sensor::getInvalidObjectInfo(std::vector<tuple<int, int, Vector3D>>& ghostInfo)
{
    if (ghostInfo.size() == 0)
    {
        return false;
    }
    return true;
}

Vector3D Sensor::calDistCoord(Vector3D _poi)	// 计算三维的坐标点在传感器处理后发生了畸变后的坐标
{
    // 对于默认情况，是不需要做畸变处理的
    return _poi;
}

Lidar::Lidar(LidarDescription description) : _description(description)
{
    Sensor::_description = (SensorDescription)description;
}

S_SP_SENSOR_INFO Lidar::getState()
{
    S_SP_SENSOR_INFO state{0};

    state.u1Type = D_SP_SENSOR_TYPE_LIDAR;
    //state.hostCategory = ASIM_OBJECT_CATEGORY_SENSOR;
    state.u8HostId = 1; // 主车持有传感器，主车Id为1
    (void)strncpy(state.au1Name, _description.name, D_SP_SIZE_OBJECT_NAME);

    state.au4FovHV[0] = _description.hFov;								// 视野（水平）
    state.au4FovHV[1] = _description.vFov; 								// 视野（垂直）
    state.au4ClipNF[0] = _description.minimumDetectRange;               // 裁剪范围（近）
    state.au4ClipNF[1] = _description.range;							// 裁剪范围（远）
    // 传感器坐标系下传感器的坐标
    state.sPos.u8X = 0.0;
    state.sPos.u8Y = 0.0;
    state.sPos.u8Z = 0.0;
    state.sPos.u4H = 0.0;
    state.sPos.u4P = 0.0;
    state.sPos.u4R = 0.0;
    // 主车坐标系下传感器的坐标
    state.sOriginCoordSys.u8X = _description.assemblePositionX;
    state.sOriginCoordSys.u8Y = _description.assemblePositionY;
    state.sOriginCoordSys.u8Z = _description.assemblePositionZ;

    state.sOriginCoordSys.u4H = _description.heading;
    state.sOriginCoordSys.u4P = _description.pitch;
    state.sOriginCoordSys.u4R = _description.roll;

    state.au4FovOffHV[0] = 0.0; // 视场偏移（水平）
    state.au4FovOffHV[1] = 0.0; // 视场偏移（垂直）

    return state;
}

double Lidar::getAssemblePositionX()
{
    return _description.assemblePositionX;
}

double Lidar::getAssemblePositionY()
{
    return _description.assemblePositionY;
}

double Lidar::getAssemblePositionZ()
{
    return _description.assemblePositionZ;
}

//取得探测范围
double Lidar::getRange()
{
    return _description.range;
}

//取得Heading Pitch Roll
double Lidar::getHeading()
{
    return _description.heading * SENSOR_OBJECT_DETECTION_PI / 180.0 ;
}
double Lidar::getPitch()
{
    return _description.pitch * SENSOR_OBJECT_DETECTION_PI / 180.0;
}

double Lidar::getRoll()
{
    return _description.roll * SENSOR_OBJECT_DETECTION_PI / 180.0;
}

//取得HFOV
double Lidar::getHFOV()
{
    return _description.hFov * SENSOR_OBJECT_DETECTION_PI / 180.0;
}

//取得VFOV
double Lidar::getVFOV()
{
    return _description.vFov * SENSOR_OBJECT_DETECTION_PI / 180.0;
}

//获取最近探测距离
double Lidar::getMinimumDetectRange()
{
    return _description.minimumDetectRange;
}

double Lidar::getDetectedPR(double distance,float radarCrossSection, float weather, float occlusionScale)
{
    log_compnt_mngr->info("Lidar::getDetectedPR start.");
    double detectPR = 1.0;
    if (_description.bExistProbInput == true)
    {
        detectPR = _description.fExistProb / 100;
    }
    else
    {
        double occlusionScaleThreshold = 0.15; // 遮挡比例影响阈值
        double reflection = 0.5;
        double falseAlarmRate = 0.0000001;
        if (radarCrossSection < 1)
        {
            reflection = 0.3;
        }
        double sigma = 3.91 / 50 * pow(550 / _description.fWavelength, 1.6);
        double Pr = (_description.peakOpticalPower * _description.fTransmittingAperture * _description.fReceivingAperture)
                        / (M_PI * pow(distance, 2) * ( (_description.fHBeamDivergenceAngle * _description.fVBeamDivergenceAngle) * SENSOR_OBJECT_DETECTION_PI / 180.0 ))
                        * reflection * _description.fTransmitTransmissivity * _description.fReceiveTransmissivity
                        * exp(-sigma * distance);
        double SignalNoiseRate = Pr / (_description.fNEP * _description.fNoiseBandwidth * 1e-6); // 信噪比

        if (weather != 0)
        {
            SignalNoiseRate *= weather;// 环境因子影响信噪比
        }

        detectPR = 0.5 * erfc ( (sqrt( -log((falseAlarmRate))) - sqrt(SignalNoiseRate + 0.5) ) ) * minPct(1 , (1 - occlusionScale)/(1 - occlusionScaleThreshold)); //获得检测概率
    }

    log_compnt_mngr->info("Lidar::getDetectedPR end.");
    return detectPR;
}

int Lidar::getFrameRate()
{
    return _description.frameRate;
}

// 判定是否使能高级设置
bool Lidar::isEnableAdvSettings()
{
    return _description.advSettingsEnable;
}

double Lidar::getRangeMSENoise()
{
    return 0.1;
}

double Lidar::getAzimuthMSENoise()
{
    return 0.1;
}

double Lidar::getElevationMSENoise()
{
    return 0.1;
}

double Lidar::getRangeResolution()
{
    return 0.5;
}

double Lidar::getAzimuthResolution()
{
    return _description.horizontalResolution;
}

double Lidar::getElevationResolution()
{
    return _description.horizontalResolution;
}

double Lidar::getEnvNoise(double distance)
{
    return 0.0;
}

Camera::Camera(CameraDescription description) : _description(description)
{
    Sensor::_description = (SensorDescription)description;
}

//取得传感器状态
S_SP_SENSOR_INFO Camera::getState()
{

    S_SP_SENSOR_INFO state{0};

    state.u1Type = D_SP_SENSOR_TYPE_CAMERA;
    //state.hostCategory = ASIM_OBJECT_CATEGORY_SENSOR;
    state.u8HostId = 1; // 主车持有传感器，主车Id为1
    (void)strncpy(state.au1Name, _description.name, D_SP_SIZE_OBJECT_NAME);
    state.au4FovHV[0] = _description.hFov;			 // 视野（水平）
    state.au4FovHV[1] = _description.vFov;			 // 视野（垂直）
    state.au4ClipNF[0] = _description.minimumDetectRange; // 裁剪范围（近）
    state.au4ClipNF[1] = _description.range;		 // 裁剪范围（远）

    // 传感器坐标系下传感器的坐标
    state.sPos.u8X = 0.0;
    state.sPos.u8Y = 0.0;
    state.sPos.u8Z = 0.0;
    state.sPos.u4H = 0.0;
    state.sPos.u4P = 0.0;
    state.sPos.u4R = 0.0;
    // 主车坐标系下传感器的坐标
    state.sOriginCoordSys.u8X = _description.assemblePositionX;
    state.sOriginCoordSys.u8Y = _description.assemblePositionY;
    state.sOriginCoordSys.u8Z = _description.assemblePositionZ;

    // 主车坐标系下传感器的角度
    state.sOriginCoordSys.u4H = _description.heading;
    state.sOriginCoordSys.u4P = _description.pitch;
    state.sOriginCoordSys.u4R = _description.roll;

    state.au4FovOffHV[0] = 0.0; // 视场偏移（水平）
    state.au4FovOffHV[1] = 0.0; // 视场偏移（垂直）

    return state;
}

double Camera::getAssemblePositionX()
{
    return _description.assemblePositionX;
}

double Camera::getAssemblePositionY()
{
    return _description.assemblePositionY;
}

double Camera::getAssemblePositionZ()
{
    return _description.assemblePositionZ;
}

//取得探测范围
double Camera::getRange()
{
    return _description.range;
}

//取得Heading Pitch Roll
double Camera::getHeading()
{
    return _description.heading * SENSOR_OBJECT_DETECTION_PI / 180.0;
}
double Camera::getPitch()
{
    return _description.pitch * SENSOR_OBJECT_DETECTION_PI / 180.0;
}

double Camera::getRoll()
{
    return _description.roll * SENSOR_OBJECT_DETECTION_PI / 180.0;
}

//取得HFOV
double Camera::getHFOV()
{
    return _description.hFov * SENSOR_OBJECT_DETECTION_PI / 180.0;
}

//取得VFOV
double Camera::getVFOV()
{
    return _description.vFov * SENSOR_OBJECT_DETECTION_PI / 180.0;
}

//获取最近探测距离
double Camera::getMinimumDetectRange()
{
    return _description.minimumDetectRange;
}

// 计算上不gamma函数
float Camera::getIncomGamma(int n, float weather)
{
    float incom_gamma = 0.0; 				// 上不完全gamma函数
    double lambdaS = 0.3; 					// 背景噪声
    double lambdaD = 0.1; 					// 暗噪声泊松分布参数
    double lambdaObj = _description.probabilityDistribution; 	// 背景噪声
    double g = CAMERA_TRUNCATIONNOISE; 		// 随机变量的截断噪声

    lambdaObj *= weather;
    double s = lambdaObj + lambdaS + lambdaD; // 各噪声参数

    double a = 0.0;
    double dt = 0.0;
    double x = 0.0;
    double m = 100;
    double step_len = 0.1;

    for (int i = 1; i < m; i++)
    {
        x = n - g/2;
        dt = s + (i * step_len);
        a = pow(dt, (x - 1)) * exp(-dt) * step_len;

        incom_gamma += a; 		// 上不完全gamma函数
    }
    return incom_gamma;
}
// 计算gamma函数
float Camera::getGamma(int n)
{
    float gamma = 0.0; 			// gamma函数
    gamma = tgamma(n); 			// gamma函数
    return gamma;
}

double Camera::getDetectedPR(double distance,float radarCrossSection, float weather, float occlusionScale)
{
    double muB = 0.2;					// B的期望
    double sigmaB = 0.1;				// B的标准差
    double sigmaR = 0.1;				// R的标准差
    double n = CAMERA_PIXELNUM; 		// 像素数
    double g = CAMERA_TRUNCATIONNOISE; 	// 随机变量的截断噪声
    double t = _description.threshold;	// 阈值

    double objEffArea = radarCrossSection / (pow(3 * distance * std::tan((_description.hFov * SENSOR_OBJECT_DETECTION_PI / 180.0) / 2), 2)); // 物体的有效面积
    double EffAreaThreshold = 0.003; // 物体有效面积影响阈值
    double occlusionScaleThreshold = 0.15; // 遮挡比例影响阈值

    double erfArray = 0.0;
    double erfArray1 = 0.0;
    double erfArray2 = 0.0;
    double res = 0.0;
    for (int i = 0; i < n; i++)
    {

        erfArray1 = erff( ( i + 1 - t - muB) / (sqrt(2 * g * ( pow(sigmaB,2) + pow(sigmaR,2) ))) );
        erfArray2 = erff( ( i - t - muB) / (sqrt(2 * g * pow(sigmaB,2) + pow(sigmaR,2))) );
        erfArray = erfArray1 - erfArray2;

        res += getIncomGamma(i, weather) / getGamma(i+1) * erfArray; 	// 累加计算
    }

    double pct = (1 - (0.5 * res)) * minPct(objEffArea/EffAreaThreshold, 1) * minPct(1 , (1 - occlusionScale)/(1 - occlusionScaleThreshold));

    return pct;
}

int Camera::getFrameRate()
{
    return _description.frameRate;
}

// 判定是否使能高级设置
bool Camera::isEnableAdvSettings()
{
    return _description.advSettingsEnable;
}

double Camera::getRangeMSENoise()
{
    return 0.0;
}

double Camera::getAzimuthMSENoise()
{
    return 0.0;
}

double Camera::getElevationMSENoise()
{
    return 0.0;
}

double Camera::getRangeResolution()
{
    return 0.0;
}

double Camera::getAzimuthResolution()
{
    return 0.0;
}

double Camera::getElevationResolution()
{
    return 0.0;
}

double Camera::getEnvNoise(double distance)
{
    return 0.0;
}

/**
 * @Date: 2023-04-18 11:34:21
 * @Author: Zale
 * @LastEditors: Zale
 * @decription: 获取相机噪音/畸变处理后的坐标值
 * @param {vector<Vector3D>} &_pois 传入未经过噪音处理的原始坐标，传出畸变/噪音处理后的坐标
 * @return {*}
 */
Vector3D Camera::calDistCoord(Vector3D _poi)
{
    log_compnt_mngr->debug("Camera::calDistCoord _poi = ({}, {}, {}), modeltype=[{}]", _poi.x(), _poi.y(), _poi.z(), _description.modelType);
    if(_description.modelType != SENSOR_MODEL_PROBABLY)
    {
        log_compnt_mngr->error("Camera::calDistCoord Not Prob model, return directly");
        return _poi;
    }
    else
    {
        log_compnt_mngr->debug("Camera::calDistCoord is Prob model, continue");
    }
    double nan = sqrt(-1);
    Vector3D ret;
    // Step 1 将相机坐标系下的三维坐标转换到像平面上
    ret = con3dToCamPlane(_poi, _description.sensorFD/1000);
    if(std::isnan(ret.x()))
    {
        return _poi;
    }
    log_compnt_mngr->debug("Camera::calDistCoord s1 conv to CamP ret = ({}, {}, {})", ret.x(), ret.y(), ret.z());
    // Step 2 对像平面上的点的坐标添加畸变
    if(_description.distSw)	// 畸变开关打开
    {
        ret = getCamDistCoord2D(ret, _description.distParamK1*1e2, _description.distParamK2*1e2, _description.distCenterL/1e3, _description.distCenterV/1e3);
    }
    else
    {
        /* do nothing */
    }
    if(std::isnan(ret.x()))
    {
        return _poi;
    }
    log_compnt_mngr->debug("Camera::calDistCoord s2 add Dist ret = ({}, {}, {})", ret.x(), ret.y(), ret.z());
    // Step 3 对Step3上畸变后的坐标添加高斯噪声，调整了高斯噪声的数量级以符合
    ret.x() = ret.x() + getGaussRand(_description.gaussSigma)/1e6;
    ret.y() = ret.y() + getGaussRand(_description.gaussSigma)/1e6;
    if(std::isnan(ret.x()))
    {
        return _poi;
    }
    log_compnt_mngr->debug("Camera::calDistCoord s3 add Noise ret = ({}, {}, {})", ret.x(), ret.y(), ret.z());
    // Step 4 将像平面上的坐标转换到相机坐标系下的三维坐标
    ret = conCamPlaneTo3d(ret, _description.sensorFD/1000);
    if(std::isnan(ret.x()))
    {
        return _poi;
    }
    log_compnt_mngr->debug("Camera::calDistCoord s4 conv to 3D ret = ({}, {}, {})", ret.x(), ret.y(), ret.z());
    return ret;
}

/**
 * @Date: 2023-04-18 13:36:16
 * @Author: Zale
 * @LastEditors: Zale
 * @decription: 将相机坐标系的三维坐标转换到相机的像平面坐标上
 * @param {Vector3D} _poi 待转换的三维坐标
 * @return {Vector3D}  转换完成的像平面二维坐标x/y 和三维坐标的Z
 */
Vector3D Camera::con3dToCamPlane(Vector3D _poi, double _fd)
{
    log_compnt_mngr->debug("Camera::con3dToCamPlane _poi = ({}, {}, {}), fd=[{}]", _poi.x(), _poi.y(), _poi.z(), _fd);
    double nan = sqrt(-1);

    if(_fd <= 0 || abs(_poi.x()) < 1e-8 || abs(_poi.y()) < 1e-8)	// fd参数异常处理
    {
        log_compnt_mngr->debug("Camera::conCamPlaneTo3d End with fd=[{}], poix=[{}]!", _fd, _poi.x());
        return Vector3D(nan, nan, nan);
    }
    double xp = _fd *_poi.y() / _poi.x();
    double yp = _fd *_description.assemblePositionZ / _poi.x();

    log_compnt_mngr->debug("Camera::con3dToCamPlane ret = ({}, {}, {})", xp, yp, _poi.z());
    return Vector3D(xp, yp, _poi.z());
}

/**
 * @Date: 2023-04-23 15:03:49
 * @Author: Zale
 * @LastEditors: Zale
 * @decription: 计算相机像平面相机的畸变误差后的三维坐标点值
 * @param {Vector3D} _poi 像平面上的坐标值，Z值为该点对应的三维坐标的Z值
 * @param {double} _distParK1 畸变参数k1
 * @param {double} _distParK2 畸变参数k2
 * @return {Vector3D} 畸变后的像平面二维坐标，Z值和输入一样
 */
Vector3D Camera::getCamDistCoord2D(Vector3D _poi, double _distParK1, double _distParK2, double _distCenterL, double _distCenterV)
{
    log_compnt_mngr->info("Camera::getCamDistCoord2D start.");
    log_compnt_mngr->debug("Camera::getCamDistCoord2D _poi = ({}, {}, {})", _poi.x(), _poi.y(), _poi.z());
    log_compnt_mngr->debug("Camera::getCamDistCoord2D disParam = [{}, {}, {}, {}]", _distParK1, _distParK2, _distCenterL, _distCenterV);

    // 以感光元件中心作为原点的坐标系下的畸变中心坐标
    double distCenCL = _distCenterL - 0.5* _description.sensorSizeL/1e3;
    double distCenCV = _distCenterV - 0.5* _description.sensorSizeW/1e3;
    // 计算到像平面中心的距离
    double dx = _poi.x() - distCenCL;
    double dy = _poi.y() - distCenCV;
    // 计算畸变参数
    double distValue = _distParK1* (pow(dx, 2)+ pow(dy, 2))+ _distParK2* pow((pow(dx, 2)+ pow(dy, 2)), 2);
    // 计算畸变后的像平面坐标
    double xdp = _poi.x() *(1 + distValue);
    double ydp = _poi.y() *(1 + distValue);

    log_compnt_mngr->debug("Camera::getCamDistCoord2D ret = ({}, {}, {})", xdp, ydp, _poi.z());
    log_compnt_mngr->info("Camera::getCamDistCoord2D end.");
    return Vector3D(xdp, ydp, _poi.z());
}

/**
 * @Date: 2023-04-23 15:06:29
 * @Author: Zale
 * @LastEditors: Zale
 * @decription: 基于像平面上的坐标反算三维世界的坐标值
 * @param {Vector3D} _poi 待转换的像平面坐标，其中Z为该点在3维坐标中的值
 * @param {double} _fd
 * @return {Vector3D} 转换完成的三维坐标
 */
Vector3D Camera::conCamPlaneTo3d(Vector3D _poi, double _fd)
{
    log_compnt_mngr->debug("Camera::conCamPlaneTo3d _poi = ({}, {}, {}), fd=[{}]", _poi.x(), _poi.y(), _poi.z(), _fd);
    double nan = sqrt(-1);

    if(_fd <= 0 || abs(_poi.x()) < 1e-8 || abs(_poi.y()) < 1e-8)	// fd参数异常处理
    {
        log_compnt_mngr->debug("Camera::conCamPlaneTo3d End with fd=[{}], poiy=[{}]!", _fd, _poi.y());
        return Vector3D(nan, nan, nan);
    }

    double xd = _fd *_description.assemblePositionZ / _poi.y();
    double yd = _poi.x() *_description.assemblePositionZ / _poi.y();

    log_compnt_mngr->debug("Camera::conCamPlaneTo3d ret = ({}, {}, {})", xd, yd, _poi.z());
    return Vector3D(xd, yd, _poi.z());
}

IdealizedSensor::IdealizedSensor(BasicSensorDescription description) : _description(description)
{
    Sensor::_description = (SensorDescription)description;
}

//取得传感器状态
S_SP_SENSOR_INFO IdealizedSensor::getState()
{

    S_SP_SENSOR_INFO state{0};

    state.u1Type = D_SP_SENSOR_TYPE_NONE;
    //state.hostCategory = ASIM_OBJECT_CATEGORY_SENSOR;
    state.u8HostId = 1; // 主车持有传感器，主车Id为1
    (void)strncpy(state.au1Name, _description.name, D_SP_SIZE_OBJECT_NAME);

    state.au4FovHV[0] = _description.hFov;				  // 视野（水平）
    state.au4FovHV[1] = _description.vFov;				  // 视野（垂直）

    state.au4ClipNF[0] = _description.minimumDetectRange; // 裁剪范围（近）
    state.au4ClipNF[1] = _description.range;			  // 裁剪范围（远）

    // 传感器坐标系下传感器的坐标
    state.sPos.u8X = 0.0;
    state.sPos.u8Y = 0.0;
    state.sPos.u8Z = 0.0;
    state.sPos.u4H = 0.0;
    state.sPos.u4P = 0.0;
    state.sPos.u4R = 0.0;
    // 主车坐标系下传感器的坐标
    state.sOriginCoordSys.u8X = _description.assemblePositionX;
    state.sOriginCoordSys.u8Y = _description.assemblePositionY;
    state.sOriginCoordSys.u8Z = _description.assemblePositionZ;

    // 主车坐标系下传感器的角度
    state.sOriginCoordSys.u4H = _description.heading;
    state.sOriginCoordSys.u4P = _description.pitch;
    state.sOriginCoordSys.u4R = _description.roll;

    state.au4FovOffHV[0] = 0.0; // 视场偏移（水平）
    state.au4FovOffHV[1] = 0.0; // 视场偏移（垂直）

    return state;
}

double IdealizedSensor::getAssemblePositionX()
{
    return _description.assemblePositionX;
}

double IdealizedSensor::getAssemblePositionY()
{
    return _description.assemblePositionY;
}

double IdealizedSensor::getAssemblePositionZ()
{
    return _description.assemblePositionZ;
}

//取得探测范围
double IdealizedSensor::getRange()
{
    return _description.range;
}

//取得Heading Pitch Roll
double IdealizedSensor::getHeading()
{
    return _description.heading * SENSOR_OBJECT_DETECTION_PI / 180.0;
}
double IdealizedSensor::getPitch()
{
    return _description.pitch * SENSOR_OBJECT_DETECTION_PI / 180.0;
}

double IdealizedSensor::getRoll()
{
    return _description.roll * SENSOR_OBJECT_DETECTION_PI / 180.0;
}

//取得HFOV
double IdealizedSensor::getHFOV()
{
    return _description.hFov * SENSOR_OBJECT_DETECTION_PI / 180.0;
}

//取得VFOV
double IdealizedSensor::getVFOV()
{
    return _description.vFov * SENSOR_OBJECT_DETECTION_PI / 180.0;
}

//获取最近探测距离
double IdealizedSensor::getMinimumDetectRange()
{
    return _description.minimumDetectRange;
}

//判定是否是物体识别类型
bool IdealizedSensor::isDetectionObject(int objectType)
{
    return ((_description.object_detection_type & objectType) != 0);
}

double IdealizedSensor::getDetectedPR(double distance,float radarCrossSection, float weather, float occlusionScale)
{
    return 1;
}

int IdealizedSensor::getFrameRate()
{
    return 60;
}

// 判定是否使能高级设置
bool IdealizedSensor::isEnableAdvSettings()
{
    return 0;
}


double IdealizedSensor::getRangeMSENoise()
{
    return 0;
}

double IdealizedSensor::getAzimuthMSENoise()
{
    return 0;
}

double IdealizedSensor::getElevationMSENoise()
{
    return 0;
}

double IdealizedSensor::getRangeResolution()
{
    return 0;
}

double IdealizedSensor::getAzimuthResolution()
{
    return 0;
}

double IdealizedSensor::getElevationResolution()
{
    return 0;
}

double IdealizedSensor::getEnvNoise(double distance)
{
    return 0.0;
}

Radar::Radar(RadarDescription description) : _description(description)
{
    Sensor::_description = (SensorDescription)description;
}

//取得传感器状态
S_SP_SENSOR_INFO Radar::getState()
{

    S_SP_SENSOR_INFO state{0};

    state.u1Type = D_SP_SENSOR_TYPE_RADAR;
    //state.hostCategory = ASIM_OBJECT_CATEGORY_SENSOR;
    state.u8HostId = 1; // 主车持有传感器，主车Id为1
    (void)strncpy(state.au1Name, _description.name, D_SP_SIZE_OBJECT_NAME);
    state.au4FovHV[0] = _description.hFov;			// 视野（水平）
    state.au4FovHV[1] = _description.vFov; 		// 视野（垂直）
    state.au4ClipNF[0] = _description.minimumDetectRange; // 裁剪范围（近）
    state.au4ClipNF[1] = _description.range;		// 裁剪范围（远）

    // 传感器坐标系下传感器的坐标
    state.sPos.u8X = 0.0;
    state.sPos.u8Y = 0.0;
    state.sPos.u8Z = 0.0;
    state.sPos.u4H = 0.0;
    state.sPos.u4P = 0.0;
    state.sPos.u4R = 0.0;
    // 主车坐标系下传感器的坐标
    state.sOriginCoordSys.u8X = _description.assemblePositionX;
    state.sOriginCoordSys.u8Y = _description.assemblePositionY;
    state.sOriginCoordSys.u8Z = _description.assemblePositionZ;
    state.sOriginCoordSys.u4H = _description.heading;
    state.sOriginCoordSys.u4P = _description.pitch;
    state.sOriginCoordSys.u4R = _description.roll;

    state.au4FovOffHV[0] = 0.0; // 视场偏移（水平）
    state.au4FovOffHV[1] = 0.0; // 视场偏移（垂直）

    return state;
}

double Radar::getAssemblePositionX()
{
    return _description.assemblePositionX;
}

double Radar::getAssemblePositionY()
{
    return _description.assemblePositionY;
}

double Radar::getAssemblePositionZ()
{
    return _description.assemblePositionZ;
}

//取得探测范围
double Radar::getRange()
{
    return _description.range;
}

//取得Heading Pitch Roll
double Radar::getHeading()
{
    return _description.heading * SENSOR_OBJECT_DETECTION_PI / 180.0;
}
double Radar::getPitch()
{
    return _description.pitch * SENSOR_OBJECT_DETECTION_PI / 180.0;
}

double Radar::getRoll()
{
    return _description.roll * SENSOR_OBJECT_DETECTION_PI / 180.0;
}

//取得HFOV
double Radar::getHFOV()
{
    return _description.hFov * SENSOR_OBJECT_DETECTION_PI / 180.0;
}

//取得VFOV
double Radar::getVFOV()
{
    return _description.vFov * SENSOR_OBJECT_DETECTION_PI / 180.0;
}

//获取最近探测距离
double Radar::getMinimumDetectRange()
{
    return _description.minimumDetectRange;
}

double Radar::getDetectedPR(double distance,float radarCrossSection, float weather, float occlusionScale)
{
    double Pd = 1.0;
    if (_description.bExistProbInput == true)
    {
        Pd = _description.fExistProb / 100;
    }
    else
    {
        const int m = 1 ; //累积数
        float SNR = 0.0; //信噪比
        double occlusionScaleThreshold = 0.15; // 遮挡比例影响阈值
        double falseAlarmRate = 0.00000001 * pow(10, _description.sensitivityThreshold);
        double fNEP = 16e-9; // 噪声水平

        double Pr = (_description.transmitterPower * _description.transmitGain * radarCrossSection * _description.effectiveArea) / (pow(4*SENSOR_OBJECT_DETECTION_PI,2) * pow(distance,4));
        SNR = Pr / fNEP;
        if (weather != 0)
        {
            SNR *= weather; // 环境因子影响信噪比
        }
        Pd = 0.5 * erfc ( (sqrt( -log((falseAlarmRate))) - sqrt(SNR + 0.5) ) ) * minPct(1 , (1 - occlusionScale)/(1 - occlusionScaleThreshold)); //获得检测概率
    }
    return Pd;
}

int Radar::getFrameRate()
{
    return _description.frameRate;
}

// 判定是否使能高级设置
bool Radar::isEnableAdvSettings()
{
    return _description.advSettingsEnable;
}

double Radar::getRangeMSENoise()
{
    return _description.rangeMSENoise;
}

double Radar::getAzimuthMSENoise()
{
    return _description.azimuthMSENoise;
}

double Radar::getElevationMSENoise()
{
    return _description.elevationMSENoise;
}

double Radar::getRangeResolution()
{
    return 2;
}

double Radar::getAzimuthResolution()
{
    return _description.horizontalResolution;
}

double Radar::getElevationResolution()
{
    return _description.horizontalResolution;
}

double Radar::getEnvNoise(double distance)
{
    return 0.0;
}

Ultrasonic::Ultrasonic(UltrasonicDescription description) : _description(description)
{
    Sensor::_description = (SensorDescription)description;
}

//取得传感器状态
S_SP_SENSOR_INFO Ultrasonic::getState()
{

    S_SP_SENSOR_INFO state{0};

    state.u1Type = D_SP_SENSOR_TYPE_ULTRASONIC;
    //state.hostCategory = ASIM_OBJECT_CATEGORY_SENSOR;
    state.u8HostId = 1; // 主车持有传感器，主车Id为1
    (void)strncpy(state.au1Name, _description.name, D_SP_SIZE_OBJECT_NAME);
    state.au4FovHV[0] = _description.hFov;			// 视野（水平）
    state.au4FovHV[1] = _description.vFov; 		// 视野（垂直）
    state.au4ClipNF[0] = _description.minimumDetectRange; // 裁剪范围（近）
    state.au4ClipNF[1] = _description.range;		// 裁剪范围（远）
    // 传感器坐标系下传感器的坐标
    state.sPos.u8X = 0.0;
    state.sPos.u8Y = 0.0;
    state.sPos.u8Z = 0.0;
    state.sPos.u4H = 0.0;
    state.sPos.u4P = 0.0;
    state.sPos.u4R = 0.0;
    // 主车坐标系下传感器的坐标
    state.sOriginCoordSys.u8X = _description.assemblePositionX;
    state.sOriginCoordSys.u8Y = _description.assemblePositionY;
    state.sOriginCoordSys.u8Z = _description.assemblePositionZ;
    state.sOriginCoordSys.u4H = _description.heading;
    state.sOriginCoordSys.u4P = _description.pitch;
    state.sOriginCoordSys.u4R = _description.roll;

    state.au4FovOffHV[0] = 0.0; // 视场偏移（水平）
    state.au4FovOffHV[1] = 0.0; // 视场偏移（垂直）

    return state;
}

double Ultrasonic::getAssemblePositionX()
{
    return _description.assemblePositionX;
}

double Ultrasonic::getAssemblePositionY()
{
    return _description.assemblePositionY;
}

double Ultrasonic::getAssemblePositionZ()
{
    return _description.assemblePositionZ;
}

//取得探测范围
double Ultrasonic::getRange()
{
    return _description.range;
}

//取得Heading Pitch Roll
double Ultrasonic::getHeading()
{
    return _description.heading * SENSOR_OBJECT_DETECTION_PI / 180.0;
}
double Ultrasonic::getPitch()
{
    return _description.pitch * SENSOR_OBJECT_DETECTION_PI / 180.0;
}

double Ultrasonic::getRoll()
{
    return _description.roll * SENSOR_OBJECT_DETECTION_PI / 180.0;
}

//取得HFOV
double Ultrasonic::getHFOV()
{
    return _description.hFov * SENSOR_OBJECT_DETECTION_PI / 180.0;
}

//取得VFOV
double Ultrasonic::getVFOV()
{
    return _description.vFov * SENSOR_OBJECT_DETECTION_PI / 180.0;
}

//获取最近探测距离
double Ultrasonic::getMinimumDetectRange()
{
    return _description.minimumDetectRange;
}

double Ultrasonic::getDetectedPR(double distance,float radarCrossSection, float weather, float occlusionScale)
{
    double Pd = 1.0;
    if (_description.bExistProbInput == true)
    {
        Pd = _description.fExistProb / 100;
    }
    else
    {
        double SNR = 0.0;	// 信噪比
        double occlusionScaleThreshold = 0.15; // 遮挡比例影响阈值
        double falseAlarmRate = 0.0000001;		//	虚警概率
        double alpha = 0.05; // 衰减系数

        double Pr = _description.SPL * exp(-alpha * distance);
        SNR = Pr / _description.noiseLevel;
        Pd = 0.5 * erfc ( (sqrt( -log(falseAlarmRate)) - sqrt(SNR + 0.5) ) ) * minPct(1 , (1 - occlusionScale)/(1 - occlusionScaleThreshold)); //获得检测概率
    }

    return Pd;
}

int Ultrasonic::getFrameRate()
{
    return _description.frameRate;
}

// 判定是否使能高级设置
bool Ultrasonic::isEnableAdvSettings()
{
    return _description.advSettingsEnable;
}

double Ultrasonic::getRangeMSENoise()
{
    return 0;
}

double Ultrasonic::getAzimuthMSENoise()
{
    return _description.azimuthGaussDistribution;
}

double Ultrasonic::getElevationMSENoise()
{
    return _description.azimuthGaussDistribution/2;
}

double Ultrasonic::getRangeResolution()
{
    return 0.5;
}

double Ultrasonic::getAzimuthResolution()
{
    return _description.horizontalResolution;
}

double Ultrasonic::getElevationResolution()
{
    return _description.horizontalResolution;
}

double Ultrasonic::getEnvNoise(double distance)
{
    double Vstd = 0.0; // 22摄氏度和相对湿度为0的空气中传播速度
    double v = 0.0; // T摄氏度和相对湿度为S的空气中传播速度
    double C0 = 331.45; // m/s, 0度时的声波速度
    double Tmpstd = 22.0;
    double Hmtstd = 0.0;
    Vstd = C0 * pow((1 + Tmpstd / 273.15) * (1 + 0.0134 * Hmtstd) , 0.5);

    v = C0 * pow((1 + _description.temp / 273.15) * (1 + 0.0134 * _description.humidity) , 0.5);

    double envNoise = (distance / v) * Vstd - distance;

    return envNoise;
}

//取得（车辆）的ID列表(探测范围内，有遮挡判断)
bool Sensor::getVehicleInViewList(std::set<int> &IDs)
{
    log_compnt_mngr->info("Sensor::getVehicleInViewList start.");

    //遍历所有筛选后的目标物体
    auto InViewVehIDs = getInViewIDsList();
    if (InViewVehIDs.empty())
    {
        log_compnt_mngr->error("InViewVehIDs is empty,return.");
        return false;
    }
    std::map<int, std::set<int>>::iterator iter = InViewVehIDs.find(SENSOR_OBJECT_DETECTION_TYPE_VEHICLE);
    if (iter->second.empty())
    {
        log_compnt_mngr->error("iter->second is empty , return.");
        return false;
    }
    if (iter != InViewVehIDs.end())
    {
        IDs = iter->second;
    }

    log_compnt_mngr->debug("Vehicle IDs.size({}).", IDs.size());
    log_compnt_mngr->info("Sensor::getVehicleInViewList end.");
    return true;
}

//取得（车辆）的ID列表(探测范围内，无遮挡判断)
bool Sensor::getVehicleList(std::set<int> &IDs)
{
    log_compnt_mngr->info("Sensor::getVehicleList start.");
    if (egoState == nullptr)
    {
        log_compnt_mngr->error( "egoState=nullptr.");
        return false;
    }
    Transform ego_transform; //获取Ego车世界坐标
    ego_transform.update(egoState->sObjectState.sPos.u8X, egoState->sObjectState.sPos.u8Y, egoState->sObjectState.sPos.u8Z);

    //遍历所有车辆
    for (auto vehIt = vehicleList.begin(); vehIt != vehicleList.end(); ++vehIt)
    {
        S_SP_MIL_OBJECT_STATE *objectState = *vehIt;
        if (objectState == nullptr)
        {
            log_compnt_mngr->warn( "objectState is nullptr , continue.");
            continue;
        }

        Transform objectTransform; //获取世界坐标
        objectTransform.update(objectState->sObjectState.sPos.u8X, objectState->sObjectState.sPos.u8Y, objectState->sObjectState.sPos.u8Z);

        std::vector<Vector3D> vehBBoxVertexVector;
        computeVehicleObstacleBBox(objectState, vehBBoxVertexVector); //获取环境车包围盒八个顶点信息

        // 世界坐标系转传感器坐标系
        Vector3D relativeCoord(0, 0, 0);
        Vector3D sensorRela(0, 0, 0);

        CoordTransform(egoState->sObjectState.sPos.u4H, egoState->sObjectState.sPos.u4P, egoState->sObjectState.sPos.u4R, ego_transform, objectTransform, getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(), getHeading(), getPitch(), getRoll(), relativeCoord, sensorRela);
        log_compnt_mngr->trace("relativeCoord[0], relativeCoord[1] relativeCoord[2] {},{},{}", relativeCoord[0],relativeCoord[1],relativeCoord[2]);
        log_compnt_mngr->trace("sensorRela[0], sensorRela[1] sensorRela[2] {},{},{}", sensorRela[0],sensorRela[1],sensorRela[2]);
        //是否在识别范围和FOV内
        if (!InCircleandInSector(egoState, ego_transform, vehBBoxVertexVector, getHFOV(), getVFOV(), getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(),
                                getRange(), getMinimumDetectRange(), getHeading(), getPitch(), getRoll()))
        {
            if (!IsIntersection(egoState, ego_transform, vehBBoxVertexVector, getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(),getHeading(), getPitch(), getRoll(),getRange(), getMinimumDetectRange()))
            {
                log_compnt_mngr->warn("object id {} not in InCircleandInSector and IsIntersection , continue.",objectState->sObjectState.u4Id);
                continue;
            }
        }
        IDs.insert(static_cast<int>(objectState->sObjectState.u4Id));
    }
    log_compnt_mngr->debug("IDs.size({}).", IDs.size());
    log_compnt_mngr->info("Sensor::getVehicleList end.");
    return true;
}

//取得（行人）的ID列表(探测范围内，有遮挡判断)
bool Sensor::getPedestrianInViewList(std::set<int> &IDs)
{
    log_compnt_mngr->info("Sensor::getPedestrianInViewList start.");
    //遍历所有筛选后的目标物体
    auto InViewPedIDs = getInViewIDsList();
    if (InViewPedIDs.empty())
    {
        log_compnt_mngr->error("InViewPedIDs is empty, return.");
        return false;
    }
    std::map<int, std::set<int>>::iterator iter = InViewPedIDs.find(SENSOR_OBJECT_DETECTION_TYPE_PEDESTRIAN);
    if (iter->second.empty())
    {
        log_compnt_mngr->error("iter->second is empty(), return.");
        return false;
    }
    if (iter != InViewPedIDs.end())
    {
        IDs = iter->second;
    }

    log_compnt_mngr->debug("Pedestrian IDs.size({}).", IDs.size());
    log_compnt_mngr->info("Sensor::getPedestrianInViewList end.");
    return true;
}

//取得（行人）的ID列表(探测范围内，无遮挡判断)
bool Sensor::getPedestrianList(std::set<int> &IDs)
{
    log_compnt_mngr->info("Sensor::getPedestrianList start.");
    if (egoState == nullptr)
    {
        log_compnt_mngr->error( "egoState=nullptr.");
        return false;
    }
    Transform ego_transform; //获取Ego车世界坐标
    ego_transform.update(egoState->sObjectState.sPos.u8X, egoState->sObjectState.sPos.u8Y, egoState->sObjectState.sPos.u8Z);

    //遍历所有行人
    for (auto pedIt = pedestrianList.begin(); pedIt != pedestrianList.end(); ++pedIt)
    {
        S_SP_MIL_OBJECT_STATE *objectState = *pedIt;
        if (objectState == nullptr)
        {
            log_compnt_mngr->warn( "objectState is nullptr , continue.");
            continue;
        }
        Transform objectTransform; //获取行人世界坐标
        objectTransform.update(objectState->sObjectState.sPos.u8X, objectState->sObjectState.sPos.u8Y, objectState->sObjectState.sPos.u8Z);

        std::vector<Vector3D> pedBBoxVertexVector; //获取行人包围盒8个顶点信息
        computePedestrianBBox(objectState, pedBBoxVertexVector);

        // 世界坐标系转传感器坐标系
        Vector3D relativeCoord(0, 0, 0);
        Vector3D sensorRela(0, 0, 0);

        CoordTransform(egoState->sObjectState.sPos.u4H, egoState->sObjectState.sPos.u4P, egoState->sObjectState.sPos.u4R, ego_transform, objectTransform, getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(), getHeading(), getPitch(), getRoll(), relativeCoord, sensorRela);
        log_compnt_mngr->trace("relativeCoord[0] , relativeCoord[1]  relativeCoord[2] {},{},{}", relativeCoord[0],relativeCoord[1],relativeCoord[2]);
        log_compnt_mngr->trace("sensorRela[0] , sensorRela[1]  sensorRela[2] {},{},{}", sensorRela[0],sensorRela[1],sensorRela[2]);
        //行人是否在识别范围和FOV内
        if (!InCircleandInSector(egoState, ego_transform, pedBBoxVertexVector, getHFOV(), getVFOV(), getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(),
                                getRange(), getMinimumDetectRange(), getHeading(), getPitch(), getRoll()))
        {
            log_compnt_mngr->warn("object id {} not in InCircleandInSector, continue.",objectState->sObjectState.u4Id);
            continue;
        }

        IDs.insert(static_cast<int>(objectState->sObjectState.u4Id));
    }
    log_compnt_mngr->debug("IDs.size({}).", IDs.size());
    log_compnt_mngr->info("Sensor::getPedestrianList end.");
    return true;
}

//取得（障碍物）的ID列表(探测范围内，有遮挡判断)
bool Sensor::getRoadObjectInViewList(std::set<int> &IDs)
{
    log_compnt_mngr->info("Sensor::getRoadObjectInViewList start.");
    //遍历所有筛选后的目标物体
    auto InViewObsIDs = getInViewIDsList();
    if (InViewObsIDs.empty())
    {
        log_compnt_mngr->error("InViewObsIDs is empty,return.");
        return false;
    }
    std::map<int, std::set<int>>::iterator iter = InViewObsIDs.find(SENSOR_OBJECT_DETECTION_TYPE_OBSTACLE);
    if (iter->second.empty())
    {
        log_compnt_mngr->error("iter->second is empty, return.");
        return false;
    }
    if (iter != InViewObsIDs.end())
    {
        IDs = iter->second;
    }

    log_compnt_mngr->debug("Obstacle IDs.size({}).", IDs.size());
    log_compnt_mngr->info("Sensor::getRoadObjectInViewList end.");
    return true;
}

//取得（障碍物）的ID列表(探测范围内，无遮挡判断)
bool Sensor::getRoadObjectList(std::set<int> &IDs)
{
    log_compnt_mngr->info("Sensor::getRoadObjectList start.");
    if (egoState == nullptr)
    {
        log_compnt_mngr->error( "egoState=nullptr.");
        return false;
    }
    Transform ego_transform; //获取Ego车世界坐标
    ego_transform.update(egoState->sObjectState.sPos.u8X, egoState->sObjectState.sPos.u8Y, egoState->sObjectState.sPos.u8Z);

    //遍历所有障碍物
    for (auto miscIt = obstacleList.begin(); miscIt != obstacleList.end(); ++miscIt)
    {
        S_SP_MIL_OBJECT_STATE *objectState = *miscIt;
        if (objectState == nullptr)
        {
            log_compnt_mngr->warn( "objectState is nullptr, continue.");
            continue;
        }
        Transform objectTransform; //获取世界坐标
        objectTransform.update(objectState->sObjectState.sPos.u8X, objectState->sObjectState.sPos.u8Y, objectState->sObjectState.sPos.u8Z);

        std::vector<Vector3D> objBBoxVertexVector;  //获取障碍物包围盒8个顶点信息
        computeVehicleObstacleBBox(objectState, objBBoxVertexVector);

        // 世界坐标系转传感器坐标系
        Vector3D relativeCoord(0, 0, 0);
        Vector3D sensorRela(0, 0, 0);

        CoordTransform(egoState->sObjectState.sPos.u4H, egoState->sObjectState.sPos.u4P, egoState->sObjectState.sPos.u4R, ego_transform, objectTransform, getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(), getHeading(), getPitch(), getRoll(), relativeCoord, sensorRela);
        log_compnt_mngr->trace("relativeCoord[0] , relativeCoord[1]  relativeCoord[2] {},{},{}", relativeCoord[0],relativeCoord[1],relativeCoord[2]);
        log_compnt_mngr->trace("sensorRela[0] , sensorRela[1]  sensorRela[2] {},{},{}", sensorRela[0],sensorRela[1],sensorRela[2]);

        //障碍物是否在识别范围和FOV内
        if (!InCircleandInSector(egoState, ego_transform, objBBoxVertexVector, getHFOV(), getVFOV(), getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(),
                                getRange(), getMinimumDetectRange(), getHeading(), getPitch(), getRoll()))
        {
            log_compnt_mngr->warn("object id {} not in InCircleandInSector, continue.",objectState->sObjectState.u4Id);
            continue;
        }

        IDs.insert(static_cast<int>(objectState->sObjectState.u4Id));
    }
    log_compnt_mngr->debug("IDs.size({}).", IDs.size());
    log_compnt_mngr->info("Sensor::getRoadObjectList end.");
    return true;
}

//取得（交通灯）的ID列表(探测范围内，无遮挡判断)
bool Sensor::getTrafficLightList(std::set<int> &IDs)
{
    log_compnt_mngr->info("Sensor::getTrafficLightList start.");
    if (egoState == nullptr)
    {
        log_compnt_mngr->error( "egoState=nullptr.");
        return false;
    }
    Transform ego_transform; //获取Ego车世界坐标
    ego_transform.update(egoState->sObjectState.sPos.u8X, egoState->sObjectState.sPos.u8Y, egoState->sObjectState.sPos.u8Z);

    //遍历所有交通灯
    for (auto itor = trafficLightList.begin(); itor != trafficLightList.end(); ++itor)
    {
        S_SP_TRAFFIC_LIGHT *trafficLight = *itor;
        if (trafficLight == nullptr)
        {
            log_compnt_mngr->warn( "trafficLight is nullptr, continue.");
            continue;
        }

        // 目标物ID
        int32_t objectId = trafficLight->u4Id;

        RoadSignal *roadSignal = RoadSystem::Instance()->getSignal(std::to_string(objectId));
        if (roadSignal == nullptr)
        {
            log_compnt_mngr->warn( "roadSignal == nullptr.");
            continue;
        }

        const Transform &trafficlight = roadSignal->getTransform();

        std::vector<Vector3D> _trafficlight;
        _trafficlight.push_back(trafficlight.v()); // 传入交通灯的世界坐标
        // 世界坐标系转传感器坐标系
        Vector3D relativeCoord(0, 0, 0);
        Vector3D sensorRela(0, 0, 0);

        CoordTransform(egoState->sObjectState.sPos.u4H, egoState->sObjectState.sPos.u4P, egoState->sObjectState.sPos.u4R, ego_transform, trafficlight, getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(), getHeading(), getPitch(), getRoll(), relativeCoord, sensorRela);
        log_compnt_mngr->trace("relativeCoord[0] , relativeCoord[1]  relativeCoord[2] {},{},{}", relativeCoord[0],relativeCoord[1],relativeCoord[2]);
        log_compnt_mngr->trace("sensorRela[0], sensorRela[1]  sensorRela[2] {},{},{}", sensorRela[0],sensorRela[1],sensorRela[2]);

        //是否在识别范围和FOV内
        if (!InCircleandInSector(egoState, ego_transform, _trafficlight, getHFOV(), getVFOV(), getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(),
                                getRange(), getMinimumDetectRange(), getHeading(), getPitch(), getRoll()))
        {
            log_compnt_mngr->warn("object id {} not in InCircleandInSector, continue.",objectId);
            continue;
        }

        IDs.insert(objectId);
    }
    log_compnt_mngr->debug("TrafficLightIDs.size({}).", IDs.size());
    log_compnt_mngr->info("Sensor::getTrafficLightList end.");
    return true;
}

//取得（TrafficSign）的ID列表
bool Sensor::getTrafficSignList(std::set<int> &IDs)
{
    log_compnt_mngr->info("Sensor::getTrafficSignList start.");
    if (egoState == nullptr)
    {
        log_compnt_mngr->error("Sensor::getTrafficSignList Error: egoState=nullptr.");
        return false;
    }

    Transform ego_transform; //获取Ego车世界坐标
    ego_transform.update(egoState->sObjectState.sPos.u8X, egoState->sObjectState.sPos.u8Y, egoState->sObjectState.sPos.u8Z);
    Road *road = RoadSystem::Instance()->getRoad(std::to_string(egoState->sObjectState.u8RoadId));
    if (road == nullptr)
    {
        log_compnt_mngr->error("Sensor::getTrafficSignList Error: road=nullptr.");
        return false;
    }

    std::vector<RoadSignal *> trafficSignVector = road->getTrafficSignVector(); //获取当前Road的TrafficSignVector
    for (int i = 0; i < static_cast<int>(trafficSignVector.size()); i++)
    {
        Transform transform = trafficSignVector[i]->getTransform(); //获取世界坐标

        std::vector<Vector3D> _transform;
        _transform.push_back(transform.v()); //传入交通标识的世界坐标
        // 世界坐标系转传感器坐标系
        Vector3D relativeCoord(0, 0, 0);
        Vector3D sensorRela(0, 0, 0);

        //是否在识别范围和FOV内
        CoordTransform(egoState->sObjectState.sPos.u4H, egoState->sObjectState.sPos.u4P, egoState->sObjectState.sPos.u4R, ego_transform,transform
                    , getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(), getHeading(), getPitch(), getRoll(), relativeCoord, sensorRela);

        if (!InCircleandInSector(egoState, ego_transform, _transform, getHFOV(), getVFOV(), getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(),
                                getRange(), getMinimumDetectRange(), getHeading(), getPitch(), getRoll()))
        {
            log_compnt_mngr->warn("object id {} not in InCircleandInSector, continue.",trafficSignVector[i]->getId());
            continue;
        }

        IDs.insert(static_cast<uint32_t>(TrafficSimulation::stoi(trafficSignVector[i]->getId())));

        //取得交通标识信息 std::set<std::pair<TrafficSignType, TrafficSignID>>
        trafficSign_infos.insert(std::make_pair(trafficSignVector[i]->getType(),TrafficSimulation::stoi(trafficSignVector[i]->getId())));
    }

    //填充nextRoad的TrafficSign元素
    std::vector<RoadSignal *> trafficSignVectorNextRoad;
    for (int i = 0; i < static_cast<int>(trafficSignVectorNextRoad.size()); i++)
    {
        Transform transform = trafficSignVectorNextRoad[i]->getTransform(); //获取世界坐标

        std::vector<Vector3D> _transform;
        _transform.push_back(transform.v()); //传入交通标识的世界坐标
        // 世界坐标系转传感器坐标系
        Vector3D relativeCoord(0, 0, 0);
        Vector3D sensorRela(0, 0, 0);

        CoordTransform(egoState->sObjectState.sPos.u4H, egoState->sObjectState.sPos.u4P, egoState->sObjectState.sPos.u4R, ego_transform, transform
                    , getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(), getHeading(), getPitch(), getRoll(), relativeCoord, sensorRela);

        //是否在识别范围和FOV内
        if (!InCircleandInSector(egoState, ego_transform, _transform, getHFOV(), getVFOV(), getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(),
                                getRange(), getMinimumDetectRange(), getHeading(), getPitch(), getRoll()))
        {
            log_compnt_mngr->warn("object id {} not in InCircleandInSector, continue.",trafficSignVectorNextRoad[i]->getId());
            continue;
        }

        IDs.insert(static_cast<uint32_t>(TrafficSimulation::stoi(trafficSignVectorNextRoad[i]->getId())));
        //取得交通标识信息 std::set<std::pair<TrafficSignType, TrafficSignID>>
        trafficSign_infos.insert(std::make_pair(trafficSignVectorNextRoad[i]->getType(),static_cast<uint32_t>(TrafficSimulation::stoi(trafficSignVectorNextRoad[i]->getId()))));
    }
    log_compnt_mngr->debug("TrafficSignIDs.size({}).", IDs.size());
    log_compnt_mngr->info("Sensor::getTrafficSignList end.");
    return true;
}

//取得交通标识信息 std::set<std::pair<TrafficSignType, TrafficSignID>>
std::set<std::pair<std::string,int>> Sensor::getTrafficSignInfo()
{
    return trafficSign_infos;
}

//x,y,z旋转;
void Sensor:: VectorRotated(float yaw, float pitch, float roll, Vector3D &SensorCoor)
{
    const double trans_x =
                            (
                                SensorCoor[0] * ( cos(yaw) * cos(pitch) )
                                + SensorCoor[1] * (-sin(yaw) * cos(pitch))   //x后旋转坐标
                                + SensorCoor[2] * sin(pitch)
                            );

    const double trans_y =
                            (
                                SensorCoor[0] * (sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll))
                                + SensorCoor[1] * (cos(yaw) * cos(roll) - sin(pitch) * sin(yaw) * sin(roll)) //y旋转后的坐标
                                + SensorCoor[2] * (-cos(pitch) * sin(roll) )
                            );
    const double trans_z =
                            (
                                SensorCoor[0] * (sin(yaw) * sin(roll) - cos(yaw) * sin(pitch) * cos(roll))
                                + SensorCoor[1] * (cos(yaw) * sin(roll) + sin(pitch) * sin(yaw) * cos(roll)) // z旋转后的坐标
                                + SensorCoor[2] * (cos(pitch) * cos(roll))
                            );
    SensorCoor.set(trans_x,trans_y,trans_z);
}

//坐标系转换
void Sensor::CoordTransform(double egoYaw, double egoPitch, double egoRoll, Transform _egoTransform, Transform _agentTransform, double _sensorX, double _sensorY, double _sensorZ, double _sensorYaw, double _sensorPitch, double _sensorRoll, Vector3D &EgoCoord, Vector3D &SensorCoorTrans)
{
    _egoTransform.relative3dCoor(egoYaw, egoPitch, egoRoll, _agentTransform.v(), EgoCoord);

    // 主车坐标系转传感器坐标系
    SensorCoorTrans = Vector3D( EgoCoord[0] - _sensorX, EgoCoord[1] - _sensorY, EgoCoord[2] - _sensorZ );

    CoordRotated(_sensorYaw, _sensorPitch, _sensorRoll, SensorCoorTrans);
}

/**
 * @Date: 2023-03-21 21:00:09
 * @Author: Zale
 * @LastEditors: Zale
 * @decription: 参考前面LS的CoordTransform，写了一个简化点输入参数的重载
 * @param {Transform} _oriTransf 作为新坐标系原点对象的Transform，世界坐标系
 * @param {Transform} _tgtTransform 待转换坐标系的对象的Transform，世界坐标系
 * @param {Vector3D} _oriAngles 作为新坐标系原点对象的姿态角（heading，pitch，roll）
 * @param {Vector3D} _senAngles 本传感器的安装姿态角（heading，pitch，roll）
 * @param {Vector3D} _senMountPos 本传感器的安装位置（x，y，z）
 * @param {Vector3D} &_egoCoord 转换后在oriTransf这个对象坐标系下的坐标
 * @param {Vector3D} &_sensorCoorTrans 转换后在sensor对象坐标系下的坐标
 * @return {*}
 */
void Sensor::CoordTransform(Transform _oriTransf, Transform _tgtTransform, Vector3D _oriAngles, Vector3D _senAngles, Vector3D _senMountPos, Vector3D &_egoCoord, Vector3D &_sensorCoorTrans)
{
    _oriTransf.relative3dCoor(_oriAngles.x(), _oriAngles.y(), _oriAngles.z(), _tgtTransform.v(), _egoCoord);
    _sensorCoorTrans = _egoCoord - _senMountPos;
    CoordRotated(_senAngles.x(), _senAngles.y(), _senAngles.z(), _sensorCoorTrans);
}

void Sensor::CoordRotated(float _yaw, float _pitch, float _roll, Vector3D &vCoorTrans)
{
    Vector3D mEx(1, 0, 0); // x轴基向量
    Vector3D mEy(0, 1, 0); // y轴基向量
    Vector3D mEz(0, 0, 1); // z轴基向量
    // 旋转基向量
    VectorRotated(_yaw, _pitch, _roll, mEx); // 旋转后的x轴基向量
    VectorRotated(_yaw, _pitch, _roll, mEy); // 旋转后的y轴基向量
    VectorRotated(_yaw, _pitch, _roll, mEz); // 旋转后的z轴基向量

    // vCoorTrans不带旋转的的坐标
    const double trans_x =
                            (
                                vCoorTrans[0] * mEx[0]
                                + vCoorTrans[1] * mEx[1]
                                + vCoorTrans[2]  * mEx[2]
                            );

    const double trans_y =
                            (
                                vCoorTrans[0] * mEy[0]
                                + vCoorTrans[1] * mEy[1]
                                + vCoorTrans[2] * mEy[2]
                            );
    const double trans_z =
                            (
                                vCoorTrans[0] * mEz[0]
                                + vCoorTrans[1] * mEz[1]
                                + vCoorTrans[2] * mEz[2]
                            );
    vCoorTrans.set(trans_x,trans_y,trans_z);
}

//计算传感器到目标物体3维最近点距离 Redmine1379 
double Sensor::CalcNearestDist(double _yaw, double _pitch, double _roll, Vector3D _relativeObjCoor, double _dimensionX, double _dimensionY,double _dimensionZ, double _centerX, double _centerY, double _centerZ, Vector3D &vRelativeCoor)
{
    log_compnt_mngr->info("Sensor::CalcNearestDist start.");
    // 目标 Bounding Box 6个面的法向量:Front, Back, Left, Right, Up, Down
    Vector3D vNf(1,0,0);
    Vector3D vNb(-1,0,0);
    Vector3D vNl(0,1,0);
    Vector3D vNr(0,-1,0);
    Vector3D vNu(0,0,1);
    Vector3D vNd(0,0,-1);
    // 向量旋转
    VectorRotated(_yaw, _pitch, _roll, vNf);
    VectorRotated(_yaw, _pitch, _roll, vNb);
    VectorRotated(_yaw, _pitch, _roll, vNl);
    VectorRotated(_yaw, _pitch, _roll, vNr);
    VectorRotated(_yaw, _pitch, _roll, vNu);
    VectorRotated(_yaw, _pitch, _roll, vNd);

    // 目标后轴中心的前/后/左/右/上/下面距离传感器的最近距离
    double fFront 	= _relativeObjCoor[0] * vNf[0] + _relativeObjCoor[1] * vNf[1] + _relativeObjCoor[2] * vNf[2];
    double fBack 	= _relativeObjCoor[0] * vNb[0] + _relativeObjCoor[1] * vNb[1] + _relativeObjCoor[2] * vNb[2];
    double fLeft 	= _relativeObjCoor[0] * vNl[0] + _relativeObjCoor[1] * vNl[1] + _relativeObjCoor[2] * vNl[2];
    double fRight	= _relativeObjCoor[0] * vNr[0] + _relativeObjCoor[1] * vNr[1] + _relativeObjCoor[2] * vNr[2];
    double fUp	 	= _relativeObjCoor[0] * vNu[0] + _relativeObjCoor[1] * vNu[1] + _relativeObjCoor[2] * vNu[2];
    double fDown	= _relativeObjCoor[0] * vNd[0] + _relativeObjCoor[1] * vNd[1] + _relativeObjCoor[2] * vNd[2];
    log_compnt_mngr->trace("fFront , fBack  fLeft , fRight, fUp  fDown {},{},{},{},{},{}", fFront,fBack,fLeft,fRight,fUp,fDown);

    // 目标前/后/左/右/上/下面距离传感器的最近距离
    double fFrontVal = fFront - ( _dimensionX / 2.0 + _centerX );
    double fBackVal  = fBack  - ( _dimensionX / 2.0 - _centerX );
    double fLeftVal  = fLeft  - ( _dimensionY / 2.0 + _centerY );
    double fRightVal = fRight - ( _dimensionY / 2.0 - _centerY );
    double fUpVal 	 = fUp	  - ( _dimensionZ ); // 目标顶部距离传感器的最近距离只用-bbox的高
    double fDownVal  = fDown; // 目标底部距离传感器的最近距离

    log_compnt_mngr->trace("fFrontVal , fBackVal  fLeftVal , fRightVal , fUpVal  fDownVal {},{},{},{},{},{}", fFrontVal,fBackVal,fLeftVal,fRightVal,fUpVal,fDownVal);

    // 计算目标坐标系下的相对最近坐标x
    const double nearest_x =
                            (
                                max(fFrontVal, 0.0) * vNf[0]
                                + max(fBackVal, 0.0)  * vNb[0]
                                + max(fLeftVal, 0.0)  * vNl[0]
                                + max(fRightVal, 0.0) * vNr[0]
                                + max(fUpVal, 0.0) 	  * vNu[0]
                                + max(fDownVal, 0.0)  * vNd[0]
                            );

    // 计算目标坐标系下的相对最近坐标y
    const double nearest_y =
                            (
                                max(fFrontVal, 0.0) * vNf[1]
                                + max(fBackVal, 0.0)  * vNb[1]
                                + max(fLeftVal, 0.0)  * vNl[1]
                                + max(fRightVal, 0.0) * vNr[1]
                                + max(fUpVal, 0.0) 	  * vNu[1]
                                + max(fDownVal, 0.0)  * vNd[1]
                            );

    // 计算目标坐标系下的相对最近坐标z
    const double nearest_z =
                            (
                                max(fFrontVal, 0.0) * vNf[2]
                                + max(fBackVal, 0.0)  * vNb[2]
                                + max(fLeftVal, 0.0)  * vNl[2]
                                + max(fRightVal, 0.0) * vNr[2]
                                + max(fUpVal, 0.0) 	  * vNu[2]
                                + max(fDownVal, 0.0)  * vNd[2]
                            );

    vRelativeCoor.set(-nearest_x, -nearest_y, -nearest_z); // 目标坐标系下的相对最近坐标

    log_compnt_mngr->debug("vRelativeCoor x, vRelativeCoor y, vRelativeCoor z {},{},{}", vRelativeCoor[0],vRelativeCoor[1],vRelativeCoor[2]);

    // 计算最近点距离
    double dNearestDist = sqrt( std::pow(max(fFrontVal, 0.0), 2) + std::pow(max(fBackVal, 0.0), 2) +
                                std::pow(max(fLeftVal, 0.0), 2) + std::pow(max(fRightVal, 0.0), 2) +
                                std::pow(max(fUpVal, 0.0), 2) + std::pow(max(fDownVal, 0.0), 2)
                                );
    log_compnt_mngr->debug("dNearestDist {}.", dNearestDist);
    log_compnt_mngr->info("Sensor::CalcNearestDist end.");
    return dNearestDist;
}

// 计算主车坐标系下的坐标
void Sensor::CalcNearestEgoCoordSys(double _yaw, double _pitch, double _roll, double _sensorX, double _sensorY, double _sensorZ, Vector3D _relativeObjCoor, Vector3D &vEgoCoorTrans)
{
    vEgoCoorTrans.set(_relativeObjCoor[0] + _sensorX, _relativeObjCoor[1] + _sensorY, _relativeObjCoor[2] + _sensorZ );
    CoordRotated( _yaw, _pitch, _roll, vEgoCoorTrans);
    log_compnt_mngr->trace("vEgoCoorTrans x, vEgoCoorTrans y, vEgoCoorTrans z {},{},{}", vEgoCoorTrans[0],vEgoCoorTrans[1],vEgoCoorTrans[2]);
}

// 计算传感器坐标系下的坐标
void Sensor::CalcNearestSensorCoordSys(double _sensorYaw, double _sensorPitch, double _sensorRoll, double _sensorX, double _sensorY, double _sensorZ, Vector3D _egoCoorTrans, Vector3D &vSensorCoorTrans)
{
    vSensorCoorTrans.set(_egoCoorTrans[0] - _sensorX, _egoCoorTrans[1] - _sensorY, _egoCoorTrans[2] - _sensorZ );
    CoordRotated( _sensorYaw, _sensorPitch, _sensorRoll, vSensorCoorTrans);
    log_compnt_mngr->trace("vSensorCoorTrans x , vSensorCoorTrans y, vSensorCoorTrans z {},{},{}", vSensorCoorTrans[0],vSensorCoorTrans[1],vSensorCoorTrans[2]);
}

// 计算传感器到物体最近点距离
// Output: [dDist]传感器到目标物最近点距离 [vEgoCoorTrans]计算主车坐标系下最近点坐标, [vSensorCoorTrans]计算传感器坐标系下最近点坐标
double Sensor::getObjNearestCoordInfo(S_SP_MIL_OBJECT_STATE *objectState, double _sensorX, double _sensorY, double _sensorZ, double _sensorYaw, double _sensorPitch, double _sensorRoll,Vector3D &vEgoCoorTrans, Vector3D &vSensorCoorTrans)
{
    log_compnt_mngr->info("Sensor::getObjNearestCoordInfo start.");
    double dDist = 0.0;
    // 主车坐标系转世界坐标系
    Vector3D vSensorWorld( _sensorX, _sensorY, _sensorZ );
    VectorRotated(egoState->sObjectState.sPos.u4H, egoState->sObjectState.sPos.u4P, egoState->sObjectState.sPos.u4R, vSensorWorld); //得到传感器坐标

    Transform tEgoTransform; // 取得主车世界坐标
    tEgoTransform.update(egoState->sObjectState.sPos.u8X, egoState->sObjectState.sPos.u8Y, egoState->sObjectState.sPos.u8Z);
    Transform tTargetTransform; // 取得目标车世界坐标
    tTargetTransform.update(objectState->sObjectState.sPos.u8X, objectState->sObjectState.sPos.u8Y, objectState->sObjectState.sPos.u8Z);

    // 传感器相对于目标坐标系下的坐标
    Vector3D vRelativeObjCoor(	(tEgoTransform.v().x() + vSensorWorld[0]) - tTargetTransform.v().x(),
                                (tEgoTransform.v().y() + vSensorWorld[1]) - tTargetTransform.v().y(),
                                (tEgoTransform.v().z() + vSensorWorld[2]) - tTargetTransform.v().z()
    );

    //[帧率控制数据传输架构整改]TODO: 所有使用sObjectState.sGeo的地方可能导致传感器移植到CM后计算结果有误差，因为之前用的是double类型
    double centerX = objectState->sObjectState.sGeo.u4OffX;
    double centerY = objectState->sObjectState.sGeo.u4OffY;
    double centerZ = objectState->sObjectState.sGeo.u4OffZ;
    uint8_t type = objectState->sObjectState.u1Type;

    // 计算最近点距离，输出目标坐标系下的相对最近坐标
    Vector3D vRelativeCoor(0,0,0);
    dDist = CalcNearestDist(objectState->sObjectState.sPos.u4H, objectState->sObjectState.sPos.u4P, objectState->sObjectState.sPos.u4R, vRelativeObjCoor,
                            objectState->sObjectState.sGeo.u4DimX, objectState->sObjectState.sGeo.u4DimY, objectState->sObjectState.sGeo.u4DimZ,
                            centerX, centerY, centerZ, vRelativeCoor);

    // 计算主车坐标系下的坐标
    CalcNearestEgoCoordSys( egoState->sObjectState.sPos.u4H, egoState->sObjectState.sPos.u4P, egoState->sObjectState.sPos.u4R,
                            _sensorX, _sensorY, _sensorZ,
                            vRelativeCoor, vEgoCoorTrans
    );

    // 计算传感器坐标系下的坐标
    CalcNearestSensorCoordSys( _sensorYaw, _sensorPitch, _sensorRoll,
                            _sensorX, _sensorY, _sensorZ,
                            vEgoCoorTrans, vSensorCoorTrans
    );
    log_compnt_mngr->info("Sensor::getObjNearestCoordInfo end.");
    return dDist;
}

std::vector<double> Sensor::getRoadMarkCoeff(int currLane)
{
    log_compnt_mngr->info("Sensor::getRoadMarkCoeff start.");
    auto nan = sqrt(-1);	// 用于返回所有的nan值
    std::vector<double> ret(8,nan);

    if (egoState == nullptr)
    {
        log_compnt_mngr->error( "Sensor::getRoadMarkCoeff egoState=nullptr.");
        return ret;
    }

    // Step 0	若主车不在车道上，则直接返回nan
    std::string roadId = std::to_string(egoState->sObjectState.u8RoadId);// 获取主车当前所在道路id
	Road *currRoad = RoadSystem::Instance()->getRoad(roadId);
	if (currRoad == nullptr)
    {
        log_compnt_mngr->error("Sensor::getRoadMarkCoeff Ego not on road!");
        return ret;
    }
    // Step 1.1 获取车道线点世界坐标系下的坐标
    Transform egoTrans;//获取Ego车世界坐标
    egoTrans.update(egoState->sObjectState.sPos.u8X, egoState->sObjectState.sPos.u8Y, egoState->sObjectState.sPos.u8Z); // 获取主车的Transform

    // auto egoHdg = egoTrans.q().getYaw();		// 获取主车的航向角
    auto egoU = egoState->sObjectState.u4RoadS;			// 获取主车在当前道路上的U坐标
    // auto currLane = egoVeh->getLane();	// 获取主车当前所在车道ID
    int egoDir = 1;	// 主车相对于道路S轴的朝向，1和S轴同向，-1为反向
    if (egoState->sObjectState.u1Dir == D_SP_ROAD_DIR_FLAG_DIR_FWD)
    {
        egoDir = 1;
    }
    else if (egoState->sObjectState.u1Dir == D_SP_ROAD_DIR_FLAG_DIR_REAR)
    {
        egoDir = -1;
    }

    Vector3D camLookTo = Vector3D(1,0,0);	// 存储相机朝向的向量
    CoordRotated(getHeading(), getPitch(), getRoll(), camLookTo);	// 计算相机在主车坐标系下，朝向方向的向量
    int camDir = camLookTo.x()>=0 ? 1 : -1;		// 摄像机的朝向，1为和主车车头朝向同向，-1为反向
    int sampleDir = egoDir * camDir;	// 采样方向，1为沿着当前道路S轴正方向，-1为负方向
    // const double camFd =0, distK1 =0, distK2 =0;	// 初始化畸变和噪声参数
    const double sampleIntv = 2.0;
    /* 基于传感器探测范围进行谁选点数的初始化 */
	double sampleRange = getRange();
	if (sampleRange > 20)
    {
        sampleRange = 20;
    }
    const int sampleNumMax = (int)(sampleRange / sampleIntv);
    std::vector<Vector3D> laneLinePoisLVec,laneLinePoisRVec;	// 用于存储左右侧的车道线采样点坐标
    std::vector<Vector3D> laneLinePoisLVecInView,laneLinePoisRVecInView;	// 用于存储在传感器探测范围内的左右侧车道线采样点坐标
    getLaneLinePois(sampleIntv, sampleNumMax, currRoad, currLane, egoU, sampleDir, laneLinePoisLVec, laneLinePoisRVec);
    // 取全向车道线
	getLaneLinePois(sampleIntv, sampleNumMax, currRoad, currLane, egoU, -sampleDir, laneLinePoisLVec, laneLinePoisRVec);

    // Step 1.2 将世界坐标系下的车道线点坐标转换至相机坐标系
    // 对于左侧的坐标点
    for(auto itp = laneLinePoisLVec.begin(); itp != laneLinePoisLVec.end(); ++itp)
    {
        log_compnt_mngr->debug("Sensor::getRoadMarkCoeff get Left Pois");
        auto pre_itp = *itp;
        // 转换到主车坐标系
        egoTrans.relative3dCoor(egoState->sObjectState.sPos.u4H, egoState->sObjectState.sPos.u4P, egoState->sObjectState.sPos.u4R, *itp, *itp);
        // 保存转换前的主车坐标系坐标
		auto temp_itp = *itp;
        // 转换到相机坐标系
        CalcNearestSensorCoordSys(getHeading(), getPitch(), getRoll(), getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(), *itp, *itp);
        // Step 2	对坐标进行噪音畸变处理
        *itp = calDistCoord(*itp);
        // Step 3	将当前点转变为球坐标系 并判断是否在相机范围内
		if (isPoisInSensorView(*itp))
        {
			log_compnt_mngr->trace("Sensor::getRoadMarkCoeff vehicle coordinate Pois in view ({}, {})", temp_itp.x(), temp_itp.y());
			// 如果在传感器探测范围内，则将主车坐标系下车道线坐标保存下来
			laneLinePoisLVecInView.push_back(temp_itp);
		}
    }
    // 对于右侧的坐标点
    for(auto itp = laneLinePoisRVec.begin(); itp != laneLinePoisRVec.end(); ++itp)
    {
        log_compnt_mngr->debug("Sensor::getRoadMarkCoeff get Right Pois");
        auto pre_itp = *itp;
        // 转换到主车坐标系
        egoTrans.relative3dCoor(egoState->sObjectState.sPos.u4H, egoState->sObjectState.sPos.u4P, egoState->sObjectState.sPos.u4R, *itp, *itp);
        // 保存转换前的主车坐标系坐标
        auto temp_itp = *itp;
        // 转换到相机坐标系
        CalcNearestSensorCoordSys(getHeading(), getPitch(), getRoll(), getAssemblePositionX(), getAssemblePositionY(), getAssemblePositionZ(), *itp, *itp);
        // Step 2	对坐标进行噪音畸变处理
        *itp = calDistCoord(*itp);
        // Step 3	将当前点转变为球坐标系 并判断是否在相机范围内
		if (isPoisInSensorView(*itp))
        {
			log_compnt_mngr->trace("Sensor::getRoadMarkCoeff vehicle coordinate Pois in view ({}, {})", temp_itp.x(), temp_itp.y());
			laneLinePoisRVecInView.push_back(temp_itp);
		}

    }
    // Step 3	使用增加了噪音之后的坐标 / 没有噪音的坐标进行车道线拟合
    auto retL = fitLaneLine(laneLinePoisLVecInView);	// 左侧车道线拟合值
	auto retR = fitLaneLine(laneLinePoisRVecInView);	// 右侧车道线拟合值
    ret.clear();
    ret.insert(ret.begin(), retL.begin(), retL.end());
    ret.insert((ret.begin() + 4), retR.begin(), retR.end());
    log_compnt_mngr->trace("Sensor::getRoadMarkCoeff ret=[{}, {}, {}, {}, {}, {}, {}, {}]", ret[0], ret[1], ret[2], ret[3], ret[4], ret[5], ret[6], ret[7]);
    log_compnt_mngr->info("Sensor::getRoadMarkCoeff end.");
    return ret;
}

/**
 * @Date: 2023-04-18 11:34:15
 * @Author: Zale
 * @LastEditors: Zale
 * @decription: 依据制定的采样间隔、采样距离、采样方向，在指定道路上采样指定数量的车道线坐标点
 * @param {double} _dis 采样间隔
 * @param {int} _num 采样点数
 * @param {Road*} _road 采样起始点所在的道路
 * @param {int} _laneId 采样起始点所在的车道id
 * @param {double} _startu 采样起始点的u坐标
 * @param {int} _dir 采样方向，1为S轴正方向，-1为反向
 * @param {vector<Vector3D>} _lPois 左侧车道线点集合
 * @param {vector<Vector3D>} _rPois 右侧车道线点集合
 * @return {int} 最终采样到的车道线数量
 */
int Sensor::getLaneLinePois(double _intv, int _num, Road* _road, int _laneId, double _startu, int _dir, std::vector<Vector3D> &_lPois, std::vector<Vector3D> &_rPois)
{
    log_compnt_mngr->info("Sensor::getLaneLinePois start.");
    int poi = 0;
    Road* currRoad = _road;
    int currLane = _laneId;
    int sampleDir = _dir;
    double sampleS = _startu;
    double currRoadLen = currRoad->getLength();
    while(poi < _num)
    {
        int currDir = 1;
        TarmacConnection *nextTmp;
        int sampleFlag = 0;	// 储存采样状态，-1为向前越界，1为向后越界，0为正常
        log_compnt_mngr->debug("Sensor::getLaneLinePois currRoad=[{}] currRoadLen=[{}] currLane=[{}] sampleDir=[{}], sampleS=[{}]",currRoad->getId(), currRoadLen, currLane, sampleDir, sampleS);
        if(sampleS < 0 )	// 采样越界，越过了道路的起点，需要从当前道路的predecessor继续采样
        {
            sampleFlag = -1;
            log_compnt_mngr->debug("Sensor::getLaneLinePois reach begin, find predecessor");
            nextTmp = currRoad->getPredecessorConnection();
        }
        else if(sampleS > currRoadLen)	// 采样越界，越过了道路的终点，需要从当前道路的successor继续采样
        {
            sampleFlag = 1;
            log_compnt_mngr->debug("Sensor::getLaneLinePois reach end, find successor");
            nextTmp = currRoad->getSuccessorConnection();
        }
        else	// 正常采样
        {
            log_compnt_mngr->debug("Sensor::getLaneLinePois in the curr road");
            sampleFlag = 0;
        }
        if(sampleFlag != 0)		// 针对采样越界时的处理
        {
            if(nextTmp == nullptr)	// 后继为空，直接break
            {
                log_compnt_mngr->warn("Sensor::getLaneLinePois no nextRoad, break");
                break;
            }
            Road *nextRoadTmp = dynamic_cast<Road *>(nextTmp->getConnectingTarmac());
            Junction *nextJuncTmp = dynamic_cast<Junction *>(nextTmp->getConnectingTarmac());
            int nextSampleLane = 0;		// 下一条采样车道的ID
            if(nextRoadTmp != nullptr)	// 后续采样对象为Road
            {
                log_compnt_mngr->debug("Sensor::getLaneLinePois next is Road");
                // 根据越界方式的不同，决定后续采样方向
                nextSampleLane = sampleFlag > 0 ? currRoad->traceLane(currLane, currRoadLen - 1.0, currRoadLen + 1.0): currRoad->traceLane(currLane, 0, -_intv);
                // 判断在下一条路是从起点开始正向采样还是从终点开始逆向采样
                int nextDir = getLaneDir(currRoad, currLane, sampleFlag, nextRoadTmp, nextSampleLane);
                if(nextDir > 0)	// 正向采样
                {
                    currDir = 1;
                    sampleS = sampleFlag > 0 ? (0) : (nextRoadTmp->getLength());	// 根据在道路的开始还是结束确定从道路的起点还是终点开始采样
                }
                else if(nextDir < 0)			// 逆向采样
                {
                    currDir = -1;
                    sampleS = sampleFlag > 0 ? (nextRoadTmp->getLength()) : (0);	// 根据在道路的开始还是结束确定从道路的起点还是终点开始采样
                }
                else		// 异常
                {
                    log_compnt_mngr->warn("Sensor::getLaneLinePois next Lane Err, break");
                    break;
                }
                currRoad = nextRoadTmp;
                currLane = nextSampleLane;
                currRoadLen = currRoad->getLength();
            }
            else if (nextJuncTmp != nullptr)		// 后续采样对象为Junction
            {
                log_compnt_mngr->debug("Sensor::getLaneLinePois next is Junc");
                PathConnectionSet connSetTmp = nextJuncTmp->getPathConnectionSet(currRoad, currLane);	// 获取后继Junction上所有和当前采样车道具有连接关系的车道
                if(connSetTmp.getFrequencySumMap().size() <1)	// map为空，跳出
                {
                    log_compnt_mngr->warn("Sensor::getLaneLinePois connSetTmp map size = 0 break");
                    break;
                }
                PathConnection *sampleCon = connSetTmp.getFrequencySumMap().begin()->second;	// 从具备连接关系的车道中选取第一条车道，作为下一个采样车道
                if(sampleCon != nullptr)	// 车道不为空
                {
                    log_compnt_mngr->debug("Sensor::getLaneLinePois sampleCon != nullptr");
                    // 更新当前道路信息
                    nextSampleLane = sampleCon->getConnectingLane(currLane, false);
                    log_compnt_mngr->debug("Sensor::getLaneLinePois getConnectingPath");
                    auto nextRoad = sampleCon->getConnectingPath();
                    log_compnt_mngr->debug("Sensor::getLaneLinePois getLength");
                    double nextoadLen = nextRoad->getLength();
                    log_compnt_mngr->debug("Sensor::getLaneLinePois currDur=[1] currRoadLen=[{}], currSampleS=[{}]", currRoadLen, sampleS);
                    // 判断在下一条路是从起点开始正向采样还是从终点开始逆向采样
                    int nextDir = getLaneDir(currRoad, currLane, sampleFlag, nextRoad, nextSampleLane);
                    if(nextDir > 0)	// 正向采样
                    {
                        currDir = 1;
                        sampleS = sampleFlag > 0 ? (0) : (nextRoad->getLength());	// 根据在道路的开始还是结束确定从道路的起点还是终点开始采样
                    }
                    else if(nextDir < 0)			// 逆向采样
                    {
                        currDir = -1;
                        sampleS = sampleFlag > 0 ? (nextRoad->getLength()) : (0);	// 根据在道路的开始还是结束确定从道路的起点还是终点开始采样
                    }
                    else		// 异常
                    {
                        log_compnt_mngr->warn("Sensor::getLaneLinePois next Lane Err, break");
                    }
                    currRoad = nextRoad;
                    currLane = nextSampleLane;
                    currRoadLen = currRoad->getLength();
                }
                else
                {
                    log_compnt_mngr->warn("Sensor::getLaneLinePois connSetTmp begin = null break");
                    break;
                }
            }
            else
            {
                log_compnt_mngr->warn("Sensor::getLaneLinePois next is not Road or Junc, break");
                break;
            }
            sampleDir = sampleDir * currDir;	// 更新采样方向
            // 如果laneID异常，终止采样
            if((abs(nextSampleLane) > 10000) or (abs(currLane) > 10000)) break;
        }
        else	// 正常采样
        {
            log_compnt_mngr->debug("Sensor::getLaneLinePois currRoad=[{}] currLane=[{}] sampleDir=[{}], sampleS=[{}]",currRoad->getId(), currLane, sampleDir, sampleS);
            RoadPoint poiOut(0,0,0,0,0,0);		// 外侧车道线的坐标信息
            RoadPoint poiIn(0,0,0,0,0,0);		// 内侧车道线的坐标信息
            double disOut = 0;					// 外侧车道线距离道路参考线的距离，仅作为下面函数的参数用
            double disIn  = 0;					// 内侧车道线距离道路参考线的距离，仅作为下面函数的参数用
            // 获取内测与外侧车道线的世界坐标
            currRoad->getLaneRoadPointsWithLimit(sampleS, currLane, poiIn, poiOut, disIn, disOut);
            int egoDir = 1; //主车相对于道路S轴的朝向，1和S轴同向，-1为反向
			if (egoState->sObjectState.u1Dir == D_SP_ROAD_DIR_FLAG_DIR_FWD)
			{
				egoDir = 1;
			}
			else if (egoState->sObjectState.u1Dir == D_SP_ROAD_DIR_FLAG_DIR_REAR)
			{
				egoDir = -1;
			}

			if (egoDir * _laneId < 0)
            {
				//主车沿车道正向行驶
				_rPois.push_back(Vector3D(poiOut.x(), poiOut.y(), poiOut.z()));	// 存储外侧车道线的采样点坐标
				_lPois.push_back(Vector3D(poiIn.x(), poiIn.y(), poiIn.z()));		// 存储内侧车道线的采样点坐标
			}
			else
            {
				//主车沿车道逆向行驶
				_lPois.push_back(Vector3D(poiOut.x(), poiOut.y(), poiOut.z()));	// 存储外侧车道线的采样点坐标
				_rPois.push_back(Vector3D(poiIn.x(), poiIn.y(), poiIn.z()));		// 存储内侧车道线的采样点坐标
			}
            log_compnt_mngr->debug("Sensor::getLaneLinePois poi[{}] PoiL=[{}, {}, {}], PoiR=[{}, {}, {}]", poi, poiIn.x(), poiIn.y(), poiIn.z(), poiOut.x(), poiOut.y(), poiOut.z());
            sampleS += _intv * sampleDir;	// 前往下一个点采样
            ++poi;
        }
    }
    log_compnt_mngr->info("Sensor::getLaneLinePois end.");
    return poi;
}

/**
 * @Date: 2023-04-23 15:09:54
 * @Author: Zale
 * @LastEditors: Zale
 * @decription: 拟合至多三次多项式的方程系数
 * @param {vector<Vector3D>} &_pois 待拟合的点
 * @return {std::vector<double>} 拟合得到的三次多项式系数0~3次项
 */
std::vector<double> Sensor::fitLaneLine(std::vector<Vector3D> &_pois)
{
    log_compnt_mngr->info("Sensor::fitLaneLine start.");
    auto nan =sqrt(-1);
    std::vector<double> ret(4, nan);
    if(_pois.size() < 1)	// 没有采样点，返回nan
    {
        log_compnt_mngr->debug("Sensor::fitLaneLine input Size = [{}], return nan", _pois.size());
        return ret;
    }
    else if(_pois.size() < 2)	// 仅有一组采样点，返回一个常数项
    {
        log_compnt_mngr->debug("Sensor::fitLaneLine input Size = [{}], return [{}]", _pois.size(), _pois[0].y());
        ret = {_pois[0].y(), 0, 0, 0};
    }
    else if(_pois.size() < 3)	// 只有两组坐标点，拟合直线
    {
        std::vector<double> xVec, yVec;
        for(auto itp =_pois.begin(); itp !=_pois.end(); ++itp)
        {
            xVec.push_back(itp->x());
            yVec.push_back(itp->y());
        }
        auto coeffs = fitStraightLine(xVec, yVec);
        ret = {	coeffs[0], coeffs[1], 0, 0};
        log_compnt_mngr->debug("Sensor::fitLaneLine input Size = [{}], return [{},{},0,0]", _pois.size(), coeffs[0], coeffs[1]);
    }
    else	// 其他情况下，拟合三次多项式
    {
        std::vector<double> xVec, yVec;
        for(auto itp =_pois.begin(); itp !=_pois.end(); ++itp)
        {
            xVec.push_back(itp->x());
            yVec.push_back(itp->y());
        }
        auto coeffs = PolynomialRegression::fit(xVec, yVec, 3);
        ret = {	coeffs[0], coeffs[1], coeffs[2], coeffs[3]};
        log_compnt_mngr->debug("Sensor::fitLaneLine input Size = [{}], return [{},{},{},{}]", _pois.size(), coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
    }
    log_compnt_mngr->info("Sensor::fitLaneLine end.");
    return ret;
}

/**
 * @Date: 2023-04-27 15:06:27
 * @Author: Zale
 * @LastEditors: Zale
 * @decription: 查询inRoad上的inLane和outRoad上的outLane的方向是否相同
 * @param {Road} *inRoad 来路
 * @param {int} inLane 来路车道
 * @param {int} inDir 来路方向：
 * 					1为正向，即待计算的连接点为来路的终点
 * 					-1为反向，即待计算的连接点为来路的起点
 * @param {Road} *outRoad 去路
 * @param {int} outLane 去路车道
 * @return {int}
 * 			1 来路的连接点和去路的起点相连
 * 			-1 来路的连接点和去路的终点相连
 * 			0异常
 */
int Sensor::getLaneDir(Road *_inRoad, int _inLane, int _inDir, Road *_outRoad, int _outLane)
{
    // 获取去路的起点和终点世界坐标
    Transform transfOutS = _outRoad->getRoadTransform(0.01, 0);
    Vector3D posOutS = transfOutS.v();
    Transform transfOutE = _outRoad->getRoadTransform(_outRoad->getLength()-0.01, 0);
    Vector3D posOutE = transfOutE.v();
    Vector3D posIn;
    if(_inDir > 0)	// 来路正向，从来路的终点查找
    {
        Transform transfIn = _inRoad->getRoadTransform(_inRoad->getLength() - 0.01, 0);
    }
    else if(_inDir <0)	// 来路反向，从来路的起点查找
    {
        Transform transfIn = _inRoad->getRoadTransform(0.01, 0);
    }
    else
    {
        log_compnt_mngr->error("Sensor::getLaneDir End Err, inDir = 0");
        return 0;
    }
    Vector3D disVecS = posIn - posOutE;
    Vector3D disVecE = posIn - posOutS;
    double disS = disVecS.length();
    double disE = disVecE.length();
    if(disS > disE)	// 来路的连接点和去路的终点更接近，和去路的终点相连
    {
        log_compnt_mngr->debug("Sensor::getLaneDir End Normal, outDir = -1");
        return -1;
    }
    else if (disE > disS)	// 来路的连接点和去路的起点更近，说明和去路的起点相连
    {
        log_compnt_mngr->debug("Sensor::getLaneDir End Normal, outDir = 1");
        return 1;
    }
    else	// 二者完全相等，异常
    {
        log_compnt_mngr->debug("Sensor::getLaneDir End Err, outDir = 0");
        return 0;
    }
}

/**
 * @Date: 2022-10-27 14:04:02
 * @Author: Zale
 * @LastEditors: Zale
 * @decription: 获取目标车辆相对于主车X/Y轴左右侧最近点的坐标 采用车辆模型轮廓（16点，不包含后视镜）
 * @return {vector<Vector3D>} 四个点的坐标，顺序为 X轴左，X轴右，Y轴左，Y轴右
 * 			若目标完全在坐标轴的一侧，取不到点的一侧坐标为(nan, nan, nan)
 * 			若目标骑跨在坐标轴上，则该坐标轴左右两侧的点取同一个点，坐标为骑跨相交线上距离原点最近的点
 */
std::vector<Vector3D> Sensor::getXYNearestPois(S_SP_MIL_OBJECT_STATE *object, Vector3D gaussRandErr, Vector3D &sensorCoord)
{
    log_compnt_mngr->info("Sensor::getXYNearestPois start.");
    std::vector<Vector3D> ret;
    ret.push_back(Vector3D(0, DBL_MAX, 0));		// X轴左侧最近点，存储为(0, DBL_MAX, 0)
    ret.push_back(Vector3D(0, -DBL_MAX, 0));	// X轴右侧最近点，存储为(0, -DBL_MAX, 0)
    ret.push_back(Vector3D(-DBL_MAX, 0, 0));	// Y轴左侧最近点，存储为(-DBL_MAX, 0, 0)
    ret.push_back(Vector3D(DBL_MAX, 0, 0));		// Y轴右侧最近点，存储为(DBL_MAX, 0, 0)
    std::vector<Vector3D> VertexPosVec; 		//存储传感器坐标系下轮廓点

    if (egoState == nullptr || object == nullptr)
    {
        log_compnt_mngr->error("Sensor::getXYNearestPois, egoState or object is null.");
        return ret;
    }

    double agentYaw 	= object->sObjectState.sPos.u4H - egoState->sObjectState.sPos.u4H - getHeading();   // 目标相对传感器的航向角
    double agentPitch 	= object->sObjectState.sPos.u4P - egoState->sObjectState.sPos.u4P - getPitch(); 	// 目标相对传感器的俯仰角
    double agentRoll	= object->sObjectState.sPos.u4R - egoState->sObjectState.sPos.u4R - getRoll();	 	// 目标相对传感器的横滚角

    std::string modelEntryName(object->au1ModelName); //模型名称
    auto iterOutline = m_modelOutline.find(modelEntryName);
    if ((iterOutline == m_modelOutline.end()) && (0 == modelEntryName.compare("Saimo")))
    {
        std::string modelCarName = "默认";
        iterOutline = m_modelOutline.find(modelCarName);
        if (iterOutline == m_modelOutline.end())
        {
            std::string modelCarName = "HongQi";
            iterOutline = m_modelOutline.find(modelCarName);
        }
    }
    // 是否有此模型轮廓点
    if (iterOutline != m_modelOutline.end())
    {
        std::vector<Vector3D> outLine = iterOutline->second;
        Vector3D paramScale(object->au8ParamScale[0], object->au8ParamScale[1], object->au8ParamScale[2]); //车模长宽高缩放比
        //对点坐标按照车模缩放比例进行相应修改
        for (auto &it : outLine)
        {
            it.set(it.x() * paramScale[0], it.y() * paramScale[1], it.z() * paramScale[2]);
            //将轮廓点转换到传感器坐标系下
            VectorRotated(agentYaw, agentPitch, agentRoll, it);
            it.x() = it.x() + sensorCoord.x() + gaussRandErr[0];
            it.y() = it.y() + sensorCoord.y() + gaussRandErr[1];
            it.z() = it.z() + sensorCoord.z() + gaussRandErr[2];
            VertexPosVec.push_back(it);
        }
    }

    // 基于16个轮廓点的坐标，取出距离坐标轴最近的对象
    for(auto const &itV : VertexPosVec)
    {
        // 与当前X轴左侧最近点比较
        if((itV.y() > 0) && (itV.y() < ret[0].y()))
        {
            ret[0] = itV;
        }
        // 与当前X轴右侧最近点比较
        if((itV.y() < 0) && (itV.y() > ret[1].y()))
        {
            ret[1] = itV;
        }
        // 与当前Y轴左侧最近点比较
        if((itV.x() < 0) && (itV.x() > ret[2].x()))
        {
            ret[2] = itV;
        }
        // 与当前Y轴右侧最近点比较
        if((itV.x() > 0) && itV.x() < ret[3].x())
        {
            ret[3] = itV;
        }
    }

    for(auto &it : ret)
    {
        if((fabs(it.x()) == DBL_MAX) || (fabs(it.y()) == DBL_MAX))
        {
            it = Vector3D(sqrt(-1), sqrt(-1), sqrt(-1));
        }
        //mod_runtime_sensor->info("getXYNearestPois vehicle POI({},{},{})",it.x(),it.y(),it.z());
    }
    // 判断是否与X轴相交
    if (  ret[0].y() >= 0 && ret[1].y() <= 0)
    {
        if(fabs(ret[0].x()) < fabs(ret[1].x()))
        {
            ret[0].set(ret[0].x(), 0, 0);
            ret[1].set(ret[0].x(), 0, 0);
        }
        else
        {
            ret[0].set(ret[1].x(), 0, 0);
            ret[1].set(ret[1].x(), 0, 0);
        }
    }
    // 判断是否与y轴相交
    if (ret[2].x() <= 0 && ret[3].x() >= 0)
    {
        if(fabs(ret[2].y()) < fabs(ret[3].y()))
        {
            ret[2].set(0, ret[2].y(), 0);
            ret[3].set(0, ret[2].y(), 0);
        }
        else
        {
            ret[2].set(0, ret[3].y(), 0);
            ret[3].set(0, ret[3].y(), 0);
        }
    }
    log_compnt_mngr->info("Sensor::getXYNearestPois end.");
    return ret;
}

/**
 * @Date: 2022-10-27 14:04:02
 * @Author: Zale
 * @LastEditors: Zale
 * @decription: 获取目标(车辆/行人/障碍物)相对于主车X/Y轴左右侧最近点的坐标
 * @return {vector<Vector3D>} 四个点的坐标，顺序为 X轴左，X轴右，Y轴左，Y轴右
 * 			若目标完全在坐标轴的一侧，取不到点的一侧坐标为(nan, nan, nan)
 * 			若目标骑跨在坐标轴上，则该坐标轴左右两侧的点取同一个点，坐标为骑跨相交线上距离原点最近的点
 */
std::vector<Vector3D> Sensor::getXYNearestPois(S_SP_MIL_OBJECT_STATE *object, Vector3D gaussRandErr)
{
    log_compnt_mngr->info("Sensor::getXYNearestPois 1 start.");
    std::vector<Vector3D> ret;
    ret.push_back(Vector3D(0, DBL_MAX, 0));		// X轴左侧最近点，存储为(0, DBL_MAX, 0)
    ret.push_back(Vector3D(0, -DBL_MAX, 0));	// X轴右侧最近点，存储为(0, -DBL_MAX, 0)
    ret.push_back(Vector3D(-DBL_MAX, 0, 0));	// Y轴左侧最近点，存储为(-DBL_MAX, 0, 0)
    ret.push_back(Vector3D(DBL_MAX, 0, 0));		// Y轴右侧最近点，存储为(DBL_MAX, 0, 0)

    Transform egoTransform;
    egoTransform.update(egoState->sObjectState.sPos.u8X, egoState->sObjectState.sPos.u8Y, egoState->sObjectState.sPos.u8Z);
    // 主车坐标系下的传感器坐标
    Vector3D senPosEgoCoor = Vector3D(_description.assemblePositionX, _description.assemblePositionY, _description.assemblePositionZ);
    double egoHdg = egoState->sObjectState.sPos.u4H;
    double egoPit = egoState->sObjectState.sPos.u4P;
    double egoRol = egoState->sObjectState.sPos.u4R;

    // 获取boundingbox顶点的信息
    std::vector<Vector3D> VertexPosVec;
    if ((object->sObjectState.u1Type == D_SP_OBJECT_TYPE_PEDESTRIAN) || (object->sObjectState.u1Type == D_SP_OBJECT_TYPE_ANIMAL)) //行人
    {
        computePedestrianBBox(object, VertexPosVec);
    }
    else //车辆或障碍物
    {
        computeVehicleObstacleBBox(object, VertexPosVec);
    }
    // 将boundingbox顶点信息转换到主车坐标系下
    for(auto &itV : VertexPosVec)
    {
        //mod_runtime_sensor->info("getXYNearestPoisPedestrain bbox in world=({},{},{})",itV.x(),itV.y(),itV.z());
        egoTransform.relative3dCoor(egoHdg, egoPit, egoRol, itV, itV);
        //mod_runtime_sensor->info("getXYNearestPoisPedestrain bbox in ego=({},{},{})",itV.x(),itV.y(),itV.z());
    }
    // 将boundingbox顶点信息转换到传感器坐标系下
    for(auto &itV : VertexPosVec)
    {
        senPosEgoCoor.relative3dCoor(getHeading(), getPitch(), getRoll(), itV, itV);
        itV.x() = itV.x() + gaussRandErr[0];
        itV.y() = itV.y() + gaussRandErr[1];
        itV.z() = itV.z() + gaussRandErr[2];
        //mod_runtime_sensor->info("getXYNearestPoisPedestrain bbox in sensor=({},{},{})",itV.x(),itV.y(),itV.z());
    }
    // 基于八个点的坐标，取出距离坐标轴最近的对象
    for(auto &itV : VertexPosVec)
    {
        // 与当前X轴左侧最近点比较
        if((itV.y()>0) && (itV.y() < ret[0].y()))
        {
            ret[0] = itV;
        }
        // 与当前X轴右侧最近点比较
        if((itV.y()<0) && (itV.y() > ret[1].y()))
        {
            ret[1] = itV;
        }
        // 与当前Y轴左侧最近点比较
        if((itV.x()<0) && (itV.x() > ret[2].x()))
        {
            ret[2] = itV;
        }
        // 与当前Y轴右侧最近点比较
        if((itV.x()>0) && itV.x() < ret[3].x())
        {
            ret[3] = itV;
        }
    }
    for(auto &it : ret)
    {
        if((fabs(it.x()) == DBL_MAX) || (fabs(it.y()) == DBL_MAX))
        {
            it = Vector3D(sqrt(-1), sqrt(-1), sqrt(-1));
        }
        //mod_runtime_sensor->info("getXYNearestPois vehicle POI({},{},{})",it.x(),it.y(),it.z());
    }
    // 判断boundingbox边缘是否与传感器坐标轴有相交，有的话需要判断骑跨的特殊情况
    std::vector<Vector3D> xCrossPois = getBoxCrossXAxis(VertexPosVec);			// 存储和x轴相交的点
    std::vector<Vector3D> yCrossPois = getBoxCrossYAxis(VertexPosVec);			// 存储和y轴相交的点

    // X轴骑跨最近点
    Vector3D tCP = Vector3D(DBL_MAX,DBL_MAX,DBL_MAX);		// 用于暂存最近点坐标

    if(xCrossPois.size() > 0)	// 如果x轴存在骑跨
    {
        for(auto &it:xCrossPois)	// 取出x坐标最小的那个点作为x轴左右侧最近的点
        {
            if(fabs(it.x()) < fabs(tCP.x()))
            {
                tCP = it;
            }
        }
        ret[0] = tCP;
        ret[1] = tCP;
    }
    // Y轴骑跨最近点
    tCP = Vector3D(DBL_MAX,DBL_MAX,DBL_MAX);		// 用于暂存最近点坐标
    if(yCrossPois.size() > 0)	// 如果x轴存在骑跨
    {
        for(auto &it:yCrossPois)	// 取出y坐标最小的那个点作为y轴左右侧最近的点
        {
            if(fabs(it.y()) < fabs(tCP.y()))
            {
                tCP = it;
            }
        }
        ret[2] = tCP;
        ret[3] = tCP;
    }

    log_compnt_mngr->info("Sensor::getXYNearestPois 1 end.");
    return ret;
}

/**
 * @Date: 2022-10-31 17:04:32
 * @Author: Zale
 * @LastEditors: Zale
 * @decription: 转换Vector3d的坐标到S_SP_COORDINATE
 * @param {Vector3D} _Vec3d
 * @return {*}
 */
S_SP_COORDINATE Sensor::Vec3dToSPCOORD(Vector3D _Vec3d)
{
    S_SP_COORDINATE ret;
    ret.u8X = _Vec3d.x();
    ret.u8Y = _Vec3d.y();
    ret.u8Z = _Vec3d.z();
    ret.u4H = 0;
    ret.u4P = 0;
    ret.u4R = 0;
}
/**
 * @Date: 2022-11-02 11:47:14
 * @Author: Zale
 * @LastEditors: Zale
 * @decription: 获取目标包围盒和X轴的交点
 * @param {vector<Vector3D>} _BBoxPois
 * @return {std::vector<Vector3D>} 所有的X轴交点坐标
 */
std::vector<Vector3D> Sensor::getBoxCrossXAxis(std::vector<Vector3D> _BBoxPois)
{
    std::vector<Vector3D> ret;
    ret.push_back(_BBoxPois[0].getAxisXCrossPos(_BBoxPois[0], _BBoxPois[1]));
    ret.push_back(_BBoxPois[0].getAxisXCrossPos(_BBoxPois[3], _BBoxPois[1]));
    ret.push_back(_BBoxPois[0].getAxisXCrossPos(_BBoxPois[3], _BBoxPois[2]));
    ret.push_back(_BBoxPois[0].getAxisXCrossPos(_BBoxPois[0], _BBoxPois[2]));
    ret.push_back(_BBoxPois[0].getAxisXCrossPos(_BBoxPois[4], _BBoxPois[5]));
    ret.push_back(_BBoxPois[0].getAxisXCrossPos(_BBoxPois[5], _BBoxPois[7]));
    ret.push_back(_BBoxPois[0].getAxisXCrossPos(_BBoxPois[7], _BBoxPois[6]));
    ret.push_back(_BBoxPois[0].getAxisXCrossPos(_BBoxPois[4], _BBoxPois[6]));
    for(auto it = ret.begin(); it != ret.end();)
    {
        log_compnt_mngr->debug("Sensor::getBoxCrossXAxis Pos=({},{},{})",it->x(), it->y(), it->z());
        if(it->z() == -1)
        {
            it = ret.erase(it);
        }
        else
        {
            ++it;
        }
    }
    return ret;
}

/**
 * @Date: 2022-11-02 11:47:14
 * @Author: Zale
 * @LastEditors: Zale
 * @decription: 获取目标包围盒和Y轴的交点
 * @param {vector<Vector3D>} _BBoxPois
 * @return {std::vector<Vector3D>} 所有的Y轴交点坐标
 */
std::vector<Vector3D> Sensor::getBoxCrossYAxis(std::vector<Vector3D> _BBoxPois)
{
    std::vector<Vector3D> ret;
    ret.push_back(_BBoxPois[0].getAxisYCrossPos(_BBoxPois[0], _BBoxPois[1]));
    ret.push_back(_BBoxPois[0].getAxisYCrossPos(_BBoxPois[3], _BBoxPois[1]));
    ret.push_back(_BBoxPois[0].getAxisYCrossPos(_BBoxPois[3], _BBoxPois[2]));
    ret.push_back(_BBoxPois[0].getAxisYCrossPos(_BBoxPois[0], _BBoxPois[2]));
    ret.push_back(_BBoxPois[0].getAxisYCrossPos(_BBoxPois[4], _BBoxPois[5]));
    ret.push_back(_BBoxPois[0].getAxisYCrossPos(_BBoxPois[5], _BBoxPois[7]));
    ret.push_back(_BBoxPois[0].getAxisYCrossPos(_BBoxPois[7], _BBoxPois[6]));
    ret.push_back(_BBoxPois[0].getAxisYCrossPos(_BBoxPois[4], _BBoxPois[6]));

    for(auto it = ret.begin(); it != ret.end();)
    {
        log_compnt_mngr->debug("Sensor::getBoxCrossYAxis Pos=({},{},{})",it->x(), it->y(), it->z());
        if(it->z() == -1)
        {
            it = ret.erase(it);
        }
        else
        {
            ++it;
        }
    }
    return ret;
}

//判定是否是物体识别类型
bool Lidar::isDetectionObject(int objectType)
{
    return ((_description.object_detection_type & objectType) != 0);
}
//判定是否是物体识别类型
bool Radar::isDetectionObject(int objectType)
{
    return ((_description.object_detection_type & objectType) != 0);
}
//判定是否是物体识别类型
bool Camera::isDetectionObject(int objectType)
{
    return ((_description.object_detection_type & objectType) != 0);
}
//判定是否是物体识别类型
bool Ultrasonic::isDetectionObject(int objectType)
{
    return ((_description.object_detection_type & objectType) != 0);
}

/// @brief 根据主车、目标物的合速度计算速度差值，并仿照 coSimuMilObjectStatePkg::getPkgData求得相对速度
/// @param objectState : 目标物
/// @return 相对速度在主车坐标系下的X、Y分量
Vector2D Sensor::getRelaVelToEgo(S_SP_MIL_OBJECT_STATE *objectState)
{
    double tgtVelxW = objectState->sObjectState.sSpeed.u8X;
    double tgtVelyW = objectState->sObjectState.sSpeed.u8Y;
    double egoVelxW = egoState->sObjectState.sSpeed.u8X;
    double egoVelyW = egoState->sObjectState.sSpeed.u8Y;

    auto relVelW = Vector3D(tgtVelxW - egoVelxW, tgtVelyW - egoVelyW, 0);
    auto egoAngles = Vector3D(egoState->sObjectState.sPos.u4H, egoState->sObjectState.sPos.u4P, egoState->sObjectState.sPos.u4R);
    auto senAnglesEgo = Vector3D(getHeading(), getPitch(), getRoll());
    auto senAngleWorld = egoAngles + senAnglesEgo;
    auto senMountPos = Vector3D(_description.assemblePositionX, _description.assemblePositionY, _description.assemblePositionZ);
    Vector3D egoCoordPos, senCoordPos;
    CoordRotated(senAngleWorld.x(), senAngleWorld.y(), senAngleWorld.z(), relVelW);
    return Vector2D(relVelW.x(), relVelW.y());
}

/// @brief 根据主车、目标物的ddu和ddv计算加速度差值，并仿照 coSimuMilObjectStatePkg::getPkgData求得相对加速度
/// @param objectState : 目标物
/// @return 相对加速度在主车坐标系下的X、Y分量
Vector2D Sensor::getRelaAccToEgo(S_SP_MIL_OBJECT_STATE *objectState)
{
    double tgtAccxW = objectState->sObjectState.sAccel.u8X;
    double tgtAccyW = objectState->sObjectState.sAccel.u8Y;
    double egoAccxW = egoState->sObjectState.sAccel.u8X;
    double egoAccyW = egoState->sObjectState.sAccel.u8Y;

    auto relAccW = Vector3D(tgtAccxW - egoAccxW, tgtAccyW - egoAccyW, 0);
    auto egoAngles = Vector3D(egoState->sObjectState.sPos.u4H, egoState->sObjectState.sPos.u4P, egoState->sObjectState.sPos.u4R);
    auto senAnglesEgo = Vector3D(getHeading(), getPitch(), getRoll());
    auto senAngleWorld = egoAngles + senAnglesEgo;
    auto senMountPos = Vector3D(_description.assemblePositionX, _description.assemblePositionY, _description.assemblePositionZ);
    Vector3D egoCoordPos, senCoordPos;
    CoordRotated(senAngleWorld.x(), senAngleWorld.y(), senAngleWorld.z(), relAccW);
    return Vector2D(relAccW.x(), relAccW.y());
}

/**
 * @description: 计算二维向量T相对于另一个二维向量O的在其方向和法向上的相对值
 * 				亦可以理解为已知世界坐标系下的坐标O和T，现在以O作为新坐标系的原点，以angDiff作为新坐标系的X轴朝向角
 * 				T点在这个新坐标系下的坐标
 * @author Tang
 * @last editor Tang
 * @param {Vector2D} _ori 描述中的向量O
 * @param {Vector2D} _tgt 描述中的向量T
 * @param {double} _angDiff 两个向量的夹角
 * @return {*}	相对值
 */
Vector2D Sensor::calRelVecByAngleDiff(Vector2D _ori, Vector2D _tgt, double _angDiff)
{
    double relDu = _tgt.x() * cos(_angDiff) - _tgt.y() * sin(_angDiff) - _ori.x();
    double relDv = _tgt.x() * sin(_angDiff) + _tgt.y() * cos(_angDiff) - _ori.y();
    return Vector2D(relDu, relDv);
}

/**
 * @Date: 2023-03-03 14:41:38
 * @Author: Zale
 * @LastEditors: Zale
 * @decription: 两点拟合直线
 * @param {vector<double>} &xVec 两个点的x坐标vector
 * @param {vector<double>} &yVec 两个点的y坐标vector
 * @return {std::vector<double>} {常数项， 一次项}
 */
std::vector<double> Sensor::fitStraightLine(std::vector<double> &_xVec, std::vector<double> &_yVec)
{
    std::vector<double> ret(2);
    ret[1] = (_yVec[1] - _yVec[0]) / (_xVec[1] - _xVec[0]);
    ret[0] = _yVec[1] - _xVec[1] * ret[1];
    return ret;
}

void Sensor::setSensorId(uint32_t _sensorId)
{
    sensorId = _sensorId;
}

// 每一帧需要做的数据重置工作
void Sensor::resetData()
{
    egoState = nullptr;
    vehicleList.clear();
    pedestrianList.clear();
    obstacleList.clear();
    trafficLightList.clear();
    trafficSignList.clear();
    environment = nullptr;

    agentFourPointRadMap.clear();
    InViewIDs.clear();
    occupiedRadVec.clear();
    objOcclusionScale.clear();
    roadMarkInfo.clear();
    sortedObjs.clear();
    ghostInfo.clear();
}

// 解析输入的数据包 返回：是否成功
bool Sensor::parseMsg(char *msgBuff, unsigned int msgSize)
{
    log_compnt_mngr->info("Sensor::parseMsg start.");
    bool result = true; //是否成功

    S_SP_MSG_HDR *msgHead = (S_SP_MSG_HDR *)msgBuff; //Msg的头部指针

    //考虑msg长度错误的情况
    if ((msgHead->u4HeaderSize + msgHead->u4DataSize) != msgSize)
    {
        log_compnt_mngr->error("Sensor::parseMsg msgSize error.");
        result = false;
        return result;
    }

    char *currentPkg = msgBuff + msgHead->u4HeaderSize; //当前Pkg的头部指针

    // 解析每个pkg
    while (true)
    {
        S_SP_MSG_ENTRY_HDR *pkgHead = (S_SP_MSG_ENTRY_HDR *)currentPkg; //Pkg的头部指针

        if (pkgHead->u2PkgId == D_SP_MIL_PKG_ID_EGO_DATA) //解析D_SP_MIL_PKG_ID_EGO_DATA
        {
            S_SP_MIL_EGO_STATE *pkgData = (S_SP_MIL_EGO_STATE *)(currentPkg + pkgHead->u4HeaderSize); //数据部分指针
            egoState = pkgData;
        }
        else if (pkgHead->u2PkgId == D_SP_MIL_PKG_ID_OBJECT_DATA) //解析S_SP_MIL_OBJECT_STATE
        {
            S_SP_MIL_OBJECT_STATE *pkgData = (S_SP_MIL_OBJECT_STATE *)(currentPkg + pkgHead->u4HeaderSize);  //数据部分指针

            int elementNum = pkgHead->u4DataSize / pkgHead->u4ElementSize; //元素数量

            for (int i = 0; i < elementNum; i++) //解析每个元素
            {
                uint8_t type = pkgData->sObjectState.u1Type;
                //车辆
                if ((type == D_SP_OBJECT_TYPE_CAR) || (type == D_SP_OBJECT_TYPE_MOTORBIKE) || (type == D_SP_OBJECT_TYPE_BICYCLE)
                    || (type == D_SP_OBJECT_TYPE_BUS) || (type == D_SP_OBJECT_TYPE_TRUCK))
                {
                    vehicleList.push_back(pkgData);
                }
                //行人
                else if ((type == D_SP_OBJECT_TYPE_PEDESTRIAN) || (type == D_SP_OBJECT_TYPE_ANIMAL))
                {
                    pedestrianList.push_back(pkgData);
                }
                //障碍物
                else
                {
                    obstacleList.push_back(pkgData);
                }

                pkgData = (S_SP_MIL_OBJECT_STATE *)(((char *)pkgData) + pkgHead->u4ElementSize);
            }
        }
        else if (pkgHead->u2PkgId == D_SP_PKG_ID_TRAFFIC_LIGHT) //解析D_SP_PKG_ID_TRAFFIC_LIGHT
        {
            S_SP_TRAFFIC_LIGHT *pkgData = (S_SP_TRAFFIC_LIGHT *)(currentPkg + pkgHead->u4HeaderSize); //数据部分指针
            trafficLightList.push_back(pkgData);
        }
        else if (pkgHead->u2PkgId == D_SP_PKG_ID_TRAFFIC_SIGN) //解析D_SP_PKG_ID_TRAFFIC_SIGN
        {
            S_SP_TRAFFIC_SIGN *pkgData = (S_SP_TRAFFIC_SIGN *)(currentPkg + pkgHead->u4HeaderSize);  //数据部分指针

            int elementNum = pkgHead->u4DataSize / pkgHead->u4ElementSize; //元素数量

            for (int i = 0; i < elementNum; i++) //解析每个元素
            {
                trafficSignList.push_back(pkgData);

                pkgData = (S_SP_TRAFFIC_SIGN *)(((char *)pkgData) + pkgHead->u4ElementSize);
            }
        }
        else if (pkgHead->u2PkgId == D_SP_PKG_ID_ENVIRONMENT) //解析D_SP_PKG_ID_ENVIRONMENT
        {
            S_SP_ENVIRONMENT *pkgData = (S_SP_ENVIRONMENT *)(currentPkg + pkgHead->u4HeaderSize); //数据部分指针
            environment = pkgData;
        }
        else if (pkgHead->u2PkgId == D_SP_PKG_ID_MODEL_OUTLINE) //解析D_SP_PKG_ID_MODEL_OUTLINE
        {
            S_SP_MODEL_OUTLINE *pkgData = (S_SP_MODEL_OUTLINE *)(currentPkg + pkgHead->u4HeaderSize);  //数据部分指针

            int elementNum = pkgHead->u4DataSize / pkgHead->u4ElementSize; //元素数量

            m_modelOutline.clear();
            for (int i = 0; i < elementNum; i++) //解析每个元素
            {
                std::vector<Vector3D> pointVec;

                //遍历每个点
                for (unsigned int j = 0; j < 16; j++)
                {
                    Vector3D xyz(pkgData->au8OutlinePoints[j][0], pkgData->au8OutlinePoints[j][1], pkgData->au8OutlinePoints[j][2]);
                    pointVec.push_back(xyz);
                }

                m_modelOutline[pkgData->au1ModelName] = pointVec;

                pkgData = (S_SP_MODEL_OUTLINE *)(((char *)pkgData) + pkgHead->u4ElementSize);
            }
        }
        else if (pkgHead->u2PkgId == D_SP_PKG_ID_END_FRAME) //如果是最后一个Pkg
        {
            break;
        }

        currentPkg += pkgHead->u4HeaderSize + pkgHead->u4DataSize; //指向下一个pkg
    }

    log_compnt_mngr->info("Sensor::parseMsg end.");
    return result;
}

/**
 * @biref                           打包SensorDetectionInfoPkg
 * @param detectionInfoVec          传感器探测结果
 * @param outputBuffer              保存SensorDetectionInfoPkg的buffer
 * @param bufferSize                buffer的空间
 * @param pkgSize                   打包生成的SensorDetectionInfoPkg大小
 * @return                          返回：是否成功
 */
bool Sensor::generateSensorDetectionInfoPkg(const std::vector<S_SP_SENSOR_DETECTION_INFO> &detectionInfoVec, char *outputBuffer, unsigned int bufferSize, unsigned int &pkgSize)
{
    log_compnt_mngr->info("Sensor::generateSensorDetectionInfoPkg start.");
    bool result = true; //返回：是否成功
    unsigned int objectNum = detectionInfoVec.size(); //元素数量

    //需要的Pkg大小
    unsigned int needPkgSize = sizeof(S_SP_MSG_ENTRY_HDR) + sizeof(S_SP_SENSOR_DETECTION_INFO) * objectNum;

    //buffer空间不足
    if (needPkgSize > bufferSize)
    {
        log_compnt_mngr->error("Sensor::generateSensorDetectionInfoPkg outputBuffer is not enough. needPkgSize={} , bufferSize={} .",needPkgSize, bufferSize);
        result = false;
        return result;
    }

    //打包生成的SensorDetectionInfoPkg大小
    pkgSize = needPkgSize;

    //开始打包pkg
    S_SP_MSG_ENTRY_HDR *pkgHead = reinterpret_cast<S_SP_MSG_ENTRY_HDR *>(outputBuffer);    //Pkg的头部指针
    S_SP_SENSOR_DETECTION_INFO *const pkgBody = reinterpret_cast<S_SP_SENSOR_DETECTION_INFO *>(outputBuffer + sizeof(S_SP_MSG_ENTRY_HDR)); //Pkg的Body指针

    //填充Pkg头部
    pkgHead->u4HeaderSize = static_cast<uint32_t>(sizeof(S_SP_MSG_ENTRY_HDR));
    pkgHead->u4DataSize = static_cast<uint32_t>(sizeof(S_SP_SENSOR_DETECTION_INFO)) * static_cast<uint32_t>(objectNum);
    pkgHead->u4ElementSize = static_cast<uint32_t>(sizeof(S_SP_SENSOR_DETECTION_INFO));
    pkgHead->u2PkgId = D_SP_PKG_ID_SENSOR_DETECTION_INFO;

    //填充Pkg数据部分
    memcpy(pkgBody, detectionInfoVec.data(), sizeof(S_SP_SENSOR_DETECTION_INFO) * objectNum);

    log_compnt_mngr->info("Sensor::generateSensorDetectionInfoPkg end.");
    return result;
}

// 输入GroundTruth，输出SensorDetectionInfoPkg
void Sensor::update(void *input, int inlen, void *output, int &outlen)
{
    log_compnt_mngr->info("Sensor::update start.");
    if ((input == nullptr) || (inlen <= 0))
    {
        log_compnt_mngr->error("Sensor::update input error");
        return;
    }

    // 每一帧需要做的数据重置工作
    resetData();

    // 解析输入的数据包
    bool parseResult = parseMsg((char *)input, inlen);
    if (!parseResult)
    {
        log_compnt_mngr->error("Sensor::update parse error");
        return;
    }

    // 执行传感器探测逻辑
    std::vector<S_SP_SENSOR_DETECTION_INFO> sensor_objects;
    if (getObjectList(sensor_objects) == true)
    {
        // 更新每个对象的传感器ID
        for (auto it = sensor_objects.begin(); it != sensor_objects.end(); it++)
        {
            it->u4SensorId = sensorId;
        }

        // 打包SensorDetectionInfoPkg
        unsigned int pkgSize = 0;
        bool generateResult = generateSensorDetectionInfoPkg(sensor_objects, (char *)output, outlen, pkgSize);
        if (generateResult)
        {
            outlen = pkgSize;
        }
    }
    log_compnt_mngr->info("Sensor::update end.");
}

/**
 * @brief               计算行人包围盒8个顶点
 * @param object        行人数据
 * @param bBoxVec       输出：包围盒8个顶点，顺序为底部左前，左后，右前，右后，顶部左前，左后，右前，右后
 */
void Sensor::computePedestrianBBox(S_SP_MIL_OBJECT_STATE *object, std::vector<Vector3D> &bBoxVec)
{
    // 计算行人底部四个顶点的世界坐标
    double pedHdg = object->sObjectState.sPos.u4H; //行人在世界坐标系下的航向角

    double pedX = object->sObjectState.sPos.u8X;
    double pedY = object->sObjectState.sPos.u8Y;
    double pedZ = object->sObjectState.sPos.u8Z;

    double dimensionX = object->sObjectState.sGeo.u4DimX;
    double dimensionY = object->sObjectState.sGeo.u4DimY;
    double dimensionZ = object->sObjectState.sGeo.u4DimZ;
    double centerX = object->sObjectState.sGeo.u4OffX;
    double centerY = object->sObjectState.sGeo.u4OffY;

    //记录行人包围盒四个顶点的坐标(Front、Rear、Left、Right)
    Vector3D coordinateLF(centerX + dimensionX / 2.0, centerY + dimensionY / 2.0, 0);  //左前
    Vector3D coordinateLR(centerX - dimensionX / 2.0, centerY + dimensionY / 2.0, 0);  //左后
    Vector3D coordinateRF(centerX + dimensionX / 2.0, centerY - dimensionY / 2.0, 0);  //右前
    Vector3D coordinateRR(centerX - dimensionX / 2.0, centerY - dimensionY / 2.0, 0);  //右后

    //根据航向角转换世界坐标
    Vector3D relativeCoordLLF((cos(pedHdg) * coordinateLF[0] - sin(pedHdg) * coordinateLF[1]) + pedX, (sin(pedHdg) * coordinateLF[0] + cos(pedHdg) * coordinateLF[1]) + pedY, pedZ);  //左前
    Vector3D relativeCoordLLR((cos(pedHdg) * coordinateLR[0] - sin(pedHdg) * coordinateLR[1]) + pedX, (sin(pedHdg) * coordinateLR[0] + cos(pedHdg) * coordinateLR[1]) + pedY, pedZ);  //左后
    Vector3D relativeCoordLRF((cos(pedHdg) * coordinateRF[0] - sin(pedHdg) * coordinateRF[1]) + pedX, (sin(pedHdg) * coordinateRF[0] + cos(pedHdg) * coordinateRF[1]) + pedY, pedZ);  //右前
    Vector3D relativeCoordLRR((cos(pedHdg) * coordinateRR[0] - sin(pedHdg) * coordinateRR[1]) + pedX, (sin(pedHdg) * coordinateRR[0] + cos(pedHdg) * coordinateRR[1]) + pedY, pedZ);  //右后
    Vector3D relativeCoordULF = relativeCoordLLF + Vector3D(0, 0, dimensionZ);
    Vector3D relativeCoordULR = relativeCoordLLR + Vector3D(0, 0, dimensionZ);
    Vector3D relativeCoordURF = relativeCoordLRF + Vector3D(0, 0, dimensionZ);
    Vector3D relativeCoordURR = relativeCoordLRR + Vector3D(0, 0, dimensionZ);

    //存储行人底部的四个顶点的世界坐标，顺序为底部左前，左后，右前，右后，顶部左前，左后，右前，右后
    bBoxVec.clear();
    bBoxVec.reserve(8);
    // 底部
    bBoxVec.push_back(relativeCoordLLF);
    bBoxVec.push_back(relativeCoordLLR);
    bBoxVec.push_back(relativeCoordLRF);
    bBoxVec.push_back(relativeCoordLRR);
    // 顶部
    bBoxVec.push_back(relativeCoordULF);
    bBoxVec.push_back(relativeCoordULR);
    bBoxVec.push_back(relativeCoordURF);
    bBoxVec.push_back(relativeCoordURR);
}

/**
 * @brief               计算车辆、障碍物包围盒8个顶点
 * @param object        车辆或障碍物数据
 * @param bBoxVec       输出：包围盒8个顶点，顺序为底部左前，左后，右前，右后，顶部左前，左后，右前，右后
 */
void Sensor::computeVehicleObstacleBBox(S_SP_MIL_OBJECT_STATE *object, std::vector<Vector3D> &bBoxVec)
{
    double dimensionX = object->sObjectState.sGeo.u4DimX;
    double dimensionY = object->sObjectState.sGeo.u4DimY;
    double dimensionZ = object->sObjectState.sGeo.u4DimZ;
    double centerX = object->sObjectState.sGeo.u4OffX;
    double centerY = object->sObjectState.sGeo.u4OffY;
    double hdgWorld = object->sObjectState.sPos.u4H;
    double pitchWorld = object->sObjectState.sPos.u4P;
    double rollWorld = object->sObjectState.sPos.u4R;

    Vector3D vehPos(object->sObjectState.sPos.u8X, object->sObjectState.sPos.u8Y, object->sObjectState.sPos.u8Z);
    Vector3D coordinateLLF(centerX + dimensionX / 2.0, centerY + dimensionY / 2.0, 0);  //左前
    Vector3D coordinateLLR(centerX - dimensionX / 2.0, centerY + dimensionY / 2.0, 0);  //左后
    Vector3D coordinateLRF(centerX + dimensionX / 2.0, centerY - dimensionY / 2.0, 0);  //右前
    Vector3D coordinateLRR(centerX - dimensionX / 2.0, centerY - dimensionY / 2.0, 0);  //右后
    // 顶部4个顶点坐标
    Vector3D coordinateULF(centerX + dimensionX / 2.0, centerY + dimensionY / 2.0, dimensionZ);  //左前
    Vector3D coordinateULR(centerX - dimensionX / 2.0, centerY + dimensionY / 2.0, dimensionZ);  //左后
    Vector3D coordinateURF(centerX + dimensionX / 2.0, centerY - dimensionY / 2.0, dimensionZ);  //右前
    Vector3D coordinateURR(centerX - dimensionX / 2.0, centerY - dimensionY / 2.0, dimensionZ);  //右后

    //根据航向角/俯仰角/横滚角转换坐标向量
    bBoxVec.clear();
    bBoxVec.reserve(8);
    // 底部4个顶点坐标
    Transform::VectorRotated(hdgWorld, pitchWorld, rollWorld, coordinateLLF);  //左前
    bBoxVec.push_back(coordinateLLF + vehPos);
    Transform::VectorRotated(hdgWorld, pitchWorld, rollWorld, coordinateLLR);  //左后
    bBoxVec.push_back(coordinateLLR + vehPos);
    Transform::VectorRotated(hdgWorld, pitchWorld, rollWorld, coordinateLRF);  //右前
    bBoxVec.push_back(coordinateLRF + vehPos);
    Transform::VectorRotated(hdgWorld, pitchWorld, rollWorld, coordinateLRR);  //右后
    bBoxVec.push_back(coordinateLRR + vehPos);
    // 顶部4个顶点坐标
    Transform::VectorRotated(hdgWorld, pitchWorld, rollWorld, coordinateULF);  //左前
    bBoxVec.push_back(coordinateULF + vehPos);
    Transform::VectorRotated(hdgWorld, pitchWorld, rollWorld, coordinateULR);  //左后
    bBoxVec.push_back(coordinateULR + vehPos);
    Transform::VectorRotated(hdgWorld, pitchWorld, rollWorld, coordinateURF);  //右前
    bBoxVec.push_back(coordinateURF + vehPos);
    Transform::VectorRotated(hdgWorld, pitchWorld, rollWorld, coordinateURR);  //右后
    bBoxVec.push_back(coordinateURR + vehPos);
}

//根据车辆转向灯，获取下一条路和在下一条路上的行驶方向
void Sensor::getNextRoadByLightMask(Road *&nextRoad, int &nextRoadDir)
{
    log_compnt_mngr->info("Sensor::getNextRoadByLightMask start.");
    // 获取主车当前所在道路
    Road *currRoad = RoadSystem::Instance()->getRoad(std::to_string(egoState->sObjectState.u8RoadId));
    if (currRoad == nullptr)
    {
        int currDir = 1;	// 主车相对于道路S轴的朝向，1和S轴同向，-1为反向
        if (egoState->sObjectState.u1Dir == D_SP_ROAD_DIR_FLAG_DIR_FWD)
        {
            currDir = 1;
        }
        else if (egoState->sObjectState.u1Dir == D_SP_ROAD_DIR_FLAG_DIR_REAR)
        {
            currDir = -1;
        }

        //获取当前Road的“所有下一条路”（所有与之连接的路）
        int nextRoadType = 0; //1非junction，2是junction
        RoadTransition nextRoadTransition;
        std::list<std::pair<double, RoadTransition>> junctionRoadTransitionList;
        currRoad->getNextRoadTransition(nextRoadType, nextRoadTransition, junctionRoadTransitionList, currDir);

        if (nextRoadType == 1)//如果下一条路非junction,则将该条路作为nextRoad
        {
            if (nextRoadTransition.road != nullptr)
            {
                nextRoad = nextRoadTransition.road;
                nextRoadDir = nextRoadTransition.direction;
            }
        }
        else if (nextRoadType == 2)//junction
        {
            std::list<std::pair<double , RoadTransition>>::iterator roadTransitionIt = junctionRoadTransitionList.begin();
            if (junctionRoadTransitionList.size() == 1)//这个路口只有一个方向
            {
                nextRoad = roadTransitionIt->second.road;
                nextRoadDir = roadTransitionIt->second.direction;
            }
            else //找与主车转向灯方向匹配的一条路
            {
                bool indicatorLeft = false;
                bool indicatorRight = false;

                if ((egoState->u4LightMask & D_SP_VEHICLE_LIGHT_INDICATOR_L) != 0)//如果左转向灯亮
                {
                    indicatorLeft = true;
                }
                if ((egoState->u4LightMask & D_SP_VEHICLE_LIGHT_INDICATOR_R) != 0)//如果右转向灯亮
                {
                    indicatorRight = true;
                }

                int turnDir = 0; //1左转 0直行 -1右转
                if (indicatorLeft && (!indicatorRight)) //如果左转向灯亮
                {
                    turnDir = 1;
                }
                else if (indicatorRight && (!indicatorLeft)) //如果右转向灯亮
                {
                    turnDir = -1;
                }
                else
                {
                    // comment: Nothing to do!
                }

                for (; roadTransitionIt != junctionRoadTransitionList.end(); roadTransitionIt++)
                {
                    if (static_cast<int>(roadTransitionIt->first) == turnDir) //1左转 0直行 -1右转
                    {
                        nextRoad = roadTransitionIt->second.road;
                        nextRoadDir = roadTransitionIt->second.direction;
                        break;
                    }
                }
            }
        }
        else
        {
            //Nothing to do!
        }
    }
    log_compnt_mngr->info("Sensor::getNextRoadByLightMask end.");
}

// 将单个Vector3D从笛卡尔坐标转换为球坐标  
SphericalCoordinates Sensor::CartesianToSpherical(const Vector3D& cart) {  
	// https://wenku.baidu.com/view/03940236f624ccbff121dd36a32d7375a517c64d.html?_wkts_=1716539107418&bdQuery=%E7%AC%9B%E5%8D%A1%E5%B0%94%E5%9D%90%E6%A0%87%E7%B3%BB%E8%BD%AC%E4%B8%BA%E7%90%83%E5%9D%90%E6%A0%87%E7%B3%BB
	// 球坐标系(r,θ,φ)与笛卡尔坐标系(x,y,z)的转换关系
	// r = √(x^2+y^2+z^2)
	// θ = arcsin(z/r)
	// φ = atan2(y,x)
    SphericalCoordinates sph;  
    sph.r = std::sqrt(cart.x() * cart.x() + cart.y() * cart.y() + cart.z() * cart.z()); // 半径 
	log_compnt_mngr->trace("vehicle coordinate Pois x={} y={} z={}", cart.x(), cart.y(), cart.z()); 
      
    if (sph.r == 0.0) {  
        // 处理原点的情况，通常球坐标在原点处未定义  
        sph.theta = 0.0;  
        sph.phi = 0.0;  
    } else {  
        sph.theta = std::asin(cart.z() / sph.r); // 极角（从z轴到xy平面的夹角）  
        if (cart.x() != 0.0) {  
            sph.phi = std::atan2(cart.y(), cart.x()); // 方位角（在xy平面内）  
        } else if (cart.y() > 0.0) {  
            sph.phi = PI / 2.0; // 当x=0且y>0时  
        } else if (cart.y() < 0.0) {  
            sph.phi = -PI / 2.0; // 当x=0且y<0时  
        } else {  
            // cart.x = cart.y = 0，这意味着在z轴上，但phi在这种情况下可以是任何值，通常设为0  
            sph.phi = 0.0;  
        }  
          
        // 确保phi在[-PI, PI]范围内  
        if (sph.phi < -PI) sph.phi += 2 * PI;  
        if (sph.phi > PI) sph.phi -= 2 * PI;  
    }  
	log_compnt_mngr->trace("vehicle coordinate Pois phi={} theta={} r={}", sph.phi, sph.theta, sph.r); 
    return sph;  
}  

// 判断该点是否在传感器探测范围内
bool Sensor::isPoisInSensorView(const Vector3D& cart)
{

	/* 获取传感器配置信息 */
	double hfov = getHFOV();
	double vfov = getVFOV();
	double range = getRange();

	/* 将点从笛卡尔坐标系转化为球坐标系 */
	SphericalCoordinates tmp = CartesianToSpherical(cart);

	log_compnt_mngr->debug("hfov={} vfov={} range={}", hfov, vfov, range); 

	/* 判断该点是否在传感器范围内 */
	if (tmp.r <= range && abs(tmp.phi) <= hfov/2 && abs(tmp.theta) <= vfov/2){
		return true;
	}
	return false;
}

bool Lidar::getBoundingBoxStatus()
{
    return _description.boundingBoxEnable;
}

bool Camera::getBoundingBoxStatus()
{
    return _description.boundingBoxEnable;
}

bool IdealizedSensor::getBoundingBoxStatus()
{
    return false;
}

bool Radar::getBoundingBoxStatus()
{
    return _description.boundingBoxEnable;
}

bool Ultrasonic::getBoundingBoxStatus()
{
    return _description.boundingBoxEnable;
}
