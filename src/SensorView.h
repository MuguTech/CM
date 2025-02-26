#ifndef __SENSOR_VIEWER_H__
#define __SENSOR_VIEWER_H__

#define OPEN_ROADSYSTEM 1
#define DEBUG_SENSER_VIEW 1

#include <osi3/osi_hostvehicledata.pb.h>
#include <osi3/osi_sensorviewconfiguration.pb.h>
#include <osi3/osi_datarecording.pb.h>
#include <osi3/osi_groundtruth.pb.h>
#include "../../include/Runtime/coSimu/SimProType.h"


#if OPEN_ROADSYSTEM
    #include "../../APF/RoadSystem/Road.h"
    #include "../../APF/RoadSystem/Junction.h"
    #include "../../APF/RoadSystem/RoadSystem.h"
    #include "../../APF/RoadSystem/Types.h"
#endif



class SensorView
{
private:
        osi3::SensorView m_OSISensorView;
        osi3::MovingObject* m_OSIhost;
        S_SP_MIL_EGO_STATE* m_pEgoMsg;
public:
    SensorView();
    ~SensorView();
    bool coverDataFrame(void *input, size_t inlen, void *output, size_t &outlen);

private:
        void afterParseMessage(osi3::GroundTruth* groundTruth);
        void printLog();
    	// 获取车道线点世界坐标
    #if OPEN_ROADSYSTEM
	    int getLaneLinePois(double _intv, int _num, Road* _road, int _laneId, double _startu, int _dir, std::vector<Vector3D> &_lPois, std::vector<Vector3D> &_rPois);
    #endif

};



#endif // __SENSOR_VIEWER_H__

