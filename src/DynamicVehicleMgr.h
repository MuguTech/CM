#ifndef __DYNAMIC_VEHICLE_MGR_H__
#define __DYNAMIC_VEHICLE_MGR_H__


#include <cstring>


class DynamicVehicleMgr
{
private:
    /* data */
public:
    DynamicVehicleMgr();
    ~DynamicVehicleMgr();

    bool isRun(void * sensor_info);
    bool coverDataFrame(void *input, size_t inlen, void *output, size_t &outlen);

};




#endif  // __DYNAMIC_VEHICLE_MGR_H__

