#include "DynamicVehicleMgr.h"
#include "log.h"
#include <iostream>


DynamicVehicleMgr::DynamicVehicleMgr()
{
}

DynamicVehicleMgr::~DynamicVehicleMgr()
{
}


bool DynamicVehicleMgr::isRun(void * sensor_info)
{
    return true;
}



bool DynamicVehicleMgr::coverDataFrame(void *input, size_t inlen, void *output, size_t &outlen)
{
    if (inlen > outlen)
    {
        log_compnt_mngr->error("inlen {} is biger the outlen {}.", inlen, outlen);
        outlen = 0;
        return false;
    }

    memcpy((char *)output, (char *)input, inlen);
    outlen = inlen;

    return true;
}



