#ifndef RDBOSIGroundTruthPkg_H
#define RDBOSIGroundTruthPkg_H

#include <string>
#include <cstring>
#include <list>
#include <set>
#include "../../include/Runtime/coSimu/SimProType.h"

class PackMsg
{

public:
    PackMsg();
	~PackMsg();

    //把所有pkg打包成msg
    static void getPkgData(void *groundTruthMsg, size_t groundTruthMsgLen, const std::list<std::tuple<char *, unsigned int>> &inputBufferList, void *output, size_t &outlen);

    static void getStopPkgData(void *input, size_t inlen, void *output, size_t &outlen);

    static void generateMsg(char *input, size_t inlen);       //生成数据

private:
    //解析SensorDetectionInfoPkg
    static void parseSensorDetectionInfoPkg(const std::list<std::tuple<char *, unsigned int>> &inputBufferList, std::set<uint32_t> &vehiclePedObsIdSet, std::set<uint32_t> &trafficLightIdSet, std::set<uint32_t> &trafficSignIdSet);

    //解析groundTruthMsg
    static void parseGroundTruthMsg(void *groundTruthMsg, size_t groundTruthMsgLen, const std::set<uint32_t> &vehiclePedObsIdSet, const std::set<uint32_t> &trafficLightIdSet, const std::set<uint32_t> &trafficSignIdSet
                                    , std::list<S_SP_MIL_OBJECT_STATE *> &vehiclePedObsList, std::list<char *> &trafficLightPkgList, std::list<S_SP_TRAFFIC_SIGN *> &trafficSignList, unsigned int &reserveSize);

    //打包groundTruthMsg中要保留的pkg
    static void generateGroundTruthPkg(void *groundTruthMsg, size_t groundTruthMsgLen, const std::list<S_SP_MIL_OBJECT_STATE *> &vehiclePedObsList, const std::list<char *> &trafficLightPkgList
                                       , const std::list<S_SP_TRAFFIC_SIGN *> &trafficSignList, void *msgBuffer, int &msgBufferUsedSize);
};

#endif//RDBOSIGroundTruthPkg_H

