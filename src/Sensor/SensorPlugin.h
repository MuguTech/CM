#ifndef SENSORPLUGIN_H
#define SENSORPLUGIN_H

#include "Sensor.h"
#include <memory>
#include "../HmiSensorCfgTcpClient/HmiPraseMsg.h"
#include "../ConfigureMngr.h"

extern "C" void init(void *desc, int inlen, uint32_t pluginId);

extern "C" void update(uint32_t pluginId, void *input, int inlen, void *output, int &outlen);

extern "C" void stop();

extern "C" void isWorking();

std::map<uint32_t, std::shared_ptr<Sensor>> sensorIdMap;

bool bIsWorking;

#endif //SENSORPLUGIN_H
