project(SimProSensorPlugin)
cmake_minimum_required(VERSION 3.16)

SET(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "-g -ggdb")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fstack-protector-all -D_FORTIFY_SOURCE=2 -O2 -s")

set(HEADERS
        BasicSensorDescription.h
        CameraDescription.h
        LidarDescription.h
        RadarDescription.h
        Sensor.h
        SensorDescription.h
        UltrasonicDescription.h
        SensorPlugin.h
)

set(SOURCES
        Sensor.cpp
        SensorPlugin.cpp
)

INCLUDE_DIRECTORIES(
)

LINK_DIRECTORIES(
)

ADD_LIBRARY(SimProSensorPlugin SHARED ${SOURCES} ${HEADERS})

target_link_libraries(SimProSensorPlugin
)

INSTALL(TARGETS SimProSensorPlugin
	LIBRARY DESTINATION lib
)
