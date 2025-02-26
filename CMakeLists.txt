cmake_minimum_required(VERSION 3.0.2)
project(component_mngr)


if(CMAKE_MODE STREQUAL "release")
    SET(CMAKE_BUILD_TYPE "Release")
    MESSAGE(STATUS "构建发布版本")
else()
    SET(CMAKE_BUILD_TYPE "Debug")
    set(CMAKE_CXX_FLAGS_DEBUG "-g -ggdb")
    MESSAGE(STATUS "构建测试版本")
endif()


# simpro帧率控制数据传输架构图整改 zxz 2023.12.25 不使用V1000HZ宏定义
# 如果不需要1000帧功能，则注释掉下面两行
# ADD_DEFINITIONS(-DV1000HZ)
# OPTION(V1000HZ "V1000HZ")

ADD_DEFINITIONS(-DSAFETYCOMPILATION)                # 不开启安全编译的时候注释本行
OPTION(SAFETYCOMPILATION "SAFETYCOMPILATION" ON)       # 不开启安全编译的时候注释本行

if(DEFINED SAFETYCOMPILATION)
message("enter ComponentManager SAFETY COMPILATION")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fpie -s")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pie -s")
set(CMAKE_EXECUTABLE_LINKER_FLAGS "${CMAKE_EXECUTABLE_LINKER_FLAGS} -pie -s")
endif()

include_directories(
        /usr/local/include/osi3
        ../Main
        ../APF
        ../APF/RoadSystem
        ../Sensor
        ./OSIGroundTruthGenerator
)


LINK_DIRECTORIES(
        /usr/local/lib/osi3
)


file(GLOB_RECURSE RoadSystemHeaders ../APF/RoadSystem/*.h)
file(GLOB_RECURSE SimProCommonHeader ../Main/Common.h)
file(GLOB_RECURSE OSIGroundTruthGeneratorSrcs ./src/OSIGroundTruthGenerator/*.cpp ./src/OSIGroundTruthGenerator/*.h ./src/OSIGroundTruthGenerator/*.cc)
file(GLOB_RECURSE CryptoSrcs ../APF/Crypto/*.cpp ../APF/Crypto/*.h)
file(GLOB_RECURSE FmiDynamicAdapterSrcs ./src/FmiDynamicAdapter/*.cpp ./src/FmiDynamicAdapter/*.h ./src/FmiDynamicAdapter/Fmi/*.h)
file(GLOB_RECURSE UnzipSrcs ../APF/Unzip/*.cpp ../APF/Unzip/*.h)


SET(V1000HZ_SRC
    ./src/common.cpp
    ./src/common.h
    ./src/ConfigureMngr.cpp
    ./src/ConfigureMngr.h
    ./src/DynamicVehicleCfg.cpp
    ./src/DynamicVehicleCfg.h
    ./src/DynamicVehicleMgr.cpp
    ./src/DynamicVehicleMgr.h
    ./src/FrameCtrlSocket.cpp
    ./src/FrameCtrlSocket.h
    ./src/log.cpp
    ./src/log.h
    ./src/main.cpp
    ./src/PluginCfg.cpp
    ./src/PluginCfg.h
    ./src/PluginMngr.cpp
    ./src/PluginMngr.h
    ./src/SocketClient.cpp
    ./src/SocketClient.h
    ./src/V_TrafficSocket.cpp
    ./src/V_TrafficSocket.h
)


aux_source_directory(./src DIR_SRCS)
aux_source_directory(./src/Sensor DIR_SENSOR_SRCS)


add_subdirectory(./src/SocketFw)
add_subdirectory(./src/simproxml)
add_subdirectory(./src/Evaluation)
add_subdirectory(./src/Sensor)
add_subdirectory(./src/HmiSensorCfgTcpClient)

if(DEFINED V1000HZ)
    add_executable(${PROJECT_NAME}
                    ${V1000HZ_SRC}
                    ${SimProCommonHeader}
    )


    target_link_libraries(${PROJECT_NAME}
            OpenCRG
            SocketFw simproxml SystemCtrol
            open_simulation_interface protobuf
            -lboost_filesystem -ldl
    )


else()
    add_executable(${PROJECT_NAME}
                    ${DIR_SRCS}
                    ${CryptoSrcs}
                    ${DIR_SENSOR_SRCS}
                    ${RoadSystemHeaders}
                    ${SimProCommonHeader}
                    ${OSIGroundTruthGeneratorSrcs}
                    ${FmiDynamicAdapterSrcs}
                    ${UnzipSrcs}
                    ../APF/RoadSystem/Types.h
                    ../APF/RoadSystem/Types.cpp
                    ../Runtime/TrafficSimulation/TrafficSimulation.h
                    ../Runtime/TrafficSimulation/TrafficSimulation.cpp
    )
    # add_executable(${PROJECT_NAME} ./src/socket_client_test.cpp ./src/v_traffic_socket.cpp ./src/socket_client.cpp)


    target_link_libraries(${PROJECT_NAME}
            SimProApfRoadSystem OpenCRG
            SocketFw simproxml SystemCtrol SimProEvaluationVC
            open_simulation_interface protobuf
            -lboost_filesystem -ldl HmiSensorCfgTcpClient
            zip
    )

endif()


INSTALL(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)


