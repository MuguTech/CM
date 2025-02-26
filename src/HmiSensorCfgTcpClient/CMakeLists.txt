cmake_minimum_required(VERSION 3.0.2)
project(HmiSensorCfgTcpClient)


SET(HEADERS
    HmiSensorCfgTcpClient.h
    HmiPraseMsg.h
    sensorMsgType.h
    ./simproSensor/simproCameraDescription.h
    ./simproSensor/simproLidarDescription.h
    ./simproSensor/simproRadarDescription.h
    ./simproSensor/simproSensorDescription.h
    ./simproSensor/simproUltrasonicDescription.h
)

SET(SOURCES
    HmiSensorCfgTcpClient.cpp
    HmiPraseMsg.cpp
)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")



if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
    add_executable(${PROJECT_NAME} test_main.cpp ${SOURCES} ${HEADERS})
else()
    # ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
    ADD_LIBRARY(${PROJECT_NAME} ${SOURCES} ${HEADERS})
endif()


# INSTALL(TARGETS ${PROJECT_NAME}
#         LIBRARY DESTINATION lib
#         )
