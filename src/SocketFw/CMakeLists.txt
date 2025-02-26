cmake_minimum_required(VERSION 3.0.2)
project(SocketFw)

SET(CMAKE_BUILD_TYPE "Debug")

SET(HEADERS
        SocketBase.h
        SocketTcpServer.h
        SocketFactory.h
        SocketTcpServerMultiClient.h
        )

SET(SOURCES
        SocketBase.cpp
        SocketTcpServer.cpp
        SocketFactory.cpp
        SocketTcpServerMultiClient.cpp
        )

SET(LOG_SRC
        ../log.cpp
        ../log.h
)

# 安全编译选项定义 开始 dyling 2022.1.28
if(DEFINED SAFETYCOMPILATION)
# 3. 地址无关   ---- so文件
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()
# 安全编译选项定义 结束


INCLUDE_DIRECTORIES(
                ../../
        )


if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
    add_executable(${PROJECT_NAME} test_main.cpp ${SOURCES} ${HEADERS} ${LOG_SRC})
else()
    # ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
    ADD_LIBRARY(${PROJECT_NAME} ${SOURCES} ${HEADERS} ${LOG_SRC})
endif()

target_link_libraries(${PROJECT_NAME} pthread )

# INSTALL(TARGETS ${PROJECT_NAME}
#         LIBRARY DESTINATION lib
#         )
