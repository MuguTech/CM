cmake_minimum_required(VERSION 3.16)                                                                                                          
project(SimProEvaluationVC)

set(CMAKE_CXX_STANDARD 14) 


SET(HEADERS
        EvaluationAPI.h
		WsAdapter.h
		SocketLocalClient.h
        )

SET(SOURCES
        ../simproxml/simproxml.cpp
        EvaluationAPI_fs.cpp
		EvaluationAPI_non_fs.cpp
		WsAdapter.cpp
		ConfigManager.cpp
		SocketLocalClient.cpp
		)


# 安全编译选项定义 开始 dyling 2022.1.28
if(DEFINED SAFETYCOMPILATION)
# 3. 地址无关   ---- so文件
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=c++11")
endif()
# 安全编译选项定义 结束


INCLUDE_DIRECTORIES(
	./
	../
	../simproxml
    ./ThirdParty/CJson/include
	/usr/include/python3
	/usr/local/include/osi3
)

LINK_DIRECTORIES(
		/usr/local/lib/osi3
        ./ThirdParty/CJson/lib/
)

ADD_LIBRARY(SimProEvaluationVC SHARED ${SOURCES} ${HEADERS})

target_link_libraries(SimProEvaluationVC
        python3
		open_simulation_interface
		# ${Protobuf_LIBRARY}
        stdc++fs
        )

INSTALL(TARGETS SimProEvaluationVC
        LIBRARY DESTINATION lib
        )

#add_executable(evaluation_trigger main.cpp)

# 添加链接库
#target_link_libraries(evaluation_trigger PRIVATE SimProEvaluationVC cjsonobject)

# 设置动态库路径
link_directories(./build)

# 设置头文件路径
include_directories(./)
