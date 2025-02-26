#include "EvaluationAPI.h"
#include "../log.h"

#include <iostream>
#include <cstring>

#ifndef ENABLE_CLOUD 
#include <signal.h>

#define D_HTTP_RECV_BUF_SIZE (256)




#endif /* no ENABLE_CLOUD */


EvaluationAPI *EvaluationAPI::_instance = nullptr;

EvaluationAPI *EvaluationAPI::Instance()
{
    log_evaluation->critical("enter");
    if (_instance == nullptr)
    {
        _instance = new EvaluationAPI();
    }
    return _instance;
}

void EvaluationAPI::Destroy()
{
    log_evaluation->critical("enter");
    // delete _instance;
    // _instance = nullptr;
}

EvaluationAPI::EvaluationAPI()
{ 
    log_evaluation->critical("enter");

    // (void )memset(mStrSimuData, 0X00, D_EVALUATION_API_SIMU_DATA_SIZE);
    // mU4SimuDataCurCount = 0;
	
	// init_non_fs();

// #if 1 /* dingjiajia 23.03.10 非广汽云仿真 [接收http消息] [start] */
// #ifndef ENABLE_CLOUD 
// 	pthread_t pid_http;
// 	int err = pthread_create(&pid_http, NULL, http_main, NULL);
// 	if (err != 0)
// 		std::cout << "can't create thread(http_main): " << err << '\n';
// 		// mod_ws_server->error("can't create thread(http_main): {}\n", err);
// #endif /* no ENABLE_CLOUD */
// #endif /* dingjiajia 23.03.10 非广汽云仿真 [接收http消息] [end] */
}

EvaluationAPI::~EvaluationAPI()
{
    log_evaluation->critical("enter");
}
