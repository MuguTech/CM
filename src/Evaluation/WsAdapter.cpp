#include "WsAdapter.h"
#include <CJsonObject.hpp>
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <algorithm>
// #include "Common.h"
#include <sys/prctl.h>
#include <linux/prctl.h>
#include <pthread.h>
#include "../log.h"

static pthread_mutex_t lo_start_app_mutex;
static pthread_mutex_t lo_snd_msg_mutex;

#include <dirent.h>

WsAdapter *WsAdapter::Instance()
{
    log_evaluation->critical("Instance start.");
    static WsAdapter __instance;
    return &__instance;
}

void WsAdapter::Destroy()
{
// 	std::cout << "Destroy start." << '\n';
// #if 1 /* dingjiajia 2020.12.30 */
// 	if (g_fd_w2s != 0) {
// 		close(g_fd_w2s);
// 	}
// 	if (g_fd_s2w != 0) {
// 		close(g_fd_s2w);
// 	}
// #endif
// 
//     // delete __instance;
//     // __instance = NULL;
// 
// 	std::cout << "testPst1." << '\n';
// 
// #if 1 /* dingjiajia 2021.04.27 */
// 	(void)pthread_mutex_destroy(&lo_start_app_mutex);
// #endif
// 
// #if 1 /* dingjiajia 2022.4.6 [禅道6258] [add]【start】 */
// 	(void)pthread_mutex_destroy(&lo_snd_msg_mutex);
// #endif /* dingjiajia 2022.4.6 [禅道6258] [add]【end】 */
// 	std::cout << "Destroy.end." << '\n';
}



int WsAdapter::startApp(const char* command, const char* childname) /* [新规方法]dingjiajia 2020.11.24 启动APP */
{
	/* 进入临界区 */
	pthread_mutex_lock(&lo_start_app_mutex);
	int fd; /* Descriptor for FIFO */
	int retry = 0;
	int flag_startapp = 0;


	while(1)
	{
		retry++;
		if (retry <= 3) {
			if ( 0 == strcmp(childname, "SCENEDESIGN\0") ) {
				if((fd = open("/tmp/.startapp", O_WRONLY)) < 0) {
					// perror("open");
					continue;
				} else {
					flag_startapp = 1;
					break;
				}
			}
			if ( 0 == strcmp(childname, "AUTOMATICTEST\0") ) {
				if((fd = open("/tmp/.startapp", O_WRONLY)) < 0) {
					// perror("open");
					continue;
				} else {
					flag_startapp = 2;
					break;
				}
			}
			if ( 0 == strcmp(childname, "ROADEDITOR\0") ) {
				if((fd = open("/tmp/.startapp", O_WRONLY)) < 0) {
					// perror("open");
					continue;
				} else {
					flag_startapp = 3;
					break;
				}
			}
			if ( 0 == strcmp(childname, "VEHICLEDYNAMICS\0") ) {
				if((fd = open("/tmp/.startapp", O_WRONLY)) < 0) {
					// perror("open");
					continue;
				} else {
					flag_startapp = 4;
					break;
				}
			}
			if ( 0 == strcmp(childname, "nautilus_\0") ) {
				if((fd = open("/tmp/.startapp", O_WRONLY)) < 0) {
					// perror("open");
					continue;
				} else {
					flag_startapp = 5;
					break;
				}
			}
			if ( 0 == strcmp(childname, "offlineEvalMain\0") ) {
				if((fd = open("/tmp/.startapp", O_WRONLY)) < 0) {
					// perror("open");
					continue;
				} else {
					flag_startapp = 6;
					break;
				}
			}
			if ( 0 == strcmp(childname, "ScenarioCaptureTool\0") ) {
				if((fd = open("/tmp/.startapp", O_WRONLY)) < 0) {
					continue;
				} else {
					flag_startapp = 7;
					break;
				}
			}
            if ( 0 == strcmp(childname, "python_script\0") ) {
                if((fd = open("/tmp/.startapp", O_WRONLY)) < 0) {
                    continue;
                } else {
                    flag_startapp = 8;
                    break;
                }
            }
			if ( 0 == strcmp(childname, "FunctionalSafety\0") ) {
				if((fd = open("/tmp/.startapp", O_WRONLY)) < 0) {
					continue;
				} else {
					flag_startapp = 9;
					break;
				}
			}
			if ( 0 == strcmp(childname, "web2simpro\0") ) {
				if((fd = open("/tmp/.web2simpro", O_WRONLY)) < 0) {
					// perror("open");
					continue;
				} else {
					break;
				}
			}
			if ( 0 == strcmp(childname, "simpro2web\0") ) {
				if((fd = open("/tmp/.simpro2web", O_WRONLY)) < 0) {
					// perror("open");
					continue;
				} else {
					break;
				}
			}
		}

		/* 离开临界区 */
		pthread_mutex_unlock(&lo_start_app_mutex);
		return 1;
	}
	/* Create the string to write */
	int len; /* Bytes written to FIFO */
	char buf[PIPE_BUF*2]={0}; /* Ensure atomic writes */
	if (flag_startapp > 0) {
		len = sprintf(buf, "%d|%s", flag_startapp, command);
	} else {
		len = sprintf(buf, "%s", command);
	}
	/*
	* Use (len+1) because sprintf does not count
	* the terminating null
	*/
	if((write(fd, buf, len + 1)) < 0) {
		perror("write");
	}

	close(fd);
	/* 离开临界区 */
	pthread_mutex_unlock(&lo_start_app_mutex);
	return 1;
}



