#ifndef WsAdapter_H
#define WsAdapter_H
#include <cstddef>
#include <iostream>
#include <list>
#include <stdlib.h>
#include <vector>

class WsAdapter
{
public:
    static WsAdapter *Instance();
    static void Destroy();
    void startOfflineEvaluation();
	int startApp(const char* command, const char* childname); /* [新规方法]dingjiajia 2020.11.24 启动APP */
};



#endif
