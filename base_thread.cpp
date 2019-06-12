////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2018      Base Code for robot
///ALL RIGHTS RESERVED
///@file:base_thread.cpp
///@brief: 线程基类源文件，封装线程操作。
///@vesion 1.0
///@author: gezp
///@email: 1350824033@qq.com
///@date: 2018.3.3
///修订历史：
//参考:http://blog.csdn.net/maotoula/article/details/18501963
////////////////////////////////////////////////////////////////////////////////

#include "base_thread.h"  


void* BaseThread::run1()  
{  
    threadStatus = THREAD_STATUS_RUNNING;  
    tid = pthread_self();  
    run();  
    threadStatus = THREAD_STATUS_EXIT;  
    tid = 0;  
    pthread_exit(NULL);  
}  

BaseThread::BaseThread()  
{  
    tid = 0;  
    threadStatus = THREAD_STATUS_NEW;  
}  

bool BaseThread::start()  
{  
    int iRet = 0;  
    return  pthread_create(&tid, NULL, thread_proxy_func, this) == 0;  
}  

pthread_t BaseThread::getThreadID()  
{  
    return tid;  
}  

int BaseThread::getState()  
{  
    return threadStatus;  
}  

void BaseThread::join()  
{  
    if (tid > 0)  
    {  
        pthread_join(tid, NULL);  
    }  
}  
void * BaseThread::thread_proxy_func(void * args)  
{  
        BaseThread * pThread = static_cast<BaseThread *>(args);

        pThread->run1();
        return pThread;
}  

void BaseThread::join(unsigned long millisTime)  
{  
    if (tid == 0)  
    {  
        return;  
    }  
    if (millisTime == 0)  
    {  
        join();  
    }else{
        unsigned long k = 0;  
        while (threadStatus != THREAD_STATUS_EXIT && k <= millisTime)  
        {  
            usleep(100); 
            k++;  
        }  
    }  
}  
