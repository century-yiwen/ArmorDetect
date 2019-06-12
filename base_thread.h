////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC ROBOMASTER2018      Base Code for robot
///ALL RIGHTS RESERVED
///@file:base_thread.h
///@brief: 线程基类头文件，封装线程操作。
///@vesion 1.0
///@author: Gezp
///@email: 1350824033@qq.com
///@date: 2018.3.3
///修订历史：
///参考:http://blog.csdn.net/maotoula/article/details/18501963
////////////////////////////////////////////////////////////////////////////////
#ifndef BASE_THREAD_H 
#define BASE_THREAD_H

#include <unistd.h>     /*Unix 标准函数定义*/
#include <iostream>  
#include <pthread.h>



using namespace std;  



class BaseThread  
{  
public:
    //构造函数  
    BaseThread(); 
private:  
    //当前线程的线程ID  
    pthread_t tid;  
    //线程的状态  
    int threadStatus;  
    //获取执行方法的指针  
    static void * thread_proxy_func(void * args);  
    //内部执行方法  
    void* run1();  
public:  
    //线程的状态－新建  
    static const int THREAD_STATUS_NEW = 0;  
    //线程的状态－正在运行  
    static const int THREAD_STATUS_RUNNING = 1;  
    //线程的状态－运行结束  
    static const int THREAD_STATUS_EXIT = -1;   
    //线程的运行实体  
    virtual void run()=0;  
    //开始执行线程  
    bool start();  
    //获取线程ID  
    pthread_t getThreadID();  
    //获取线程状态  
    int getState();  
    //等待线程直至退出  
    void join();  
    //等待线程退出或者超时  
    void join(unsigned long millisTime);  
};  
#endif


/*

example
class Test : public BaseThread
{

public:
void run(){
    //线程逻辑.
}

}

using:

Test test;
test.start(); //开启线程

*/


