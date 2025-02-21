/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-06-13 14:53:12
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-06-20 15:29:17
 * @FilePath: /mysylar/mytest/test_thread.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "sylar.h"
#include "thread.h"
#include <iostream>
#include <mutex>

mysylar::Logger::ptr g_logger = SYLAR_LOG_NAME("root");
mysylar::Logger::ptr g_logger_system = SYLAR_LOG_NAME("system");
int count =0;
mysylar::RWMutex s_mutex;
void fun1(){
    SYLAR_LOG_INFO(g_logger)<< "name : " << mysylar::Thread::GetName()
        << " this name: " << mysylar::Thread::GetThis()->getName()
        << " id :"<< mysylar::GetThreadId()
        << " this.id:" << mysylar::Thread::GetThis()->getId();
        // 如果不加锁的话，会出现总数不为500000
        // count++ 可以简单理解为 count = count + 1 ；
        // count 取值时候值为10 然后会发生中断，被其他线程修改成了12 ，然后再回到这里，就将count修改成了11
        // 这样就会出现了少加的情况 
        for(int i=0;i<100000;i++){
            mysylar::RWMutex::WriteLock lock(s_mutex);
            count = count + 1;
        }
}
void fun2(){
    while(true){
        SYLAR_LOG_INFO(g_logger) << "***************************************************************";
    }
} 
void func3(){
    while (true)
    {
        SYLAR_LOG_INFO(g_logger) << "--------------------------------------------------------------------";
    }
}

int main(){
    YAML::Node node = YAML::LoadFile("/home/jiangz/CODE/mysylar/config/log.yml");
    mysylar::Config::LoadFileFromYaml(node);
    //为什么重新Load之后找到的还是原来的system
    SYLAR_LOG_INFO(g_logger) << "start to test Thread ";
    std::vector<mysylar::Thread::ptr> thrs;
    // std::vector<mysylar::Thread::ptr> thr1s;
    for(int i =0;i<5;i++){
        mysylar::Thread::ptr thr(new mysylar::Thread(fun1,"name_"+std::to_string(i)));
        // mysylar::Thread::ptr thr1(new mysylar::Thread(func3,"name_"+std::to_string(i)));
        thrs.push_back(thr);
        // thr1s.push_back(thr1);
    }
   
    for(int i=0;i<5;i++){
        thrs[i]->join();
        // thr1s[i]->join();
    } 
    SYLAR_LOG_INFO(g_logger) << "end to test Thread ";
    SYLAR_LOG_INFO(g_logger) << count;



    return 0;
}