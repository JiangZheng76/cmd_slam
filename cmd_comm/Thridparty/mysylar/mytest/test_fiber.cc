/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-06-21 18:27:36
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-02-09 12:31:14
 * @FilePath: /mysylar/mytest/test_fiber.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "sylar.h"
#include "fiber.h"
#include <vector>
#include <string>
void run_in_fiber(){
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) <<"run_in_fiber begin. ";
    mysylar::Fiber::YieldToHold();
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) <<"run_in_fiber centerl.";
    mysylar::Fiber::YieldToHold();
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) <<"run_in_fiber end";
}

void test_fiber(){
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "main begin -1";
    {
        mysylar::Fiber::GetThis();
        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) <<mysylar::Thread::GetName() <<  " main begin.";
        mysylar::Fiber::ptr fiber(new mysylar::Fiber(run_in_fiber));
        fiber->swapIn();
        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) <<mysylar::Thread::GetName() <<  " main after swap in.";
        fiber->swapIn();
        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) <<mysylar::Thread::GetName() <<  " main after swao in 2.";
        fiber->swapIn();
    }
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "main end-1";
}

int main(){
    mysylar::Thread::SetName("main");

    std::vector<mysylar::Thread::ptr> thrs;
    for(size_t i =0 ;i<5;i++){  
        // 每个线程执行一个携程
        thrs.push_back(mysylar::Thread::ptr(new mysylar::Thread(test_fiber,"name_"+std::to_string(i))));
    }
    
    for(auto i : thrs){
        i->join();
    }
    return 0;
}