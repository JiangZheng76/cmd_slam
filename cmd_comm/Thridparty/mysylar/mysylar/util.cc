/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-05-17 21:50:39
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-02-25 15:19:41
 * @FilePath: /mysylar/mysylar/util.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "util.h"
#include <unistd.h>
#include <sys/syscall.h>
#include <sylar.h>
#include <exception>
#include <typeinfo>
#include <string>
#include <sys/time.h>
#include "fiber.h"
namespace mysylar{

mysylar::Logger::ptr g_log_system = SYLAR_LOG_NAME("system");

pid_t GetThreadId(){
    
    return syscall(__NR_gettid);
    // return getpid();
}

u_int64_t GetFiberId() { return Fiber::GetFiberId(); } 

void showAllNode(std::list<std::pair<std::string,const YAML::Node>> all_node){
    std::stringstream ss;
    for(auto & l : all_node){
        if(l.second.IsScalar()){
            std::cout << l.first ;
            // std::cout << " " << ls.second.Scalar();
            std::cout<<std::endl;    
            continue;
        }
        ss << l.second;
        std::cout << l.first ;
        // std::cout << " " << s.str();
        std::cout<<std::endl; 
        ss.str("");
    }
}
void Backtrace(std::vector<std::string>& bt,int size,int skip){
    // void 不占用字节，void*万能指针，大小为4个字节 
    void** buffer = (void**)malloc(sizeof (void*) * size);
    size_t s = backtrace(buffer,size);
    char** strings = backtrace_symbols(buffer,size);
    if(strings == NULL){
        SYLAR_LOG_ERROR(g_log_system) << "backtrace_symbols error";
        return ;
    }
    // 跳过头几层 ？ 不是很懂头几层是什么？
    for(size_t i=skip;i<s;i++){
        // 没有经过名字改编
        bt.push_back(strings[i]);
        // bt.push_back(demangle(strings[i]));
    }
    // malloc 需要手动释放
    free(buffer);
    free(strings);   
}

const std::string BacktraceToString(int size,int skip,const std::string& prefix){
    std::vector<std::string> bt;
    Backtrace(bt,size,skip);
    std::stringstream ss;
    for(size_t i=0;i<bt.size();i++){
        ss << prefix << bt[i] << std::endl;
    }
    return ss.str();
}
/**
 * @brief: 
 * @description: 获取当前时间
 * @return {*}
 */
uint64_t GetCurrentMs(){
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec * 1000ul + tv.tv_usec / 1000;
}

uint64_t GetCurrentUs(){
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec * 1000 * 1000ul + tv.tv_usec;
}
}
