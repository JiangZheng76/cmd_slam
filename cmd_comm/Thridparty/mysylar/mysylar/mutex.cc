/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-06-14 10:35:01
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-14 16:06:53
 * @FilePath: /mysylar/mysylar/mutex.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "mutex.hpp"

namespace mysylar{
/**
 * @brief RAII 封装信号量构造函数
 * @param {uint32_t} num
 * @description: 
 * @return {*}
 */
Semaphore::Semaphore(uint32_t num ){
    // 0 表示进程内共享
    // 1 表示跨进程共享
    if(sem_init(&m_semaphore,0,num)){
        throw std::logic_error("sem_init error.");
    }
}
/**
 * @brief RAII 思想释放信号量
 * @description: 
 * @return {*}
 */
Semaphore::~Semaphore(){
    sem_destroy(&m_semaphore);
}
/**
 * @brief 信号量-1 信号量会先-1，然后再查看值 <0 的话就阻塞，一直到信号量变回为 0，表示有人释放了notify 了一个信号量，就唤醒
 * @description: 
 * @return {*}
 */
void Semaphore::wait(){
    if(sem_wait(&m_semaphore)){
        throw std::logic_error("sem_wait error");
    }
}
/**
 * @brief 信号量+1 释放一个信号量 
 * @description: 
 * @return {*}
 */
void Semaphore::notify(){
    if(sem_post(&m_semaphore)){
        throw std::logic_error("sem_post error");
    }
}

}