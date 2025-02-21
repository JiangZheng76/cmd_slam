/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-06-13 10:49:13
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-14 20:41:03
 * @FilePath: /mysylar/mysylar/thread.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __MYSYLAR_THREAD_HH__
#define __MYSYLAR_THREAD_HH__
#include <thread>
#include <pthread.h>
#include <functional>
#include <vector>
#include <memory>
#include <sys/types.h>
#include <semaphore.h>
#include <mutex.h>
#include "config.h"
#include "log.h"

namespace mysylar{

class Thread : public Noncopyable{
public:
    typedef std::shared_ptr<Thread> ptr;

    Thread(std::function<void()> cb,const std::string& name);
    ~Thread();
    
    void join();

    static Thread* GetThis();
    static void SetName(const std::string& name);
    static const std::string& GetName();

    const std::string& getName(){ return m_name; }
    pid_t getId(){ return m_id; }

    void detach();

private:
    Thread(const Thread& ) = delete;
    Thread(const Thread && ) = delete;
    Thread operator=(const Thread& ) = delete;

    static void* run(void* argc);
private:
    // 线程id
    pid_t m_id = -1;
    // 线程标识
    pthread_t m_thread = 0;
    std::function<void()> m_cb;
    std::string m_name;
    Semaphore m_semaphore;
};

}

#endif