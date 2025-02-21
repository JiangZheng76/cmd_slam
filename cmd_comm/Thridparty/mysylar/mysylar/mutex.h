/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-06-14 10:34:55
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-14 16:59:08
 * @FilePath: /mysylar/mysylar/mutex.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SYLAR_MUTEX_HPP__
#define __SYLAR_MUTEX_HPP__
#include <semaphore.h>
#include <pthread.h>
#include <stdint.h>
#include <stdexcept>
#include <atomic>
#include "noncopyable.h"

namespace mysylar{

class Semaphore : public Noncopyable{
public:
    Semaphore(uint32_t num = 0);
    ~Semaphore();

    void wait();
    void notify();
    
private:
    // 只有一种构造方式，将其他的都删除
    // 因为定义了构造函数，所以不会有默认构造函数
    Semaphore(Semaphore& ) = delete;
    Semaphore(Semaphore&& ) = delete;
    Semaphore operator=(Semaphore& ) = delete;
private:
    sem_t m_semaphore;
};

// RAII思想，将mutex封装起来
// 创建一个锁类来方式忘记解锁
// 使用一个模板类来封装将 RAII 封装好的pthread 锁对象，这样就不会忘记析构解锁
// 初始化加锁，析构解锁
template<class T>
struct ScopedLockImpl : public Noncopyable{
public:
    /**
     * @brief 使用引用的方式因为是同一个锁，创建上锁
     * @param {T&} mutex
     * @description: 
     * @return {*}
     */
    ScopedLockImpl(T& mutex):m_mutex(mutex){
        lock();
    }
    /**
     * @brief 析构解锁
     * @description: 
     * @return {*}
     */    
    ~ScopedLockImpl(){
        unlock();
    }

    void lock(){
        if(!m_locked){
            m_mutex.lock();
            m_locked = true;
        }
    }

    void unlock(){
        if(m_locked){
            m_locked = false;
            m_mutex.unlock();
        }
    }
private:
    T& m_mutex;
    bool m_locked = false;
};

// 读锁模版
template<class T>
struct ReadScopedLockImpl : public Noncopyable{
public:
    ReadScopedLockImpl(T& mutex):m_mutex(mutex){
        m_mutex.rdlock();
        m_locked = true;
    }
    ~ReadScopedLockImpl(){
        unlock();
    }

    void lock(){
        if(!m_locked){
            m_mutex.rdlock();
            m_locked = true;
        }
    }

    void unlock(){
        if(m_locked){
            m_locked = false;
            m_mutex.unlock();
        }
    }
private:
    T& m_mutex;
    bool m_locked;
};

// 写锁模板
template<class T>
struct WriteScopedLockImpl : public Noncopyable{
public:
    WriteScopedLockImpl(T& mutex):m_mutex(mutex){
        m_mutex.wrlock();
        m_locked = true;
    }
    ~WriteScopedLockImpl(){
        unlock();
    }

    void lock(){
        if(!m_locked){
            m_mutex.wrlock();
            m_locked = true;
        }
    }

    void unlock(){
        if(m_locked){
            m_locked = false;
            m_mutex.unlock();
        }
    }
private:
    T& m_mutex;
    bool m_locked;
};

// RAII 封装pthread读写锁
class RWMutex : public Noncopyable{
public:

    typedef ReadScopedLockImpl<RWMutex> ReadLock;
    typedef WriteScopedLockImpl<RWMutex> WriteLock;
    
    RWMutex() {
        pthread_rwlock_init(&m_lock,nullptr);
    }
    ~RWMutex() {
        pthread_rwlock_destroy(&m_lock);
    }
    void rdlock(){
        pthread_rwlock_rdlock(&m_lock);
    }
    void wrlock() {
        pthread_rwlock_wrlock(&m_lock);
    }
    void unlock() {
        pthread_rwlock_unlock(&m_lock);
    }
private:
    pthread_rwlock_t m_lock;

};
// 互斥锁
// 上锁的时候释放 cpu 进入到阻塞的状态，锁资源释放之后再进入到就绪状态等待 cpu 的调度
class Mutex: public Noncopyable{
public:
    /**
     * @brief 
     * @description: 
     * @return {*}
     */    
    typedef ScopedLockImpl<Mutex> Lock;
    Mutex() {
        pthread_mutex_init(&m_lock,nullptr);
    }
    ~Mutex(){
        pthread_mutex_destroy(&m_lock);
    }
    void lock() {
        pthread_mutex_lock(&m_lock);
    } 
    void unlock(){
        pthread_mutex_unlock(&m_lock);
    }
private:
    pthread_mutex_t m_lock;
};
// 自旋锁
// 自旋锁不会释放 cpu 资源，而是会使用 while 循环等待，或者是让 cpu 停止执行一会再询问，期间保持对 cpu 的占用
class SpinLock : public Noncopyable{
public:
    typedef ScopedLockImpl<SpinLock> Lock;
    SpinLock() {
        pthread_spin_init(&m_lock,0);
    }
    ~SpinLock() {
        pthread_spin_destroy(&m_lock);
    }
    void lock(){
        pthread_spin_lock(&m_lock);
    }
    void unlock(){
        pthread_spin_unlock(&m_lock);
    }

private:
    pthread_spinlock_t m_lock;
};


// CAS自旋锁 
// 这是一个乐观锁，乐观锁不会上锁，只会先拿资源，实现完之后再比较，查看资源是否可以使用
// CAS 只是一次操作，去拿资源，比较，可以实现返回 true，不可以实现返回 false；循环调用 CAS 就可以实现自旋锁的效果
// https://segmentfault.com/q/1010000021946513
class CASLock : public Noncopyable{
public:
    typedef ScopedLockImpl<CASLock> Lock;
    CASLock(){
        m_mutex.clear();
    }
    ~CASLock(){

    }
    void lock(){
        //  atomic_flag是一次原子操作
        // 如果之前 atmoic_flag 没有设置过就返回 false，并且将他修改
        // 如果之前 被设置过了就返回 true
        // 原子操作查询，实现 CAS 自旋锁
        // http://hxz.ink/2016/07/10/C++11-atomic-flag/
        while(std::atomic_flag_test_and_set_explicit(&m_mutex,std::memory_order_acquire));
    }

    void unlock(){
        // 清除 atomic_flag
        std::atomic_flag_clear_explicit(&m_mutex,std::memory_order_release);
    }

private:
    std::atomic_flag m_mutex;
};



class NullRWMutex : public Noncopyable{
public:
    typedef ReadScopedLockImpl<NullRWMutex>  ReadLock;
    typedef WriteScopedLockImpl<NullRWMutex>  WriteLock;
    typedef ScopedLockImpl<NullRWMutex> Lock;
    NullRWMutex(){}
    ~NullRWMutex(){}

    void rdlock(){}
    void wrlock(){}
    void lock(){}
    void unlock(){}

};

}
#endif
