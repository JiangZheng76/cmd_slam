/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-06-27 11:06:06
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-02-24 11:50:59
 * @FilePath: /mysylar/mysylar/scheduler.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __MYSYLAR_SCHEDULER_HPP__
#define __MYSYLAR_SCHEDULER_HPP__
#include <memory>
#include "thread.h"
#include "fiber.h"
#include "mutex.h"
#include <vector>
namespace mysylar{

class Scheduler{
public:
    typedef std::shared_ptr<Scheduler> ptr;
    typedef Mutex MutexType;
    /// @brief 
    /// @param thread 调度线程号
    /// @param use_caller 是否当前调用线程,是否将当前创建该调度器的线程放到调度器里面去，
        // 如果不放作为一个单独管理调度器的线程使用，如果放的话，需要用一个单独的协程来存放这个调度器
    /// @param name 调度器名称
    Scheduler(size_t thread = 1,bool use_caller = true,const std::string& name="");

    /// @brief 虚函数 这是个基类需要后面优化
    virtual ~Scheduler();

    template<class FiberOrCb>
    void schedule(FiberOrCb fc,int thread = -1){
        bool need_tickle = false;
        {
            MutexType::Lock lock(m_mutex);
            need_tickle = schedulerNoLock(fc,thread);
        }
        if(need_tickle){
            tickle();
        }
    }
    /// @brief 批量协程调度，附属的线程默认为-1，批量放入的认识在什么时候处理？？？？从现在来看好像是-1的都不进行处理
    /// @tparam InputIterator 
    /// @param fcs 
    /// @param thread 
    template<class InputIterator>
    void schedule(InputIterator begin,InputIterator end){
        bool need_tickle = false;
        {
            MutexType::Lock lock(m_mutex);
            while(begin!=end){
                need_tickle = schedulerNoLock(&*begin,-1) || need_tickle;
                ++begin;
            }
        }
        if(need_tickle){
            tickle();
        }
    }

    const std::string& getName() const {return m_name; }

    /// @brief 获取调度器本身
    /// @return 
    static Scheduler* GetThis();

    /// @brief 不懂？？？？？
    /// @return 
    static Fiber* GetMainFiber();

    void setThis();

    void start();

    void run();
    
    void stop();

    
protected: // 子类定义的函数
    virtual void tickle();
    virtual void idle();
    virtual bool stopping();
private:
    /// @brief 放协程或函数进去协程队列中，如果原来是空的，就返回true
    /// @tparam FiberOrCb 
    /// @param fc 
    /// @param thread 
    /// @return 
    template<class FiberOrCb>
    bool schedulerNoLock(FiberOrCb fc,int thread = -1){
        bool need_tickle = m_fibers.empty();
        FiberAndThread ft(fc,thread);
        if(ft.fiber || ft.cb){
            m_fibers.push_back(ft);
        }
        return need_tickle;
    }
// fiber 或 是 cb实体，选择性包含thread id指定执行的线程。
struct FiberAndThread{
    Fiber::ptr fiber;
    std::function<void()>cb;
    int  thread;
    FiberAndThread(Fiber::ptr fb,size_t thr)
        :fiber(fb),thread(thr){
    }

    FiberAndThread(Fiber::ptr * fb,size_t thr)
        :thread(thr){
        fiber.swap(*fb);
    }
    FiberAndThread(std::function<void()> f,size_t thr)
        :cb(f),thread(thr){

    }
    FiberAndThread(std::function<void()>* f,size_t thr)
        :thread(thr){
        cb.swap(*f);
    }
    FiberAndThread():thread(-1){}

    void reset(){
        thread = -1;
        fiber = nullptr;
        cb = nullptr;
    }
};


protected:
    /// @brief 线程池
    std::vector<Thread::ptr> m_threads;
    /// @brief 任务队列 
    std::list<FiberAndThread> m_fibers; 
    MutexType m_mutex;
    /// @brief 可能是fiber或者是callback
    std::string m_name;
    Fiber::ptr m_rootFiber; // 调度器绑定的主携程
    int m_rootThreadId; // 创建本调度器的线程id, == -1表示不在线程池中

protected:
    std::vector<int> m_threadIds;
    std::atomic<size_t> m_threadCount = {0}; 
    std::atomic<size_t> m_idleThreadCount = {0};
    /// @brief 表示活跃（正在处理任务）的线程数
    std::atomic<size_t> m_activeThreadCount = {0};
    bool m_stopping = true;
    /// @brief 停止调度器之后剩余线程自动停止
    bool m_autoStop = false;
};

}



#endif