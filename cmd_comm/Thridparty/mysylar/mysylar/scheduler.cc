/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-06-27 11:06:12
 * @LastEditors: Johnathan 2440877322@qq.com
 * @LastEditTime: 2024-08-14 00:54:38
 * @FilePath: /mysylar/mysylar/scheduler.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "log.h"
#include "scheduler.h"
#include "hook.h"
namespace mysylar{

static thread_local Scheduler * t_scheduler = nullptr; // 当前线程执行的调度器
static thread_local Fiber* t_scheduler_fiber = nullptr; // 调度器当前执行线程的执行协程（可以是任务执行协程，也可以是负责调度的主协程）

// static Logger::ptr g_logger_sys = SYLAR_LOG_ROOT();
static Logger::ptr g_logger_sys = SYLAR_LOG_NAME("system");
/// @brief 创建一个调度器，并且判断是否将当前的创建调度器的线程存放到线程池中去
/// @param thread 创建的线程数量
/// @param use_caller 是否将创建线程加入到线程池里面去
/// @param name 创建调度器线程名称
Scheduler::Scheduler(size_t threads,bool use_caller,const std::string& name)
    :m_name(name){
    SYLAR_ASSERT(threads > 0);
    
    // 如果将创建调度器的线程加入到线程池中
    if(use_caller){
        Fiber::GetThis(); // 创建当前线程的作为主携程
        --threads; // 这是什么操作？？？

        SYLAR_ASSERT(GetThis() == nullptr); // 防止创建了多个调度器
        t_scheduler = this;

        // 创建调度器自己的协程
        m_rootFiber.reset(new Fiber(std::bind(&Scheduler::run,this)));
        Thread::SetName(m_name);

        t_scheduler_fiber = m_rootFiber.get();
        m_rootThreadId = GetThreadId();
        m_threadIds.push_back(m_rootThreadId);
    }else {
        m_rootThreadId = -1;
    }
    m_threadCount= threads; // 线程号是叠加上去的
}
Scheduler::~Scheduler(){
    SYLAR_ASSERT(m_stopping);
    if(GetThis() == this){
        // 防止变成野指针，资源的回收系统会做
        t_scheduler = nullptr;
    }
}
Scheduler* Scheduler::GetThis(){ return t_scheduler; }
Fiber* Scheduler::GetMainFiber(){ return t_scheduler_fiber; }
void Scheduler::setThis() { t_scheduler = this; } 
/**
 * @brief 唤醒所有线程池的线程，并加入到调度器的调度中，每个线程都归一个调度器管理。
 * @description: 
 * @return {*}
 */
void Scheduler::start(){
    if(!m_stopping){
        return ;
    }
    MutexType::Lock lk(m_mutex);
    m_stopping = false;
    SYLAR_ASSERT(m_threads.empty());
    m_threads.resize(m_threadCount);
    for(size_t i = 0;i<m_threadCount;i++){
        m_threads[i].reset(new Thread(std::bind(&Scheduler::run,this)
            ,m_name + "_"+std::to_string(i)));
        m_threadIds.push_back(m_threads[i]->getId());
    }
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "\e[1;32m" << "+++ START +++"<< "\e[0m"; 
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "create " << m_threads.size() << " threads "; 
}
/**
 * @brief 停止调度器
 * @description: 判断是否还有线程或者说当前主Fiber是否正在运行，如果已经停止直接停止调度器
 *  如果没有停止的话，就将所有线程全部启动起来，然后接上join() 让其执行结束之后自动回收，再停止调度器
 * @return {*}
 */
void Scheduler::stop(){
    m_autoStop = true;              // 命令各个线程陆续停止
    if(m_rootFiber && 
        m_threadCount == 0 && 
        (m_rootFiber->getState() == Fiber::INIT ||
        m_rootFiber->getState() == Fiber::TERM)){
        SYLAR_LOG_INFO(g_logger_sys) << this << " stoped.";
        if(stopping()){             // 没有活跃的线程，同时任务队列都已经完成了，直接停止
            return ;
        }
    }
    // 没有停止成功，仍然有活跃的线程，或者是任务队列里面还有任务，需要手动提醒

    // 需要自动停止，直到所有线程执行起来
    if(m_rootThreadId != -1){
        // 意味着创建调度器的线程不在线程池中
        SYLAR_ASSERT(GetThis() == this);
    }else{
        // 意味着创建调度器的线程在线程池中「不懂？？？？？？」
        SYLAR_ASSERT(GetThis() != this);
    }

    m_stopping = true;

    // 不懂 tickle 为什么需要重新去遍历之后tickle？？？
    // 因为是边缘模式，tickle 之后只会唤醒一个线程
    // tickle m_threadCount次唤醒所有的线程去执行任务队列的任务
    // 如果没有任务，就让他们 auto_stoping跳出循环
    for(size_t i = 0;i < m_threadCount;i++){
        tickle();
    }
    // 不懂???????
    //这个 主携程只有在stop 的时候做收尾的工作，平时不干活
    if(m_rootFiber){
        tickle();
        if(!stopping()){
            // 主携程收尾完了回来，可以忽略这个设置，有问题
            m_rootFiber->call();
        }
    }

    std::vector<Thread::ptr >thrs;
    {
        // 对线程池操作需要加锁
        MutexType::Lock lk(m_mutex);
        thrs.swap(m_threads);
    }
    //  join 每一个线程
    for(auto &i: thrs){
        // 阻塞等待所有线程结束
        i->join();
        SYLAR_LOG_DEBUG(g_logger_sys) << "join.";     
    }
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "\e[1;32m" << "--- STOP ---"<< "\e[0m"; 

}
/**
 * @brief 每个线程使用调度器时执行的函数
 * @description: 
 * 1、执行当前线程的scheduler 
 * 2、设置当前执行线程的执行协程
 * 协程调取循环while
 *  查看任务队列里面是否有属于当前线程的任务，并执行
 *  没有任务进入idle函数，在idle——fiber 里面空转
 * @return {*}
 */
void Scheduler::run(){
    // 1、设置当前线程的调度器

    setThis();
    set_hook_enable(true); //  只有 scheduler 可以使用这个库
    // 2、 设置调度协程为主携程
    if(m_rootThreadId != GetThreadId()){
        // 设置主携程
        t_scheduler_fiber = Fiber::GetThis().get();
    }

    Fiber::ptr idle_fiber(new Fiber(std::bind(&Scheduler::idle,this),0,false,Fiber::IDLE));
    Fiber::ptr cb_fiber;

    // 每个线程都是使用同一个对象的话 函数内的资源是否共享的？
    FiberAndThread ft;
    // 3、循环从任务队列中获取可执行的任务
    // 执行停止就结束所有的thread
    while(true){
        ft.reset();
        bool tickle_me = false;
        bool is_active = false; // 找到任务
        {
            // 多个线程之间互斥访问任务队列
            MutexType::Lock lock(m_mutex);
            auto it = m_fibers.begin();
            while(it != m_fibers.end()){
                // 只有找到属于当前执行线程或是无指定线程的任务时候才会进入下一步处理
                if(it->thread != -1 && it->thread != GetThreadId()){
                    // 如果这个任务不是交给当前线程去处理的就跳过不处理
                    ++it;
                    // 对于存在非当前执行线程的任务 利用tickle_me来提醒其他执行线程来处理
                    tickle_me = true;
                    continue;
                }
                SYLAR_ASSERT(it->fiber || it->cb);
                // 对于执行中的任务暂不处理，等待执行完成
                if(it->fiber && it->fiber->getState() == Fiber::EXEC){
                    it++ ;
                    continue;
                }
                // 从队列中提取此任务，并跳出寻找任务的循环，执行此任务
                ft = *it;
                m_fibers.erase(it);
                is_active = true;
                m_activeThreadCount++;
                break;
            }
            // 队列中还有任务，提醒其他执行线程检查。
            tickle_me |= it!=m_fibers.end();
        }
        // 通知下其他线程 队列里面还有任务
        if(tickle_me)tickle();
        // 启动fiber 
        if(ft.fiber && ft.fiber->getState() != Fiber::TERM
            && ft.fiber->getState() != Fiber::EXCEPT){
            ft.fiber->swapIn();
            m_activeThreadCount--;
            if(ft.fiber->getState() == Fiber::READY){
                // 还是在READY状态，还没有开始执行的重新回到队列中等待下一次的执行
                schedule(ft.fiber);
            }else if(ft.fiber->getState() != Fiber::TERM
                && ft.fiber->getState() != Fiber::EXCEPT){
                // 不为执行结束也不为出现异常，就会标志任务被搁置了，可能在等待IO相应等操作
                // 回到这里表示让出了执行的时间，状态需要变回hold
                ft.fiber->setState(Fiber::HOLD);
            }
            // 不是很懂，HOLD的状态也认为任务被执行完毕了么？？？不是还没有完全结束么？？？
            // 对于事件任务，因为hook 模块中重写了，所以还继续在 epoll 中注册了，所以可以不需要重新挂载回去。
            
            // fiber 没有人指向它了就会自动析构
            ft.reset();
        }else if(ft.cb){
            // 启动 cb
            if(cb_fiber){
                cb_fiber->reset(ft.cb);
            }else {
                cb_fiber.reset(new Fiber(ft.cb));
            }
            cb_fiber->swapIn();
            m_activeThreadCount--;
            if(cb_fiber->getState() == Fiber::READY){
                schedule(cb_fiber);
                cb_fiber.reset();
            }else if(cb_fiber->getState() == Fiber::TERM 
                || cb_fiber->getState() == Fiber::EXCEPT){
                // 执行结束了
                // 出现了异常 直接释放掉这个协程的执行函数
                cb_fiber->reset(nullptr);
            }else {
                // 其他情况执行还没有结束的话，就设为HOLD？？？不懂，为什么不需要放回去重新调度
                cb_fiber->setState(Fiber::HOLD);
                // 是
                cb_fiber.reset();
            }
            ft.reset();
        }else {
            
            if(is_active){ // 表示有任务
                // 没有任务执行的时候才会去执行idle
                --m_activeThreadCount;
                continue;
            }
            // 判断是否有命令要求停止
            if(idle_fiber->getState() == Fiber::TERM){
                SYLAR_LOG_INFO(g_logger_sys) << "idle fiber term! ";
                break;
            }

            ++m_idleThreadCount;
            idle_fiber->swapIn();
            --m_idleThreadCount;

            if(idle_fiber->getState() != Fiber::TERM && 
                idle_fiber->getState() != Fiber::EXCEPT){
                idle_fiber->setState(Fiber::HOLD);
                SYLAR_LOG_DEBUG(g_logger_sys)  << "idle fiber state=" << idle_fiber->getState();
            }else {
                SYLAR_LOG_DEBUG(g_logger_sys)  << "idle fiber is term or except state=" << idle_fiber->getState();
            }
        }
    }
}
void Scheduler::tickle(){
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT() ) << "tickle";
}
void Scheduler::idle(){
    
    while(!m_stopping){
        SYLAR_LOG_INFO(SYLAR_LOG_ROOT() ) << "idle ";
        usleep(100);
        Fiber::YieldToHold();
    }
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "end idle ";
}
bool Scheduler::stopping(){ 
    MutexType::Lock lock(m_mutex);
    // 只有命令停止，没有任务，没有正在活动线程时才可以停止。
    return m_autoStop && m_stopping
        && m_fibers.empty() && m_activeThreadCount == 0;
    }

}
