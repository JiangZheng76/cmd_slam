/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-02-25 09:42:35
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-15 17:53:38
 * @FilePath: /mysylar/mysylar/time.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "timer.h"

namespace mysylar{

static Logger::ptr g_Logger_sys = SYLAR_LOG_ROOT();
/**
 * @brief: 需要得到一个升序的 所以是 lhs < rhs = > trues
 * @param {ptr&} lhs
 * @param {ptr &} rhs
 * @description: 
 * @return {*}
 */
bool Timer::Comparator::operator() (const Timer::ptr& lhs,const Timer::ptr & rhs){
    if(!lhs && !rhs){
        return false;
    }
    if(!lhs){
        return true;
    }
    if(!rhs){
        return false;
    }
    if(lhs->m_next < rhs->m_next ){
        return true;
    }
    if(lhs->m_next > rhs->m_next){
        return false;
    }
    // 相同的时候比较地址
    return lhs.get() < rhs.get();
}
Timer::Timer(uint64_t ms,std::function<void()> cb,
    bool recurring, TimerManager* manager)
    :m_ms(ms)
    ,m_cb(cb)
    ,m_recurring(recurring)
    ,m_manager(manager){
    m_next = GetCurrentMs() + m_ms;
}
Timer::Timer(uint64_t now_ms){
    m_next = now_ms;
}
/**
 * @brief: 
 * @description: 删除一个计时器
 * @return {*}
 */
bool Timer::cancel(){
    TimerManager::RWMutexType::WriteLock lock(m_manager->m_mutex);
    if(m_cb){
        m_cb = nullptr;
        auto it = m_manager->m_timers.find(shared_from_this());
        m_manager->m_timers.erase(it);
        return true;
    }
    return false;
}
/**
 * @brief: 
 * @description: 刷新一个计时器
 * @return {*}
 */
bool Timer::refresh(){
    TimerManager::RWMutexType::WriteLock lock(m_manager->m_mutex);
    if(!m_cb){
        return false;
    }
    auto it = m_manager->m_timers.find(shared_from_this());
    if(it == m_manager->m_timers.end()){
        return false;
    }
    m_manager->m_timers.erase(it);
    m_next = GetCurrentMs() + m_ms;
    m_manager->m_timers.insert(shared_from_this());
    return true;
}
/**
 * @brief: 
 * @param {uint64_t} ms
 * @param {bool} from_now 启示时间从现在开始
 * @description: 
 * @return {*}
 */
bool Timer::reset(uint64_t ms ,bool from_now){
    TimerManager::RWMutexType::WriteLock lock(m_manager->m_mutex);
    if(!m_cb){
        return false;
    }
    if(m_ms == ms && !from_now){
        return true;
    }
    auto it = m_manager->m_timers.find(shared_from_this());
    if(it == m_manager->m_timers.end()){
        return false;
    }
    m_manager->m_timers.erase(it);
    uint64_t start = 0;
    if(from_now){
        start = GetCurrentMs();
    }else {
        start = m_next - m_ms;
    }
    m_ms = ms;
    m_next = start + m_ms;
    // 因为修改了时间之后，有可能去到了第一的位置，所以需要用这个接口来修改TimerManger最早开始时间
    m_manager->addTimer(shared_from_this(),lock);
    return true;
}

TimerManager::TimerManager(){

}
TimerManager::~TimerManager(){
    
}
/**
 * @brief 添加一个计时器 
 * @param {uint64_t} ms
 * @description: 
 * @return {*}
 */
Timer::ptr TimerManager::addTimer(uint64_t ms, std::function<void()> cb
    ,bool recurring){
    Timer::ptr timer(new Timer(ms,cb,recurring,this));
    RWMutexType::WriteLock lock(m_mutex);
    addTimer(timer,lock);
    return timer;
}
Timer::ptr TimerManager::addTimer(Timer::ptr val,RWMutexType::WriteLock& lock){
    auto it = m_timers.insert(val).first;
    // 查看插入之后是否在最前面
    bool at_front = (it == m_timers.begin()) && !m_tickle;
    lock.unlock();
    if(at_front){
        // 在最前面的话使用 tickle来修改
        // 因为 tickle 可以直接唤醒 epoll_wait 阻塞的线程，让他重新执行一次循环，更新等待时间
        m_tickle = true;
    }
    // 插入到了最开始的位置，说明在iomanager中设置的下一个活动执行时间发生了改变，
    // 所以需要唤醒一下iomanager，修改下一个活动执行时间
    if(at_front){
        // 只是封装了一个 tickle()
        onTimerInsertedAtFront();
    }
    return val;
}
/**
 * @brief 检查条件测存活，来决定是否执行函数 
 * @param {weak_ptr<void>} weak_ptr
 * @param {function<void()>} cb
 * @description: 
 * @return {*}
 */
static void OnTimer(std::weak_ptr<void> weak_ptr, std::function<void()> cb){
    std::shared_ptr<void> tmp = weak_ptr.lock();
    if(tmp){
        cb();
    }else{
        SYLAR_LOG_DEBUG(g_Logger_sys) << "OnTimer weak_ptr have been release.";
    }
}
/**
 * @brief: 
 * @param {uint64_t} ms
 * @param {function<void()>} cb
 * @param {weak_ptr<void>} weak_cond 只能指针做条件，智能指针不存在了，就不需要做这个任务了
 * @param {bool} recurring
 * @description: 
 * @return {*}
 */
Timer::ptr TimerManager::addConditionTimer(uint64_t ms, std::function<void()> cb
    ,std::weak_ptr<void> weak_cond, bool recurring){
    // bind重新封装成一个函数来执行
    return addTimer(ms,std::bind(&OnTimer,weak_cond,cb),recurring);
}  
/**
 * @brief: 返回执行下一个timer 剩余的执行时间
 * @description: 
 * @return {*}
 */
uint64_t TimerManager::getNextTimer(){
    RWMutexType::ReadLock lock(m_mutex);
    // 获取了最新的时间了，如果后面再有的需要修改的，就要改变了
    m_tickle = false;
    if(m_timers.empty()){
        // 返回一个最大的数 0的取反 
        return ~0ull;
    }
    Timer::ptr next = *m_timers.begin();
    // 如果在当前时间前就应该执行的话，就返回0立刻执行
    uint64_t now_ms = GetCurrentMs();
    if(next->m_next <= now_ms){
        return 0;
    }else{
        return next->m_next - now_ms;
    }
}
void TimerManager::listExpiredCbs(std::vector<std::function<void()>>& cbs){
    uint64_t now_ms = GetCurrentMs();
    std::vector<Timer::ptr> expired;
    {
        RWMutexType::ReadLock lock(m_mutex);
        if(m_timers.empty()){
            return;
        }
    }
    RWMutexType::WriteLock lock(m_mutex);
    Timer::ptr now_timer(new Timer(now_ms));
    auto it = m_timers.lower_bound(now_timer);
    // 找到第一个大于now_timer的timer
    while(it != m_timers.end() && (*it)->m_next == now_ms){
        ++it;
    }
    // 将超时的都拿出来
    expired.insert(expired.begin(), m_timers.begin(),it);
    // 原队列中清除
    m_timers.erase(m_timers.begin(),it);
    cbs.reserve(expired.size());
    for(auto& timer : expired){
        cbs.push_back(timer->m_cb);
        // 如果是循环执行的话，重新放入到计时器中
        if(timer->m_recurring){
            timer->m_next = now_ms + timer->m_ms;
            m_timers.insert(timer);
        }else {
            timer->m_cb = nullptr;
            }
    }
    return;
}
bool TimerManager::hasTimer(){
    RWMutexType::ReadLock lock(m_mutex);
    return !m_timers.empty();
}

}