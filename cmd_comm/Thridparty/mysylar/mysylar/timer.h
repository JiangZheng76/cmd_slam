/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-02-25 09:42:29
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-15 17:38:45
 * @FilePath: /mysylar/mysylar/timer.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SYLAR_TIMER_H__
#define __SYLAR_TIMER_H__

#include <memory>
#include "thread.h"
#include "sylar.h"

namespace mysylar{
    
class TimerManager;
// ？？？为什么计时器没有fiber类型呢？
class Timer : public std::enable_shared_from_this<Timer>{
friend class TimerManager;
public:
    typedef std::shared_ptr<Timer> ptr;
private:
    // 设计成不会自动创建，只会通过Timermanager来创建管理
    Timer(uint64_t ms,std::function<void()> cb,
        bool recurring, TimerManager* manager);
    Timer(uint64_t now_ms);
public:
    bool cancel();
    bool refresh();
    bool reset(uint64_t ms ,bool from_now);
private:
    uint64_t m_ms = 0;          // 执行周期
    uint64_t m_next = 0;        // 下一个执行周期精确的时间
    std::function<void()> m_cb; // 任务函数
    bool m_recurring = false;   // 是否循环
    TimerManager* m_manager;    // 所属Manager 指针

private:
    struct Comparator {
        bool operator() (const Timer::ptr& lhs,const Timer::ptr& rhs);
    };
};

class TimerManager{

friend class Timer;
public:
    typedef RWMutex RWMutexType;
public:
    TimerManager();
    // 留给iomanager继承用的
    virtual ~TimerManager();

    Timer::ptr addTimer(uint64_t ms, std::function<void()> cb
        ,bool recurring = false);
    Timer::ptr addTimer(Timer::ptr val,RWMutexType::WriteLock& lock);
    
    Timer::ptr addConditionTimer(uint64_t ms, std::function<void()> cb,
        std::weak_ptr<void> weak_cond, bool recurring = false);  // 只能指针做条件，智能指针不存在了，就不需要做这个任务了

    // 给iomanager修改下一次执行时间用的
    virtual void onTimerInsertedAtFront() = 0;

    uint64_t getNextTimer();
    bool hasTimer();

    void listExpiredCbs(std::vector<std::function<void()>>& cbs);

private:
    RWMutexType m_mutex;                            // 时间队列的修改锁，因为涉及读和写，使用读写锁
    std::set<Timer::ptr,Timer::Comparator> m_timers;// 时间队列
    bool m_tickle = false;                          //  防止频繁tickle修改nextimer，如果多个出发，只需要一次tickle就足够了
};

}



#endif