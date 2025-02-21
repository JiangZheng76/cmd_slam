/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-02-23 16:22:07
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-19 12:24:41
 * @FilePath: /mysylar/mysylar/iomanager.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SYLAR_IOMANAGER_H__
#define __SYLAR_IOMANAGER_H__

#include "sylar.h"
#include "scheduler.h"
#include "timer.h"
#include <sys/epoll.h>

namespace mysylar{

class IOManager : public Scheduler ,public TimerManager{
public :
    typedef std::shared_ptr<IOManager> ptr;
    typedef RWMutex RWMutexType;

    enum Event{
        NONE = 0x000,
        READ = EPOLLIN, // EPOLLIN
        WRITE = EPOLLOUT // EPOLLOUT
    };
private:
    struct FdContext {
        struct EventContext{
            Scheduler* scheduler;       // 带执行的scheduler ,可能会有很多个调度器
            Fiber::ptr fiber;           // 事件协程
            std::function<void()> cb;   // 事件的回调函数

            
        };
        EventContext& getEventContext(Event event);     // 返回事件的执行主体
        void resetContext(EventContext& ctx);
        void triggerEvent(Event event);
        int fd;                 // 事件fd
        EventContext read;      // 读事件
        EventContext write;     // 写事件
        Event events = NONE;   // 已经注册的事件
        MutexType mutex;        
    };
public:
    IOManager(size_t threads = 1, bool use_caller = true, const std::string name = "");
    ~IOManager();

    // 0 success ,-1 fail
    int addEvent(int fd ,Event event, std::function<void()> cb = nullptr);
    bool delEvent(int fd,Event event);
    // 取消掉某个监听
    bool cancelEvent(int fd,Event event);
    // 将所有的监听都取消掉
    bool cancelAllEvent(int fd);
    bool hasIdleThreads() {return m_idleThreadCount > 0;}

    
    static IOManager *GetThis();

protected:
    void tickle() override;
    void idle() override;
    bool stopping() override;
    bool stopping(uint64_t& timeout);
    void onTimerInsertedAtFront() override;
    void contextResize(size_t size);

private:
    int m_epfd = 0;
    int m_tickleFds[2]; // 这是一个提醒有任务的pipe,0是读的，1是写的
    
    // 监听中的任务数,一个fd可能为系统添加多个event，read 和 write
    std::atomic<size_t> m_pendingEventCount = {0};
    RWMutexType m_mutex;
    // 管理的所有fd
    std::vector<FdContext* > m_fdContexts;
    

};
}


#endif
