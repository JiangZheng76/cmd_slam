/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-02-23 16:22:12
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-17 18:00:32
 * @FilePath: /mysylar/mysylar/iomanager.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "iomanager.hpp"
#include "macro.hpp"
#include <sys/epoll.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
namespace mysylar {
    
// static Logger::ptr g_logger_sys = SYLAR_LOG_ROOT();
static Logger::ptr g_logger_sys = SYLAR_LOG_NAME("system");

IOManager::FdContext::EventContext& IOManager::FdContext::getEventContext(Event event){
    switch(event){
        case READ:
            return read;
        case WRITE:
            return write;
        default : 
            SYLAR_ASSERT2(false,"getEventConetxt()");
    }
    return write;
}
/**
 * @brief 清空scheduler fiber 和 cb
 * @param {EventContext&} ctx
 * @description: 
 * @return {*}
 */
void IOManager::FdContext::resetContext(EventContext& ctx){
    ctx.scheduler = nullptr;
    ctx.fiber.reset();
    ctx.cb = nullptr;
}
/**
 * @brief: 触发这个任务，然后清除这个event任务
 * @param {Event} event
 * @description: 
 * @return {*}
 */
void IOManager::FdContext::triggerEvent(Event event){
    SYLAR_ASSERT(events & event);
    events = (Event)(events & ~event);
    EventContext &ctx = getEventContext(event);
    if(ctx.cb){
        ctx.scheduler->schedule(ctx.cb);
    }else if(ctx.fiber) {
        ctx.scheduler->schedule(ctx.fiber);
    }
    ctx.scheduler = nullptr;
    return ;
}

/**
 * @brief 初始化调度器，将 tickle 的管道读监听注册到 epoll 中
 * @param {size_t} threads
 * @param {bool} use_caller
 * @param {string} name
 * @description: 
 * @return {*}
 */
IOManager::IOManager(size_t threads, bool use_caller, const std::string name)
    :Scheduler(threads,use_caller,name){
    // 创建 epoll，上限是 5000 个 fd
    m_epfd = epoll_create(5000);            
    SYLAR_ASSERT(m_epfd > 0);
    // 提醒还有任务的管道
    int rt = pipe(m_tickleFds);
    SYLAR_ASSERT(!rt);

    // 注册 tickle 管道到 epoll 中
    epoll_event event;
    memset(&event,0,sizeof(epoll_event));
    event.events = EPOLLIN | EPOLLET;

    event.data.fd = m_tickleFds[0];
    // 设置成非堵塞
    // 所有的读都不阻塞的，将等待执行放到后台
    rt = fcntl(m_tickleFds[0],F_SETFL,O_NONBLOCK);
    SYLAR_ASSERT(!rt);

    // epoll 中添加管道的可读监听
    rt = epoll_ctl(m_epfd,EPOLL_CTL_ADD,m_tickleFds[0],&event);
    SYLAR_ASSERT(!rt);

    contextResize(64);
    // 创建线程池，开始调度
    start();
}
/**
 * @brief: 关闭句柄
 * @description: 
 * @return {*}
 */
IOManager::~IOManager(){
    SYLAR_LOG_INFO(g_logger_sys) << "~IOManager.";
    stop();
    close(m_epfd);
    close(m_tickleFds[0]);
    close(m_tickleFds[1]);

    for(size_t i =0 ;i< m_fdContexts.size(); ++i){
        if(m_fdContexts[i]){
            delete m_fdContexts[i];
        }
    }
}
/**
 * @brief  时间换空间 一次性分配多个，下一次就不需要重新加上写锁，可以节省一点时间
 * @param {size_t} size
 * @description: 
 * @return {*}
 */
void IOManager::contextResize(size_t size){
    m_fdContexts.resize(size);
    for(size_t i =0;i<size;++i){
        if(!m_fdContexts[i]){
            m_fdContexts[i] = new FdContext();
            m_fdContexts[i]->fd = i;
        }
    }
}
/**
 * @brief 添加 fd 的事件监听
 * @param {int} fd
 * @param {Event} event
 * @param {function<void()>} cb 如果 cb 为 0，那么就绑定当前这个 fiber的环境给当前的 event_ctx
 * @description: 
 * 1、申请 fdctx
 * 2、epoll 注册监听的事件，并在event 中挂上 event_ctx，方便响应时候调用处理函数
 * 3、给事件挂载上调度器的指针和响应事件的执行函数
 * @return {*} 1 success,0 retry ,-1 fail
 */
int IOManager::addEvent(int fd ,Event event, std::function<void()> cb){
    FdContext* fd_ctx = nullptr;
    // 判断 fd 的空间是否足够
    // 1、申请 fd
    RWMutexType::ReadLock lock(m_mutex);
    if(m_fdContexts.size() > (size_t)fd){
        fd_ctx = m_fdContexts[fd];
    }else {
        lock.unlock();
        RWMutexType::WriteLock lock2(m_mutex);
        // 写锁一次，创建多个
        contextResize(m_fdContexts.size() * 1.5);
        fd_ctx = m_fdContexts[fd];
    }

    // 对 fd 资源的操作上锁，防止其他线程也介入
    MutexType::Mutex::Lock lock2(fd_ctx->mutex);
    // 如果需要注册的event 都已经存在的话，说明有两个不同的线程在操作同一个句柄
    // 因为 m_fdContexts 在close fd的时候会清空内容
    if(fd_ctx->events & event) { // 1 & 1 = 1 ； 0 & 1 = 0
        SYLAR_LOG_ERROR(g_logger_sys) << "addEvent assert fd=" << fd
            << "event=" << event 
            << "fd_ctx.event=" << fd_ctx->events;
        SYLAR_ASSERT(fd_ctx->events & event);
        
    }
    // 对本 fd 的操作
    int op = fd_ctx->events ? EPOLL_CTL_MOD : EPOLL_CTL_ADD;
    epoll_event epevent;
    // 2、epoll 注册监听的事件，并在event 中挂上 event_ctx，方便响应时候调用处理函数
    epevent.events = EPOLLET | fd_ctx->events | event;
    // 可以用户自定义数据的数据
    epevent.data.ptr = fd_ctx;
    
    // epoll注册新事件
    int rt = epoll_ctl(m_epfd,op,fd,&epevent);
    if(rt){
        SYLAR_LOG_ERROR(g_logger_sys) << "epoll_ctl("<< m_epfd <<"," <<op << ","
            <<  fd << "," << epevent.events << "):"
            << rt << "(" << errno << ") (" << strerror(errno) << ")";
        return -1;
    }
    ++m_pendingEventCount;
    // 同步 FdContext 和 epoll 的事件类型
    fd_ctx->events = (Event)(fd_ctx->events | event);

    // 3、给事件挂载上调度器的指针和响应事件的执行函数
    FdContext::EventContext& event_ctx = fd_ctx->getEventContext(event);
    // 保证新注册的事件是全新的
    SYLAR_ASSERT(!event_ctx.scheduler 
        && (!event_ctx.fiber 
        || !event_ctx.cb));
    event_ctx.scheduler = GetThis();
    if(cb){
        event_ctx.cb.swap(cb);
    }else {
        // 如果这个协程在执行添加读，下一次进来就继续这个fiber
        // 调用addEvent的就是当前在执行的这个协程？？？为什么可以回来这个协程继续？
        event_ctx.fiber = Fiber::GetThis();
        SYLAR_ASSERT(event_ctx.fiber->getState() == Fiber::EXEC);
    }
    return  0;

}
/**
 * @brief 删除event 监听
 * @param {int} fd
 * @param {Event} event event 类型
 * @description: 
 * @return {*}
 */
bool IOManager::delEvent(int fd,Event event){
    RWMutex::ReadLock lock(m_mutex);
    if(m_fdContexts.size() <= (size_t)fd){
        return false;
    }
    FdContext* fd_ctx = m_fdContexts[fd];
    lock.unlock();

    MutexType::Lock lock2(fd_ctx->mutex);
    // 没有需要删除的event
    if(!(fd_ctx->events & event)){
        return false;
    }
    // 获取去掉event之后的new_event
    Event new_event = (Event)(fd_ctx->events & ~event);
    // 全部去掉的话就是删除，不是全部去掉就是重新设置
    int op = new_event ? EPOLL_CTL_MOD : EPOLL_CTL_DEL;
    epoll_event epevent;
    epevent.events = EPOLLET | new_event;
    epevent.data.ptr = fd_ctx;
    // ？？？怎么觉得这里的逻辑有问题？event如果和原来的一模一样的话就是空，那么epevent也是空，如果是空的话del什么？
    int rt = epoll_ctl(m_epfd,op,fd,&epevent);

    if(rt){
        SYLAR_LOG_ERROR(g_logger_sys) << "epoll_ctl("<< m_epfd <<"," <<op << ","
            <<  fd << "," << epevent.events << "):"
            << rt << "(" << errno << ") (" << strerror(errno) << ")";
        return false;
    }
    // 这里也有问题，删掉了event，可能并不是删掉所有的event呀，只是删除了其中一个事件？？？
    --m_pendingEventCount;

    fd_ctx->events = new_event;

    FdContext::EventContext& event_ctx = fd_ctx->getEventContext(event);
    fd_ctx->resetContext(event_ctx);
    return true;

}
/**
 * @brief 找到这个事件强制触发执行,然后取消监听，不是等轮到他再执行
 * @param {int} fd
 * @param {Event} event
 * @description: 取消掉某个监听
 * @return {*}
 */
bool IOManager::cancelEvent(int fd,Event event){
    RWMutex::ReadLock lock(m_mutex);
    if(m_fdContexts.size() <= (size_t)fd){
        return false;
    }
    FdContext* fd_ctx = m_fdContexts[fd];
    lock.unlock();

    MutexType::Lock lock2(fd_ctx->mutex);
    // 没有需要删除的event
    if(!(fd_ctx->events & event)){
        return false;
    }
    // 获取去掉event之后的new_event
    Event new_event = (Event)(fd_ctx->events & ~event);
    // 全部去掉的话就是删除，不是全部去掉就是重新设置
    int op = new_event ? EPOLL_CTL_MOD : EPOLL_CTL_DEL;
    epoll_event epevent;
    epevent.events = EPOLLET | new_event;
    epevent.data.ptr = fd_ctx;
    // ？？？怎么觉得这里的逻辑有问题？event如果和原来的一模一样的话就是空，那么epevent也是空，如果是空的话del什么？
    int rt = epoll_ctl(m_epfd,op,fd,&epevent);

    if(rt){
        SYLAR_LOG_ERROR(g_logger_sys) << "epoll_ctl("<< m_epfd <<"," <<op << ","
            <<  fd << "," << epevent.events << "):"
            << rt << "(" << errno << ") (" << strerror(errno) << ")";
        return false;
    }
    // 将删除的event触发
    fd_ctx->triggerEvent(event);
    // 这里也有问题，删掉了event，可能并不是删掉所有的event呀，只是删除了其中一个事件？？？
    --m_pendingEventCount;
    return true;

}
// 将所有的监听都取消掉
bool IOManager::cancelAllEvent(int fd){
    RWMutex::ReadLock lock(m_mutex);
    if(m_fdContexts.size() <= (size_t)fd){
        return false;
    }
    FdContext* fd_ctx = m_fdContexts[fd];
    lock.unlock();

    MutexType::Lock lock2(fd_ctx->mutex);
    // 没有event
    if(!(fd_ctx->events)){
        return false;
    }
    // 全部去掉的话就是删除，不是全部去掉就是重新设置
    int op = EPOLL_CTL_DEL;
    epoll_event epevent;
    epevent.events = 0;
    epevent.data.ptr = fd_ctx;
    int rt = epoll_ctl(m_epfd,op,fd,&epevent);
    if(rt){
        SYLAR_LOG_ERROR(g_logger_sys) << "epoll_ctl("<< m_epfd <<"," <<op << ","
            <<  fd << "," << epevent.events << "):"
            << rt << "(" << errno << ") (" << strerror(errno) << ")";
        return false;
    }

    // 将删除的event触发
    if(fd_ctx->events & READ){
        fd_ctx->triggerEvent(READ);
        --m_pendingEventCount;
    }
    if(fd_ctx->events & WRITE){
        fd_ctx->triggerEvent(WRITE);
        --m_pendingEventCount;
    }
    // 在 triggerEvent中会修改events，取消完之后理论上应该是0
    SYLAR_ASSERT(fd_ctx->events == 0);
    return true;
}
IOManager *IOManager::GetThis(){
    // ？？？ 这里可以将父类转化成子类么？
    // 没有什么问题的，dynamic_cast会进行检查，并且此程序里面不会直接调用 Scheduler对象，都是在 IOmanager 里面调用的。
    // 所以一般都可以转化成功。
    return dynamic_cast<IOManager*>(Scheduler::GetThis());
}
/**
 * @brief: 唤醒有新的任务了,只有有idle线程的时候才会唤醒
 * @description: 
 * @return {*}
 */
void IOManager::tickle(){
    if(!hasIdleThreads()){ return; }
    int rt  = write(m_tickleFds[1],"T",1);
    SYLAR_ASSERT(rt==1);
}
/**
 * @brief: 只有没有任务和计时器了才可以停止调度
 * @description: 
 * @return {*}
 */
bool IOManager::stopping(){
    uint64_t timeout = 0;
    return stopping(timeout);
}
/**
 * @brief: 获取next_timeout的同时，判断是否没有任务和计时器，是否可以停止
 * @param {uint64_t&} next_timeout
 * @description: 
 * @return {*}
 */
bool IOManager::stopping(uint64_t& next_timeout){
    next_timeout = getNextTimer();
    // 只有没有计时器和监听event，还有任务队列没有任务且没有活跃线程时候
    return next_timeout == ~0ull 
        && m_pendingEventCount == 0
        && Scheduler::stopping();
}
/**
 * @brief idle线程执行的主要函数
 * 1、判断是否结束，并获取计时器下一个执行时间
 * 2、利用 epoll_wait 的超时时间设置成下一个计时器执行时间，来实现定时
 * 3、获取超时的计时器，将他们放入到调度器中
 * 4、将 epoll_wait 响应的事件，设置监听项，并triggerEvent 放到调度器中
 * @description: 
 * @return {*}
 */
void IOManager::idle(){
    epoll_event* epevents = new epoll_event[64]();
    // 利用智能指针防止内存泄露
    std::shared_ptr<epoll_event> events_ptr(epevents,[](epoll_event* ptr){
        delete [] ptr;
    });

    while(true){
        // 1、判断是否结束且获取计时器下一个时间
        uint64_t next_timeout;
        if(stopping(next_timeout)){
            SYLAR_LOG_INFO(g_logger_sys) << "Iomanager name=" << m_name << "idle stopping and exist.";
            break;
        }
        int rt;
        do{
            // epoll 的等待是毫秒级的
            static const int MAX_TIMEOUT = 5000;
            if(next_timeout!= ~0ull){
                next_timeout = (int)next_timeout > MAX_TIMEOUT
                                ? MAX_TIMEOUT : next_timeout;
            }else {
                next_timeout = MAX_TIMEOUT;
            }
            // 返回实际长度，阻塞等待事件
            rt = epoll_wait(m_epfd,epevents,64,(int)next_timeout);
            // 一般时间到了之后，rt 返回0 ，所以只会等待一个next_timeout的时间跨度
            if(rt < 0 && errno == EINTR){
                // 时间结束，继续循环检测
                SYLAR_LOG_ERROR(g_logger_sys) << "continue epoll_wait.";
            }else {
                // 有问题 或者是检测到events
                break;
            }
        }while(true);
        SYLAR_LOG_DEBUG(g_logger_sys) << "check epoll_wait rt=" << rt;
        // TimerManegr 计时器的作用:
        // 1、调用更新函数onTimerInsertedAtFront()的作用就是为了提前停止epoll_wait，查看是否有符合条件的timer
        // 2、timer的实现逻辑就是获取下一个执行的时间，然后epoll_wait执行，
        // 时间到了无论是不是有新的event，都会结束epoll_wait出来调度计时器的任务。
        std::vector<std::function<void()>> cbs;
        listExpiredCbs(cbs);
        if(!cbs.empty()){
            schedule(cbs.begin(),cbs.end());
            cbs.clear();
        }

        // 处理 epoll 返回的事件
        for(int i = 0;i<rt;++i){
            epoll_event& event = epevents[i];
            if(event.data.fd == m_tickleFds[0]){
                uint8_t dummy;
                // 读干净 tickle 信号，防止后面干扰
                while(read(m_tickleFds[0],&dummy,1) == 1);
                continue;
            }
            FdContext* fd_ctx = (FdContext*)event.data.ptr;
            // 如果出了问题，手动将这两个标志添加上去
            MutexType::Lock lock(fd_ctx->mutex);
            // EPOLLHUP 对端关闭了 
            // EPOLLERR 对端出现问题
            if(event.events & (EPOLLERR | EPOLLHUP)){
                // 只会添加注册了的事件，没有注册的不会帮忙添加
                event.events |= (EPOLLIN | EPOLLOUT) & fd_ctx->events;
            }
            int real_event = NONE;
            if(event.events & EPOLLIN){
                real_event |= READ;
            }
            if(event.events & EPOLLOUT){
                real_event |= WRITE;
            }
            // 只会处理预先注册了的事件
            if(fd_ctx->events && real_event == NONE){
                continue;
            }
            // 将现在需要处理的事件抽离出来之后
            int left_event = event.events & ~real_event;
            // 还有事件的话就重新设置监听
            int op = left_event ? EPOLL_CTL_MOD : EPOLL_CTL_DEL;
            event.events = EPOLLET | left_event;
            int rt2 = epoll_ctl(m_epfd,op,fd_ctx->fd,&event);
            if(rt2){
                SYLAR_LOG_ERROR(g_logger_sys) << "epoll_ctl("<< m_epfd <<"," <<op << ","
                    <<  fd_ctx->fd << "," << event.events << "):"
                    << rt2 << "(" << errno << ") (" << strerror(errno) << ")";
            }
            // 将这一次涉及到的事件加入调度器中调度
            if(real_event &READ){
                fd_ctx->triggerEvent(READ);
                --m_pendingEventCount;
            }
            if(real_event & WRITE){
                fd_ctx->triggerEvent(WRITE);
                --m_pendingEventCount;
            }
        }
        
        // idle 线程执行完毕回去。
        Fiber::ptr cur = Fiber::GetThis();
        auto raw_ptr = cur.get(); 
        cur.reset();

        raw_ptr->swapOut();
    }
}
/**
 * @brief: ？？？？这里的逻辑暂时还没有懂,计时器最近的时间更新了，调用 tickle 让 idle 的线程重新更新等待时间
 * @description: 
 * @return {*}
 */
void IOManager::onTimerInsertedAtFront() {
    tickle();
}

}