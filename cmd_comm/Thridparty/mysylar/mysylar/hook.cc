/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-02-27 10:44:10
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-04-14 22:51:04
 * @FilePath: /mysylar/mysylar/hook.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "hook.h"
#include "sylar.h"
#include "iomanager.h"
#include "fd_manager.h"
#include <dlfcn.h>
#include <sys/types.h>          /* See NOTES */
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
 #include <stdarg.h>
mysylar::Logger::ptr g_logger_sys = SYLAR_LOG_ROOT();
namespace mysylar{

/// @brief 已经忘了 ConfigVar的原理是什么了 ？？？ 在配置器map 中创建了一个名为tcp.connect.timeout的项
static mysylar::ConfigVar<int>::ptr g_tcp_connect_timeout = 
    mysylar::Config::Lookup("tcp.connect.timeout",5000,"tcp connect timeout");
static thread_local bool t_enable_hook = false;

/**
 * @brief 需要 Hook 重写的系统调用函数 
 * @description: 
 * @return {*}
 */
#define HOOK_FUN(XX) \
    XX(sleep) \
    XX(usleep) \
    XX(nanosleep) \
    XX(accept) \
    XX(connect) \
    XX(socket) \
    XX(readv) \
    XX(read) \
    XX(recv) \
    XX(recvfrom) \
    XX(recvmsg) \
    XX(write) \
    XX(writev) \
    XX(send) \
    XX(sendto) \
    XX(sendmsg) \
    XX(close) \
    XX(fcntl) \
    XX(ioctl) \
    XX(getsockopt) \
    XX(setsockopt) 

/**
 * @brief: 初始化钩子，将系统调用函数都用 name_f 来代替，这些名字和它类型都已经在头文件中定义了
 * @description: 
 * @return {*}
 */
void init_hook(){
    static bool is_init = false;
    if(is_init){
        return ;
    }
// RTLD_NEXT ： 返回下一个定义了 name 名字的共享库的函数。
// https://csstormq.github.io/blog/计算机系统篇之链接（16）：真正理解%20RTLD_NEXT%20的作用
// dlsym 函数用于将操作柄中的 name 名字的函数的指针返回回来，一般在运行时动态链接中使用，但是在这里没有使用 dlopen 动态打开，所以应该还是编译时链接
#define XX(name) name ## _f = (name ## _fun) dlsym(RTLD_NEXT,#name);
    HOOK_FUN(XX);
#undef XX
}
// ？？？为什么需要在前面加短_，这表示什么？
static uint64_t s_connect_timeout = -1;
/**
 * @brief 将 hook 的初始化封装在一个结构体中，通过构造函数实现在 main 函数前将 hook 函数初始化 
 * @description: 
 * @return {*}
 */
struct _HookIniter{
    _HookIniter(){
        init_hook();
        s_connect_timeout = g_tcp_connect_timeout->getValue();
        // ？？？ 已经忘了这个addlistener 的实现还有作用是什么了
        g_tcp_connect_timeout->addListener([](const int& old_value, const int& new_value){
            SYLAR_LOG_INFO(g_logger_sys) << "tcp connect timeout changed from " 
                    << old_value << " to " << new_value;
            s_connect_timeout = new_value;
        });
    }
};

// 在主函数前初始化
static _HookIniter s_hook_init;

bool is_enable_hook(){
    return t_enable_hook;
}
void set_hook_enable(bool flag){
    t_enable_hook = flag;
}

}
/**
 * @brief 定时器的条件结构体
 * @description: 
 * @return {*}
 */
struct time_info{
    int cancelled = 0;
};
// ？？？ OrginFun是属于指针么？？改名之后的原系统调用函数
// ？？？ 多参数模板
/**
 * @brief 一个通用的封装io函数，将socket的操作由原来的同步变成异步处理
 * @param {int} fd 
 * @param {OrginFun} fun 处理的原函数
 * @param {char*} hook_fun_name 钩子的函数名称，也就是原函数名称
 * @param {uint32_t} event 事件类型
 * @param {int} timeout_so 超时类型 user 还是 sys
 * @param {Args&& ...} args 处理原函数的参数
 * @description: 
 * 1、判断fd 是否还有效，是不是 socket 或者 是不是非堵塞的
 * 2、监听 io 现在是否就绪，就绪直接用 原来的fun() 来io
 * 3、否则 在当前的给当前调度器添加一个计时器，然后再添加一个事件监听，事件监听挂载的是当前的协程环境；最后 swap 回去。
 * 4、时间到了或者是epoll 给出了事件的提醒回来处理。超时直接退出，事件的话就回来重新处理，处理完回到 2 继续循环，直到 2 返回-1.
 * @return {*}
 */
template<typename OrginFun,typename ... Args>
static ssize_t do_io(int fd,OrginFun fun,const char* hook_fun_name
    , uint32_t event, int timeout_so, Args&& ... args){ // ？？？ Args&& 是什么意思？？Univesal 引用，可以穿左值或者是右值
    if(!mysylar::t_enable_hook){
        // 万能引用 Args&& ...
        // 确保了传进去的参数类型是和原来一样的
        return fun(fd,std::forward<Args>(args)...);
    }
    // ？？？ 为什么限定这几个条件是不执行自定义的？？？这个和iomanager 里面的 FdContext 有什么关系么？
    mysylar::FdCtx::ptr ctx = mysylar::FdMgr::GetInstance()->get(fd);
    if(!ctx){
        // fd 没有 注册
        return fun(fd,std::forward<Args>(args)...);
    }
    if(ctx->isClose()){
        
        errno = EBADF;
        return fun(fd,std::forward<Args>(args)...);
    }
    if(!ctx->isSocket() || ctx->getUserNonblock()){
        // 只有socket才使用这个
        // ？？？ 用户级的非堵塞是什么意思？
        return fun(fd,std::forward<Args>(args)...);
    }

    // 获取超时时间
    uint64_t to = ctx->getTimeout(timeout_so);
    // 创建条件
    std::shared_ptr<time_info> tinfo(new time_info);
retry:
    // 因为进来的socket已经是非阻塞的了，所以可以循环检测
    ssize_t n = fun(fd,std::forward<Args>(args)...);
    while(n == -1 && errno == EINTR){
        // EINTR 表示有其他信号中断了
        // 如果是遇到了某些信号中断了fun的执行，就继续执行
        n = fun(fd,std::forward<Args>(args)...);
    } 
    /// EAGAGIN表示重新启动的意思，这个在O_NONBLOCK中很常见
    // Resource temporarily unavailable 资源暂时不够，重启尝试就好，在O_NONBLOCK中不是错误
    if(n == -1  && errno == EAGAIN){
        mysylar::IOManager* iom = mysylar::IOManager::GetThis();
        mysylar::Timer::ptr timer;
        std::weak_ptr<time_info> winfo(tinfo);
        if(to != (uint64_t)-1){
            // ？？？ 这里是不是没有必要使用addConditionTimer，addTimer也可以吧？？
            timer = iom->addConditionTimer(to,[winfo,fd,iom,event](){
                // 这个判断是不是已经做了么？？？
                auto t = winfo.lock();
                // 时间到了，或者说结束计时
                // 这里是不是冗余了，winfo 已经判断了是否已经超时了
                // 超时才能设置 t->cancelled ,理论上应该不可能会提前出现 t->cancelled 下面log测试一下
                if(!t || t->cancelled){
                    SYLAR_LOG_ERROR(g_logger_sys) << "tinfo has been cancelled in timer.";
                    return ;
                }
                t->cancelled = ETIMEDOUT;
                iom->cancelEvent(fd,(mysylar::IOManager::Event)event);
            },winfo);
        }
        // 在 epoll 中绑定一个 event
        // 将当前的 Fiber 环境绑定到 event_ctx 那样回去调度器之后就可以直接 reset 了
        // 在发生了事件的时候还可以通过 event_ctx回来
        int rt = iom->addEvent(fd,(mysylar::IOManager::Event)event);
        // SYLAR_LOG_DEBUG(g_logger_sys) << hook_fun_name << " addEvent("
        //         << fd << "," << event << ")";
        // 添加失败
        if(rt){
            SYLAR_LOG_ERROR(g_logger_sys) << hook_fun_name << " addEvent("
                << fd << "," << event << ")";
            if(timer){
                timer->cancel();
            }
            return -1;
        }else {
            // 回去，重新丢到
            mysylar::Fiber::YieldToHold();
            // 可能是事件触发 或者是 timer时间到了回到这里
            if(timer){
                timer->cancel();
            }
            if(tinfo->cancelled){
                // 超时回来的
                errno = tinfo->cancelled;
                SYLAR_LOG_DEBUG(g_logger_sys) << hook_fun_name <<"() timeout after " << to << " ms wait!";
                return -1;
            }
            // 回去重新尝试读
            goto retry;
        }
    }
    return n;
} 
// extern C的编译顺序是什么？
extern "C"{
// 前面只是声明了，需要在这里定义
#define XX(name) name ## _fun name ## _f = nullptr;
    HOOK_FUN(XX);
#undef XX
/**
 * @brief 异步 sleep 重写 
 * @param {unsigned int} seconds
 * @description: 
 * 添加计时器 执行的函数是 重新 schedule（）目前的 fiber 环境
 * @return {*}
 */
unsigned int sleep(unsigned int seconds){
    if(mysylar::is_enable_hook()){
        return sleep_f(seconds);
    }
    mysylar::Fiber::ptr fiber = mysylar::Fiber::GetThis();
    mysylar::IOManager* iom = mysylar::IOManager::GetThis();
    // (void(mysylar::Scheduler::*) (mysylar::Fiber::ptr,int thread))&mysylar::IOManager::schedule 指定重载哪一个函数
    // 添加计时器 执行的函数是 重新 schedule（）目前的 fiber 环境
    iom->addTimer(seconds * 1000 ,std::bind((void(mysylar::Scheduler::*)
            (mysylar::Fiber::ptr,int thread))&mysylar::IOManager::schedule,iom,fiber,-1));
    mysylar::Fiber::YieldToHold();
    return 0;
}
int usleep(useconds_t usec ){
    if(mysylar::is_enable_hook()){
        return usleep_f(usec);
    }
    mysylar::Fiber::ptr fiber = mysylar::Fiber::GetThis();
    mysylar::IOManager* iom = mysylar::IOManager::GetThis();
    iom->addTimer(usec / 1000,std::bind(
            (void(mysylar::Scheduler::*)(mysylar::Fiber::ptr,int thread))&mysylar::IOManager::schedule
            ,iom,fiber,-1));
    mysylar::Fiber::YieldToHold();
    return 0;
}
int nanosleep(const struct timespec *req, struct timespec *rem){
    if(mysylar::is_enable_hook()){
        return nanosleep_f(req,rem);
    }
    mysylar::Fiber::ptr fiber = mysylar::Fiber::GetThis();
    mysylar::IOManager* iom = mysylar::IOManager::GetThis();
    uint64_t time = req->tv_sec * 1000 + req->tv_nsec / 1000 / 1000;
    iom->addTimer(time,std::bind(
            (void(mysylar::Scheduler::*)(mysylar::Fiber::ptr,int thread))&mysylar::IOManager::schedule
            ,iom,fiber,-1));
    mysylar::Fiber::YieldToHold();
    return 0;
}

int socket(int domain, int type, int protocol){
    if(!mysylar::is_enable_hook()){
        return socket_f(domain,type,protocol);
    }
    int fd = socket_f(domain,type,protocol);
    // 创建socket时候，默认添加到管理器中
    if(fd != -1){
        mysylar::FdMgr::GetInstance()->get(fd,true);
    }
    return fd;
}
/**
 * @brief 添加上定时器的 connect
 * @param {int} sockfd
 * @param {sockaddr} *addr
 * @param {socklen_t} addrlen
 * @param {uint64_t} timeout_ms
 * @description:
 * 1、检查合法和是否在 socket
 * 2、添加定时器和监听事件，实现逻辑和 do_io一样，Event 挂载当前 Fiber，然后 swapOut 出去 
 * 3、时间到或者是捕获到了 event
 * 4、利用 SO_ERROR 判断 socket 是否有错误
 * @return {*}
 */
int connect_with_timeout(int sockfd, const struct sockaddr *addr, socklen_t addrlen, uint64_t timeout_ms){
    if(!mysylar::t_enable_hook){
        return connect_f(sockfd,addr,addrlen);
    }
    auto ctx = mysylar::FdMgr::GetInstance()->get(sockfd);
    if(!ctx || ctx->isClose()){
        // EBADF 表示有问题的 fd
        errno = EBADF;
        return connect_f(sockfd,addr,addrlen);
    }
    // ？？？为什么用户非堵塞直接执行
    if(!ctx->isSocket() || ctx->getUserNonblock()){
        return connect_f(sockfd,addr,addrlen);
    }
    
    int n = connect_f(sockfd,addr,addrlen);
    if(n == 0){
        // 成功链接
        return 0;
    }else if(n != -1 || errno != EINPROGRESS){
        // 其他错误，EINPROGRESS表示正在链接中，后续可以通过 epoll 来捕获
        // ？？？这里也不懂是什么意思(表示内核没有正在连接了connecting)出现问题了，就出来
        // 正在连接中返回值为-1
        return n;
    }

    mysylar::IOManager* iom = mysylar::IOManager::GetThis();
    mysylar::Timer::ptr timer;
    std::shared_ptr<time_info> tinfo(new time_info);
    std::weak_ptr<time_info> winfo(tinfo);
    // 2、添加定时器和监听事件，实现逻辑和 do_io一样，Event 挂载当前 Fiber，然后 swapOut 出去
    if(iom){
        // 设置了超时时间
        if(timeout_ms != (uint64_t)-1){
            timer = iom->addConditionTimer(timeout_ms,[sockfd,winfo,iom](){
                auto t = winfo.lock();
                if(!t || t->cancelled){
                    return ;
                }
                t->cancelled = ETIMEDOUT;
                iom->cancelEvent(sockfd,mysylar::IOManager::WRITE);
            },winfo);
        }
        // connect 是写 SYN 第一次握手包等待 SYN+ACK 第二次握手包回来
        int rt = iom->addEvent(sockfd,mysylar::IOManager::WRITE);

        if(!rt){
            mysylar::Fiber::YieldToHold();
            if(timer){
                // 从epoll_wait唤醒
                timer->cancel();
            }
            if(tinfo->cancelled){
                // 从计时器中唤醒
                errno = tinfo->cancelled;
                return -1;
            }
        }else {
            if(timer){
                timer->cancel();
            }
            SYLAR_LOG_ERROR(SYLAR_LOG_ROOT()) << "connect addEvent(" << sockfd 
                << "," << mysylar::IOManager::WRITE << ")";
        }
    } else {
        SYLAR_LOG_ERROR(SYLAR_LOG_ROOT()) << "connect IOManger::Getthis() return nullptr.";
        return -1;
    }
    int error = 0;
    socklen_t len = sizeof(int);

    // 4、SO_ERROR 检查connect 的错误
    // ？？？ SOL_SOCKET的作用？ 会在socket 自身layer 进行搜索
    // 获取这次connect 有没有什么错误
    if(-1 == getsockopt(sockfd,SOL_SOCKET,SO_ERROR,&error,&len)){
        return -1;
    }
    // 没有错误返回0
    if(!error) {
        return 0;
    }else {
        errno = error;
        return -1;
    }
}

int connect(int sockfd, const struct sockaddr *addr, socklen_t addrlen){
    // SYLAR_LOG_DEBUG(SYLAR_LOG_ROOT()) << "connect timeout=" << mysylar::s_connect_timeout;
    return connect_with_timeout(sockfd,addr,addrlen,mysylar::s_connect_timeout);
}

int accept(int s, struct sockaddr *addr, socklen_t *addrlen){
    int fd = do_io(s,accept_f,"accept",mysylar::IOManager::READ,SO_RCVTIMEO,addr,addrlen);
    // 接受socket时候，默认添加到管理器中
    if(fd >= 0){
        mysylar::FdMgr::GetInstance()->get(fd,true);
    }
    return fd;
}
ssize_t read(int fd, void *buf, size_t count){
    return do_io(fd,read_f,"read",mysylar::IOManager::READ,SO_RCVTIMEO,buf,count);
}
ssize_t readv(int fd, const struct iovec *iov, int iovcnt){
    return do_io(fd,readv_f,"readv",mysylar::IOManager::READ,SO_RCVTIMEO,iov,iovcnt);
}
ssize_t recv(int sockfd, void *buf, size_t len, int flags){
    return do_io(sockfd,recv_f,"recv",mysylar::IOManager::READ,SO_RCVTIMEO,buf,len,flags);
}
ssize_t recvfrom(int sockfd, void *buf, size_t len, int flags, struct sockaddr *src_addr, socklen_t *addrlen){
    return do_io(sockfd,recvfrom_f,"recvfrom",mysylar::IOManager::READ,SO_RCVTIMEO,buf,len,flags,src_addr,addrlen);
}
ssize_t recvmsg(int sockfd, struct msghdr *msg, int flags){
    return do_io(sockfd,recvmsg_f,"recvmsg",mysylar::IOManager::READ,SO_RCVTIMEO,msg,flags);
}
ssize_t write(int fd, const void *buf, size_t count){
    return do_io(fd,write_f,"write",mysylar::IOManager::WRITE,SO_SNDTIMEO,buf,count);
}
ssize_t writev(int fd, const struct iovec *iov, int iovcnt){
    return do_io(fd,writev_f,"writev",mysylar::IOManager::WRITE,SO_SNDTIMEO,iov,iovcnt);
}
ssize_t send(int s, const void *msg, size_t len, int flags){
    return do_io(s,send_f,"send",mysylar::IOManager::WRITE,SO_SNDTIMEO,msg,len,flags);
}
ssize_t sendto(int s, const void *msg, size_t len, int flags, const struct sockaddr *to, socklen_t tolen){
    return do_io(s,sendto_f,"sendto",mysylar::IOManager::WRITE,SO_SNDTIMEO,msg,len,flags,to,tolen);
}
ssize_t sendmsg(int s, const struct msghdr *msg, int flags){
    return do_io(s,sendmsg_f,"sendmsg",mysylar::IOManager::WRITE,SO_SNDTIMEO,msg,flags);
}
int close(int fd){
    if(!mysylar::t_enable_hook){
        return close_f(fd);
    }
    mysylar::FdCtx::ptr ctx =  mysylar::FdMgr::GetInstance()->get(fd);
    if(!ctx){
        return -1;
    }else {
        auto iom = mysylar::IOManager::GetThis();
        // 删除fd，默认取消所有的和它相关的event
        if(iom){
            iom->cancelAllEvent(fd);
        }
        mysylar::FdMgr::GetInstance()->del(fd);
    }
    return close_f(fd);
}
/**
 * @brief 在设置fd的状态标记，在socket 类型 用户线程（协程）默认不阻塞，系统级是否会发生阻塞按照控制来。F_GETFL 获取socket的O_Nonblock状态是用户级的
 * @param {int} fd
 * @param {int} cmd
 * @param {* arg} *
 * @description: ？？？不是很懂里面的tag各自的作用，重点记住在设置阻塞的 Flag 的时候会顺带设置了 fdctx 的就行了。
 * @return {*}
 */
int fcntl(int fd, int cmd, ... /* arg */ ) {
    va_list va;
    va_start(va, cmd);
    switch(cmd) {
        case F_SETFL:
            {
                int arg = va_arg(va, int);
                va_end(va);
                mysylar::FdCtx::ptr ctx = mysylar::FdMgr::GetInstance()->get(fd);
                if(!ctx || ctx->isClose() || !ctx->isSocket()) {
                    return fcntl_f(fd, cmd, arg);
                }

                // socket 类型 用户线程（协程）默认不阻塞
                ctx->setUserNonblock(arg & O_NONBLOCK);
                // socket 类型 系统级是否会发生阻塞按照控制来
                if(ctx->getSysNonblock()) {
                    arg |= O_NONBLOCK;
                } else {
                    arg &= ~O_NONBLOCK;
                }
                return fcntl_f(fd, cmd, arg);
            }
            break;
        case F_GETFL:
            {
                va_end(va);
                int arg = fcntl_f(fd, cmd);
                mysylar::FdCtx::ptr ctx = mysylar::FdMgr::GetInstance()->get(fd);
                if(!ctx || ctx->isClose() || !ctx->isSocket()) {
                    return arg;
                }
                // socket
                if(ctx->getUserNonblock()) {
                    return arg | O_NONBLOCK;
                } else {
                    return arg & ~O_NONBLOCK;
                }
            }
            break;
        case F_DUPFD:
        case F_DUPFD_CLOEXEC:
        case F_SETFD:
        case F_SETOWN:
        case F_SETSIG:
        case F_SETLEASE:
        case F_NOTIFY:
#ifdef F_SETPIPE_SZ
        case F_SETPIPE_SZ:
#endif
            {
                int arg = va_arg(va, int);
                va_end(va);
                return fcntl_f(fd, cmd, arg); 
            }
            break;
        case F_GETFD:
        case F_GETOWN:
        case F_GETSIG:
        case F_GETLEASE:
#ifdef F_GETPIPE_SZ
        case F_GETPIPE_SZ:
#endif
            {
                va_end(va);
                return fcntl_f(fd, cmd);
            }
            break;
        case F_SETLK:
        case F_SETLKW:
        case F_GETLK:
            {
                struct flock* arg = va_arg(va, struct flock*);
                va_end(va);
                return fcntl_f(fd, cmd, arg);
            }
            break;
        case F_GETOWN_EX:
        case F_SETOWN_EX:
            {
                struct f_owner_exlock* arg = va_arg(va, struct f_owner_exlock*);
                va_end(va);
                return fcntl_f(fd, cmd, arg);
            }
            break;
        default:
            va_end(va);
            return fcntl_f(fd, cmd);
    }
}
/**
 * @brief 设置阻塞的时候，也是默认设置用户级的阻塞
 * @param {int} d
 * @param {unsigned long int} request
 * @description: 
 * @return {*}
 */
int ioctl(int d, unsigned long int request, ...) {
    va_list va;
    va_start(va, request);
    void* arg = va_arg(va, void*);
    va_end(va);

    if(FIONBIO == request) {
        bool user_nonblock = !!*(int*)arg;
        mysylar::FdCtx::ptr ctx = mysylar::FdMgr::GetInstance()->get(d);
        if(!ctx || ctx->isClose() || !ctx->isSocket()) {
            return ioctl_f(d, request, arg);
        }
        ctx->setUserNonblock(user_nonblock);
    }
    return ioctl_f(d, request, arg);
}

int getsockopt(int sockfd, int level, int optname, void *optval, socklen_t *optlen){
    return getsockopt_f(sockfd,level,optname,optval,optlen);
}
int setsockopt(int sockfd, int level, int optname, const void *optval, socklen_t optlen){
    if(mysylar::t_enable_hook){
        return setsockopt_f(sockfd,level,optname,optval,optlen);
    }
    // 如果是设置socket的时间的话，也同步fd manager 中维护的时间
    if(level == SOL_SOCKET){
        if(optname == SO_RCVTIMEO || optname == SO_SNDTIMEO){
            auto ctx = mysylar::FdMgr::GetInstance()->get(sockfd);
            if(ctx ){
                const timeval* v = (const timeval*)optval;
                // 设置读或者写的超时时间
                ctx->setTimeout(optname,v->tv_sec * 1000 + v->tv_usec / 1000);
            }
        }
    }
    return setsockopt_f(sockfd,level,optname,optval,optlen);
}

}


