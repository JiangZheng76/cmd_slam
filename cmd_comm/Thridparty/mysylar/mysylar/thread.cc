/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-06-13 10:49:20
 * @LastEditors: Johnathan 2440877322@qq.com
 * @LastEditTime: 2024-08-13 23:35:58
 * @FilePath: /mysylar/mysylar/thread.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "thread.h"
namespace mysylar{
// static Logger::ptr g_log_system = SYLAR_LOG_NAME("system");
static Logger::ptr g_log_system = SYLAR_LOG_ROOT();


// thread_local 可以理解为各个线程中互不影响的变量 https://zhuanlan.zhihu.com/p/624614590
// 当前线程中正在执行的线程对象，一般就是自己
// 在 run 函数中进行初始化，也就是当前线程开启的时候初始化
static thread_local Thread* t_thread = nullptr;
// 当前线程的名字
static thread_local std::string t_thread_name = "UNKONW";

/**
 * @brief  
 * @param {function<void()>} cb Thread 的主要执行函数，需要套一层 run（）封装，在 run（）函数中执行
 * @param {string&} name
 * @description: 
 * @return {*}
 */
Thread::Thread(std::function<void()> cb,const std::string& name)
:m_cb(cb){
    if(name.empty()){
        m_name = "UNKNOW";
    }else {
        m_name = name;
    }
    // 成功返回0 失败返回error码
    // 创建线程 pthread_create(pthread_t *thread, const pthread_attr_t *attr, void *(*start_routine) (void *), void *arg);
    // 第四个参数 this 是传给 run 函数的指针。
    // 为什么run 使用静态函数？？？我估计是为了可以传 this 进去使用吧
    // pthread 创建成功之后，run 函数就开始执行了，run 函数里面就是在创建的 pthread线程环境里面
    // 构造函数这里还是在主线程的环境里面
    int rt = pthread_create(&m_thread,nullptr,Thread::run,this);
    if(rt){
        SYLAR_LOG_ERROR(g_log_system) << "craete thread error rt: "<< rt
            << "name = " << m_name;
        throw std::logic_error("craete thread error name: " + m_name);
    }
    // 在 run 函数中释放了信号量表示创建的线程环境初始化完成了
    // 线程的 m_cb 函数已经开始执行了
    // 保证线程创建成功之后肯定是跑起来的
    // 确保 Thread 构造函数调用成功之后，线程是已经启动了
    m_semaphore.wait();
}
/**
 * @brief 主线程中主动关闭 
 * @description: 
 * @return {*}
 */
Thread::~Thread(){
    if(m_thread){
        // detach 和 join 的区别：
        // https://blog.csdn.net/WuLex/article/details/89390547?utm_medium=distribute.pc_relevant.none-task-blog-2~default~baidujs_baidulandingword~default-0-89390547-blog-119109979.235^v43^pc_blog_bottom_relevance_base5&spm=1001.2101.3001.4242.1&utm_relevant_index=3
        // detach 将当前线程和主线程分离，分离之后主线程和子线程没有任何关系了
        // join 主线程阻塞等待子线程结束，然后回收子线程资源。
        // 主线程和子线程在 pthread 里面都是平等的关系，主线程结束，子线程也结束是因为主线程结束调用了 exit(0)杀死了进程。
        pthread_detach(m_thread);
    }
}
void Thread::detach(){
    if(m_thread){
        pthread_detach(m_thread);
    }
}
/**
 * @brief 获取当前的执行线程，A线程里面调用获取A线程自己 
 * @description: 
 * @return {*}
 */
Thread* Thread::GetThis(){
    return t_thread;
}

void Thread::SetName(const std::string& name){
    if(name.empty()){
        return ;
    }
    if(t_thread){
        t_thread->m_name = name;
    }
    t_thread_name = name;
}

const std::string& Thread::GetName(){
    return t_thread_name;
}
/**
 * @brief 主线程调用的join 线程接口，等待子线程的结束回收子线程资源
 * @description: 
 * @return {*}
 */
void Thread::join(){
    if(m_thread){
        int rt = pthread_join(m_thread,nullptr);
        if(rt) {
                SYLAR_LOG_ERROR(g_log_system) << "pthread_join thread fail" 
                << rt << " name= " << m_name;
            throw std::logic_error("pthread_join error");
        }
        m_thread = 0;
    }
    
}

/**
 * @brief 
 * @param {void*} arg 将arg变量作为一个初始化的信息包传进去，一般传送的是封装好的 Thread 线程指针
 * @description: 
 * @return {*}
 */    
void* Thread::run(void* arg){
    Thread* thread = (Thread*)arg;
    t_thread= thread;
    t_thread_name = thread->m_name;
    thread->m_id = mysylar::GetThreadId();
    pthread_setname_np(pthread_self(),thread->m_name.substr(0,15).c_str());

    // 自己创建一个 cb 函数来调用
    // 这样操作可以让主线程结束之后，也不会影响自己执行
    std::function<void()> cb;
    cb.swap(thread->m_cb);

    // 需要等待线程开始跑了，提醒结束初始化
    thread->m_semaphore.notify();
    cb();
    SYLAR_LOG_INFO(g_log_system) << "thraed end.";
    return 0;
}


}