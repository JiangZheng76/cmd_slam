/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-06-21 12:25:54
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-14 21:59:09
 * @FilePath: /mysylar/mysylar/fiber.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "fiber.hpp"

namespace mysylar{

static uint64_t s_fiber_id{0};
static uint32_t s_fiber_count{0};

/// @brief 执行协程
static thread_local Fiber* t_fiber = nullptr;
/// @brief 主协程
static thread_local Fiber::ptr t_threadfiber = nullptr;

ConfigVar<uint32_t>::ptr g_fiber_stack_size = 
    Config::Lookup<uint32_t>("fiber.stack_size", 128 * 1024,"fiber stack size");
    
// Logger::ptr g_logger = SYLAR_LOG_ROOT();
Logger::ptr g_logger = SYLAR_LOG_NAME("system");

/// @brief RAII 思想进行 malloc stack
class MallocStackAllocator{
public:
    static void* Alloc(size_t size){
        return malloc(size);
    }
    static void Dealloc(void* vp,size_t size = 0){
        free(vp);
    }
};
using StackAllocator = MallocStackAllocator;

/**
 * @brief main_fiber 的创建入口, 不允许其他人通过默认构造函数创建，只可以通过 GetThis ，在还没有开始子协程执行的时候在协程中创建，主要目的是为了保存 主携程的上下文
 * @description: 
 * @return {*}
 */
Fiber::Fiber() 
    :m_id(++s_fiber_id){
    // main_fiber 主携程永远处于执行态
    m_state = State::EXEC;
    SetThis(this);
    // getcontext 保存当前主携程上下文
    if(getcontext(&m_ctx)){
        SYLAR_ASSERT2(false,"getcontext error.");
    }
    s_fiber_count++;
}
/**
 * @brief  创建子协程的构造函数
 * @param {function<void()>} cb
 * @param {size_t} stacksize
 * @param {bool} user_call
 * @param {Type} type
 * @description: 
 * @return {*}
 */
Fiber::Fiber(std::function<void()> cb,size_t stacksize,bool user_call,Type type)
    // 初始化的顺序最好和声明的顺序保持一致
    :m_id(++s_fiber_id),m_cb(cb){
    ++s_fiber_count;
    m_type = type;
    // 预设好分配给协程大栈大小
    m_stack_size = stacksize ? stacksize : g_fiber_stack_size->getValue();
    // 保存协程上下文环境
    if(getcontext(&m_ctx)){
        SYLAR_ASSERT2(false,"getcontext error.");
    }
    // 给协程分配栈的空间
    m_stack = StackAllocator::Alloc(m_stack_size);
    
    m_ctx.uc_link = nullptr;
    m_ctx.uc_stack.ss_sp = m_stack;
    m_ctx.uc_stack.ss_size = m_stack_size;
    SYLAR_LOG_DEBUG(g_logger) << "Fiber::Fiber() id=" << m_id
                              << " total=" << s_fiber_count;
    // 创建一个新的上下文
    // m_ctx 提供上下文信息（必须要经过 getcontext 初始化），MainFunc 是开始执行的函数，后面的是参数
    // 这里不立马开始，需要等调用swapcontext 才进入到 MainFunc 中
    makecontext(&m_ctx,&Fiber::MainFunc,0);
}

Fiber::~Fiber(){
    --s_fiber_count;
    if(m_stack){
        // 只有状态是处于 init 和 term 才回收空间，那如果不是这些状态的协程什么时候回收呢？？？表明协程没有执行完，不会回收还在调度器里面调度
        SYLAR_ASSERT2(m_state == State::INIT || m_state == State::TERM ," fiber state error.");
        StackAllocator::Dealloc(m_stack);
    }else {
        // 这里不懂 没有分配空间，有函数 或者 不处于运行态的都出现了异常么？？？？没有被分配栈的就是主携程
        // 这一块都不懂？？？
        // 主协程是没有栈的。
        SYLAR_ASSERT(!m_cb);                    // 主携程是没有执行函数的，只会保存用于切换的上下文
        SYLAR_ASSERT(m_state == State::EXEC);   // 当前正在执行，而如果不是处于执行态说明出问题了

        Fiber* cur = t_fiber;
        if(cur == this){
            SetThis(nullptr);                   // 如果主线程在执行的话，取消主线程指针
        }
    }
    SYLAR_LOG_DEBUG(g_logger) << "Fiber::~Fiber id=" << m_id
                              << " total=" << s_fiber_count;
}

/**
 * @brief 只有处于执行态或者是阻塞态的执行协程（非主携程）可以重新配置
 * @param {function<void()>} cb
 * @description: 
 * @return {*}
 */
void Fiber::reset(std::function<void()> cb) {
    // 只有子携程会参与工作
    SYLAR_ASSERT(m_stack);
    // 因为是 reset 动作是 this 自己发起的，所以自己不在执行态，就不可能会 reset 自己的状态，发生了逻辑错误。其实也不是很懂？？？别人不可以重设你么？？？
    // 只有自己可以在执行态的时候设置自己，或者是 this 是 HOLD 状态，别人设置你
    SYLAR_ASSERT(m_state == INIT || m_state == EXCEPT || m_state==TERM);

    m_cb = cb;
    if(getcontext(&m_ctx)){                     // 因为执行的入口发生了改变，整个环境需要重新配置
        SYLAR_ASSERT2(false,"getcontext error.");
    }
    // 这个上下文不是已经是这个了么？？为什么还需要重新声明一次？
    m_ctx.uc_link = nullptr;
    m_ctx.uc_stack.ss_sp = m_stack;
    m_ctx.uc_stack.ss_size = m_stack_size;

    // MainFunc 代表什么东西？？？封装好的函数入口
    makecontext(&m_ctx,&Fiber::MainFunc,0);
    m_state = INIT;                         // 重新配置状态，回到开始
}
/**
 * @brief 转入处理当前协程 
 * @description: 
 * @return {*}
 */
void Fiber::swapIn(){
    SetThis(this);          // 将当前执行线程 t_fiber 变为发起 swapin 的协程
    // 只有不在执行态，处于 HOLD，READY 等状态的协程可以被 swapin。
    SYLAR_ASSERT(m_state!= EXEC);
    m_state = EXEC;
    // 跳转到MainFunc中执行fiber的任务。 
    SYLAR_LOG_DEBUG(g_logger) << "swapIn() " << m_id;
    // 报错当前上下文到主协程，将当前协程换上来
    // 子携程调用 swapin 的时候就是主携程在工作的，在主携程下子携程发起 swapin
    if(swapcontext(&t_threadfiber->m_ctx,&m_ctx)){
        SYLAR_ASSERT2(false,"swapcontext error.");
    }
    // 子携程执行完毕，或者中途 HOLD 了，回到主携程入口这里
    SYLAR_LOG_DEBUG(g_logger) << "swapIn() out." << m_id;
}
/**
 * @brief  子携程在工作完了，想回去主携程
 * @description: 
 * @return {*}
 */
void Fiber::swapOut() {
    // 子携程要离开，当前执行协程切换回主携程
    SetThis(t_threadfiber.get());
    if(swapcontext(&m_ctx,&t_threadfiber->m_ctx)){
        SYLAR_ASSERT2(false,"swapcontext error.");
    }
}

void Fiber::SetThis(Fiber* f){
    t_fiber = f;
}
/**
 * @brief 获取当前协程
 * 如果是本线程第一次调用（当前协程没有设置），那么就默认调用的是主线程，并创建主线程对象（默认构造函数）和初始化本线程环境
 * @description: 
 * @return {*}
 */
Fiber::ptr Fiber::GetThis(){
    if(t_fiber){
        return t_fiber->shared_from_this();
    }
    // 本线程第一次调用（当前线程还没有配置协程）
    Fiber::ptr main_fiber(new Fiber);
    SYLAR_ASSERT(main_fiber.get() == t_fiber);
    // 那么就默认调用的是主线程，并创建主线程对象（默认构造函数）和初始化本线程环境
    t_threadfiber = main_fiber;
    return t_fiber->shared_from_this();
}
// 当前协程切换到后台
void Fiber::YieldToHold(){
    Fiber::ptr cur = GetThis();
    SYLAR_ASSERT(cur->m_state == EXEC);
    cur->m_state = HOLD;
    cur->swapOut();
}

void Fiber::YieldToReady(){
    Fiber::ptr cur = GetThis();
    SYLAR_ASSERT(cur->m_state == EXEC);
    cur->m_state = READY;
    cur->swapOut();
}

uint64_t Fiber::TotalFiber(){
    return s_fiber_count;
}
/**
 * @brief 封装协程执行的主函数
 * @description: 
 * 1、正常执行完成之后会自己释放掉主函数的内容资源
 * 2、如果是执行出现异常是不会自动回收函数cb的内容资源
 * 3、执行完成之后修改自己的状态值为TERM或者是EXCEPT swapOut回到mainFiber 中
 * @return {*}
 */
void Fiber::MainFunc(){
    // 智能指针数+1
    Fiber::ptr cur = GetThis(); 
    SYLAR_ASSERT(cur != t_threadfiber); // 主协程进不来
    try{
        cur->m_cb();
        cur->m_cb = nullptr;
        cur->m_state = TERM; // 执行完毕，终止状态
    }catch (std::exception& ex){
        cur->m_state = EXCEPT;
        SYLAR_LOG_ERROR(g_logger) << "Fiber Exception : "<<ex.what()
            << "fiber_id =" << cur->m_id
            << std::endl
            << mysylar::BacktraceToString(100);
    // 怎么理解
    } catch (...){
        cur->m_state = EXCEPT;
        SYLAR_LOG_ERROR(g_logger) << "Fiber Exception : "
            << "fiber_id =" << cur->m_id
            << std::endl
            << mysylar::BacktraceToString(100);
    }
    auto raw_ptr = cur.get();
    cur.reset(); // 智能指针数-1
    // 执行结束之后 回到主Fiber
    raw_ptr->swapOut();
    // 完成任务的子协程回去之后应该被delete，永远不可能回来
    SYLAR_ASSERT2(false, "never reach fiber_id=" + std::to_string(raw_ptr->getId()));
}

uint64_t Fiber::GetFiberId(){
    if(t_fiber) {
        return GetThis()->m_id;
    }
    return 0;
}

void Fiber::call(){
    SetThis(this);
    m_state = EXEC;
    // 保存t_threadFiber的上下文 然后将当前协程的上下文m_ctx部署上来
    SYLAR_LOG_INFO(g_logger) << "call()";
    if(swapcontext(&t_threadfiber->m_ctx, &m_ctx)) {
        SYLAR_ASSERT2(false, "swapcontext");
    }
    SYLAR_LOG_INFO(g_logger) << "call() swapOut()";
}


}

