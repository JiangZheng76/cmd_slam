/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-06-21 12:25:49
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-02-21 11:37:25
 * @FilePath: /mysylar/mysylar/fiber.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __MYSYLAR_FIBER_HPP__
#define __MYSYLAR_FIBER_HPP__

#include <sylar.h>
#include <memory>
#include <functional>
#include <ucontext.h>
namespace mysylar {
    
class Fiber : public std::enable_shared_from_this<Fiber>{
public:
    typedef std::shared_ptr<Fiber> ptr;
    
    enum State{
        // 初始化
        INIT,
        // 暂停
        HOLD,
        // 执行
        EXEC,
        // 结束 
        TERM,
        // 就绪
        READY,
        // 异常
        EXCEPT
    };

    enum Type{
        // 普通类型
        NORM,
        // 发呆类型
        IDLE
    };
private:
    Fiber();
public:
    /**
     * @brief 构造函数
     * @param {function<void()>} cb 毁掉函数
     * @param {size_t} stack_size 协程分配栈的大小
     * @param {bool} usr_caller 是否在MainFiber上面进行调度
     * @description: 
     * @return {*}
     */    
    Fiber(std::function<void()> cb, size_t stack_size = 0,bool usr_caller = false,Type type = Fiber::NORM);
    
    ~Fiber();
    /// @brief 将当前的fiber 转换成新的函数执行的协程，不用重复创建，节省资源
    /// @param cb 
    void reset(std::function<void()> cb);

    /**
     * @brief 将当前的协程切换到运行状态
     * @description: 
     * @return {*}
     */    
    void swapIn();

    /// @brief 当前协程转换到后台
    void swapOut();

    /// @brief 将当前线程转到前台，执行主协程
    void call();

    /// @brief 将当前线程切换到后台，执行当前线程
    void back();

    uint64_t getId() const { return m_id; }
    
    State getState() const { return m_state; }
    void setState(Fiber::State state){ m_state = state; } 

public:
    /// @brief 设置当前线程的运行协程
    /// @param f 运行的协程
    static void SetThis(Fiber* f);

    /// @brief 返回当前所在协程
    /// @return 
    static Fiber::ptr GetThis();

    /// @brief 将当前的协程切换到后台
    static void YieldToReady();

    static void YieldToHold();

    /// @brief 返回当前协程的总量
    /// @return 
    static uint64_t TotalFiber();

    /// @brief 执行当前协程函数，执行完成返回到线程主协程
    static void MainFunc();

    static void CallerMainFunc();

    static uint64_t GetFiberId();

private:
    // 协程 id
    uint64_t m_id = 0;
    // 栈大小
    uint32_t m_stack_size = 0;
    // 当前状态
    State m_state = State::INIT;
    // Fiber 类型
    Type m_type = Type::NORM;
    /// 协程运行上下文
    ucontext_t m_ctx;
    /// @brief 协程栈指针
    void* m_stack = nullptr;
    /// @brief 协程运行函数
    std::function<void()> m_cb;

};

}


#endif