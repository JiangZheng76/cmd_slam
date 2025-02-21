/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-02-27 20:14:48
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-25 16:01:27
 * @FilePath: /mysylar/mysylar/fd_manager.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SYLAR__FD_MANAGER_H__
#define __SYLAR__FD_MANAGER_H__
#include <memory>
#include "thread.h"
#include "iomanager.h"
#include "singleton.h"
#include <sys/types.h>          /* See NOTES */
#include <sys/socket.h>
namespace mysylar{

// fd的管理器
class FdCtx : public std::enable_shared_from_this<FdCtx>{
public:
    typedef std::shared_ptr<FdCtx> ptr;

    FdCtx(int fd);
    ~FdCtx();

    bool init();
    bool isInit(){return m_isInit;}
    bool isSocket(){return m_isSocket;}
    bool isClose(){return m_isClosed;}
    bool close();

    void setUserNonblock(bool v){ m_userNonblock = v;}
    bool getUserNonblock(){return m_userNonblock;}
    
    void setSysNonblock(bool v){ m_sysNonblock = v;}
    bool getSysNonblock(){return m_sysNonblock;}
    enum type{
        SO_RECVTIMEO = SO_RCVTIMEO,
        SO_SENDTIMEO = SO_SNDTIMEO
    };
    void setTimeout(int type , uint64_t v);
    uint64_t getTimeout(int type);
private :
    //  c++ : 是什么语法？？？
    // 是否初始化
    bool m_isInit: 1;
    // 是否socket
    bool m_isSocket: 1;
    // 系统态是否堵塞
    bool m_sysNonblock : 1;
    // 用户态是否堵塞 用户态是指协程是否会发生堵塞，默认socket 在协程是非堵塞的
    bool m_userNonblock : 1;
    bool m_isClosed : 1;
    int m_fd;
    // 接收超时
    uint64_t m_recvTimeout;
    // 发送超时
    uint64_t m_sendTimeout;
};

class FdManager {
public:
    typedef RWMutex RWMutexType;

    FdManager();
    /**
     * @brief 获取 fd，自动创建模式在空间不足时候会自动扩建 
     * @param {int} fd
     * @param {bool} auto_create
     * @description: 
     * @return {*}
     */    
    FdCtx::ptr get(int fd,bool auto_create = false);
    void del(int fd);
private :
    RWMutexType m_mutex;
    std::vector<FdCtx::ptr> m_fds;
};
// 单例模式，全局 fd 管理器
typedef SingletonPtr<FdManager> FdMgr;
}

#endif