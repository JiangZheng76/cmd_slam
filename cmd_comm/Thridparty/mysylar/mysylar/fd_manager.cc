/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-02-27 20:14:53
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-19 11:37:37
 * @FilePath: /mysylar/mysylar/fd_manager.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AEme
 */
#include "fd_manager.h"
#include <sys/types.h>
#include <sys/stat.h>
#include "hook.h"
#include <fcntl.h>
#include <sys/types.h>          /* See NOTES */
#include <sys/socket.h>
namespace mysylar{

FdCtx::FdCtx(int fd)
    :m_isInit(false)
    ,m_isSocket(false)
    ,m_sysNonblock(false)
    ,m_userNonblock(false)
    ,m_isClosed(false)
    ,m_fd(fd)
    ,m_recvTimeout(-1)
    ,m_sendTimeout(-1){
    init();
}
FdCtx::~FdCtx(){
}
bool FdCtx::init(){
    if(m_isInit){
        return true;
    }
    m_recvTimeout = 5000;
    m_sendTimeout = 5000;
    struct stat fd_stat;

    if(-1 == fstat(m_fd,&fd_stat)){
        m_isInit = false;
        m_isSocket = false;
    }else {
        m_isInit = true;
        // 判断是否是socket
        m_isSocket = S_ISSOCK(fd_stat.st_mode);
    }

    if(m_isSocket){
        // ？？？为什么系统设计如果是socket就走hook，不是socket(普通文件)就按照原来的走
        // 主要是面向 socket
        // 获取 socket 的 flag
        int flag = fcntl_f(m_fd,F_GETFL,0);
        // 为什需要保证socket是非阻塞的？？？ 
        if(!(flag & O_NONBLOCK)){
            fcntl_f(m_fd,F_SETFL,flag | O_NONBLOCK);
        }
        m_sysNonblock = true;
    }else {
        m_sysNonblock = false;
    }
    m_userNonblock = false;
    m_isClosed = false;
    return m_isInit;
}
void FdCtx::setTimeout(int type , uint64_t v){
    if(type == SO_RCVTIMEO){
        m_recvTimeout = v;
    }else {
        m_sendTimeout = v;
    }
}
uint64_t FdCtx::getTimeout(int type){
    if(type == SO_RCVTIMEO){
        return m_recvTimeout ;
    }else {
        return m_sendTimeout;
    }

}
bool close();

FdManager::FdManager(){
    m_fds.resize(64);
}

FdCtx::ptr FdManager::get(int fd,bool auto_create){
    RWMutexType::ReadLock lock(m_mutex);
    // ？？？ 为什么说同一时间只会有一个进来，所以不需要关心线程安全？？（后面反口了，但还是想想为什么吧）
    if((int)m_fds.size() <= fd){
        if(!auto_create){
            return nullptr;
        }
        m_fds.resize(2 * m_fds.size());
    }else {
        if(m_fds[fd] || !auto_create){
            return m_fds[fd];
        }
    }
    lock.unlock();

    RWMutexType::WriteLock lock2(m_mutex);
    FdCtx::ptr ctx(new FdCtx(fd));
    m_fds[fd] = ctx;
    return ctx;
}
void FdManager::del(int fd){
    RWMutexType::WriteLock lock(m_mutex);
    if((int)m_fds.size() <= fd){
        return;
    }
    m_fds[fd].reset();
}

}