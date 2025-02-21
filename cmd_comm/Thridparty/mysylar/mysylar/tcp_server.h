/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-03-16 15:12:48
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-26 10:57:44
 * @FilePath: /mysylar/mysylar/tcp_server.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SYLAR__TCP_SERVER_H__
#define __SYLAR__TCP_SERVER_H__
#include "iomanager.h"
#include "sylar.h"
#include "socket.h"
#include "streams/socket_stream.h"
#include <memory>

namespace mysylar{


class TcpServer : public std::enable_shared_from_this<TcpServer> , Noncopyable{
public:

    typedef std::shared_ptr<TcpServer> ptr;

    TcpServer(IOManager* io_woker = IOManager::GetThis()
                ,IOManager* accept_worker = IOManager::GetThis());
    virtual ~TcpServer();
    /**
     * @brief 根据地址创建好 socket
     * @param {ptr} addr
     * @description: 
     * @return {*}
     */
    virtual bool bind(Address::ptr addr);
    /**
     * @brief 批量绑定
     * @description: 
     * @return {*}返回 bind 失败的地址
     */
    virtual bool bind(const std::vector<Address::ptr>& addrs,std::vector<Address::ptr>& fails);
    /**
     * @brief socket 执行的等待accept 新的 socket
     * @param {ptr} sock
     * @description: 
     * @return {*}
     */
    virtual bool startAccept(Socket::ptr sock);
    /**
     * @brief 启动服务器
     * @description: 
     * 启动 accept 调度器
     * @return {*}
     */    
    virtual bool start();
    /**
     * @brief 停止服务器
     * @description: 
     * @return {*}
     */    
    virtual void stop();

    std::string getName()const {return m_name;}
    void setRecvTime(uint64_t v){m_recvTimeout = v;}
    void setName(const std::string& v){ m_name = v; }

    bool isStop(){return m_isStop;}

protected:
    /**
     * @brief 新建立 socket 的任务执行函数
     * @param {ptr} client
     * @description: 
     * @return {*}
     */    
    virtual void handleClient(Socket::ptr client);

private:
    std::vector<Socket::ptr> m_socks;           // 多监听，多网卡
    IOManager* m_worker;                        // 新连接的Socket工作的调度器
    IOManager* m_acceptWorker;                  // 服务器Socket接收连接的调度器
    uint64_t m_recvTimeout;                     // 接收超时时间
    std::string m_name;                         // 服务器名称
    bool m_isStop;                              // 服务是否停止
};

}
#endif