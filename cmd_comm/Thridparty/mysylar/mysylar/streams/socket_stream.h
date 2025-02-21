/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-03-16 16:17:43
 * @LastEditors: Johnathan 2440877322@qq.com
 * @LastEditTime: 2024-07-06 12:28:18
 * @FilePath: /mysylar/mysylar/socket_stream.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE#inc
 */
#ifndef __SYLAR_SOCKET_STREAM_H__
#define __SYLAR_SOCKET_STREAM_H__
#include "stream.h"
#include "socket.h"

namespace mysylar{
/**
 * @brief 封装了 socket 和 Stream，用于对 socket 的读写操作，如果有权限可以自动释放 socket
 * @description: 
 * @return {*}
 */
class SocketStream : public Stream{
public:    
    SocketStream(Socket::ptr sock=nullptr,bool owner = true);
    virtual ~SocketStream();

    bool isConnected();
    virtual void close() override;

    /**
     * @brief 封装 socket的 recv
     * @param {void*} buffer
     * @param {size_t} length
     * @description: 
     * @return {*}
     */    
    virtual int read(void* buffer,size_t length) override;
    /**
     * @brief 封装 socket 的 read 
     * @param {ptr} ba 接收的位置，调用getwriteBuffers
     * @param {size_t} length
     * @description: 
     * @return {*}
     */    
    virtual int read(ByteArray::ptr ba,size_t length) override;
    /**
     * @brief 封装 socket的 send
     * @param {void*} buffer
     * @param {size_t} length
     * @description: 
     * @return {*}
     */    
    virtual int write(void* buffer,size_t length) override;
    /**
     * @brief 封装 socket的 write 
     * @param {ptr} ba 需要写出去的资源，调用getReadBuffers
     * @param {size_t} length
     * @description: 
     * @return {*}
     */        
    virtual int write(ByteArray::ptr ba,size_t length) override;

    void reset(Socket::ptr socket,bool owner);
    Address::ptr getRemoteAddress();
    Address::ptr getLocalAddress();
    std::string getRemoteAddressString();
    std::string getLocalAddressString();
    
    Socket::ptr getSocket(){ return m_socket; }

private:
    Socket::ptr m_socket;   // 这里不会出现循环引用么？？？socket 里面没有他
    bool m_owner;           // 不懂这个主控是什么？？？自主控制释放sock 资源，sock 是否跟这个释放
};

}


#endif


