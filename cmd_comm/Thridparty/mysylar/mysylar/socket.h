#ifndef __SYALAR_SOCKET_H__
#define __SYALAR_SOCKET_H__
#include "address.h"
#include <memory>
#include "noncopyable.h"
namespace mysylar{
/**
 * @brief Socket 是不可以复制资源
 * @description: 
 * @return {*}
 */
class Socket : public std::enable_shared_from_this<Socket>,Noncopyable{
public:
    typedef std::shared_ptr<Socket> ptr;
    typedef std::weak_ptr<Socket> weak_ptr;

    enum Type{
        TCP = SOCK_STREAM,
        UDP = SOCK_DGRAM
    };

    enum Family{
        IPv4 = AF_INET,
        IPv6 = AF_INET6,
        Unix = AF_UNIX
    };
    /**
     * @brief 按照 addr 的类型创建一个 tcp 类型的 sock 
     * @param {ptr} addr
     * @description: 
     * @return {*}
     */    
    static Socket::ptr CreateTCP(Address::ptr addr);
    /**
     * @brief 创建一个 IPv6 的 tcp sock
     * @description: 
     * @return {*}
     */    
    static Socket::ptr CreateTCPSocket6();
    /**
     * @brief 创建一个 IPv4 的 sock
     * @description: 
     * @return {*}
     */    
    static Socket::ptr CreateTCPSocket();
    /**
     * @brief 按照 addr 的类型创建一个 udp 类型的 sock 
     * @param {ptr} addr
     * @description: 
     * @return {*}
     */ 
    static Socket::ptr CreateUDP(Address::ptr addr);
    /**
     * @brief 创建一个 IPv6 的 udp sock
     * @description: 
     * @return {*}
     */    
    static Socket::ptr CreateUDPSocket6();
    /**
     * @brief 创建一个 IPv4 的 udp sock
     * @description: 
     * @return {*}
     */    
    static Socket::ptr CreateUDPSocket();

    static Socket::ptr CreateUnixTCPSocket(Address::ptr addr);
    static Socket::ptr CreateUnixUDPSockett();

    /**
     * @brief Socket 对象构造函数，并不会分配 sock fd，只是构建一个壳
     * @param {int} family
     * @param {int} type
     * @param {int} protocol
     * @description: 
     * @return {*}
     */    
    Socket(int family,int type,int protocol = 0);
    ~Socket();

    int64_t getSendTimeout();
    int64_t getRecvTimeout();

    bool setSendTimeout(int64_t v); 
    bool setRecvTimeout(int64_t v); 

    /**
     * @brief 封装好的 getsockopt（）方法
     * @param {int} level
     * @param {int} option
     * @param {void} *result
     * @param {socklen_t*} len
     * @description: 
     * @return {*}
     */    
    bool getOption(int level , int option,void *result,socklen_t* len);
    bool setOption(int level ,int option,const void* result,socklen_t* len);

    /**
     * @brief 为了自动推导出 set 项 或者是 get 项的长度
     * @param {int} level
     * @param {int} option
     * @param {T&} result
     * @description: 
     * ？？？result 是什么东西，为什么可以传void进去
     * @return {*}
     */    
    template<class T>
    bool getOption(int level, int option,T& result){
        size_t len = sizeof(T);
        return getOption(level,option,result,&len);
    }
    /**
     * @brief 为了自动推导出 set 项 或者是 get 项的长度
     * @description: 
     * @return {*}
     */    
    template<class T>
    bool setOption(int level, int option,T& result){
        socklen_t len = sizeof(T);
        return setOption(level,option,&result,&len);
    }
    
    // 封装的 accept，bind，connect，listen 接口
    Socket::ptr accept();
    bool init(int sock);
    bool bind(const Address::ptr addr);
    bool connect(const Address::ptr addr, uint64_t timeout_ms = -1);
    bool listen(int backlog = SOMAXCONN);
    bool close();
    // TCP 使用
    int send(const void* buffer,size_t length,int flags = 0);
    int send(const iovec* buffers,size_t length,int flags = 0);
    int recv(void * buffer,size_t length,int flags = 0);
    int recv(iovec * buffers,size_t length,int flags = 0);
    // UDP使用
    int sendTo(const void* buffer,size_t length,const Address::ptr to,int flags = 0);
    int sendTo(const iovec* buffers,size_t length,const Address::ptr to ,int flags = 0);
    int recvFrom(iovec * buffers,size_t length, Address::ptr from ,int flags = 0);
    int recvFrom(void * buffers,size_t length, Address::ptr from ,int flags = 0);

    Address::ptr getRemoteAddress();
    Address::ptr getLocalAddress();

    int getFamily() const {return m_family;} 
    int getType() const {return m_type;}
    int getProtocol() const {return m_protocol;}

    int isConnected() const{ return m_isConnected;}
    bool isVaild() const;
    int getError();

    // ？？？有啥用，用来输出sock 信息的
    std::ostream& dump(std::ostream& os) const;
    int getSocket() const { return m_sock;}

    // 取消调度器中对 sock 的监听接口
    bool cancelRead();
    bool cancelWrite();
    bool cancelAccept();
    bool cancellAll();

private:
    /**
     * @brief 内部使用初始化自身的方法
     * @description: 
     * ？？？内部 initSock，主要是 option 相关的东西
     * @return {*}
     */
    void initSock();
    /**
     * @brief socket 对象构造函数不会创建 sockfd，使用这个接口来创建 
     * @description: 
     * @return {*}
     */    
    void newSock();

private:
    int m_sock;
    int m_family;
    int m_type;
    int m_protocol;
    bool m_isConnected;

    Address::ptr m_localAddress;
    Address::ptr m_remoteAddress;

};
/**
 * @brief 流式输出socket
 * @param[in, out] os 输出流
 * @param[in] sock Socket类
 */
std::ostream& operator<<(std::ostream& os, const Socket& sock);

}


#endif