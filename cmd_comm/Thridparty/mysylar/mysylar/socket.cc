#include "socket.h"
#include "fd_manager.h"
#include "hook.h"
#include <sys/types.h>          /* See NOTES */
#include <sys/socket.h>
#include <netinet/tcp.h>
namespace mysylar{

static Logger::ptr g_logger_sys = SYLAR_LOG_ROOT();

Socket::ptr Socket::CreateTCP(Address::ptr addr){
    Socket::ptr sock(new Socket(addr->getFamily(),TCP));
    return sock;
}
Socket::ptr Socket::CreateUDP(Address::ptr addr){
    Socket::ptr sock(new Socket(addr->getFamily(),UDP));
    return sock;
}

Socket::ptr Socket::CreateTCPSocket(){
    Socket::ptr sock(new Socket(IPv4,TCP,0));
    return sock;
}
Socket::ptr Socket::CreateUDPSocket(){
    Socket::ptr sock(new Socket(IPv4,UDP,0));
    return sock;
}

Socket::ptr Socket::CreateTCPSocket6(){
    Socket::ptr sock(new Socket(IPv6,TCP,0));
    return sock;
}
Socket::ptr Socket::CreateUDPSocket6(){
    Socket::ptr sock(new Socket(IPv6,UDP,0));
    return sock;
}
Socket::ptr Socket::CreateUnixTCPSocket(Address::ptr addr){
    Socket::ptr sock(new Socket(addr->getFamily(),TCP,0));
    return sock;
}
Socket::ptr Socket::CreateUnixUDPSockett(){
    Socket::ptr sock(new Socket(Unix,TCP,0));
    return sock;
}

Socket::Socket(int family,int type,int protocol)
    :m_sock(-1)
    ,m_family(family)
    ,m_type(type)
    ,m_protocol(protocol)
    ,m_isConnected(false){
        // newSock();
}
Socket::~Socket(){
    close();
}

int64_t Socket::getSendTimeout(){
    FdCtx::ptr ctx = FdMgr::GetInstance()->get(m_sock);
    if(ctx){
        return ctx->getTimeout(SO_SNDTIMEO);
    }
    return -1;
}
bool Socket::setSendTimeout(int64_t v){
    struct timeval tv;
    tv.tv_sec = v / 1000;
    tv.tv_usec = (v % 1000 ) * 1000;
    return setOption(SOL_SOCKET,SO_SNDTIMEO,tv);
}

int64_t Socket::getRecvTimeout(){
    FdCtx::ptr ctx = FdMgr::GetInstance()->get(m_sock);
    if(ctx){
        return ctx->getTimeout(SO_RCVTIMEO);
    }
    return -1;
}
bool Socket::setRecvTimeout(int64_t v){
    struct timeval tv;
    tv.tv_sec = v / 1000;
    tv.tv_usec = (v % 1000 ) * 1000;
    auto fdctx = FdMgr::GetInstance()->get(m_sock);
    if(fdctx){
        fdctx->setTimeout(SO_RCVTIMEO,v);
    }
    return setOption(SOL_SOCKET,SO_RCVTIMEO,tv);
}

bool Socket::getOption(int level , int option,void *result,socklen_t* len){
    int rt = getsockopt(m_sock,level,option,result,len);
    if(!rt){
        SYLAR_LOG_ERROR(g_logger_sys) << "Socket::getOption getsockopt(" << m_sock << ", level=" << level 
            << ",option=" << option << "result, len) errno=" << std::strerror(errno);
        return false;
    }
    return true;
}

bool Socket::setOption(int level ,int option,const void* result,socklen_t* len){
    int rt = setsockopt(m_sock,level,option,result,*len);
    if(rt){
        SYLAR_LOG_ERROR(g_logger_sys) << "Socket::getOption getsockopt(" << m_sock << ", level=" << level 
            << ",option=" << option << "result, len) errno=" << std::strerror(errno);
        return false;
    }
    return true;
}
/**
 * @brief accept 一个 socket对象
 * @description: 
 * ？？？ 这里面的逻辑不是很懂
 * 1、创建一个新的 sylar::sock 对象，准备接受tcp 链接的新 sock
 * 2、获取  新tcp 链接的 sock fd
 * 3、使用 sock fd 初始化 sylar::sock 对象
 * 4、返回 socket 对象
 * @return {*}
 */
Socket::ptr Socket::accept(){
    Socket::ptr sock(new Socket(m_family,m_type,m_protocol));
    // :: 表示不是自己的accept 
    // 为什么不传一个address 接收那个远端地址？
    // accept 成功之后返回新创建的 sock fd
    int netsocket = ::accept(m_sock,nullptr,nullptr); 
    if(!netsocket){
        SYLAR_LOG_INFO(g_logger_sys)<< "Socket::accept() sock=" << m_sock << " errno=" << errno 
            << "strerrno=" << std::strerror(errno);
        return nullptr;
    }
    if(netsocket>=0 && sock->init(netsocket)){
        return sock;
    }
    return nullptr;
}
/**
 * @brief 使用sock fd 初始化 socket 对象
 * @param {int} sock
 * @description: 
 * 初始化,？？？这个是什么时候用的？
 * @return {*}
 */
bool Socket::init(int sock){
    FdCtx::ptr ctx = FdMgr::GetInstance()->get(sock);
     // sock 即有自己的对象，也有在 fd 中保存起来
    //  创建了
    if(ctx && ctx->isSocket() && !ctx->isClose()){
        m_sock = sock;
        m_isConnected = true;
        // 内部 option 的设置
        initSock();
        // 记录当前这个 sock 的远端地址（客户端）和本地地址（服务器新分配的）
        getLocalAddress();
        getRemoteAddress();
        return true;
    }
    return false;
}
bool Socket::isVaild() const{
    return m_sock != -1;
}
/**
 * @brief 对原来 sock 的 bind 封装
 * @param {ptr} addr
 * @description: 
 * @return {*}
 */
bool Socket::bind(const Address::ptr addr){
    // 判断是否合法
    if(!isVaild()){
        newSock();
        if(SYLAR_UNLIKELY(!isVaild())){
            return false;
        }
    }
    if(addr->getFamily() != m_family){
        SYLAR_LOG_ERROR(g_logger_sys) << "bind.sock.family(" << m_family<<")" 
            << " addr.family(" << addr->getFamily() << ") not equal. addr=" << addr->toString();
        return false;
    }
    // 给 sock 绑定
    if(::bind(m_sock,addr->getAddr(),addr->getAddrlen())){
        SYLAR_LOG_ERROR(g_logger_sys) << "Socket::bind  ::bind("<<m_sock << " ," << addr->toString() << "," << addr->getAddrlen() 
            << ") errno=" << strerror(errno) ;
        return false;
    }
    // 初始化下本地地址
    getLocalAddress();
    return true;
}
bool Socket::connect(const Address::ptr addr, uint64_t timeout_ms){
    if(!isVaild()){
        newSock();
        if(SYLAR_UNLIKELY(!isVaild())){
            return false;
        }
    }
    if(addr->getFamily() != m_family){
        SYLAR_LOG_ERROR(g_logger_sys) << "connect.sock.family(" << m_family<<")" 
            << " addr.family(" << addr->getFamily() << ") not equal. addr=" << addr->toString();
        return false;
    }

    if(timeout_ms == (uint64_t)-1){
        // 没有设定时间的话使用默认时间
        if(::connect(m_sock,addr->getAddr(),addr->getAddrlen())){
            SYLAR_LOG_ERROR(g_logger_sys) << "Socket::connect  ::connect("<<m_sock << " ," << addr->toString() << "," << addr->getAddrlen() 
                << ") errno=" << strerror(errno) ;
            close();
            return false;
        }
    } else {
        if(::connect_with_timeout(m_sock,addr->getAddr(),addr->getAddrlen(),timeout_ms)){
            SYLAR_LOG_ERROR(g_logger_sys) << "Socket::connect  ::connect_with_timeout("<<m_sock << " ," << addr->toString() << "," << addr->getAddrlen() 
                << "," << timeout_ms << ") errno=" << strerror(errno) ;
            close();
            return false;
        }
    }
    // ？？？ 为什么不直接调用 init?
    m_isConnected = true;
    getLocalAddress();
    getRemoteAddress();
    SYLAR_LOG_INFO(g_logger_sys) << "connect to " << getRemoteAddress()->toString();
    return true;
}
/**
 * @brief 开启监听端口
 * @param {int} backlog
 * @description: 
 * ？？？怎么用的？listen 没有重写他会一直堵塞在这里么？（listen 不会堵塞，调用创建半链接队列和全连接队列）
 * @return {*}
 */
bool Socket::listen(int backlog){
    if(!isVaild()){
        SYLAR_LOG_ERROR(g_logger_sys) << "listen error sock=-1";
        return false;
    }
    if(::listen(m_sock,backlog)){
        SYLAR_LOG_ERROR(g_logger_sys) << "::listen(" << m_sock << ", backlog=" 
            << backlog << ") errno=" << errno << " strerrno=" <<strerror(errno) ;
        return false;
    }
    return true;
}
bool Socket::close(){
    if(!m_isConnected && m_sock == -1){
        return true;
    }
    m_isConnected = false;
    if(m_sock!=-1){
        ::close(m_sock);
        m_sock = -1;
        return true;
    }
    return false;
}

int Socket::send(const void* buffer,size_t length,int flags){
    if(isConnected()){
        return ::send(m_sock,buffer,length,flags);
    }
    return -1;
}
int Socket::send(const iovec* buffers,size_t length,int flags){
    if(isConnected()){
        struct msghdr msg;
        memset(&msg,0,sizeof(msghdr));
        // ？？？为什么 const 可以直接改？？这样的话不就可以修改 buffer 里面的内容了么？
        msg.msg_iov = (iovec*)buffers;
        msg.msg_iovlen = length;
        return ::sendmsg(m_sock,&msg,flags);
    }
    return -1;
}
// UDP使用
int Socket::sendTo(const void* buffer,size_t length,const Address::ptr to,int flags){
    if(isConnected()){
        return ::sendto(m_sock,buffer,length,flags,to->getAddr(),to->getAddrlen());
    }
    return -1;
}
int Socket::sendTo(const iovec* buffers,size_t length,const Address::ptr to ,int flags){
        if(isConnected()){
        struct msghdr msg;
        memset(&msg,0,sizeof(msghdr));
        // ？？？为什么 const 可以直接改？？这样的话不就可以修改 buffer 里面的内容了么？
        msg.msg_iov = (iovec*)buffers;
        msg.msg_iovlen = length;
        // ？？？为什么可以添加其他的 addr，作用是什么
        msg.msg_name = (void*)to->getAddr();
        msg.msg_namelen = to->getAddrlen();
        return ::sendmsg(m_sock,&msg,flags);
    }
    return -1;
}

int Socket::recv(void * buffer,size_t length,int flags){
    if(isConnected()){
        return ::recv(m_sock,buffer,length,flags);
    }
    return -1;
}
int Socket::recv(iovec * buffers,size_t length,int flags){
    
    if(isConnected()){
        struct msghdr msg;
        // ？？？为什么需要 memset？？？
        memset(&msg,0,sizeof(msghdr));
        msg.msg_iov = buffers;
        msg.msg_iovlen = length;
        return ::recvmsg(m_sock,&msg,flags);
    }
    return -1;
}
int Socket::recvFrom(void * buffers,size_t length, Address::ptr from ,int flags ){
    if(isConnected()){
        socklen_t len = from->getAddrlen();
        return ::recvfrom(m_sock,buffers,length,flags, from->getAddr(),&len);
    }
    return -1;
}
int Socket::recvFrom(iovec * buffers,size_t length, Address::ptr from ,int flags){
    if(isConnected()){
        struct msghdr msg;
        // ？？？为什么需要 memset？？？
        memset(&msg,0,sizeof(msghdr));
        msg.msg_iov = buffers;
        msg.msg_iovlen = length;
        // 限定了对象的就需要在 name 里面写对象是么？
        msg.msg_name = (void*)from->getAddr();
        msg.msg_namelen = from->getAddrlen();
        // msg里面已经限定了对象了
        return ::recvmsg(m_sock,&msg,flags);
    }
    return -1;
}

Address::ptr Socket::getRemoteAddress(){
    if(m_remoteAddress){
        return m_remoteAddress;
    }

    Address::ptr result;
    switch (m_family)
    {
        case AF_INET:
            result.reset(new IPv4Address());
            break;
        case AF_INET6:
            result.reset(new IPv6Address());
            break;
        case AF_UNIX:
            result.reset(new UnixAdress());
            break;
        default:
            result.reset(new UnkonwAddress(m_family));
            break;
    }
    socklen_t len = result->getAddrlen();
    if(getpeername(m_sock,result->getAddr(),&len)){
        SYLAR_LOG_INFO(g_logger_sys) << "getpeername error sock=" << m_sock << 
            ", errno=" << errno << ", strerrno(" << strerror(errno) << ")" ;
        return Address::ptr(new UnkonwAddress(m_family));
    }
    if(m_family == AF_UNIX){
        // unix 地址长度不一样，需要修改
        UnixAdress::ptr addr = std::dynamic_pointer_cast<UnixAdress>(result);
        addr->setAddrlen(len);
    }
    m_remoteAddress = result;
    return m_remoteAddress;
}
Address::ptr Socket::getLocalAddress(){
    if(m_localAddress){
        return m_localAddress;
    }

    Address::ptr result;
    switch (m_family)
    {
        case AF_INET:
            result.reset(new IPv4Address());
            break;
        case AF_INET6:
            result.reset(new IPv6Address());
            break;
        case AF_UNIX:
            result.reset(new UnixAdress());
            break;
        default:
            result.reset(new UnkonwAddress(m_family));
            break;
    }
    socklen_t len = result->getAddrlen();
    // 获取localAddress
    if(getsockname(m_sock,result->getAddr(),&len)){
        SYLAR_LOG_INFO(g_logger_sys) << "getsockname error sock=" << m_sock << 
            ", errno=" << errno << ", strerrno(" << strerror(errno) << ")" ;
        return Address::ptr(new UnkonwAddress(m_family));
    }
    if(m_family == AF_UNIX){
        // unix 地址长度不一样，需要修改
        UnixAdress::ptr addr = std::dynamic_pointer_cast<UnixAdress>(result);
        addr->setAddrlen(len);
    }
    m_localAddress = result;
    return m_localAddress;
}
/**
 * @brief 获取 sock 当前情况
 * @description: 
 * @return {*}
 */
int Socket::getError(){
    int error = 0;
    socklen_t len = sizeof(error);
    if(getOption(SOL_SOCKET,SO_ERROR,(void*)&error,&len)){
        return -1;
    }
    return error;
}

// ？？？有啥用？？?通过 operator<<封装起来
/**
 * @brief socket 相关的信息都输出来 
 * @param {ostream&} os
 * @description: 
 * @return {*}
 */
std::ostream& Socket::dump(std::ostream& os) const{
    os << "[Socket sock=" << m_sock 
        << " isConnected=" << m_isConnected
        << " family=" << m_family
        << " type=" << m_type
        << " protocol=" << m_protocol;
    if(m_localAddress){
        os << " localAddress : " << m_localAddress->toString(); 
    }
    if(m_remoteAddress){
        os<< " remoteAddress: " << m_remoteAddress->toString();
    }
    os << "]";
    return os;
}

// callback
bool Socket::cancelRead(){
    return IOManager::GetThis()->cancelEvent(m_sock,IOManager::READ);
}
bool Socket::cancelWrite(){
    return IOManager::GetThis()->cancelEvent(m_sock,IOManager::WRITE);
}
bool Socket::cancelAccept(){
    return IOManager::GetThis()->cancelEvent(m_sock,IOManager::READ);
}
bool Socket::cancellAll(){
    return IOManager::GetThis()->cancelAllEvent(m_sock);
}

void Socket::initSock(){
    int val = 1;
    // 一般来说，一个端口释放后会等待两分钟之后才能再被使用，SO_REUSEADDR是让端口释放后立即就可以被再次使用。
    // ？？？ 为什么要等待两分钟呢？什么原理？？？
    // 因为在 wait_time 里面阻塞了
    setOption(SOL_SOCKET,SO_REUSEADDR,val);
    if(m_type == SOCK_STREAM){
        // 如果是 tcp 的话，取消Nagle算法
        // nagle 算法会延迟发送数据：1、等待下一个ACK 报文到了再发 2、mss 满了再发
        // 因为是高并发，我们需要及时发送
        setOption(IPPROTO_TCP,TCP_NODELAY,val);
    }

}
void Socket::newSock(){
    m_sock = socket(m_family,m_type,m_protocol);
    if(SYLAR_LIKELY(m_sock != -1)){
        initSock();
    }else {
        SYLAR_LOG_ERROR(g_logger_sys) << "socket(" << m_family << "," << m_type
            << "," << m_protocol << ") errno=" << errno << ":" << std::strerror(errno);
    }
}
std::ostream& operator<<(std::ostream& os ,const Socket& sock){
    return sock.dump(os);
}

struct _SSLInit {
    _SSLInit() {
        // 初始化 ssl
        // OPENSSL_init_ssl();
        OPENSSL_init_ssl(OPENSSL_INIT_LOAD_SSL_STRINGS | OPENSSL_INIT_LOAD_CRYPTO_STRINGS, NULL);
        //  调试使用，遇到错误的时候加载人类可读的错误字
        SSL_load_error_strings();
        // 添加所有可用的加密算法
        OpenSSL_add_all_algorithms();
    }
};
static _SSLInit s_init;

SSLSocket::SSLSocket(int family,int type,int protocol )
    :Socket(family,type,protocol){
}
/// @brief 主要是工具的作用，返回一个 SSlSocket 的 socket 指针
/// @return 
Socket::ptr SSLSocket::accept(){
    // 先建立一个 sock 链接
    SSLSocket::ptr sock(new SSLSocket(m_family, m_type, m_protocol));
    int newsock = ::accept(m_sock, nullptr, nullptr);
    if(newsock == -1) {
        SYLAR_LOG_ERROR(g_logger_sys) << "accept(" << m_sock << ") errno="
            << errno << " errstr=" << strerror(errno);
        return nullptr;
    }
    sock->m_ctx = m_ctx;
    // 初始化
    if(sock->init(newsock)) {
        return sock;
    }
    return nullptr;
}
SSLSocket::ptr SSLSocket::CreateTCP(Address::ptr addr){
    SSLSocket::ptr ssl_sock(new SSLSocket(addr->getFamily(),TCP));
    return ssl_sock;
}
/// 初始化本地地址
bool SSLSocket::bind(const Address::ptr addr){
    return Socket::bind(addr);
}
/// SSLSocket 和普通 Socket 不同的地方在于建立链接之后还要进行 ssl 握手
bool SSLSocket::connect(const Address::ptr addr, uint64_t timeout_ms){
    bool v = Socket::connect(addr, timeout_ms);
    if(v) {
        /**
        1.	创建并初始化一个SSL上下文。
	    2.	使用该上下文创建一个SSL对象。
	    3.	将一个套接字文件描述符与SSL对象关联。
	    4.	尝试通过SSL握手与服务器建立SSL连接，并将结果存储在变量v中。
        */
        // 创建ssl上下文环境
        m_ctx.reset(SSL_CTX_new(SSLv23_client_method()), SSL_CTX_free);
        // 利用上下文环境创建 ssl
        m_ssl.reset(SSL_new(m_ctx.get()),  SSL_free);
        // 将套接字的文件描述符合 ssl 绑定起来
        SSL_set_fd(m_ssl.get(), m_sock);
        // 建立一个 ssl 加密链接
        v = (SSL_connect(m_ssl.get()) == 1);
    }
    return v;
}
bool SSLSocket::listen(int backlog){
    return Socket::listen(backlog);
}
/// 
bool SSLSocket::close(){
    // SSL 握手不需要断开
    return close();
}
/// 发送使用 SSL_write加密发送数据
int SSLSocket::send(const void* buffer, size_t length, int flags) {
    if(m_ssl) {
        return SSL_write(m_ssl.get(), buffer, length);
    }
    return -1;
}

int SSLSocket::send(const iovec* buffers, size_t length, int flags) {
    if(!m_ssl) {
        return -1;
    }
    int total = 0;
    for(size_t i = 0; i < length; ++i) {
        int tmp = SSL_write(m_ssl.get(), buffers[i].iov_base, buffers[i].iov_len);
        if(tmp <= 0) {
            return tmp;
        }
        total += tmp;
        if(tmp != (int)buffers[i].iov_len) {
            break;
        }
    }
    return total;
}

int SSLSocket::sendTo(const void* buffer, size_t length, const Address::ptr to, int flags) {
    SYLAR_ASSERT(false);
    return -1;
}

int SSLSocket::sendTo(const iovec* buffers, size_t length, const Address::ptr to, int flags) {
    SYLAR_ASSERT(false);
    return -1;
}

int SSLSocket::recv(void* buffer, size_t length, int flags) {
    if(m_ssl) {
        return SSL_read(m_ssl.get(), buffer, length);
    }
    return -1;
}

int SSLSocket::recv(iovec* buffers, size_t length, int flags) {
    if(!m_ssl) {
        return -1;
    }
    int total = 0;
    for(size_t i = 0; i < length; ++i) {
        int tmp = SSL_read(m_ssl.get(), buffers[i].iov_base, buffers[i].iov_len);
        if(tmp <= 0) {
            return tmp;
        }
        total += tmp;
        if(tmp != (int)buffers[i].iov_len) {
            break;
        }
    }
    return total;
}
int SSLSocket::recvFrom(void* buffer, size_t length, Address::ptr from, int flags) {
    SYLAR_ASSERT(false);
    return -1;
}

int SSLSocket::recvFrom(iovec* buffers, size_t length, Address::ptr from, int flags) {
    SYLAR_ASSERT(false);
    return -1;
}

// 是 服务端侧 加载自己的证书用的
bool SSLSocket::loadCertificates(const std::string& cert_file, const std::string& key_file) {
    m_ctx.reset(SSL_CTX_new(SSLv23_server_method()), SSL_CTX_free);
    // 加载证书链文件
    if(SSL_CTX_use_certificate_chain_file(m_ctx.get(), cert_file.c_str()) != 1) {
        SYLAR_LOG_ERROR(g_logger_sys) << "SSL_CTX_use_certificate_chain_file("
            << cert_file << ") error";
        return false;
    }
    // 将私钥文件加载到SSL上下文中。key_file是私钥文件的路径
    if(SSL_CTX_use_PrivateKey_file(m_ctx.get(), key_file.c_str(), SSL_FILETYPE_PEM) != 1) {
        SYLAR_LOG_ERROR(g_logger_sys) << "SSL_CTX_use_PrivateKey_file("
            << key_file << ") error";
        return false;
    }
    // 检查证书和密钥是否匹配
    if(SSL_CTX_check_private_key(m_ctx.get()) != 1) {
        SYLAR_LOG_ERROR(g_logger_sys) << "SSL_CTX_check_private_key cert_file="
            << cert_file << " key_file=" << key_file;
        return false;
    }
    return true;
}
std::ostream& SSLSocket::dump(std::ostream& os) const {
    os << "[SSLSocket sock=" << m_sock
       << " is_connected=" << m_isConnected
       << " family=" << m_family
       << " type=" << m_type
       << " protocol=" << m_protocol;
    if(m_localAddress) {
        os << " local_address=" << m_localAddress->toString();
    }
    if(m_remoteAddress) {
        os << " remote_address=" << m_remoteAddress->toString();
    }
    os << "]";
    return os;
}

// 将建立的链接套接字初始化到自己的服务中,通常是在 accept 之后调用
bool SSLSocket::init(int sock) {
    int v = Socket::init(sock);
    if(v){
        // 初始化 ssl
        m_ssl.reset(SSL_new(m_ctx.get()), SSL_free);
        // 将套接字的文件描述符合 ssl 绑定起来
        SSL_set_fd(m_ssl.get(), m_sock);
        // 与链接发起方，进行 ssl 四次握手
        v= (SSL_accept(m_ssl.get()) == 1 );
    }
    return v;
}


}


