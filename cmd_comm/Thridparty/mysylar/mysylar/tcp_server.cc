/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-03-16 15:13:21
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-26 10:59:11
 * @FilePath: /mysylar/mysylar/tcp_server.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AEname
 */
#include "tcp_server.h"


namespace mysylar{

// static Logger::ptr g_logger = SYLAR_LOG_NAME("system");
static Logger::ptr g_logger = SYLAR_LOG_ROOT();
static ConfigVar<uint64_t>::ptr s_recv_timeout 
    = Config::Lookup("tcp.server.recv_timeout",(uint64_t)(60 * 1000 * 2),"tcp server recv timeout");

TcpServer::TcpServer(IOManager* worker,IOManager* accept_worker)
    :m_worker(worker)
    ,m_acceptWorker(accept_worker)
    ,m_recvTimeout(s_recv_timeout->getValue())
    ,m_name("sylar/1.0.0")
    ,m_isStop(true){
}
/**
 * @brief 防止 socket 的泄露
 * @description: 
 * @return {*}
 */
TcpServer::~TcpServer(){
    for(auto& i : m_socks){
        i->close();
    }
    m_socks.clear();
}

bool TcpServer::bind(Address::ptr addr){
    std::vector<Address::ptr> addrs;
    std::vector<Address::ptr> fails;
    addrs.push_back(addr);
    bool isSuccess = bind(addrs,fails);
    return isSuccess;
}
bool TcpServer::bind(const std::vector<Address::ptr>& addrs,std::vector<Address::ptr>& fails){
    for(auto& addr: addrs){
        Socket::ptr sock = Socket::CreateTCP(addr);
        if(!sock->bind(addr)){
            SYLAR_LOG_DEBUG(g_logger) << "bind fail errno = "
                << errno << " strerr = " << strerror(errno)
                << " addr = [" << addr->toString() << "]";
            // bind失败放入失败数组
            fails.push_back(addr);
            continue;
        }
        // ？？？这里不会堵塞么？
        // listen 不会堵塞，只是开启了半链接队列和全连接队列
        if(!sock->listen(10)){
            SYLAR_LOG_DEBUG(g_logger) << "listen fail errno = "
                << errno << "strerr = " << strerror(errno)
                << " addr = [" << addr->toString() << "]";
            // bind失败放入失败数组
            fails.push_back(addr);
            continue;
        }
        m_socks.push_back(sock);
    }
    // ？？？看不懂
    // 直到所有都被 bind 之后再启动
    if(!fails.empty()){
        m_socks.clear();
        for(auto& s : fails){
            SYLAR_LOG_INFO(g_logger)<< "server bind fail " << s->toString();
        }
        return false;
    }
    for(auto& sock : m_socks){
        SYLAR_LOG_INFO(g_logger)<< "server bind success " << *sock;
    }
    return true;
}
    

bool TcpServer::startAccept(Socket::ptr sock){
    // 只要服务器不停止
    while(!m_isStop){
        // 阻塞等待全连接队列
        // 接收一个新的 socket
        Socket::ptr client = sock->accept();
        if(client){
            SYLAR_LOG_INFO(g_logger) << "ACCEPT CLIENT: \n" << *client << "\n";
            client->setRecvTimeout(m_recvTimeout);
            // 将新建立 sock 送到任务执行调度器里面处理
            m_worker->schedule(std::bind(&TcpServer::handleClient
                ,shared_from_this(),client)); // 传递shared_from_this为为了防止 m_work 还没有结束，但是这里的结束了
                // 一般来说 bind 是绑定所属对象的 this 指针的。
        }else {
            SYLAR_LOG_ERROR(g_logger) << "accept errno = " << errno
                << " strerr = " << strerror(errno);
            return false;
        }
    }
    return true;
}
bool TcpServer::start(){
    if(!m_isStop){
        // 判断是否已经开始
        return true;
    }
    m_isStop = false;
    for(auto& sock: m_socks){
        // accept 调度器执行 accept 任务
        sock->setRecvTimeout(m_recvTimeout);
        m_acceptWorker->schedule(std::bind(&TcpServer::startAccept
            ,shared_from_this(),sock));
    }
    return true;
}
void TcpServer::stop(){
    m_isStop = true;
    auto self = shared_from_this();
    // 传一个智能指针，确保 TcpServer 对象一直没有被析构
    m_acceptWorker->schedule([this,self]() {
        for(auto& sock : m_socks){
            sock->cancellAll();
            sock->close();
        }
        m_socks.clear();
    });
}

void TcpServer::handleClient(Socket::ptr client){
    SYLAR_LOG_INFO(g_logger) << "han dleClient() " << *client;
}

}
