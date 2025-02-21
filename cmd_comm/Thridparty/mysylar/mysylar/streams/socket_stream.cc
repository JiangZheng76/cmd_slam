/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-03-16 16:17:52
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-19 17:18:11
 * @FilePath: /mysylar/mysylar/socket_stream.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AEe
 */
#include "streams/socket_stream.h"

namespace mysylar{

SocketStream::SocketStream(Socket::ptr sock,bool owner)
    :m_socket(sock)
    ,m_owner(owner){

}
SocketStream::~SocketStream(){
    // stream 没有的时候 sock 是否自主释放
    if(m_socket && m_owner){
        m_socket->close();
    }
}
bool SocketStream::isConnected(){
    return m_socket && m_socket->isConnected();
}
void SocketStream::close(){
    if(m_socket){
        m_socket->close();
    }
}
int SocketStream::read(void* buffer,size_t length){
    if(!isConnected()){
        return -1;
    }
    int rt = m_socket->recv(buffer,length);
    // std::string strstr((char*)buffer,length);
    
    return rt;
}
int SocketStream::read(ByteArray::ptr ba,size_t length){
    if(!isConnected()){
        return -1;
    }
    std::vector<iovec> iovs;
    // 从 bytearray 中获取可以写的空间，并且将每一块空间保存在 iov 中
    ba->getWriteBuffers(iovs,length);
    // 从 socket 中将数据写到 iov 里面
    int rt = m_socket->recv(&iovs[0],length);
    if(rt > 0){
        // 写完之后跳帧 bytearray 的位置
        ba->setPosition(ba->getPosition() + rt);
    }
    return rt;
}
int SocketStream::write(void* buffer,size_t length){
    if(!isConnected()){
        return -1;
    }
    return m_socket->send(buffer,length);
    
}
int SocketStream::write(ByteArray::ptr ba,size_t length){
    if(!isConnected()){
        return -1;
    }
    std::vector<iovec> iovs;
    // 读 bytearray 中的数据块，并用 iov 存起来
    ba->getReadBuffers(iovs,length);
    // 将 iov 中的数据块通过 socket 发出去
    int rt = m_socket->send(&iovs[0],length);
    if(rt){
        ba->setPosition(ba->getPosition() + rt);
    }
    return rt;
}
Address::ptr SocketStream::getRemoteAddress(){
    if(m_socket){
        return m_socket->getRemoteAddress();
    }
    return nullptr;
}
Address::ptr SocketStream::getLocalAddress(){
    if(m_socket){
        return m_socket->getLocalAddress();
    }
    return nullptr;
}
std::string SocketStream::getRemoteAddressString(){
    if(m_socket){
        return m_socket->getRemoteAddress()->toString();
    }
    return "";
}
std::string SocketStream::getLocalAddressString(){
    if(m_socket){
        return m_socket->getLocalAddress()->toString();
    }
    return "";
}

void SocketStream::reset(Socket::ptr socket,bool owner){
    if(m_socket){
        m_socket->close();
    }
    m_socket = socket;
    m_owner = owner;
}

}