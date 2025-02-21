/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-03-05 17:50:40
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-09 17:31:08
 * @FilePath: /mysylar/mytest/test_socket.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "socket.h"
#include "iomanager.hpp"

void test_socket(){
    // mysylar::IPv4Address::ptr addr = 
        // std::dynamic_pointer_cast<mysylar::IPv4Address>(mysylar::IPAddress::Create("www.baidu.com",0));
    // mysylar::IPAddress::ptr addr = mysylar::Address::LookupAnyIPAddress("www.baidu.com");
    mysylar::IPAddress::ptr addr = mysylar::Address::LookupAnyIPAddress("10.0.0.1");
    if(addr){
        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "address = " << addr->toString();
    }else {
        SYLAR_LOG_ERROR(SYLAR_LOG_ROOT()) << "get address fail.";
        return;
    }
    mysylar::Socket::ptr sock = mysylar::Socket::CreateTCP(addr);
    addr->setPort(666);
    while (!sock->connect(addr)){}
    
    // if(sock->connect(addr)){
        const char buf[] = "GET / HTTP/1.0\r\n\r\n";
        if(sock->send(buf,sizeof(buf)) <= 0){
            SYLAR_LOG_ERROR(SYLAR_LOG_ROOT()) << "send error.";
            return ;
        }
        std::string buffers;
        buffers.resize(2048);
        sock->setRecvTimeout(500000);
        while(1){
            
            int rt = sock->recv(&buffers[0],buffers.size());
            if(rt <=0 ){
                SYLAR_LOG_ERROR(SYLAR_LOG_ROOT()) << "recv error.";
                return;
            }
            SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << buffers;
        }
    // }
}

int main(){
    mysylar::IOManager iom(3,false);
    iom.schedule(test_socket);
}