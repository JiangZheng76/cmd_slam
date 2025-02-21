/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-02-27 16:40:10
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-02-28 20:55:53
 * @FilePath: /mysylar/mytest/test_hook.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "sylar.h"
#include "hook.h"
#include "iomanager.h"
#include <sys/types.h>          /* See NOTES */
#include <sys/socket.h>
#include <arpa/inet.h>
void test_hook(){
    mysylar::IOManager iom(2,false);
    iom.schedule([](){
        sleep(2);
        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << 2 << "s times up.";
    });
    iom.schedule([](){
        sleep(3);
        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << 3 << "s times up.";
    });
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "test_sleep";
}

void test_socket(){
    int sock = socket(AF_INET,SOCK_STREAM,0);
    sockaddr_in addr;
    memset(&addr,0,sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(9999);
    inet_pton(AF_INET,"157.148.69.74",&addr.sin_addr.s_addr);

    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "begin connect.";
    int rt = connect(sock,(const sockaddr*)&addr,sizeof(addr));
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "connect() rt=" << rt << ", errno="<< errno;
    if(rt < 0){
        SYLAR_LOG_ERROR(SYLAR_LOG_ROOT()) << "connect() error"<< errno;
        return ;
    }
    const char str[] = "GET / HTTP/1.0\r\n\r\n";
    rt = send(sock,str,sizeof(str),0);
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "send() rt=" << rt << ", errno="<< errno;

    std::string buff;
    buff.resize(4096);

    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "begin recv().";
    rt = recv(sock,&buff[0],buff.size(),0);
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "recv() rt=" << rt << ", errno="<< errno;
    if(rt < 0){
        return ;
    }
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << buff;
}


int main(){
    mysylar::IOManager iom(3,false);
    iom.schedule(test_socket);
}