/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-03-17 18:54:02
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-19 11:49:42
 * @FilePath: /mysylar/mytest/test_tcp_server.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "tcp_server.h"
#include "iomanager.hpp"
#include "log.h"
#include "socket_stream.h"

mysylar::Logger::ptr g_logger = SYLAR_LOG_ROOT();
// static mysylar::ConfigVar<uint64_t>::ptr s
void run() {
    auto addr = mysylar::Address::LookupAnyIPAddress("0.0.0.0:8033");
    //auto addr2 = sylar::UnixAddress::ptr(new sylar::UnixAddress("/tmp/unix_addr"));
    std::vector<mysylar::Address::ptr> addrs;
    addrs.push_back(addr);
    //addrs.push_back(addr2);
    mysylar::TcpServer::ptr tcp_server(new mysylar::TcpServer);
    std::vector<mysylar::Address::ptr> fails;
    while(!tcp_server->bind(addrs, fails)) {
        sleep(2);
    }
    tcp_server->start();
    
}
int main(int argc, char** argv) {
    mysylar::IOManager iom(2,false);
    iom.schedule(run);
    return 0;
}
