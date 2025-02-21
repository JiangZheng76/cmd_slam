/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-03-19 11:50:26
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-19 14:18:49
 * @FilePath: /mysylar/examples/echo_tcp_server.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "tcp_server.h"
#include "log.h"
#include "iomanager.h"
#include "bytearray.h"
#include "address.h"

using namespace mysylar;
static Logger::ptr g_logger = SYLAR_LOG_ROOT();
class EchoServer : public TcpServer {
public:
    EchoServer(int type);
    void handleClient(Socket::ptr client);

private:
    int m_type = 0;
};

EchoServer::EchoServer(int type)
    :m_type(type) {
}
const char test_response_data[] = "HTTP/1.1 200 OK\r\n"
        "Date: Tue, 04 Jun 2019 15:43:56 GMT\r\n"
        "Server: Apache\r\n"
        "Last-Modified: Tue, 12 Jan 2010 13:48:00 GMT\r\n"
        "ETag: \"51-47cf7e6ee8400\"\r\n"
        "Accept-Ranges: bytes\r\n"
        "Content-Length: 81\r\n"
        "Cache-Control: max-age=86400\r\n"
        "Expires: Wed, 05 Jun 2019 15:43:56 GMT\r\n"
        "Connection: Close\r\n"
        "Content-Type: text/html\r\n\r\n"
        "<html>\r\n"
        "<meta http-equiv=\"refresh\" content=\"0;url=http://www.baidu.com/\">\r\n"
        "</html>\r\n";
void EchoServer::handleClient(mysylar::Socket::ptr client) {
    SYLAR_LOG_INFO(g_logger) << "handleClient " << *client;   
    ByteArray::ptr ba(new mysylar::ByteArray);
    while(true) {
        ba->clear();
        std::vector<iovec> iovs;
        ba->getWriteBuffers(iovs, 1024);
        client->send(test_response_data,sizeof(test_response_data));
        // 这里不知道为什么会堵塞，发送了事件之后 epoll_wait没有反应
        int rt = client->recv(&iovs[0], iovs.size());
        if(rt == 0) {
            SYLAR_LOG_INFO(g_logger) << "client close: " << *client;
            break;
        } else if(rt < 0) {
            SYLAR_LOG_INFO(g_logger) << "client error rt=" << rt
                << " errno=" << errno << " errstr=" << strerror(errno);
            break;
        }
        ba->setPosition(ba->getPosition() + rt);
        ba->setPosition(0);
        //SYLAR_LOG_INFO(g_logger) << "recv rt=" << rt << " data=" << std::string((char*)iovs[0].iov_base, rt);
        if(m_type == 1) {//text 
            std::cout << ba->toString();// << std::endl;
        } else {
            std::cout << ba->toHexString();// << std::endl;
        }
        std::cout.flush();
    }
}

int type = 1;

void run() {
    SYLAR_LOG_INFO(g_logger) << "server type=" << type;
    EchoServer::ptr es(new EchoServer(type));
    auto addr = mysylar::Address::LookupAny("0.0.0.0:8020");
    while(!es->bind(addr)) {
        sleep(2);
    }
    es->start();
}

int main(int argc, char** argv) {
    if(argc < 2) {
        SYLAR_LOG_INFO(g_logger) << "used as[" << argv[0] << " -t] or [" << argv[0] << " -b]";
        return 0;
    }

    if(!strcmp(argv[1], "-b")) {
        type = 2;
    }

    mysylar::IOManager iom(2);
    
    iom.schedule(run);
    return 0;
}
