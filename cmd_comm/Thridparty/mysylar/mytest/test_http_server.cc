/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-03-19 16:00:40
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-04-19 13:43:10
 * @FilePath: /mysylar/mytest/test_http_server.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "http_server.h"
#include "log.h"
#include "http.h"
using namespace mysylar;
static mysylar::Logger::ptr g_logger = SYLAR_LOG_ROOT();

#define XX(...) #__VA_ARGS__


mysylar::IOManager::ptr worker;
void run() {
    // g_logger->setLevel(mysylar::LogLevel::INFO);
    //sylar::http::HttpServer::ptr server(new sylar::http::HttpServer(true, worker.get(), sylar::IOManager::GetThis()));
    mysylar::http::HttpServer::ptr server(new mysylar::http::HttpServer(true));
    mysylar::Address::ptr addr = mysylar::Address::LookupAnyIPAddress("0.0.0.0:8020");
    while(!server->bind(addr)) {
        sleep(2);
    }
    server->getServletDispatch()->addServlet("/jiangzheng",[](http::HttpRequest::ptr req
        ,http::HttpResponse::ptr rsp,http::HttpSession::ptr session){
            rsp->setBody(req->toString());
            return 0;
    });
    server->getServletDispatch()->addGlobsServlet("/jiangzheng/*",[](http::HttpRequest::ptr req
        ,http::HttpResponse::ptr rsp,http::HttpSession::ptr session){
            rsp->setBody("Glob:\r\n" + req->toString());
            return 0;
    });

    server->start();
}
class A{
public:
    A(){std::cout << "A constructor." << std::endl;};
    ~A(){std::cout << "A disconstructor." << std::endl;};
    // A(A& b){std::cout << "A copy constructor." << std::endl;}
    // A(A && b){std::cout << "A move constructor." << std::endl;}
    void func(){}
    long f;// 8
    // char a; // 1 4
    // int b; // 4 4
    
    // char c;// 1
    // char d;// 1
    short e;// 2 4
    A operator+ (A b){
        return b;
    }

};
class B : public A{
public:
    B(A a):m_a(a){
        std::cout << " B " << std::endl;
    };
    B(const B& b){std::cout << "B copy constructor." << std::endl;}
    // B(B && b){std::cout << "B move constructor." << std::endl;}
    A m_a;
};
struct test{
    A a;
    B b;
    int c;
};

A&& func(A &a ){
    a.f = 3;
    return std::move(a);
};
// A&& add(A a,A b){
//     return a+b;
// };
int main(int argc, char** argv) {
    // mysylar::IOManager iom(3, false, "main");
    // worker.reset(new mysylar::IOManager(3, false, "worker"));
    // iom.schedule(run);
    // std::vector<B> a(10);
    // B * b ;
    
    // std::cout << sizeof(a) << " sizeof a" << std::endl; 
    // std::cout << sizeof(long) << " sizeof long" << std::endl; 
    
    
    // B b(a);
    // std::cout << b.m_a << std::endl;
    // std::unique_ptr<int> ptr;
    // B c(std::move(a));
    // std::cout << c.m_a << std::endl;
    // A a;
    // A a;
    // a.f = 1;
    // func(a);
    // A b = func(a);
    // A&& c = func(a);
    // std::cout << c.f << b.f ;
    // std::vector<int> va;
    // va.emplace_back(1,2,3);
    // // va.push_back(1,2);
    // std::map<int,int>  vec;
    // for(auto i: vec){
        
    // }
    // std::cout << sizeof(struct test) << std::endl;
    // std::cout << sizeof(A) << std::endl;
    A a; 
    a.e = 2;
    // std::cout << a.e << std::endl;
    A&& b = std::move(a);
    b.func();
    b.e = 1;
    A c;
    // add(a,c);
    // std::cout << a.e << std::endl;
    // std::cout << b.e << std::endl;
    A * m = new A;
    m->e = 1;
    int sad = 10;
    static int staticb = sad;
    std::cout << staticb << std::endl;
    return 0;
}
