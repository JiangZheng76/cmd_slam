/*
 * @Author: Johnathan 2440877322@qq.com
 * @Date: 2024-06-10 10:27:39
 * @LastEditors: Johnathan 2440877322@qq.com
 * @LastEditTime: 2024-07-06 12:28:59
 * @FilePath: /mysylar/mysylar/http/http_connection.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SYLAR_HTTP_CONNECTION_H__
#define __SYLAR_HTTP_CONNECTION_H__

#include "sylar.h"

#include "http/http.h"
#include "streams/socket_stream.h"


namespace mysylar{
namespace http{

class HttpResult
{
public:
    using ptr = std::shared_ptr<HttpResult>;

    enum class Error{
        OK = 0,
        INVALID_URL = 1,
        INVALID_HOST = 2,
        CONNECT_FAIL = 3,
        SEND_CLOSE_BY_PEER = 4,
        SEND_SOCKET_ERROR = 5,
        TIMEOUT = 6,
        CREATE_SOCKET_ERROR = 7,
        POOL_GET_CONNECTION = 8,
        /// 无效的连接
        POOL_INVALID_CONNECTION = 9,
    };

    HttpResult(int _result,HttpResponse::ptr rsp,const std::string& _error);
    std::string toString() const;

    int m_result;
    HttpResponse::ptr m_rsp;
    std::string m_error;
};

class HttpConnection : public SocketStream{
public:
    using ptr = std::shared_ptr<HttpConnection>;

    HttpConnection(Socket::ptr sock,bool is_owner = true);

    virtual ~HttpConnection();

    HttpResponse::ptr recvResponse();

    int sendRequest(HttpRequest::ptr req); 
    

    uint64_t m_createTime = 0; //  创建链接时间
    uint64_t m_request = 0; // 在连接池中被调用的次数
};

class HttpConnectionPool {
public:
    using ptr = std::shared_ptr<HttpConnectionPool>;
    using MutexType = Mutex;

    static HttpConnectionPool::ptr Create(const std::string& url,
                                        const std::string& vhost,
                                        uint32_t max_size,
                                        uint32_t max_alive_time,
                                        uint32_t max_request);

    HttpConnectionPool(const std::string& url
                        , const std::string& vhost
                        ,uint32_t port
                        ,bool isHttps
                        ,uint32_t max_size
                        ,uint32_t max_alive_time
                        ,uint32_t max_request);
    
    HttpConnection::ptr getConnection();

    HttpResult::ptr doGet(const std::string& url
                        ,uint64_t timeout
                        ,const std::map<std::string,std::string>& headers
                        ,const std::string& body);

    HttpResult::ptr doPost(const std::string& url
                        ,uint64_t timeout
                        , const std::map<std::string,std::string>& headers
                        ,const std::string& body);

    HttpResult::ptr doRequest(HttpMethod method
                        ,const std::string& url
                        ,uint64_t timeout
                        ,const std::map<std::string,std::string>& headers
                        ,const std::string& body);
    
    HttpResult::ptr doRequest(HttpRequest::ptr req,uint64_t timeout);
private:
    static void ReleasePtr(HttpConnection* ptr,HttpConnectionPool* pool);
private:
    std::string m_host;
    std::string m_vhost;
    uint32_t m_port;
    bool m_isHttps;

    uint32_t m_maxAliveTime;
    uint32_t m_maxRequest;
    uint32_t m_maxSize;

    MutexType m_mutex;
    std::list<HttpConnection*>  m_conns;
    std::atomic<int32_t> m_totals=  {0};
};

}

}

#endif