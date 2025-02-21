/*
 * @Author: Johnathan 2440877322@qq.com
 * @Date: 2024-06-10 10:27:29
 * @LastEditors: Johnathan 2440877322@qq.com
 * @LastEditTime: 2024-07-06 12:44:05
 * @FilePath: /mysylar/mysylar/http/http_connection.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "http/http_connection.h"
#include "http/http_parser.h"
#include "httpclient_parser.h"
#include "streams/zlib_stream.h"

#include "uri.h"

namespace mysylar
{

    namespace http
    {
        Logger::ptr g_logger = SYLAR_LOG_NAME("system");
        HttpResult::HttpResult(int _result, HttpResponse::ptr rsp, const std::string &_error)
            : m_result(_result), m_rsp(rsp), m_error(_error)
        {
        }
        std::string HttpResult::toString() const
        {
            std::stringstream ss;
            ss << "[HttpResult result=" << m_result
               << " error=" << m_error
               << " response=" << (m_rsp ? m_rsp->toString() : "nullptr")
               << "]";
            return ss.str();
        }

        HttpConnection::HttpConnection(Socket::ptr sock, bool is_owner)
            : SocketStream(sock, is_owner)
        {
        }
        HttpConnection::~HttpConnection()
        {
            SYLAR_LOG_DEBUG(g_logger) << "HttpConnection::~HttpConnection";
        }
        HttpResponse::ptr HttpConnection::recvResponse()
        {
            HttpResponseParser::ptr parser(new HttpResponseParser);
            uint64_t buff_size = HttpRequestParser::GetHttpRequestBufferSize();
            // uint64_t buff_size = 100;
            //  读取 buffer 一个最大 http 请求大小
            std::shared_ptr<char> buffer(
                new char[buff_size + 1], [](char *ptr)
                { delete[] ptr; });
            char *data = buffer.get();
            int offset = 0;
            // 读取字节流
            do
            {
                int len = read(data + offset, buff_size - offset);
                if (len <= 0)
                {
                    close();
                    return nullptr;
                }
                len += offset;
                data[len] = '\0';
                // 一边读取一边解析字节流
                size_t nparse = parser->execute(data, len);
                if (parser->hasError())
                {
                    close();
                    return nullptr;
                }
                offset = len - nparse;
                if (offset == (int)buff_size)
                {
                    close();
                    return nullptr;
                }
                // header 读取完毕
                if (parser->isFinish())
                {
                    break;
                }
            } while (true);
            auto &client_parser = parser->getParser();
            std::string body;

            if (client_parser.chunked)
            { // 这里不是很懂？？？chunked 的作用https://blog.csdn.net/tianyeshiye/article/details/88017268，只有客户端的请求会有 chunked 么？？？
                int len = offset;
                do
                {
                    bool begin = true;
                    // 解析body
                    do
                    {
                        if (!begin || len == 0)
                        {
                            int rt = read(data + len, buff_size - len);
                            if (rt <= 0)
                            {
                                close();
                                return nullptr;
                            }
                            len += rt;
                        }
                        data[len] = '\0';
                        size_t nparse = parser->execute(data, len);
                        if (parser->hasError())
                        {
                            close();
                            return nullptr;
                        }
                        len -= nparse;
                        if (len == (int)buff_size)
                        {
                            close();
                            return nullptr;
                        }
                        begin = false;
                    } while (!parser->isFinish());
                    // len -= 2;

                    SYLAR_LOG_DEBUG(g_logger) << "content_len=" << client_parser.content_len;
                    if (client_parser.content_len + 2 <= len)
                    {
                        body.append(data, client_parser.content_len);
                        memmove(data, data + client_parser.content_len + 2, len - client_parser.content_len - 2);
                        len -= client_parser.content_len + 2;
                    }
                    else
                    {
                        body.append(data, len);
                        int left = client_parser.content_len - len + 2;
                        while (left > 0)
                        {
                            int rt = read(data, left > (int)buff_size ? (int)buff_size : left);
                            if (rt <= 0)
                            {
                                close();
                                return nullptr;
                            }
                            body.append(data, rt);
                            left -= rt;
                        }
                        body.resize(body.size() - 2);
                        len = 0;
                    }
                } while (!client_parser.chunks_done);
            }
            else
            {
                int64_t length = parser->getContentLength();
                if (length > 0)
                {
                    body.resize(length);

                    int len = 0;
                    if (length >= offset)
                    {
                        memcpy(&body[0], data, offset);
                        len = offset;
                    }
                    else
                    {
                        memcpy(&body[0], data, length);
                        len = length;
                    }
                    length -= offset;
                    if (length > 0)
                    {
                        if (readFixSize(&body[len], length) <= 0)
                        {
                            close();
                            return nullptr;
                        }
                    }
                }
            }
            // 解析 body
            if (!body.empty())
            {
                auto content_encoding = parser->getData()->getHeaders("content-encoding");
                SYLAR_LOG_DEBUG(g_logger) << "content_encoding: " << content_encoding
                                          << " size=" << body.size();
                // 解析压缩的内容
                if (strcasecmp(content_encoding.c_str(), "gzip") == 0)
                {
                    auto zs = ZlibStream::CreateGzip(false);
                    zs->write((void *)body.c_str(), body.size());
                    zs->flush();
                    zs->getResult().swap(body);
                }
                else if (strcasecmp(content_encoding.c_str(), "deflate") == 0)
                {
                    auto zs = ZlibStream::CreateDeflate(false);
                    zs->write((void *)body.c_str(), body.size());
                    zs->flush();
                    zs->getResult().swap(body);
                }
                parser->getData()->setBody(body);
            }
            return parser->getData();
        }
        int HttpConnection::sendRequest(HttpRequest::ptr req)
        {
            std::stringstream ss;
            ss << *req;
            std::string data = ss.str();
            // ？？？ writeFixSize 和 write 有什么区别？
            return writeFixSize((void *)data.c_str(), data.size());
        }
        /// @brief 创建一个连接池
        /// @param url
        /// @param vhost
        /// @param max_size
        /// @param max_alive_time
        /// @param max_request
        /// @return
        HttpConnectionPool::ptr HttpConnectionPool::Create(const std::string &url,
                                                           const std::string &vhost,
                                                           uint32_t max_size,
                                                           uint32_t max_alive_time,
                                                           uint32_t max_request)
        {
            Uri::ptr uri = Uri::Create(url);
            if (!uri)
            {
                SYLAR_LOG_ERROR(g_logger) << "invalid uri=" << uri;
                return nullptr;
            }
            return std::make_shared<HttpConnectionPool>(uri->getHost(), vhost, uri->getPort(), uri->getScheme() == "https", max_size, max_alive_time, max_request);
        }

        HttpConnectionPool::HttpConnectionPool(const std::string &url, const std::string &vhost, uint32_t port, bool isHttps, uint32_t max_size, uint32_t max_alive_time, uint32_t max_request)
            : m_host(url),
              m_vhost(vhost),
              m_port(port),
              m_isHttps(isHttps),
              m_maxAliveTime(max_size),
              m_maxRequest(max_alive_time),
              m_maxSize(max_request)
        {
        }

        HttpConnection::ptr HttpConnectionPool::getConnection()
        {
            HttpConnection *ptr;
            uint64_t now_ms = GetCurrentMs();
            std::list<HttpConnection *> invaild_conn;
            MutexType::Lock lk(m_mutex);
            while (!m_conns.empty())
            {
                HttpConnection *conn = m_conns.front();
                if (!conn->isConnected())
                {
                    invaild_conn.push_back(conn);
                    continue;
                }
                if (conn->m_createTime + m_maxAliveTime >= now_ms)
                {
                    invaild_conn.push_back(conn);
                    continue;
                }
                ptr = conn;
                break;
            }
            lk.unlock();
            // 删除无效的链接
            for (auto i : invaild_conn)
            {
                delete i;
            }
            m_totals -= invaild_conn.size();
            // 如果没有符合的链接,创建一个新的连接
            if (!ptr)
            {
                IPAddress::ptr addr = Address::LookupAnyIPAddress(m_host);
                if (!addr)
                {
                    SYLAR_LOG_ERROR(g_logger) << "get addr fail: " << m_host;
                    return nullptr;
                }
                Socket::ptr sock = m_isHttps ? SSLSocket::CreateTCP(addr) : Socket::CreateTCP(addr);

                if (!sock)
                {
                    SYLAR_LOG_ERROR(g_logger) << "create sock fail: " << addr;
                    return nullptr;
                }
                if (!sock->connect(addr))
                {
                    SYLAR_LOG_ERROR(g_logger) << "sock connect fail: " << addr;
                    return nullptr;
                }

                ptr = new HttpConnection(sock);
                ++m_totals;
            }
            // 给智能指针绑定一个释放函数
            // std::placeholders::_1 会默认智能指针本身传进去
            return HttpConnection::ptr(ptr, std::bind(&HttpConnectionPool::ReleasePtr, std::placeholders::_1, this));
        }
        /// @brief 连接池的链接释放函数
        /// @param ptr
        /// @param pool
        void HttpConnectionPool::ReleasePtr(HttpConnection *ptr, HttpConnectionPool *pool)
        {
            // 这里面的逻辑还是离的不是很清楚？？？
            ++ptr->m_request; // http 链接的请求次数
            // 一般都是析构的时候自动释放，所以 isConnected 一般都是true
            // 一般来说没有超时和没有到达最多请求的次数连接池的链接都可复用
            if (!ptr->isConnected() || ((ptr->m_createTime + pool->m_maxAliveTime) >= mysylar::GetCurrentMs()) || (ptr->m_request >= pool->m_maxRequest))
            {
                delete ptr;
                --pool->m_totals;
                return;
            }
            // 发复用链接
            MutexType::Lock lock(pool->m_mutex);
            pool->m_conns.push_back(ptr);
        }

        HttpResult::ptr HttpConnectionPool::doGet(const std::string &url, uint64_t timeout, const std::map<std::string, std::string> &headers, const std::string &body)
        {
            return doRequest(HttpMethod::GET, url, timeout, headers, body);
        }

        HttpResult::ptr HttpConnectionPool::doPost(const std::string &url, uint64_t timeout, const std::map<std::string, std::string> &headers, const std::string &body)
        {
            return doRequest(HttpMethod::POST, url, timeout, headers, body);
        }

        HttpResult::ptr HttpConnectionPool::doRequest(HttpMethod method, const std::string &url, uint64_t timeout, const std::map<std::string, std::string> &headers, const std::string &body)
        {

            HttpRequest::ptr req = std::make_shared<HttpRequest>();
            req->setPath(url);
            req->setMethod(method);
            req->setClose(false); // 是否长连接
            bool has_host = false;
            for (auto &i : headers)
            {
                // 判断头部是否有长连接
                if (strcasecmp(i.first.c_str(), "connection") == 0)
                {
                    if (strcasecmp(i.second.c_str(), "keep-alive") == 0)
                    {
                        req->setClose(false);
                    }
                    continue;
                }
                // 判断是否有 host，没有 host 一般就访问默认的 index.html ？？？
                if (!has_host && strcasecmp(i.first.c_str(), "host") == 0)
                {
                    has_host = !i.second.empty();
                }
                // 配置头部
                req->setHeaders(i.first, i.second);
            }
            // vhost 一般为空
            if (!has_host)
            {
                if (m_vhost.empty())
                {
                    req->setHeaders("Host", m_host);
                }
                else
                {
                    req->setHeaders("Host", m_vhost);
                }
            }
            req->setBody(body);
            return doRequest(req, timeout);
        }

        HttpResult::ptr HttpConnectionPool::doRequest(HttpRequest::ptr req, uint64_t timeout_ms)
        {
            // 从链接池中拿连接
            auto conn = getConnection();
            if (!conn)
            {
                return std::make_shared<HttpResult>((int)HttpResult::Error::POOL_GET_CONNECTION
                    , nullptr, "pool host:" + m_host + " port:" + std::to_string(m_port));
            }
            // 检查套接字是否还合法
            auto sock = conn->getSocket();
            if (!sock)
            {
                return std::make_shared<HttpResult>((int)HttpResult::Error::POOL_INVALID_CONNECTION, nullptr, "pool host:" + m_host + " port:" + std::to_string(m_port));
            }
            // 设置套接字超时时间
            sock->setRecvTimeout(timeout_ms);
            // 发送 httprequest
            int rt = conn->sendRequest(req);
            if (rt == 0)
            {
                return std::make_shared<HttpResult>((int)HttpResult::Error::SEND_CLOSE_BY_PEER, nullptr, "send request closed by peer: " + sock->getRemoteAddress()->toString());
            }
            if (rt < 0)
            {
                return std::make_shared<HttpResult>((int)HttpResult::Error::SEND_SOCKET_ERROR, nullptr, "send request socket error errno=" + std::to_string(errno) + " errstr=" + std::string(strerror(errno)));
            }
            // 接收回复，这个在 hook 环境下是会利用协程 hook 出去的
            auto rsp = conn->recvResponse();
            if (!rsp)
            {
                return std::make_shared<HttpResult>((int)HttpResult::Error::TIMEOUT, nullptr, "recv response timeout: " + sock->getRemoteAddress()->toString() + " timeout_ms:" + std::to_string(timeout_ms));
            }
            return std::make_shared<HttpResult>((int)HttpResult::Error::OK, rsp, "ok");
        }

    }

}