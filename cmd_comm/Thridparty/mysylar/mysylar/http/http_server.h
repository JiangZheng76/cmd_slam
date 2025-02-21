/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-03-19 15:39:52
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-19 21:20:51
 * @FilePath: /mysylar/mysylar/http_server.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __HTTP_SERVER_H__
#define __HTTP_SERVER_H__
#include "tcp_server.h"
#include "servlet.h"
namespace mysylar{

namespace http{

class HttpServer : public TcpServer{
public:
    typedef std::shared_ptr<HttpServer> ptr;

    HttpServer(bool keetp_alive = false,IOManager* worker = IOManager::GetThis()
        ,IOManager* accept_worker = IOManager::GetThis());
    /**
     * @brief 主执行函数 resquest -> response 
     * @param {ptr} client
     * @description: 
     * @return {*}
     */    
    virtual void handleClient(Socket::ptr client) override;
    bool isKeepalive(){return m_keep_alive;}
    ServletDispatch::ptr getServletDispatch(){return m_dispatch;}
private:
    bool m_keep_alive;
    ServletDispatch::ptr m_dispatch;

};

}

}


#endif
