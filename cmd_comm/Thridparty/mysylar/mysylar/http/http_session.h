/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-03-19 14:32:38
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-26 10:51:10
 * @FilePath: /mysylar/mysylar/http_session.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SYLAR_HTTP_SESSION_H__
#define __SYLAR_HTTP_SESSION_H__
#include "streams/socket_stream.h"
#include "http/http_parser.h"
#include <memory>

namespace mysylar{

namespace http{

class HttpSession : public SocketStream{
public:
    typedef std::shared_ptr<HttpSession> ptr;
   
    HttpSession(Socket::ptr sock,bool is_ower = false);
    /**
     * @brief 读请求，并封装parser，解析成 HttpRequest
     * @param {ptr} sock
     * @param {bool} is_ower
     * @description: 
     * @return {*}
     */ 
    HttpRequest::ptr recvRequest();
    /**
     * @brief 发送回复
     * @param {ptr} rsp
     * @description: 
     * @return {*}
     */    
    int sendResponse(HttpResponse::ptr rsp);

};

}

}


#endif
