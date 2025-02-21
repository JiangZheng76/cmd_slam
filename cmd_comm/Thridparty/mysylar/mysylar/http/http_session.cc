/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-03-19 14:32:46
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-26 12:05:40
 * @FilePath: /mysylar/mysylar/http_session.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "http/http_session.h"

namespace mysylar{

namespace http{

HttpSession::HttpSession(Socket::ptr sock,bool is_ower)
    :SocketStream(sock,is_ower){

}

HttpRequest::ptr HttpSession::recvRequest(){
    HttpRequestParser::ptr parser(new HttpRequestParser);
    size_t buff_size = HttpRequestParser::GetHttpRequestBufferSize();
    // 头部 buffer 最大只可以是 4KB
    std::shared_ptr<char> buffer(new char[buff_size],[](char* ptr){
        delete[] ptr;
    });
    char* data = buffer.get();
    int offset = 0;
    // 读取头部 buffer 部分
    do {
        int len = read(data + offset, buff_size - offset);
        if(len <= 0) {
            close();
            return nullptr;
        }
        len += offset;
        size_t nparse = parser->execute(data, len);
        if(parser->hasError()) {
            close();
            return nullptr;
        }
        offset = len - nparse;
        if(offset == (int)buff_size) {
            close();
            return nullptr;
        }
        if(parser->isFinish()) {
            break;
        }
    } while(true);
    //  读取 body 部分
    int64_t length = parser->getContentLength();
    if(length  > 0) {
        std::string body;
        body.reserve(length);
        // 这里没有看懂？？？
        // 因为上面读取头部的时候，可能读了一部分的 body，需要将超读的 body 部分写回到 body 后面
        if(length >= offset){
            body.append(data,offset);
        }else {
            body.append(data,length);
        }
        length-=offset;
        if(length > 0){
            // 如果第一次是没有读完 body 的，将剩下的 body 读完
            if(readFixSize(&body[body.size()],length)){
                return nullptr;
            }
        }
        parser->getData()->setBody(body);
    }
    // 返回请求报文对象
    return parser->getData();
}
int HttpSession::sendResponse(HttpResponse::ptr rsp){
    std::stringstream ss;
    // 将对象转化为string
    rsp->dump(ss);
    std::string data = ss.str();
    return writeFixSize((void*)data.c_str(),data.length());
}


}
}