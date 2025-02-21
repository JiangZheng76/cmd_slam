/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-03-18 22:18:52
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-19 17:49:07
 * @FilePath: /mysylar/mysylar/http/http_parser.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "http_parser.h"
#include "log.h"
#include "config.hpp"

namespace mysylar{

static Logger::ptr g_logger_sys = SYLAR_LOG_ROOT();

// 设置最大的 buffer 和 body 上限，防止别人恶意攻击
static ConfigVar<uint64_t>::ptr g_http_request_buffer_size  
    = Config::Lookup<uint64_t>("http.request.max_buffer_size",1024 * 4,"http request max_buffer_size");
static ConfigVar<uint64_t>::ptr g_http_request_body_size  
    = Config::Lookup<uint64_t>("http.request.max_body_size",64 * 1024 * 1024,"http request max body size");

namespace http{
static uint64_t s_http_request_buffer_size = 0;
static uint64_t s_http_request_body_size = 0;
uint64_t HttpRequestParser::GetHttpRequestBufferSize(){
    return s_http_request_buffer_size;
}
uint64_t HttpRequestParser::GetHttpRequestBodySize(){
    return s_http_request_body_size;
}
/**
 * @brief 参数初始化的结构体
 * @description: 
 * 在main 函数前初始化全局值改变的时候同时改变 s_http_request_buffer_size s_http_request_body_size
 * @return {*}
 */
struct _RequestSizeIniter{
    _RequestSizeIniter(){
        s_http_request_buffer_size = g_http_request_buffer_size->getValue();
        s_http_request_body_size = g_http_request_body_size->getValue();
        g_http_request_buffer_size->addListener([](const uint64_t& ov , uint64_t nv){
            s_http_request_buffer_size = nv;
        });
        g_http_request_body_size->addListener([](const uint64_t& ov , uint64_t nv){
            s_http_request_body_size = nv;
        });
    }
};
static _RequestSizeIniter _requestInit; // 初始化参数

/**
 * @brief parser 解析完成之后的回调函数，返回 对应的字段的 val 和 长度
 * @param {void} *data  ？？？为什会返回我们自己定义的对象呢？
 * @param {char} *at
 * @param {size_t} length
 * @description: 
 * @return {*}
 */    
void on_request_method(void *data, const char *at, size_t length){
    HttpRequestParser* parser = static_cast<HttpRequestParser*>(data);
    HttpMethod m = StringToHttpMethod(std::string(at,length));
    if(m == HttpMethod::INVALID_METHOD){
        SYLAR_LOG_WARN(g_logger_sys) << "invaild http method parser.";
        parser->setError(1000);
        return;
    }
    parser->getData()->setMethod(m);
}
void on_request_uri(void *data, const char *at, size_t length){
    // HttpRequestParser* parser = static_cast<HttpRequestParser*>(data);
    
}
void on_request_path(void *data, const char *at, size_t length){
    HttpRequestParser* parser = static_cast<HttpRequestParser*>(data);
    parser->getData()->setPath(std::string(at,length));
}
void on_request_fragment(void *data, const char *at, size_t length){
    HttpRequestParser* parser = static_cast<HttpRequestParser*>(data);
    parser->getData()->setFragment(std::string(at,length));
}
void on_request_query_string(void *data, const char *at, size_t length){
    HttpRequestParser* parser = static_cast<HttpRequestParser*>(data);
    parser->getData()->setQuery(std::string(at,length));
}
void on_request_http_version(void *data, const char *at, size_t length){
    HttpRequestParser* parser = static_cast<HttpRequestParser*>(data);
    uint8_t version;
    std::string s(at,length);
    if(strcmp(s.c_str(),"HTTP/1.1") == 0){
        version = 0x11;
    }else if(strcmp(s.c_str(),"HTTP/1.0") == 0){
        version = 0x10;
    }else {
        SYLAR_LOG_WARN(g_logger_sys) << "invaild  http request version "
            << s;
        parser->setError(1001);
        return ;
    }
    parser->getData()->setVersion(version);
}
void on_request_header_done(void *data, const char *at, size_t length){
    // HttpRequestParser* parser = static_cast<HttpRequestParser*>(data);
    
}
/**
 * @brief header 的字段 解析，key+val
 * @param {void} *data
 * @param {char} *field 字段key
 * @param {size_t} flen 
 * @param {char} *value 值 val
 * @param {size_t} vlen
 * @description: 
 * @return {*}
 */
void on_request_http_field(void *data, const char *field, size_t flen, const char *value, size_t vlen){
    HttpRequestParser* parser = static_cast<HttpRequestParser*>(data);
    if(flen == 0){
        SYLAR_LOG_WARN(g_logger_sys) << "invaild http field parser.";
        return ;
    }
    parser->getData()->setHeaders(std::string(field,flen),std::string(value,vlen));
}
HttpRequestParser::HttpRequestParser(){
    // 初始化状态机
    http_parser_init(&m_parser);
    m_data.reset(new HttpRequest());
    // 绑定动作函数
    m_parser.request_method = on_request_method;
    m_parser.request_uri = on_request_uri;
    m_parser.request_path = on_request_path;
    m_parser.fragment = on_request_fragment;
    m_parser.query_string = on_request_query_string;
    m_parser.http_version = on_request_http_version;
    m_parser.header_done = on_request_header_done;
    m_parser.http_field = on_request_http_field;
    m_parser.data = this;
    
    m_error = 0;
}
/**
 * @brief 开始执行解析，可能一次解析不完，socket 一次没有发送完
 * @param {char*} data
 * @param {size_t} len
 * @param {size_t} off
 * @description: 
 * @return {*} 1：成功完成，其他：解析的长度
 */
size_t HttpRequestParser::execute(char* data,size_t len){
    // 返回处理的长度
    size_t rt = http_parser_execute(&m_parser,data,len,0);
    // 执行完一次之后会使用回调函数将结果写在对象里面
    // 为了不浪费，直接将已经解析原来的覆盖
    // 一般来说是 用 0将已经解析的覆盖吧？？？
    memmove(data,data + rt,(len- rt));
    return rt;
}
/**
 * @brief 判断是否解析完成，一次解析不完
 * @description: 
 * @return {*}
 */
int HttpRequestParser::isFinish(){
    return http_parser_finish(&m_parser);
}
int HttpRequestParser::hasError(){
    return m_error || http_parser_has_error(&m_parser);
}
size_t HttpRequestParser::getContentLength(){
    return m_data->getHeaderAs<uint64_t>("conetnt-length",0);
}

void on_response_reason_phrase(void *data, const char *at, size_t length){
    HttpResponseParser* parser = static_cast<HttpResponseParser*>(data);
    parser->getData()->setReason(std::string(at,length));
}
void on_response_status_code(void *data, const char *at, size_t length){
    HttpResponseParser* parser = static_cast<HttpResponseParser*>(data);
    // parser->getData()->setStatus(StringToHttpStatus(std::string(at,length)));
    HttpStatus status = (HttpStatus)(atoi(at));
    parser->getData()->setStatus(status);
}
void on_response_http_version(void *data, const char *at, size_t length){
    HttpResponseParser* parser = static_cast<HttpResponseParser*>(data);
    uint8_t version;
    std::string v(at,length);
    if(strcmp(v.c_str(),"HTTP/1.1") == 0){
        version = 0x11;
    }else if(strcmp(v.c_str(),"HTTP/1.0") == 0){
        version = 0x10;
    }else {
        SYLAR_LOG_WARN(g_logger_sys) << "invaild  http request version "
            << v;
        parser->setError(1001);
        return ;
    }
    parser->getData()->setVersion(version);
}
void on_response_header_done(void *data, const char *at, size_t length){
    
}
void on_response_last_chunk(void *data, const char *at, size_t length){
    
}
void on_response_chunk_size(void *data, const char *at, size_t length){
    
}
void on_response_http_field(void *data, const char *field, size_t flen, const char *value, size_t vlen){
    HttpResponseParser* parser = static_cast<HttpResponseParser*>(data);
    if(flen == 0){
        SYLAR_LOG_WARN(g_logger_sys) << "invaild http field parser.";
        return ;
    }
    std::string key(field,flen);
    std::string val(value,vlen);
    parser->getData()->setHeaders(key,val);
}
size_t HttpResponseParser::getContentLength(){
    return m_data->getHeaderAs<uint64_t>("conetnt-length",0);
}
HttpResponseParser::HttpResponseParser(){
    // 初始化解析器
    httpclient_parser_init(&m_parser);
    m_data.reset(new HttpResponse());
    // 绑定动作函数
    m_parser.reason_phrase = on_response_reason_phrase;
    m_parser.status_code = on_response_status_code;
    m_parser.http_field = on_response_http_field;
    m_parser.http_version = on_response_http_version;
    m_parser.header_done = on_response_header_done;
    m_parser.last_chunk = on_response_last_chunk;
    m_parser.chunk_size = on_response_chunk_size;
    m_parser.data = this;

    m_error = 0;
}
size_t HttpResponseParser::execute(char* data,size_t len){
    size_t rt = httpclient_parser_execute(&m_parser,data,len,0);
    memmove(data,data+rt,len - rt);
    return rt;
}
int HttpResponseParser::isFinish(){
    return httpclient_parser_finish(&m_parser);
}
int HttpResponseParser::hasError(){
    return m_error || httpclient_parser_has_error(&m_parser);
}


}

}