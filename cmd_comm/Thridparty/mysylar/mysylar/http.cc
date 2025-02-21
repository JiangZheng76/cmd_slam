/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-03-10 09:30:34
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-26 10:31:46
 * @FilePath: /mysylar/mysylar/http.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "http.h"
#include <ostream>
namespace mysylar{
    
namespace http{

bool CaseInsensitiveLess::operator()(const std::string& lhs ,const std::string& rhs) const{
    return strcasecmp(lhs.c_str() ,rhs.c_str()) < 0;
}

HttpMethod StringToHttpMethod(const std::string& m){
    // name 是枚举型字段名字，name 才是 HttpMethod，虽然两个一样
    // num 是字段对应的号码
    #define XX(num,name,string) \
    if(strcasecmp(#string, m.c_str()) == 0){ \
        return HttpMethod::name; \
    }  
    HTTP_METHOD_MAP(XX); 
    #undef XX
    return HttpMethod::INVALID_METHOD;
}
HttpMethod CharsToHttpMethod(const char* m){
    #define XX(num,name,string) \
    if(strcmp(#string, m) == 0){ \
        return HttpMethod::name; \
    }  
    HTTP_METHOD_MAP(XX); 
    #undef XX
    return HttpMethod::INVALID_METHOD;
}

const std::string HttpMethodToString(const HttpMethod& m){
    #define XX(num,name,string) \
    if(m == HttpMethod::name){ \
        return # string; \
    }  
    HTTP_METHOD_MAP(XX); 
    #undef XX
    return "<unknow>";
}
const char* HttpMethodToChar(const HttpMethod& m){
    #define XX(num,name,string) \
    if(m == HttpMethod::name){ \
        return # string; \
    }  
    HTTP_METHOD_MAP(XX); 
    #undef XX
    return "<unknow>";
}

HttpStatus StringToHttpStatus(const std::string& m){
    // name 是枚举型字段名字，name 才是 HttpMethod，虽然两个一样
    // num 是字段对应的号码
    #define XX(num,name,string) \
    if(strcasecmp(#string, m.c_str()) == 0){ \
        return HttpStatus::name; \
    }  
    HTTP_STATUS_MAP(XX); 
    #undef XX
    return HttpStatus::HTTP_INVALID_STATUS;
}
HttpStatus CharsToHttpStatus(const char* m){
    #define XX(num,name,string) \
    if(strcasecmp(#string, m) == 0){ \
        return HttpStatus::name; \
    }  
    HTTP_STATUS_MAP(XX); 
    #undef XX
    return HttpStatus::HTTP_INVALID_STATUS;
}

std::string HttpStatusToString(const HttpStatus& m){
    #define XX(num,name,string) \
    if(m == HttpStatus::name){ \
        return # string; \
    }  
    HTTP_STATUS_MAP(XX); 
    #undef XX
    return "<unknow>";
}
const char* HttpStatusToChar(const HttpStatus& m){
    #define XX(num,name,string) \
    if(m == HttpStatus::name){ \
        return # string; \
    }  
    HTTP_STATUS_MAP(XX); 
    #undef XX
    return "<unknow>";
}

HttpRequest::HttpRequest(uint8_t version ,bool close )
    :m_method(HttpMethod::GET)
    ,m_version(version)
    ,m_close(close){
    m_path = "/index.html";
}

std::string HttpRequest::getHeaders(const std::string& key,const std::string& def) const{
    auto it = m_headers.find(key);
    if(it == m_headers.end()){
        return def;
    }
    return it->second;
}
std::string HttpRequest::getParams(const std::string& key,const std::string& def ) const{
    auto it = m_params.find(key);
    if(it == m_params.end()){
        return def;
    }
    return it->second;
}
std::string HttpRequest::getCookies(const std::string& key,const std::string& def ) const{
    auto it = m_cookies.find(key);
    if(it == m_cookies.end()){
        return def;
    }
    return it->second;
}

void HttpRequest::setHeaders(const std::string& key,const std::string& val){
    m_headers[key] = val;
}
void HttpRequest::setParams(const std::string& key,const std::string& val){
    m_params[key] = val;
}
void HttpRequest::setCookies(const std::string& key,const std::string& val){
    m_cookies[key] = val;
}

void HttpRequest::delHeaders(const std::string& key){
    m_headers.erase(key);
}
void HttpRequest::delParams(const std::string& key){
    m_params.erase(key);
}
void HttpRequest::delCookies(const std::string& key){
    m_cookies.erase(key);
}

bool HttpRequest::hasHeaders(const std::string& key,std::string* val ){
    auto it = m_headers.find(key);
    if(it == m_headers.end()){
        return false;
    }
    val = &it->second;
    return true;
}
bool HttpRequest::hasParams(const std::string& key,std::string* val ){
    auto it = m_params.find(key);
    if(it == m_params.end()){
        return false;
    }
    val = &it->second;
    return true;
}
bool HttpRequest::hasCookies(const std::string& key,std::string* val){
    auto it = m_cookies.find(key);
    if(it == m_cookies.end()){
        return false;
    }
    val = &it->second;
    return true;
}

std::ostream& HttpRequest::dump(std::ostream& os){
    // GET /uri HTTP/1.1
    // Host: www.sylar.top 
    // 
    // 头部
    os << HttpMethodToString(m_method) << " "
        // uri
        << m_path << " "
        << (m_query.empty() ? "" : "?")
        << m_query << " "
        << (m_fragment.empty() ? "" : "#")
        << m_fragment<< " "
        // version
        << " HTTP/"
        << (uint32_t)(m_version>>4) // version 默认写法 0x11
        <<"."
        <<(uint32_t)(m_version & 0x0F)
        <<"\r\n" // \r回车 \n换行
        <<"connection: " << (m_close ? "close":"keep-alive") << "\r\n";
    // 字段
    for(auto& i : m_headers){
        if(strcasecmp(i.first.c_str(),"connection") == 0){
            continue;
        }
        os << i.first << ": " << i.second << "\r\n";
    }
    // body 请求数据
    if(!m_body.empty()){
        os << "content-length: " << m_body.size() << "\r\n\r\n"
            << m_body;
    }
    os << "\r\n";

    return os;
}
std::string HttpRequest::toString(){
    std::stringstream ss;
    dump(ss);
    return std::move(ss.str());
}

HttpResponse::HttpResponse(uint8_t version,bool close)
    :m_status(HttpStatus::OK)
    ,m_version(version)
    ,m_close(close){
}
std::ostream& HttpResponse::dump(std::ostream& os){
    // HTTP 版本 ， 状态码， 状态码原因
    os << "HTTP/"
        <<(uint32_t)(m_version >> 4)
        << "."
        << (uint32_t)(m_version & 0x0F) << " "
        << (uint32_t)m_status << " "
        << (m_reason.empty() ? HttpStatusToString(m_status) : m_reason)
        << "\r\n"
        // 头部字段
        <<"connection: " << (m_close ? "close":"keep-alive") << "\r\n";
    // 头部字段
    for(auto& h : m_headers){
        if(strcasecmp(h.first.c_str(),"connection") == 0){
            continue;
        }
        os << h.first << ": "  << h.second << "\r\n";
    }
    // 主体
    if(!m_body.empty()){
        os << "content-length: "<<m_body.size() << "\r\n\r\n";
        os << m_body;
    }
    os << "\r\n";
    return os;
}
void HttpResponse::setHeaders(const std::string& key,const std::string& val){
    m_headers[key] = val;
}
std::string HttpResponse::getHeaders(const std::string& key,const std::string& def ) const {
    auto it = m_headers.find(key);
    if(it == m_headers.end()){
        return def;
    }
    return it->second;
}
void HttpResponse::delHeaders(const std::string& key){
    m_headers.erase(key);
}

std::string HttpResponse::toString(){
    std::stringstream ss;
    dump(ss);
    return std::move(ss.str());
}

template<class T>
bool HttpRequest::checkGetHeaderAs(const std::string& key,T& val,const T& def ){
    return checkGetAs(m_headers,key,val,def);
}
template<class T>
bool HttpRequest::checkGetParamsAs(const std::string& key,T& val,const T& def ){
    return checkGetAs(m_params,key,val,def);
}

template<class T>
bool HttpRequest::checkGetCookiesAs(const std::string& key,T& val,const T& def ){
    return checkGetAs(m_cookies,key,val,def);
}

}
}