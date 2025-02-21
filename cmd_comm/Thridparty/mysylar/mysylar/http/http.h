#ifndef __SYLAR_HTTP_H__
#define __SYLAR_HTTP_H__
#include <memory>
#include <map>
#include <string.h>
#include "sylar.h"
// #include "http/http_parser.h"
namespace mysylar{

namespace http{

/* Request Methods */
#define HTTP_METHOD_MAP(XX)         \
  XX(0,  DELETE,      DELETE)       \
  XX(1,  GET,         GET)          \
  XX(2,  HEAD,        HEAD)         \
  XX(3,  POST,        POST)         \
  XX(4,  PUT,         PUT)          \
  /* pathological */                \
  XX(5,  CONNECT,     CONNECT)      \
  XX(6,  OPTIONS,     OPTIONS)      \
  XX(7,  TRACE,       TRACE)        \
  /* WebDAV */                      \
  XX(8,  COPY,        COPY)         \
  XX(9,  LOCK,        LOCK)         \
  XX(10, MKCOL,       MKCOL)        \
  XX(11, MOVE,        MOVE)         \
  XX(12, PROPFIND,    PROPFIND)     \
  XX(13, PROPPATCH,   PROPPATCH)    \
  XX(14, SEARCH,      SEARCH)       \
  XX(15, UNLOCK,      UNLOCK)       \
  XX(16, BIND,        BIND)         \
  XX(17, REBIND,      REBIND)       \
  XX(18, UNBIND,      UNBIND)       \
  XX(19, ACL,         ACL)          \
  /* subversion */                  \
  XX(20, REPORT,      REPORT)       \
  XX(21, MKACTIVITY,  MKACTIVITY)   \
  XX(22, CHECKOUT,    CHECKOUT)     \
  XX(23, MERGE,       MERGE)        \
  /* upnp */                        \
  XX(24, MSEARCH,     M-SEARCH)     \
  XX(25, NOTIFY,      NOTIFY)       \
  XX(26, SUBSCRIBE,   SUBSCRIBE)    \
  XX(27, UNSUBSCRIBE, UNSUBSCRIBE)  \
  /* RFC-5789 */                    \
  XX(28, PATCH,       PATCH)        \
  XX(29, PURGE,       PURGE)        \
  /* CalDAV */                      \
  XX(30, MKCALENDAR,  MKCALENDAR)   \
  /* RFC-2068, section 19.6.1.2 */  \
  XX(31, LINK,        LINK)         \
  XX(32, UNLINK,      UNLINK)       \
  /* icecast */                     \
  XX(33, SOURCE,      SOURCE)       \
  XX(100,INVALID_METHOD , INVALID_METHOD)

#define HTTP_STATUS_MAP(XX)                                                 \
  XX(100, CONTINUE,                        Continue)                        \
  XX(101, SWITCHING_PROTOCOLS,             Switching Protocols)             \
  XX(102, PROCESSING,                      Processing)                      \
  XX(200, OK,                              OK)                              \
  XX(201, CREATED,                         Created)                         \
  XX(202, ACCEPTED,                        Accepted)                        \
  XX(203, NON_AUTHORITATIVE_INFORMATION,   Non-Authoritative Information)   \
  XX(204, NO_CONTENT,                      No Content)                      \
  XX(205, RESET_CONTENT,                   Reset Content)                   \
  XX(206, PARTIAL_CONTENT,                 Partial Content)                 \
  XX(207, MULTI_STATUS,                    Multi-Status)                    \
  XX(208, ALREADY_REPORTED,                Already Reported)                \
  XX(226, IM_USED,                         IM Used)                         \
  XX(300, MULTIPLE_CHOICES,                Multiple Choices)                \
  XX(301, MOVED_PERMANENTLY,               Moved Permanently)               \
  XX(302, FOUND,                           Found)                           \
  XX(303, SEE_OTHER,                       See Other)                       \
  XX(304, NOT_MODIFIED,                    Not Modified)                    \
  XX(305, USE_PROXY,                       Use Proxy)                       \
  XX(307, TEMPORARY_REDIRECT,              Temporary Redirect)              \
  XX(308, PERMANENT_REDIRECT,              Permanent Redirect)              \
  XX(400, BAD_REQUEST,                     Bad Request)                     \
  XX(401, UNAUTHORIZED,                    Unauthorized)                    \
  XX(402, PAYMENT_REQUIRED,                Payment Required)                \
  XX(403, FORBIDDEN,                       Forbidden)                       \
  XX(404, NOT_FOUND,                       Not Found)                       \
  XX(405, METHOD_NOT_ALLOWED,              Method Not Allowed)              \
  XX(406, NOT_ACCEPTABLE,                  Not Acceptable)                  \
  XX(407, PROXY_AUTHENTICATION_REQUIRED,   Proxy Authentication Required)   \
  XX(408, REQUEST_TIMEOUT,                 Request Timeout)                 \
  XX(409, CONFLICT,                        Conflict)                        \
  XX(410, GONE,                            Gone)                            \
  XX(411, LENGTH_REQUIRED,                 Length Required)                 \
  XX(412, PRECONDITION_FAILED,             Precondition Failed)             \
  XX(413, PAYLOAD_TOO_LARGE,               Payload Too Large)               \
  XX(414, URI_TOO_LONG,                    URI Too Long)                    \
  XX(415, UNSUPPORTED_MEDIA_TYPE,          Unsupported Media Type)          \
  XX(416, RANGE_NOT_SATISFIABLE,           Range Not Satisfiable)           \
  XX(417, EXPECTATION_FAILED,              Expectation Failed)              \
  XX(421, MISDIRECTED_REQUEST,             Misdirected Request)             \
  XX(422, UNPROCESSABLE_ENTITY,            Unprocessable Entity)            \
  XX(423, LOCKED,                          Locked)                          \
  XX(424, FAILED_DEPENDENCY,               Failed Dependency)               \
  XX(426, UPGRADE_REQUIRED,                Upgrade Required)                \
  XX(428, PRECONDITION_REQUIRED,           Precondition Required)           \
  XX(429, TOO_MANY_REQUESTS,               Too Many Requests)               \
  XX(431, REQUEST_HEADER_FIELDS_TOO_LARGE, Request Header Fields Too Large) \
  XX(451, UNAVAILABLE_FOR_LEGAL_REASONS,   Unavailable For Legal Reasons)   \
  XX(500, INTERNAL_SERVER_ERROR,           Internal Server Error)           \
  XX(501, NOT_IMPLEMENTED,                 Not Implemented)                 \
  XX(502, BAD_GATEWAY,                     Bad Gateway)                     \
  XX(503, SERVICE_UNAVAILABLE,             Service Unavailable)             \
  XX(504, GATEWAY_TIMEOUT,                 Gateway Timeout)                 \
  XX(505, HTTP_VERSION_NOT_SUPPORTED,      HTTP Version Not Supported)      \
  XX(506, VARIANT_ALSO_NEGOTIATES,         Variant Also Negotiates)         \
  XX(507, INSUFFICIENT_STORAGE,            Insufficient Storage)            \
  XX(508, LOOP_DETECTED,                   Loop Detected)                   \
  XX(510, NOT_EXTENDED,                    Not Extended)                    \
  XX(511, NETWORK_AUTHENTICATION_REQUIRED, Network Authentication Required) 


/**
 * @brief 方法宏 封装所有的请求方法,比如GET,POST等
 * @description: 
 * @return {*}
 */
enum class HttpMethod {
#define XX(num, name,string) name = num,
    HTTP_METHOD_MAP(XX)
#undef XX  
    HTTP_INVALID_METGOD
};

/**
 * @brief 红方法 封装所有的状态
 * @description: 
 * @return {*}
 */
enum class HttpStatus {
#define XX(code,name ,desc) name = code,
    HTTP_STATUS_MAP(XX)
#undef XX
    HTTP_INVALID_STATUS
};

/**
 * @brief 描述转到数子
 * @param {string&} m
 * @description: 
 * @return {*}
 */
HttpMethod StringToHttpMethod(const std::string& m);
/**
 * @brief 数字和描述互转 GET<-> 1
 * @param {HttpMethod&} m
 * @description: 
 * @return {*}
 */
HttpMethod CharsToHttpMethod(const char* m);

/**
 * @brief 数字和描述互转 GET<-> 1
 * @param {HttpMethod&} m
 * @description: 
 * @return {*}
 */
const std::string HttpMethodToString(const HttpMethod& m);
/**
 * @brief 数字和描述互转 GET<-> 1
 * @param {HttpMethod&} m
 * @description: 
 * @return {*}
 */
const char* HttpMethodToChar(const HttpMethod& m);


/**
 * @brief 描述转到数子
 * @param {string&} m
 * @description: 
 * @return {*}
 */
HttpStatus StringToHttpStatus(const std::string& m);
/**
 * @brief 数字和描述互转 GET<-> 1
 * @param {HttpMethod&} m
 * @description: 
 * @return {*}
 */
HttpStatus CharsToHttpStatus(const char* m);

/**
 * @brief 数字和描述互转 GET<-> 1
 * @param {HttpMethod&} m
 * @description: 
 * @return {*}
 */
std::string HttpStatusToString(const HttpStatus& m);
/**
 * @brief 数字和描述互转 GET<-> 1
 * @param {HttpMethod&} m
 * @description: 
 * @return {*}
 */
const char* HttpStatusToChar(const HttpStatus& m);
/**
 * @brief 字段的排序按照忽略的大小写字符段顺序
 * @description: 
 * @return {*}
 */
struct CaseInsensitiveLess
{ 
  /**
   * @brief  忽略大小写的比较
   * @param {string&} lhs
   * @param {string&} rhs
   * @description: 
   * @return {*}
   */  
  bool operator()(const std::string& lhs ,const std::string& rhs) const;
};
/**
 * @brief 从 maptype 里面转换类型，string 转 bool 或者是其他等
 * @description: 
 * @return {*}
 */  
template<class T>
bool checkGetAs(const std::map<std::string, std::string
  , mysylar::http::CaseInsensitiveLess>& m
  ,const std::string& key,T& val,const T& def = T()){
  std::string str;
  auto& it =m.find(key);
  if(it == m.end()){
    val = def;
    return false;
  }
  try{
    val = boost::lexical_cast<T>(it->second);
    return true;
  }catch(...){
    val = def;
  }
  return false;
}

template<class T>
T getAs(const std::map<std::string, std::string
  , mysylar::http::CaseInsensitiveLess>& m
  ,const std::string& key,const T& def = T()){
  auto it = m.find(key);
  if(it == m.end()){
    return def;
  }
  try
  {
    return boost::lexical_cast<T>(it->second);
  }
  catch(...){}
  return def;
}



/**
 * @brief 对于 http 请求的封装 
 * @description: 
 * uri: http://www.sylar.top:80/page/xxx?id=10&v=20*fr
 *  www.sylar.top host
 *  80 : 端口
 *  /page/xxx ： path
 *  id=10&v=20*fr ： query
 * @return {*}
 */
class HttpRequest{
public:
  typedef std::shared_ptr<HttpRequest> ptr;
  typedef std::map<std::string,std::string,CaseInsensitiveLess> MapType;
public:
  HttpRequest(uint8_t version = 0x11,bool close = true);
  HttpMethod getMethod(){return m_method;}
  HttpStatus getStatus(){return m_status;}

  const std::string& getPath(){ return m_path;}
  const std::string& getQuery(){ return m_query;}
  const std::string& getBody(){ return m_body; }
  const uint8_t& getVersion(){ return m_version; }

  const MapType& getHeaders() { return m_headers;}
  const MapType& getParams() { return m_params;}
  const MapType& getCookies() { return m_cookies;}

  void setMethod(HttpMethod method){ m_method = method;}
  void setStatus(HttpStatus status){ m_status = status;}
  void setVersion(uint8_t v){m_version = v;}

  void setPath(std::string path){m_path = path;}
  void setQuery(std::string query){m_query = query;}
  void setFragment(std::string fragment){m_fragment = fragment;}
  void setBody(std::string body){m_body = body;}

  void setHeaders(const MapType& headers) { m_headers = headers;}
  void setParams(const MapType& params) { m_params = params;}
  void setCookies(const MapType& cookies) { m_cookies = cookies;}

  std::string getHeaders(const std::string& key,const std::string& def = "") const;
  std::string getParams(const std::string& key,const std::string& def = "") const;
  std::string getCookies(const std::string& key,const std::string& def = "") const;

  bool isClose(){return m_close;}
  void setClose(bool v){m_close = v;}

  void setHeaders(const std::string& key,const std::string& val);
  void setParams(const std::string& key,const std::string& val);
  void setCookies(const std::string& key,const std::string& val);

  void delHeaders(const std::string& key);
  void delParams(const std::string& key);
  void delCookies(const std::string& key);

  bool hasHeaders(const std::string& key,std::string* val = nullptr);
  bool hasParams(const std::string& key,std::string* val = nullptr);
  bool hasCookies(const std::string& key,std::string* val = nullptr);

  std::ostream& dump(std::ostream& os);
  std::string toString();
  /**
   * @brief 检查并获取 Header 里面的字段 
   * @description: 
   * @return {*}
   */  
  template<class T>
  bool checkGetHeaderAs(const std::string& key,T& val,const T& def = T());

  template<class T>
  T getHeaderAs(const std::string& key, const T& def = T()){
    return getAs(m_headers,key,def);
  }
  /**
   * @brief 检查并获取 Params 里面的字段 
   * @description: 
   * @return {*}
   */  
  template<class T>
  bool checkGetParamsAs(const std::string& key,T& val,const T& def = T());
  /**
   * @brief 检查并获取 Cookies 里面的字段 
   * @description: 
   * @return {*}
   */  
  template<class T>
  bool checkGetCookiesAs(const std::string& key,T& val,const T& def = T());



private:

  HttpMethod m_method;  // 请求方法
  HttpStatus m_status;  // 请求状态
  uint8_t m_version;    // HTTP 版本
  bool m_close;         // 长连接
  // 格式: [scheme:][//host:port][path][?query][#fragment]
  // 例子：http://www.java2s.com:8080/yourpath/fileName.htm?stove=10&path=32&id=4#harvic
  std::string m_path;     // 请求路径
  std::string m_query;    // ？之后的数据
  std::string m_fragment; // # 之后的数据
  std::string m_body;     // 请求的主体数据

  MapType m_headers;  // header的头部字段
  MapType m_params;   // params 
  MapType m_cookies;  // cookies 的字段

};
class HttpResponse{
public:
  typedef std::shared_ptr<HttpResponse> ptr;
  typedef std::map<std::string,std::string,CaseInsensitiveLess> MapType;
  
private:
  HttpStatus m_status;  // 状态码
  uint8_t m_version;    // Http 版本
  bool m_close;         // 长连接标识
  
  std::string m_body;   // 主体
  std::string m_reason; // 状态码 翻译结果

  MapType m_headers;    // 头部字段
public:
  HttpResponse(uint8_t version = 0x11,bool close = true);
  HttpStatus getStatus(){return m_status;}
  uint8_t getVersion(){return m_version;}
  const std::string& getBody(){return m_body;}
  const std::string& getReason(){return m_reason;}
  const MapType& getHeaders(){return m_headers;}
  bool isClose(){return m_close;}

  void setStatus(HttpStatus v){ m_status = v;}
  void setVersion(uint8_t v){ m_version = v;}
  void setBody(const std::string& v){ m_body = v;}
  void setReason(const std::string& v){ m_reason = v;}
  void setHeaders(const MapType& v){m_headers = v;}
  void setHeaders(const std::string& key,const std::string& val);
  void setClose(bool v){m_close = v;}

  std::string getHeaders(const std::string& key,const std::string& def = "") const;
  void delHeaders(const std::string& key);
  bool hasHeaders(const std::string& key,std::string* val = nullptr);
  
  template<class T>
  bool checkGetHeaderAs(const std::string& key,T& val,const T& def = T()){
    return checkGetAs(m_headers,key,val,def);
  }
  template<class T>
  T getHeaderAs(const std::string& key, const T& def = T()){
    return getAs(m_headers,key,def);
  }

  std::ostream& dump(std::ostream& os);
  std::string toString();

};

static std::ostream& operator<<(std::ostream& os,HttpRequest& req){
  return req.dump(os);
}
static std::ostream& operator<<(std::ostream& os,HttpResponse& rsp){
  return rsp.dump(os);
}

}
}


#endif