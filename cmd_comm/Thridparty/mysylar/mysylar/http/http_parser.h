/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-03-18 22:14:52
 * @LastEditors: Johnathan 2440877322@qq.com
 * @LastEditTime: 2024-06-19 23:13:34
 * @FilePath: /mysylar/mysylar/http/http_parser.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AEHTTP
 */
# ifndef __SYLAR_HTTP_PARSER_H__
# define __SYLAR_HTTP_PARSER_H__
#include "http/http11_parser.h"
#include "http/httpclient_parser.h"
#include "http/http.h"
#include <memory>
namespace mysylar{

namespace http{
/**
 * @brief 对 http_parser 做封装，解析的结果利用 parser 回调函数修改 HttpRequest对象的值 
 * @description: 
 * @return {*}
 */
class HttpRequestParser{
public:
  typedef std::shared_ptr<HttpRequestParser> ptr;
  HttpRequestParser();
  size_t execute(char* data,size_t len);
  int isFinish();
  int hasError();

  HttpRequest::ptr getData(){ return m_data;}
  size_t getContentLength();
  void setError(int v){ m_error = v;}
  http_parser& getParser() {return m_parser;}
public:

  static uint64_t GetHttpRequestBufferSize();
  static uint64_t GetHttpRequestBodySize();

private:
  http_parser m_parser;       // http 有限状态机解析，解析到什么地方会调用什么回调函数，实时的。
  HttpRequest::ptr m_data;
  // 1000 : invaild parser
  // 1001 : invaild version
  // 1002 : invaild field
  int m_error;
};

class HttpResponseParser{
public:
  typedef std::shared_ptr<HttpResponseParser> ptr;
  HttpResponseParser();
  size_t execute(char* data,size_t len);
  int isFinish();
  int hasError();

  HttpResponse::ptr getData(){return m_data;}
  size_t getContentLength();
  void setError(int v){ m_error = v;}
  httpclient_parser& getParser() {return m_parser;}
private:
  httpclient_parser m_parser;     // http 有限状态机解析
  HttpResponse::ptr m_data;
  int m_error;

};


    
}

}


#endif