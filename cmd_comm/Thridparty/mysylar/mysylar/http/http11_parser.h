/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-03-18 21:44:27
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-18 21:52:18
 * @FilePath: /mysylar/mysylar/http/http11_parser.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef http11_parser_h
#define http11_parser_h

#include "http/http11_common.h"

typedef struct http_parser { 
  int cs;
  size_t body_start;
  int content_len;
  size_t nread;
  size_t mark;
  size_t field_start;
  size_t field_len;
  size_t query_start;
  int xml_sent;
  int json_sent;

  void *data;

  int uri_relaxed;
  field_cb http_field;
  element_cb request_method;
  element_cb request_uri;
  element_cb fragment;
  element_cb request_path;
  element_cb query_string;
  element_cb http_version;
  element_cb header_done;
  
} http_parser;

int http_parser_init(http_parser *parser);
int http_parser_finish(http_parser *parser);
size_t http_parser_execute(http_parser *parser, const char *data, size_t len, size_t off);
int http_parser_has_error(http_parser *parser);
int http_parser_is_finished(http_parser *parser);

#define http_parser_nread(parser) (parser)->nread 

#endif
