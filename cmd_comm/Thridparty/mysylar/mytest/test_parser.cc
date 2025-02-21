/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-03-19 00:26:57
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-19 09:47:32
 * @FilePath: /mysylar/mytest/test_parser.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "http/http_parser.h"
#include "log.h"

static mysylar::Logger::ptr g_logger = SYLAR_LOG_ROOT();

const char test_request_data[] = "POST / HTTP/1.1\r\n"
                                "Host: www.sylar.top\r\n"
                                "Content-Length: 10\r\n\r\n"
                                "1234567890";

void test_request() {
    mysylar::http::HttpRequestParser parser;
    std::string tmp = test_request_data;
    size_t s = parser.execute(&tmp[0], tmp.size());
    SYLAR_LOG_ERROR(g_logger) << "\nexecute rt=" << s
        << " has_error=" << parser.hasError()
        << " is_finished=" << parser.isFinish()
        << " total=" << tmp.size()
        << " content_length=" << parser.getContentLength();
    tmp.resize(tmp.size() - s);
    SYLAR_LOG_INFO(g_logger) << "\n" << parser.getData()->toString();
    SYLAR_LOG_INFO(g_logger) << "\n" << tmp;
}

const char test_response_data[] = "HTTP/1.1 200 OK\r\n"
        "Date: Tue, 04 Jun 2019 15:43:56 GMT\r\n"
        "Server: Apache\r\n"
        "Last-Modified: Tue, 12 Jan 2010 13:48:00 GMT\r\n"
        "ETag: \"51-47cf7e6ee8400\"\r\n"
        "Accept-Ranges: bytes\r\n"
        "Content-Length: 81\r\n"
        "Cache-Control: max-age=86400\r\n"
        "Expires: Wed, 05 Jun 2019 15:43:56 GMT\r\n"
        "Connection: Close\r\n"
        "Content-Type: text/html\r\n\r\n"
        "<html>\r\n"
        "<meta http-equiv=\"refresh\" content=\"0;url=http://www.baidu.com/\">\r\n"
        "</html>\r\n";

void test_response() {
    mysylar::http::HttpResponseParser parser;
    std::string tmp = test_response_data;
    size_t s = parser.execute(&tmp[0], tmp.size());
    SYLAR_LOG_ERROR(g_logger) << "\n" << "execute rt=" << s
        << " has_error=" << parser.hasError()
        << " is_finished=" << parser.isFinish()
        << " total=" << tmp.size()
        << " content_length=" << parser.getContentLength()
        << " tmp[s]=" << tmp[s];

    tmp.resize(tmp.size() - s);

    SYLAR_LOG_INFO(g_logger) << "\n" << parser.getData()->toString();
    SYLAR_LOG_INFO(g_logger) << "\n" << tmp;
}

int main(int argc, char** argv) {
    test_request();
    SYLAR_LOG_INFO(g_logger) << "--------------";
    test_response();
    return 0;
}
