/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-03-18 21:19:46
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-18 21:36:29
 * @FilePath: /mysylar/mytest/test_http.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "http/http.h"
#include <iostream>
void test_request(){
    mysylar::http::HttpRequest request;
    request.setHeaders("host","www.jiangzheng.top");
    request.setMethod(mysylar::http::HttpMethod::POST);
    request.setBody("hello jiangzheng");

    request.dump(std::cout) << std::endl;
}
void test_response(){
    mysylar::http::HttpResponse rsp;
    rsp.setHeaders("host","jiangzheng niubi");
    rsp.setBody("jiangzheng very niubi");
    rsp.setStatus(mysylar::http::HttpStatus::BAD_REQUEST);
    rsp.setClose(false);

    rsp.dump(std::cout) << std::endl;
}

int main(){
    test_request();
    test_response();
}