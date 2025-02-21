/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-06-21 11:42:58
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-06-21 18:29:58
 * @FilePath: /mysylar/mytest/text_util.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <sylar.h>

mysylar::Logger::ptr g_logger = SYLAR_LOG_ROOT();

void testAssert(){
    SYLAR_LOG_INFO(g_logger) << mysylar::BacktraceToString(10);
    SYLAR_ASSERT2( 1 ==0 ,"absdas d");
}

int main(int argc,char** agrv){
    testAssert();
}