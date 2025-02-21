/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-05-17 14:29:46
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-05-22 11:26:04
 * @FilePath: /mysylar/mytest/test.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include "../mysylar/log.h"
#include "../mysylar/util.h"
using namespace mysylar;
int main(int argc,char** argv){
    Logger::ptr logger(new Logger);
    LogAppender::ptr file_appender(new FileLogAppender());
    LogFormatter::ptr fmt(new LogFormatter("%d%T%p%T%m%n"));
    file_appender->setLevel(LogLevel::Level::ERROR);
    file_appender->setLogFormatter(fmt);
    logger->addAppender(LogAppender::ptr(new StdoutLogAppender()));
    logger->addAppender(file_appender);
    SYLAR_LOG_INFO(logger)<<"测试一下";
    SYLAR_LOG_ERROR(logger) << "出现问题测试";

    auto l = LoggerMgr::GetInstance()->getLogger("xx");
    SYLAR_LOG_INFO(l) << "xxx";
    return 1;
}