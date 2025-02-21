/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-05-17 21:45:56
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-02-25 11:13:51
 * @FilePath: /mysylar/mysylar/util.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __MYSYLAR_UTIL_H__
#define __MYSYLAR_UTIL_H__
#include <pthread.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <list>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>
#include <execinfo.h>

namespace mysylar{

pid_t GetThreadId();

u_int64_t GetFiberId();

void showAllNode(std::list<std::pair<std::string,const YAML::Node>> all_node);

void Backtrace(std::vector<std::string>& bt,int size,int skip);

const std::string BacktraceToString(int size,int skip = 2,const std::string& prefix = "");

uint64_t GetCurrentMs();

uint64_t GetCurrentUs();
}


#endif