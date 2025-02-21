/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-06-20 15:38:17
 * @LastEditors: Johnathan 2440877322@qq.com
 * @LastEditTime: 2024-07-06 12:53:57
 * @FilePath: /mysylar/mysylar/macro.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __MYSYLAR_MARCRO_HPP__
#define __MYSYLAR_MARCRO_HPP__
#include <log.h>
#include <util.h>
#include <assert.h>
// ？？？__GNUC__ __llvm__分别代表什么？
// UNLIKELY小概率事件触发，提升流水线分支预测准确率
#if defined __GNUC__ || defined __llvm__
#    define SYLAR_LIKELY(x) __builtin_expect(!!(x),1)
#    define SYLAR_UNLIKELY(x) __builtin_expect(!!(x),0)
#else 
#   define SYLAR_LIKELY(x) (x)
#   define SYLAR_UNLIKELY(x) (x)
#endif
// 宏定义
#define SYLAR_ASSERT(x) \
    if( SYLAR_UNLIKELY(!(x))){ \
        SYLAR_LOG_ERROR(SYLAR_LOG_ROOT()) << "ASSERTION: "<< #x  \
            << "\nbacktrace:\n"  \
            << mysylar::BacktraceToString(100,2,"\t"); \
        assert(x); \
    }

#define SYLAR_ASSERT2(x,w) \
    if(SYLAR_UNLIKELY(!(x))){ \
        SYLAR_LOG_ERROR(SYLAR_LOG_ROOT()) << "ASSERTION: "<< #x<< " "  << w \
            << "\nbacktrace:\n"  \
            << mysylar::BacktraceToString(100,2,"\t"); \
        assert(x); \
    }

#endif