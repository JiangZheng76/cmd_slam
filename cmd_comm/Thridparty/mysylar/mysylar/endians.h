/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-02-29 14:06:42
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-01 11:13:57
 * @FilePath: /mysylar/mysylar/endian.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SYLAR_ENDIAN_H__
#define __SYLAR_ENDIAN_H__

#define SYLAR_LITTLE_ENDIAN 1
#define SYLAR_BIG_ENDIAN 2

#include <byteswap.h>
#include <stdint.h>
#include  <type_traits>
// traits 技法
// 使用内嵌的模板参数来实现参数推导
// 模板元编程
template<class T >
typename std::enable_if<sizeof(T) == sizeof(uint64_t),T>::type
byteswap(T value){
    return(T)bswap_64((uint64_t)value);
}

template<class T >
typename std::enable_if<sizeof(T) == sizeof(uint32_t),T>::type
byteswap(T value){
    return(T)bswap_32((uint32_t)value);
}

template<class T >
typename std::enable_if<sizeof(T) == sizeof(uint16_t),T>::type
byteswap(T value){
    return(T)bswap_16((uint16_t)value);
}

///根据系统定义的大小端，来实现大小端的转化
// 这样实现的好处是可以跨平台 ？？？ 不是很懂
# if BYTE_ORDER == BIG_ENDIAN
#define SYLAR_BYTE_ORDER SYLAR_BIG_ENDIAN
#else 
#define SYLAR_BYTE_ORDER SYLAR_LITTLE_ENDIAN
#endif

# if SYLAR_BYTE_ORDER ==  SYLAR_BIG_ENDIAN
template<class T>
T swapToLittleEndian(T t){
    return byteswap(t);
}
template<class T>
T swapToBigEndian(T t){
    return t;
}
#else 
template<class T>
T swapToLittleEndian(T t){
    return t;
}
template<class T>
T swapToBigEndian(T t){
    return byteswap(t);
}
#endif

#endif