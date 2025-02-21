/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-03-16 16:03:14
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-26 10:44:56
 * @FilePath: /mysylar/mysylar/stream.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SYLAR__STREAM_H__
#define __SYLAR__STREAM_H__

#include "sylar.h"
#include "bytearray.h"

namespace mysylar{

class Stream{ // 一个流操作的接口，read 和 write 都是纯虚函数
public:
    Stream(){}
    virtual ~Stream() {}
protected:
    virtual int read(void* buffer,size_t len) = 0;
    virtual int read(ByteArray::ptr buffer,size_t len) = 0;
    virtual int write(void* buffer,size_t len) = 0;
    virtual int write(ByteArray::ptr buffer,size_t len) = 0;

    virtual void close() = 0;
public:
    /**
     * @brief 封装 read,循环读取到一定读完 len 之后再返回
     * @param {void*} buffer
     * @param {size_t} len
     * @description: 
     * @return {*}
     */
    virtual int readFixSize(void* buffer,size_t len);
    /**
     * @brief 封装 read,循环读取到一定读完 len 之后再返回
     * @param {void*} buffer
     * @param {size_t} len
     * @description: 
     * @return {*}
     */
    virtual int readFixSize(ByteArray::ptr ba,size_t len);
    /**
     * @brief 封装 write,循环读取到一定读、写完 len 之后再返回
     * @param {void*} buffer
     * @param {size_t} len
     * @description: 
     * @return {*}
     */
    virtual int writeFixSize(void* buffer,size_t len);
        /**
     * @brief 封装 write,循环读取到一定写完 len 之后再返回
     * @param {void*} buffer
     * @param {size_t} len
     * @description: 
     * @return {*}
     */
    virtual int writeFixSize(ByteArray::ptr ba,size_t len);

};

}


#endif