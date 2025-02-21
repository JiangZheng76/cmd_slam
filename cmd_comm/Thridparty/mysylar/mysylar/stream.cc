#include "stream.h"

namespace mysylar{

int Stream::readFixSize(void* buffer,size_t len){
    size_t offset = 0;
    size_t left = len;
    while(left > 0){
        // 要么读 len 的长度，要么读取失败返回 0 或者 -1
        int l = read((char*)buffer+offset,left);
        if(l <= 0){
            return l;
        }
        offset += l;
        left -= l;
    }
    return len;
}

int Stream::readFixSize(ByteArray::ptr ba,size_t len){
    size_t left = len;
    while(left > 0){
        size_t l = read(ba,left);
        if(l <= 0){
            return l;
        }
        left -= l;
    }
    return len;
}
int Stream::writeFixSize(void* buffer,size_t len){
    size_t offset = 0;
    size_t left = len;
    while(left > 0){
        // 要么读 len 的长度，要么读取失败返回 0 或者 -1
        int l = write((char*)buffer+offset,left);
        if(l <= 0){
            return l;
        }
        offset += l;
        left -= l;
    }
    return len;
}
int Stream::writeFixSize(ByteArray::ptr ba,size_t len){
    size_t left = len;
    while(left > 0){
        size_t l = write(ba,left);
        if(l <= 0){
            return l;
        }
        left -= l;
    }
    return len;
}

}