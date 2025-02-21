/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-03-07 12:32:20
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-17 17:10:47
 * @FilePath: /mysylar/mytest/test_bytearray.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "bytearray.h"
#include "sylar.h"
#include <vector>
#include "hook.h"
static mysylar::Logger::ptr g_logger = SYLAR_LOG_ROOT();
void test() {
#define XX(type, len, write_fun, read_fun, base_len) {\
    std::vector<type> vec; \
    for(int i = 0; i < len; ++i) { \
        vec.push_back(rand()); \
    } \
    mysylar::ByteArray::ptr ba(new mysylar::ByteArray(base_len)); \
    for(auto& i : vec) { \
        ba->write_fun(i); \
    } \
    ba->setPosition(0); \
    for(size_t i = 0; i < vec.size(); ++i) { \
        type v = ba->read_fun(); \
        SYLAR_ASSERT(v == vec[i]); \
    } \
    SYLAR_ASSERT(ba->getReadSize() == 0); \
    SYLAR_LOG_INFO(g_logger) << #write_fun "/" #read_fun \
                    " (" #type " ) len=" << len \
                    << " base_len=" << base_len \
                    << " size=" << ba->getSize(); \
}

    XX(int8_t,  100, writeFint8, readFint8, 1);
    XX(uint8_t, 100, writeFuint8, readFuint8, 1);
    XX(int16_t,  100, writeFint16,  readFint16, 1);
    XX(uint16_t, 100, writeFuint16, readFuint16, 1);
    XX(int32_t,  100, writeFint32,  readFint32, 1);
    XX(uint32_t, 100, writeFuint32, readFuint32, 1);
    XX(int64_t,  100, writeFint64,  readFint64, 1);
    XX(uint64_t, 100, writeFuint64, readFuint64, 1);

    XX(int32_t,  100, writeInt32,  readInt32, 1);
    XX(uint32_t, 100, writeUint32, readUint32, 1);
    XX(int64_t,  100, writeInt64,  readInt64, 1);
    XX(uint64_t, 100, writeUint64, readUint64, 1);
#undef XX

#define XX(type, len, write_fun, read_fun, base_len) {\
    std::vector<type> vec; \
    for(int i = 0; i < len; ++i) { \
        vec.push_back(rand()); \
    } \
    mysylar::ByteArray::ptr ba(new mysylar::ByteArray(base_len)); \
    for(auto& i : vec) { \
        ba->write_fun(i); \
    } \
    ba->setPosition(0); \
    for(size_t i = 0; i < vec.size(); ++i) { \
        type v = ba->read_fun(); \
        SYLAR_ASSERT(v == vec[i]); \
    } \
    SYLAR_ASSERT(ba->getReadSize() == 0); \
    SYLAR_LOG_INFO(g_logger) << #write_fun "/" #read_fun \
                    " (" #type " ) len=" << len \
                    << " base_len=" << base_len \
                    << " size=" << ba->getSize(); \
    ba->setPosition(0); \
    std::string file_name = "/tmp/" #type "_" #len "-" #read_fun ".dat"; \
    SYLAR_LOG_INFO(g_logger) << file_name; \
    SYLAR_ASSERT(ba->writeToFile(file_name)); \
    mysylar::ByteArray::ptr ba2(new mysylar::ByteArray(base_len * 2)); \
    SYLAR_ASSERT(ba2->readFromFile(file_name)); \
    ba2->setPosition(0); \
    SYLAR_ASSERT(ba->toString() == ba2->toString()); \
    SYLAR_ASSERT(ba->getPosition() == 0); \
    SYLAR_ASSERT(ba2->getPosition() == 0); \
}
    XX(int8_t,  100, writeFint8, readFint8, 1);
    XX(uint8_t, 100, writeFuint8, readFuint8, 1);
    XX(int16_t,  100, writeFint16,  readFint16, 1);
    XX(uint16_t, 100, writeFuint16, readFuint16, 1);
    XX(int32_t,  100, writeFint32,  readFint32, 1);
    XX(uint32_t, 100, writeFuint32, readFuint32, 1);
    XX(int64_t,  100, writeFint64,  readFint64, 1);
    XX(uint64_t, 100, writeFuint64, readFuint64, 1);

    XX(int32_t,  100, writeInt32,  readInt32, 1);
    XX(uint32_t, 100, writeUint32, readUint32, 1);
    XX(int64_t,  100, writeInt64,  readInt64, 1);
    XX(uint64_t, 100, writeUint64, readUint64, 1);

#undef XX
}
// uint32_t test_a(){
    // uint32_t必须要初始化？？？
//     int32_t i;
//     int32_t j = 1;
//     i |= j;
//     return i;
// }
void test_fun(){
    int len = 100;
    int base_len = 1;
    std::vector<int32_t> vec; 
    for(int i = 0; i < len; ++i) { 
        vec.push_back(rand()); 
    } 
    mysylar::ByteArray::ptr ba(new mysylar::ByteArray(base_len)); 
    for(auto& i : vec) { 
        ba->writeInt32(i); 
    } 
    ba->setPosition(0); 
    for(size_t i = 0; i < vec.size(); ++i) { 
        int32_t v = ba->readInt32(); 
        SYLAR_ASSERT(v == vec[i]); 
    } 
    SYLAR_ASSERT(ba->getReadSize() == 0); 
    SYLAR_LOG_INFO(g_logger) << "writeInt32/readInt32" 
                    " (int32_t ) len=" << len 
                    << " base_len=" << base_len 
                    << " size=" << ba->getSize(); 
    ba->setPosition(0); 
    std::string file_name("/tmp/int32_t_-readInt32.dat"); 
    SYLAR_LOG_INFO(g_logger) << file_name; 
    SYLAR_ASSERT(ba->writeToFile(file_name)); 
    mysylar::ByteArray::ptr ba2(new mysylar::ByteArray(base_len * 2)); 
    SYLAR_ASSERT(ba2->readFromFile(file_name)); 
    ba2->setPosition(0); 
    while(ba2->getPosition() != ba2->getSize()){
        SYLAR_ASSERT(ba2->readInt32() == ba->readInt32());
    }
    ba->setPosition(0); 
    ba->toString();
    ba2->setPosition(0); 
    auto tmp2 = ba2->toString();
    auto tmp1 = ba->toString();
    // SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << tmp1;
    SYLAR_ASSERT(tmp1 == tmp2); 
    SYLAR_ASSERT(ba->getPosition() == 0); 
    SYLAR_ASSERT(ba2->getPosition() == 0); 
}
class Per {
public:
    void play(){};
    int a;
};
class Ber : public Per{

};
int main(int argc, char** argv) {
    // test();
    // std::vector<int> array;
    // SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << array[4] << " " << array[45];
    // Per p;
    // p.a = 1;
    // Ber ber;
    // ber.play();
    // Per b = std::move(p);
    // std::cout << sizeof(b) << " " << b.a << " " << &b << std::endl;
    // std::cout << sizeof(p) << " " << p.a << " " << &p << std::endl;
    mysylar::Thread* detch_tread = new mysylar::Thread ([](){
        int i=0;
        while(1){
            sleep_f(1);
            // SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "keep running.";
            mysylar::Thread* t = mysylar::Thread::GetThis();
            std::cout << t->getId() << std::endl;     
            std::cout << "keep running" << i++ <<  std::endl;
        }
    },"abc");
    sleep_f(2);
    // detach 之后还在执行
    std::cout << "main:" << detch_tread <<  std::endl;    
    delete detch_tread;
    // sleep_f(10);
    // 主线程死了
    // pthread_exit(pthread_self());
    std::cout << "main2:" << detch_tread <<  std::endl;    
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "main thread cancel.";
    pthread_exit(NULL);
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "unreachable .";
    
    int a = 1;
    std::function<void(int & a)> fun = [](int & a){};
    fun(a);

    return 0; // return 会杀死进程。
}