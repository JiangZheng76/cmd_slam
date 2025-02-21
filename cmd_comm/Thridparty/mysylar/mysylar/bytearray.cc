/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-03-06 09:47:27
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-27 15:35:48
 * @FilePath: /mysylar/mysylar/bytearray.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%Aname
 */
#include "bytearray.h"
#include "endians.h"
#include <memory>
#include "string.h"
#include "log.h"
#include <fstream>
#include <sstream>
#include <iomanip>
namespace mysylar{

static Logger::ptr g_logger_sys = SYLAR_LOG_NAME("system");

ByteArray::Node::Node(size_t s)
    :ptr(new char[s])
    ,next(nullptr)
    ,size(s){
}
        
ByteArray::Node::Node()
    :ptr(nullptr)
    ,next(nullptr)
    ,size(0){

}
ByteArray::Node::~Node(){
    if(ptr){
        delete[] ptr;
    }
}

ByteArray::ByteArray(size_t base_size)
    :m_position(0)
    ,m_capacity(base_size)
    ,m_baseSize(base_size)
    ,m_size(0)
    ,m_endian(SYLAR_BIG_ENDIAN)
    ,m_root(new Node(base_size))
    ,m_cur(m_root){

}
ByteArray::~ByteArray(){
    Node* tmp = m_root;
    while (tmp)
    {
        m_cur = tmp;
        tmp = tmp->next;
        delete m_cur;
    }
}

bool ByteArray::isLittleEndian() const{
    return m_endian == SYLAR_LITTLE_ENDIAN;
}
void ByteArray::setIsLitttleEndian(bool val){
    if(val){
        m_endian = SYLAR_LITTLE_ENDIAN;
    }else {
        m_endian = SYLAR_BIG_ENDIAN;
    }
}

/**
 * @brief ？？？固不固定长度有什么区别？
 * @param {int8_t&} value
 * @description: 
 * @return {*}
 */
void ByteArray::writeFint8(int8_t value){
    write( &value , sizeof(value));
}
void ByteArray::writeFuint8(uint8_t value){
    write( &value , sizeof(value));
}
void ByteArray::writeFint16(int16_t value){
    // 为什么需要考虑字节序的问题呢？？？为什么我们平时 write 和 read 不需要考虑呢？
    // 为什么固定长度才会有字节序呢？？？
    if(m_endian != SYLAR_BYTE_ORDER){
        value = byteswap(value);
    }   
    write( &value,sizeof(value));
}
void ByteArray::writeFuint16(uint16_t value){
    if(m_endian != SYLAR_BYTE_ORDER){
        value = byteswap(value);
    }   
    write( &value,sizeof(value));
}
void ByteArray::writeFint32(int32_t value){
    if(m_endian != SYLAR_BYTE_ORDER){
        value = byteswap(value);
    }   
    write( &value,sizeof(value));
}
void ByteArray::writeFuint32(uint32_t value){
    if(m_endian != SYLAR_BYTE_ORDER){
        value = byteswap(value);
    }   
    write( &value,sizeof(value));
}
void ByteArray::writeFint64(int64_t value){
    if(m_endian != SYLAR_BYTE_ORDER){
        value = byteswap(value);
    }   
    write( &value,sizeof(value));
}
void ByteArray::writeFuint64(uint64_t value){
    if(m_endian != SYLAR_BYTE_ORDER){
        value = byteswap(value);
    }   
    write( &value,sizeof(value));
}
// 压缩类型
/**
 * @brief 将有符号转化为无符号？？？具体压缩和解压缩的方法看https://blog.csdn.net/weixin_43708622/article/details/111397290
 * @param {int32_t} v
 * @description: 
 * @return {*}
 */
static uint32_t EncodeZigzag32(const int32_t& v){
    if(v < 0){
        return ((uint32_t)(-v)) * 2 -1;
    }else {
        return v * 2;
    }
}
static uint64_t EncodeZigzag64(const int64_t& v){
    if(v < 0){
        return ((uint64_t)(-v)) * 2 -1;
    }else {
        return v * 2;
    }
}
/**
 * @brief 解无符号数成有符号数，具体的公式解释https://blog.csdn.net/weixin_43708622/article/details/111397290
 * @param {uint32_t&} v
 * @description: 
 * 转化逻辑 0 - 0 ；-1 — 1；1 - 2； -2 - 3
 * @return {*}
 */
static int32_t DecodeZigzag32(const uint32_t& v) {
    return (v >> 1) ^ -(v & 1);
}
static int64_t DecodeZigzag64(const uint64_t& v){
    return (v >> 1) ^ -(v & 1);
}
/**
 * @brief 有符号的需要转化成无符号的
 * @param {int32_t} value
 * @description: 
 * @return {*}
 */
void ByteArray::writeInt32(int32_t value){
    writeUint32(EncodeZigzag32(value));
}
/**
 * @brief 7bit压缩法
 * @param {uint32_t} value
 * @description: https://blog.csdn.net/yzf279533105/article/details/132149559
 * ，怎么还原呢？？？？https://blog.csdn.net/iAm333/article/details/38038879
 * ？？？为什么可变长度不需要管字节序了？
 * @return {*}
 */
void ByteArray::writeUint32(uint32_t value){
    uint8_t tmp[5]; // 最高的上限是 5 字节
    uint8_t i =0;
    // 7bit 压缩法实现思路：
    // 1、 tmp 是一个小端的表示，最低位放最前面
    // 2、 tmp 中的每一位都只有后 7 位才是有效数据，第 8 位表示是否已经结束，1 表示后面还有数据，0 表示已经没有数据了
    while(value >= 0x80){
        // 后 7 位才是表示真实数据的，最高位 0x80只是表示后面还有没有数据
        tmp[i++] = (value & 0x7f) | 0x80;
        // ？？？这样放的话信息不是反而多了么？看后面解码怎么解决
        value >>= 7;
    }
    tmp[i++] = value;
    write( tmp,i);
}
void ByteArray::writeInt64(int64_t value){
    writeUint64(EncodeZigzag64(value));
}
void ByteArray::writeUint64(uint64_t value){
    uint8_t tmp[10];
    uint8_t i = 0;
    while(value >=0x80){
        tmp[i++] = (value & 0x7f) | 0x80;
        value >>= 7;
    }
    tmp[i++] = value;
    write(  tmp,i);
}
/**
 * @brief 将 float 当成是 uint 类型写入 ？？？为什么可以
 * @param {float&} value
 * @description: 
 * @return {*}
 */
void ByteArray::writeFloat(const float& value){
    uint32_t v;
    // float 直接将比特位写入
    memcpy(&v,&value,sizeof(float));
    write( &v,sizeof(uint32_t));
}
void ByteArray::writeDouble(const double& value){
    uint64_t v;
    memcpy(&v,&value,sizeof( double));
    write( &v,sizeof(uint64_t));
}

// 固定 16 位的 int 表示长度，以此类推 如何理解？？
/**
 * @brief Fxx表示使用多少位数来表示 string 的长度， 先写长度（固定），然后再写本体
 * @param {string&} value
 * @description: 
 * @return {*}
 */
void ByteArray::writeStringF16(const std::string& value){
    // 使用固定 16 位来表示 string 的长度
    writeFuint16(value.size());
    
    write(value.c_str(),value.size());
}
void ByteArray::writeStringF32(const std::string& value){
    // 使用固定 32 位来表示 string 的长度
    writeFuint32(value.size());
    write(value.c_str(),value.size());
}
void ByteArray::writeStringF64(const std::string& value){
    // 使用固定 16 位来表示 string 的长度
    writeFuint64(value.size());
    write(value.c_str(),value.size());
}
/**
 * @brief 可变长度的 string，固定使用 64 位来表示 string 长度
 * @param {string&} value
 * @description: 
 * @return {*}
 */
void ByteArray::writeStringVint(const std::string& value){
    // 为什么使用 64 的？？？sylar 说是使用最大长度，不怕他会超过。
    writeFuint64(value.size());
    write(value.c_str(),value.size());
}
/**
 * @brief 不需要先定义长度直接写
 * @param {string&} value
 * @description: 
 * ，那么解包的时候不会造成混乱么？？？
 * @return {*}
 */
void ByteArray::writeStringWithoutLength(const std::string& value){
    write(value.c_str(),value.size());
}

// read
int8_t ByteArray::readFint8(){
    int8_t v;
    read(&v,sizeof(int8_t));
    return v;
}
uint8_t ByteArray::readFuint8(){
    uint8_t v;
    // read读的时候也不用标明类型这么万能的么？？？
    read(&v,sizeof(uint8_t));
    return v;
}

#define XX(type) \
    type v; \
    read(&v,sizeof(v)); \
    if(m_endian == BYTE_ORDER) { \
        return v; \
    }else { \
        return byteswap(v); \
    } 


int16_t ByteArray::readFint16(){
    // 多个字节的话 就需要判断字节序是否和
    XX(int16_t);
}
uint16_t ByteArray::readFuint16(){
    XX(uint16_t);
}
int32_t ByteArray::readFint32(){
    XX(int32_t);
}
uint32_t ByteArray::readFuint32(){
    XX(uint32_t);
}
int64_t ByteArray::readFint64(){
    XX(int64_t);
}
uint64_t ByteArray::readFuint64(){
    XX(uint64_t);
}
#undef XX
/**
 * @brief 读取压缩的数据
 * @description: 
 * @return {*}
 */
int32_t  ByteArray::readInt32() {
    return DecodeZigzag32(readUint32());
}
uint32_t ByteArray::readUint32(){
    uint32_t result = 0;
    // 压缩最大不是 5 个字节么？？？i 应该小于 40 吧？？？
    for(int i=0;i<32;i+=7){
        uint8_t b = readFuint8();
        if(b < 0x80){
            result |= (uint32_t(b)) << i;
            // 最高位不为 0x80了，已经读完整个数据，所以跳出
            break;
        }else {
            result |= (uint32_t(b & 0x7f)) << i;
        }
    }
    return result;
}
int64_t ByteArray::readInt64(){
    return DecodeZigzag64(readUint64());
}
uint64_t ByteArray::readUint64(){
    uint64_t result = 0;
    for(int i=0;i<64;i+=7){
        uint8_t b = readFuint8();
        if(b < 0x80){
            result |= (uint64_t(b)) << i;
            // 最高位不为 0x80了，已经读完整个数据，所以跳出
            break;
        }else {
            result |= (uint64_t(b & 0x7f)) << i;
        }
    }
    return result;
}
float ByteArray::readFloat(){
    float result;
    uint32_t v = readFuint32();
    memcpy(&result,&v,sizeof(uint32_t));
    return result;
}
double ByteArray::readDouble(){
    double result;
    uint64_t v = readFuint64();
    memcpy(&result,&v,sizeof(uint64_t));
    return result;
}
/**
 * @brief 读取固定长度的 string，传输格式是 len ， string
 * @description: 
 * @return {*}
 */
std::string ByteArray::readStringF16(){
    uint16_t len = readFint16();
    std::string buff;
    buff.resize(len);
    read(&buff[0],len);
    return buff;
}
std::string ByteArray::readStringF32(){
    uint32_t len = readFint32();
    std::string buff;
    buff.resize(len);
    read(&buff[0],len);
    return buff;
}
std::string ByteArray::readStringF64(){
    uint64_t len = readFint64();
    std::string buff;
    buff.resize(len);
    read(&buff[0],len);
    return buff;
}
/**
 * @brief 可变的默认标识长度也是 64 位，格式 uint64_t string
 * @description: 
 * @return {*}
 */
std::string ByteArray::readStringVint(){
    uint64_t len = readFint64();
    std::string buff;
    buff.resize(len);
    read(&buff[0],len);
    return buff;
}
// std::string ByteArray::readStringWithoutLength(){

// }
/**
 * @brief 和析构差不多，将数据清理掉，只留下一个节点，什么时候使用？？
 * @description: 
 * @return {*}
 */
void ByteArray::clear(){
    m_position = m_size = 0;
    m_capacity = m_baseSize;
    Node* tmp = m_root->next;
    while(tmp) {
        m_cur = tmp;
        tmp = tmp->next;
        delete m_cur;
    }
    m_cur = m_root;
    m_cur->next = nullptr;
}
/**
 * @brief 将内容写入块中
 * @param {void*} buf
 * @param {size_t} size
 * @description: 
 * @return {*}
 */
void ByteArray::write(const void* buf,size_t size){
    if(size == 0){
        return;
    }
    // 保证空间足够
    addCapacity(size);

    // 块中位置
    size_t npos = m_position % m_baseSize;
    // 当前块剩余空间
    size_t ncap = m_cur->size - npos;
    // buf中的位置
    size_t bpos = 0;
    while(size > 0){
        if(ncap >= size){
            memcpy(m_cur->ptr + npos,(const char*)buf + bpos , size);
            if((size + npos) == m_cur->size){
                // 当前的写满了，指针自动跳到下一个 node
                m_cur = m_cur->next;
            }
            m_position += size;
            bpos += size;
            ncap -= size;
            size = 0;
        }else{
            memcpy(m_cur->ptr + npos,(char*)buf + bpos,ncap);
            m_position += ncap;
            bpos += ncap;
            size -= ncap;
            m_cur = m_cur->next;
            ncap = m_cur->size;
            npos = 0;
        }
    }

    if(m_position > m_size){
        m_size = m_position;
    }
}
/**
 * @brief read 是用于固定长度的读写的，如果长度不够的话表明不能正确表示和这个 uinxx_t
 * @param {void*} buf
 * @param {size_t} size
 * @description: 
 * 不是应该有一个读指针和一个写指针的么？这里实现成和 write 一样不是会导致m_position指针混乱么？？？
 * @return {*}
 */
void ByteArray::read(void* buf,size_t size){
    // 为什么是大于？大于不是正常的么？小于不是读不完嘛？？？？
    // 因为 read 是用于固定长度的读写的，如果长度不够的话表明不能正确表示和这个 uinxx_t
    // 从 root 节点开始往后读
    if(size > getReadSize()){
        throw std::out_of_range("not enough len");
    }

    size_t npos = m_position % m_baseSize;
    size_t ncap = m_cur->size - npos;
    // buf postion 
    size_t bpos = 0;

    while(size > 0){
        if(ncap >= size ){
            memcpy((char*)buf + bpos,m_cur->ptr + npos,size);
            if(m_cur->size == npos + size){
                // 如果读完之后当前 node 满了增加下一个
                // 你怎么知道够不够空间？？？因为这个是读？？
                m_cur = m_cur->next;
            }
            m_position += size;
            bpos -=size;
            npos += size;
            size = 0;
        }else {
            memcpy((char*)buf + bpos,m_cur->ptr + npos,ncap);
            size -= ncap;
            m_position += ncap;
            bpos += ncap;
            m_cur = m_cur->next;
            ncap = m_cur->size;
            npos = 0;
        }
    }
}

/**
 * @brief 从 position 开始读内容，而且不会改变 ByteArray 成员变量
 * @param {void*} buf
 * @param {size_t} size
 * @param {size_t} position
 * @description: 
 * @return {*}
 */
void ByteArray::read(void* buf,size_t size, size_t position) const{
    if(size > getReadSize()){
        throw std::out_of_range("not enough len");
    }
    Node* tmp_cur = m_cur;
    size_t npos = position % m_baseSize;
    size_t ncap = tmp_cur->size - npos;
    // buf postion 
    size_t bpos = 0;

    while(size > 0){
        if(ncap >= size ){
            memcpy((char*)buf + bpos,tmp_cur->ptr + npos,size);
            if(tmp_cur->size == npos + size){
                // 如果读完之后当前 node 满了增加下一个
                // 你怎么知道够不够空间？？？因为这个是读？？
                tmp_cur = tmp_cur->next;
            }
            position += size;
            bpos +=size;
            size = 0;
            npos += size;
        }else {
            memcpy((char*)buf + bpos,tmp_cur->ptr + npos,ncap);
            size -= ncap;
            position += ncap;
            bpos += ncap;
            tmp_cur = tmp_cur->next;
            ncap = tmp_cur->size;
            npos = 0;
        }
    }
}

void ByteArray::setPosition(size_t v){
    if(v > m_size){
        throw std::out_of_range("set_position out of range.");
    }
    m_position = v;
    m_cur = m_root;
    while(v >= m_cur->size){
        v -= m_cur->size;
        m_cur = m_cur->next;
    }
    if(v == m_cur->size){
        m_cur = m_cur->next;
    }
}
/**
 * @brief 将m_position之后的数据都写入到 file 里面去 
 * @param {string&} name
 * @description: 
 * @return {*}
 */
bool ByteArray::writeToFile(const std::string& name) const{
    std::ofstream ofs;
    // 以二进制形式写
    ofs.open(name,std::ios::trunc | std::ios::binary);
    if(!ofs){
        SYLAR_LOG_ERROR(g_logger_sys) << "ByteArray::writeToFile ofstream open("<< name <<") error "
            << "errono=" << errno << " strerrno(" << strerror(errno) << ")";
        return false;
    }
    // read_size 到底指什么？？？剩余的总容量
    int64_t read_size = getReadSize();
    // 这个 m_position 到底是跟谁走的？？？
    int64_t pos = m_position;
    Node* cur = m_cur;
    // 以块为单位进行读写
    while(read_size > 0){
        int64_t diff = pos % m_baseSize;
        // 这里有歧义？？？和作者不一样
        int64_t len = (read_size > (int64_t)cur->size - diff ) ? cur->size - diff : read_size;
        // int64_t len = (read_size > (int64_t)m_baseSize ? m_baseSize : read_size) - diff;
        ofs.write(cur->ptr + diff,len);
        cur = cur->next;
        pos += len;
        read_size -= len;
    }
    return true;
}
/**
 * @brief 读取 name 文件中所有的数据，并 write（） 到我们的 ByteArray 中 
 * @param {string&} name
 * @description: 
 * @return {*}
 */
bool ByteArray::readFromFile(std::string& name){
    std::ifstream ifs;
    ifs.open(name ,std::ios::binary);
    if(!ifs){
        SYLAR_LOG_ERROR(g_logger_sys) << "ByteArray::readFromFile ifstream open("<< name <<") error "
            << "errono=" << errno << " strerrno(" << strerror(errno) << ")";
        return false;
    }
    std::shared_ptr<char> buff(new char[m_baseSize],[](char* ptr){ delete[] ptr;});
    while (!ifs.eof()){
        // 也是以块为单位读取
        ifs.read(buff.get(),m_baseSize);
        // gcount 返回的是上一次读到的长度
        write(buff.get(),ifs.gcount());
    }
    return true;
}
/**
 * @brief 给 ByteArray 分配容量，确保我们有足够多的容量进行写入操作
 * @param {size_t} size
 * @description: 
 * @return {*}
 */
void ByteArray::addCapacity(size_t size){
    if(size == 0){
        return ;
    }
    size_t old_cap = getCapacity();
    if(old_cap > size){
        return ;
    }
    size = size - old_cap;
    // size_t count = (size / m_baseSize) + ((size % m_baseSize) ? 1 : 0);
    // 当size 和 old_cap 两者相等的时候也需要扩充一个 Node
    size_t count = (size / m_baseSize) + 1;
    Node* tmp = m_root;
    while( tmp ->next){
        tmp = tmp->next;
    }
    Node* first = NULL;
    for(size_t i=0 ; i<count;i++){
        tmp->next = new Node(m_baseSize);
        if(!first){
            first = tmp->next;
        }
        tmp = tmp->next;
        m_capacity += m_baseSize;
    }
    // 如果洽好已经满了，那么就把当前指针 m_cur指向第一个新创建的 Node
    if(old_cap == 0){
        m_cur = first;
    }
}
/**
 * @brief 可视化 ByteArray 里面的内容，而且不会影响原有的成员变量
 * @description: 
 * @return {*}
 */
std::string ByteArray::toString() const{
    std::string str;
    // char str[getReadSize()];
    str.resize(getReadSize());
    if(getReadSize() == 0) {
        return str;
    }
    read(&str[0], getReadSize(), m_position);
    return str;
}
/**
 * @brief Hex 是什么意思？？内容转 16 进制
 * @description: 
 * @return {*}
 */
std::string ByteArray::toHexString() const{
    std::string str = toString();
    std::stringstream ss;
    for(size_t i =0;i<str.size();i++){
        if(i> 0 && i % 32 == 0){
            ss << std::endl;
        }
        // std::setw(n) https://en.cppreference.com/w/cpp/io/manip/setw
        // std::setfill(c) https://en.cppreference.com/w/cpp/io/manip/setfill
        ss << std::setw(2) << std::setfill('0') << std::hex 
            << (int)(uint8_t)str[i] << " ";
    }
    return ss.str();
}
/**
 * @brief 只获取内容不修改 m_position
 * @param {uint64_t} len 
 * @description: 读取 len 长度的数据到 iovec 中，每个iovec存放一个 Node 数据
 * @return {*}
 */
uint64_t ByteArray::getReadBuffers(std::vector<iovec>& buffers,uint64_t len) const{
    len = len > getReadSize() ? getReadSize() : len;
    if(len == 0){
        return 0;
    }
    uint64_t size = len;
    // node 中读位置
    size_t npos = m_position % m_baseSize;
    // node 中容量
    size_t ncap = m_cur->size - npos;
    struct iovec iov;
    Node* cur = m_cur;
    while(len > 0){
        if(len <= ncap){
            iov.iov_base = (void*)(cur->ptr + npos);
            iov.iov_len = len;
            npos += len;
            ncap -= len;
            len = 0;
        }else {
            iov.iov_base = (void*)(cur->ptr + npos);
            iov.iov_len = ncap;
            len -= ncap;
            cur = cur->next;
            ncap = cur->size;
            npos = 0;
        }
        // 🈯️创建一个 iov 就可以了，因为push_back 会自动创建一个新的
        buffers.push_back(iov);
    }
    return size;
}
/**
 * @brief 从position 获取内容，只获取内容不修改 m_position
 * @param {uint64_t} len
 * @param {uint64_t} position
 * @description: 读取 len 长度的数据到 iovec 中，每个iovec存放一个 Node 数据
 * @return {*}
 */
uint64_t ByteArray::getReadBuffers(std::vector<iovec>& buffers,uint64_t len, uint64_t position) const{
    len = len > getReadSize() ? getReadSize() : len;
    if(len == 0){
        return 0;
    }
    uint64_t size = len;
    // node 中读位置
    size_t npos = position % m_baseSize;
    // 获取 position 位置的 Node
    size_t count = (position / m_baseSize) + ((position % m_baseSize) ? 1 :0);
    Node* cur = m_root;
    while (count > 0)
    {
        cur = cur->next;
        --count;
    }

    // node 中容量
    size_t ncap = cur->size - npos;
    struct iovec iov;
    
    while(len > 0){
        if(len <= ncap){
            iov.iov_base = (void*)(cur->ptr + npos);
            iov.iov_len = len;
            npos += len;
            ncap -= len;
            len = 0;
        }else {
            iov.iov_base = (void*)(cur->ptr + npos);
            iov.iov_len = ncap;
            len -= ncap;
            cur = cur->next;
            ncap = cur->size;
            npos = 0;
        }
        // 🈯️创建一个 iov 就可以了，因为push_back 会自动创建一个新的
        buffers.push_back(iov);
    }
    return size;
}
/**
 * @brief 申请 len 的可写空间，并将iovec 指向每一块空间，但是先不修改 position 的位置
 * @param {uint64_t} len
 * @description: 不懂为什么读和写基本上都是从 ByteArray 中拿数据？？？
 * @return {*}
 */
uint64_t ByteArray::getWriteBuffers(std::vector<iovec>& buffers,uint64_t len){
    if(len == 0){
        return 0;
    }
    // 为什么拿数据还需要 addCapacity 呢？？？
    addCapacity(len);
    uint64_t size = len;
    
    size_t npos = m_position % m_baseSize;
    size_t ncap = m_cur->size - npos;
    Node* cur = m_cur;
    struct iovec iov;
    while(len > 0){
        if(len <= ncap){
            iov.iov_base = (void*)(cur->ptr + npos);
            iov.iov_len = len;
            len = 0;
        }else {
            iov.iov_base = (void*)(cur->ptr + npos);
            iov.iov_len = ncap;
            len -= ncap;
            ncap = cur->size;
            cur = cur->next;
            npos = 0;
        }
        buffers.push_back(iov);
    }
    return size;
}
} 