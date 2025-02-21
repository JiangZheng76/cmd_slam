/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-03-05 20:48:13
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-25 16:18:14
 * @FilePath: /mysylar/mysylar/bytearray.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SYLAR_BYTEARRAY_H__
#define __SYLAR_BYTEARRAY_H__

#include <memory>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <vector>

namespace mysylar{

class ByteArray{

public:
    typedef std::shared_ptr<ByteArray> ptr;

    struct Node {
        Node(size_t s);
        Node();
        ~Node();

        char* ptr;
        Node* next;
        size_t size;
    };

    ByteArray(size_t base_size = 4096);
    ~ByteArray();

    /**
     * 关于写固定长度和可变长度的解释：
     * 无论固定长度还是可变长度最后实现写的底层都是 write 函数
     * 
     * 固定长度：
     * 1、不对数据进行压缩
     * 2、比如原数据长度为int32_t 那么就写 32 位 的长度的数据
     * 
     * 可变长度：
     * 1、对数据进行压缩
     * 2、使用 varint 方法进行压缩
     * 3、压缩的结果在 1-5 个字节区间，数值越大，压缩后的结果越大
     * 4、因为压缩后的结果长度不定，所以为可变长度
     */

    // write 
    // fix固定长度 ？？？ 如何理解
    // 都是调用 write ，sizeof(type)来写入
    void writeFint8(int8_t value);
    void writeFuint8(uint8_t value);
    void writeFint16(int16_t value);
    void writeFuint16(uint16_t value);
    void writeFint32(int32_t value);
    void writeFuint32(uint32_t value);
    void writeFint64(int64_t value);
    void writeFuint64(uint64_t value);
    
    // 可变长度？？？如何理解，用来压缩的？？？怎么实现
    void writeInt32(int32_t value);
    void writeUint32(uint32_t value);
    void writeInt64(int64_t value);
    void writeUint64(uint64_t value);

    void writeFloat(const float& value);
    void writeDouble(const double& value);

    // 固定 16 位的 int 表示长度，以此类推 如何理解？？
    void writeStringF16(const std::string& value);
    void writeStringF32(const std::string& value);
    void writeStringF64(const std::string& value);
    // 可变的 int
    void writeStringVint(const std::string& value);
    void writeStringWithoutLength(const std::string& value);

    // read
    int8_t readFint8();
    uint8_t readFuint8();
    int16_t readFint16();
    uint16_t readFuint16();
    int32_t readFint32();
    uint32_t readFuint32();
    int64_t readFint64();
    uint64_t readFuint64();

    int32_t readInt32();
    uint32_t readUint32();
    int64_t readInt64();
    uint64_t readUint64();
    float readFloat();
    double readDouble();

    std::string readStringF16();
    std::string readStringF32();
    std::string readStringF64();
    std::string readStringVint();
    // std::string readStringWithoutLength();

    // 内部操作
    void clear();

    void write(const void* buf,size_t size);
    void read(void* buf,size_t size);
    void read(void* buf,size_t size, size_t position) const;

    size_t getPosition() const {return m_position;}
    void setPosition(size_t v);

    bool writeToFile(const std::string& name) const;
    bool readFromFile(std::string& name);

    size_t getBaseSize() const {return m_baseSize;}
    size_t getReadSize() const {return m_size - m_position;} // 还有多少数据可以读
    
    bool isLittleEndian() const;
    void setIsLitttleEndian(bool val);

    std::string toString() const;
    std::string toHexString() const;

    uint64_t getReadBuffers(std::vector<iovec>& buffers,uint64_t len = ~0ull) const;
    uint64_t getReadBuffers(std::vector<iovec>& buffers,uint64_t len, uint64_t position) const;
    uint64_t getWriteBuffers(std::vector<iovec>& buffers,uint64_t len);

    size_t getSize()const {return m_size;}

private:
    /**
     * @brief 检查容量是否足够，不够和刚好够都进行块扩充
     * @param {size_t} size
     * @description: 
     * @return {*}
     */    
    void addCapacity(size_t size);
    /**
     * @brief 剩余容量
     * @description: 
     * 当前可以操作的空余空间大小？？？为什么不是减 m_size？
     * @return {*}
     */    
    size_t getCapacity() const {return m_capacity - m_position;}

private:
    size_t m_position;  // 当前操作到的位置
    size_t m_capacity;  // 当前总容量
    size_t m_baseSize;  // 基础容量？？？作用是什么？（一个扩充块的单位大小）
    size_t m_size;      // 当前已使用大小

    int8_t m_endian;    // 存储数据大小端标识，默认是大端和网络字节序一致,表明网络字节序的问题

    Node* m_root;       // 初始节点指针
    Node* m_cur;        // 当前节点指针

};

}

#endif