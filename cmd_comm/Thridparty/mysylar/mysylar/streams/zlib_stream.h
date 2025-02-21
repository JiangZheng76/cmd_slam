/*
 * @Author: Johnathan 2440877322@qq.com
 * @Date: 2024-06-19 23:17:44
 * @LastEditors: Johnathan 2440877322@qq.com
 * @LastEditTime: 2024-06-21 23:09:54
 * @FilePath: /mysylar/mysylar/streams/zlib_stream.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SYLAR_ZLIB_STREAM_H__
#define __SYLAR_ZLIB_STREAM_H__

#include "stream.h"
#include <zlib.h>

namespace mysylar{

class ZlibStream : public Stream{

public:
    using ptr = std::shared_ptr<ZlibStream>;
    enum Type{
        ZLIB,
        DEFLATE,
        GZIP
    };

    // strategy 和 compress 有什么区别？？？
    enum Strategy {
        DEFAULT = Z_DEFAULT_STRATEGY, // 默认策略，使用Zlib的默认压缩方式
        FILTERED = Z_FILTERED, // 这种策略，在压缩前对数据进行预处理，尝试识别并消除冗余信息
        HUFFMAN = Z_HUFFMAN_ONLY, // 霍夫曼编码策略，这种策略仅使用霍夫曼编码进行压缩，而不考虑数据的重复性
        FIXED = Z_FIXED, // 固定编码策略，这种策略使用一种固定的、预定义的霍夫曼树进行编码，而不是根据输入数据动态构建霍夫曼树
        RLE = Z_RLE // 运行长度编码策略，这种策略通过计算连续相同数据的个数来进行压缩。
    };

    enum CompressLevel{
        NO_COMPRESSION = Z_NO_COMPRESSION,
        BEST_COMPRESSION = Z_BEST_COMPRESSION,
        BEST_SPEED = Z_BEST_SPEED,
        DEFAULT_COMPRESSION = Z_DEFAULT_COMPRESSION
    };

    // encode是什么意思？？？有什么作用 标识解码还是编码
    static ZlibStream::ptr CreateGzip(bool encode,uint32_t buff_size = 4096);
    static ZlibStream::ptr CreateZlib(bool encode,uint32_t buff_size = 4096);
    static ZlibStream::ptr CreateDeflate(bool encode,uint32_t buff_size = 4096);
    static ZlibStream::ptr Create(bool encode,uint32_t buff_size = 4096
            ,Type type = DEFLATE,int level = Z_DEFAULT_COMPRESSION ,int window_bits = 15
            ,int memlevel = 0,Strategy strategy = DEFAULT);

    ZlibStream(bool encode,uint32_t buff_size = 4096);
    virtual ~ZlibStream();

    virtual int read(void* buffer,size_t len) override;
    virtual int read(ByteArray::ptr buffer,size_t len) override;
    virtual int write(void* buffer,size_t len) override;
    virtual int write(ByteArray::ptr buffer,size_t len) override;
    virtual void close() override;

    // 压缩解压缩
    int encode(iovec* v,const uint64_t& size,bool finish);
    int decode(iovec* v,const uint64_t& size,bool finish);
    
    int init(Type type, int level, int window_bits,int memlevel,Strategy strategy);
    /// @brief 关闭这个流,释放这个流的资源
    /// @return 
    int flush();

    bool isFree(){return m_free;}
    void setFree(bool v) {m_free = v;}

    bool isEncode() {return m_encode;}
    void setEncode(bool v) {m_encode = v;}

    std::vector<iovec>& getBuffers(){ return m_buffs; }
    std::string getResult() const;
    ByteArray::ptr getByteArray();

private:
    z_stream m_zstream;
    bool m_encode; // 这个 encode 有什么作用？？？encode 用于标识释放 m_zstream 的资源，目前还没懂原理？？？
    uint32_t m_buff_size; // 一个块的大小
    bool m_free; // ？？？有什么作用么？用于表示释放，m_buffs里面缓冲区资源，和 socket_stream的功能类似
    std::vector<iovec> m_buffs;

};

}


#endif
