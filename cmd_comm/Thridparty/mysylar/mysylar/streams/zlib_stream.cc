/*
 * @Author: Johnathan 2440877322@qq.com
 * @Date: 2024-06-19 23:17:50
 * @LastEditors: Johnathan 2440877322@qq.com
 * @LastEditTime: 2024-06-21 23:05:06
 * @FilePath: /mysylar/mysylar/streams/zlib_stream.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AEname
 */
#include "zlib_stream.h"

namespace mysylar{

ZlibStream::ptr ZlibStream::CreateGzip(bool encode,uint32_t buff_size){
    return Create(encode,buff_size,GZIP);
}

ZlibStream::ptr ZlibStream::CreateZlib(bool encode,uint32_t buff_size){
    return Create(encode,buff_size,ZLIB);
}

ZlibStream::ptr ZlibStream::CreateDeflate(bool encode,uint32_t buff_size){
    return Create(encode,buff_size,DEFLATE);
}

ZlibStream::ptr ZlibStream::Create(bool encode,uint32_t buff_size
    ,Type type ,int level ,int window_bits
    ,int memlevel ,Strategy strategy ){
    ZlibStream::ptr rt(new ZlibStream(encode,buff_size));
    if(rt->init(type,level,memlevel,window_bits,strategy) == Z_OK){
        return rt;
    }
    return nullptr;
}

ZlibStream::ZlibStream(bool encode,uint32_t buff_size)
    :m_encode(encode)
    ,m_buff_size(buff_size)
    ,m_free(true){
}

ZlibStream::~ZlibStream(){}

int ZlibStream::read(void* buffer,size_t len) {
    throw std::logic_error("ZlibStream::read is invaild");
}

int ZlibStream::read(ByteArray::ptr buffer,size_t len) {
    // 抛出异常表示这个是不可用的
    throw std::logic_error("ZlibStream::read is invaild");
}

int ZlibStream::write(void* buffer,size_t len) {
    iovec ivc;
    ivc.iov_base = (void*)buffer;
    ivc.iov_len = len;
    if(m_encode){
        return encode(&ivc,1,false);
    }else {
        return decode(&ivc,1,false);
    }
}

int ZlibStream::write(ByteArray::ptr buffer,size_t len) {
    std::vector<iovec> buffers;
    // 存疑 个人感觉是获取可以写入的空间但是源码是获取可以读取的 buffer？？？
    buffer->getWriteBuffers(buffers,len);
    if(m_encode){
        return encode(&buffers[0],buffers.size(),false);
    }else {
        return decode(&buffers[0],buffers.size(),false);
    }
}

int ZlibStream::encode(iovec* v,const uint64_t& size,bool finish){
    int ret = 0;
    int flush = 0;
    for(uint64_t i = 0; i < size; ++i) {
        // 定义压缩源的大小和输入的 buf 位置
        m_zstream.avail_in = v[i].iov_len;
        m_zstream.next_in = (Bytef*)v[i].iov_base;

        flush = finish ? (i == size - 1 ? Z_FINISH : Z_NO_FLUSH) : Z_NO_FLUSH; // 判断当前是否已经解压完了？？？flush 有什么用？

        iovec* ivc = nullptr; // 输出结果的位置
        do {
            // 判断当前是否 ivc 已经满了，没有满继续填充，满了的话创建一个新的m_buff_size 大小的块
            if(!m_buffs.empty() && m_buffs.back().iov_len != m_buff_size) {
                ivc = &m_buffs.back();
            } else {
                iovec vc;
                vc.iov_base = malloc(m_buff_size);
                vc.iov_len = 0;
                m_buffs.push_back(vc);
                ivc = &m_buffs.back();
            }
            // zstream 中剩余的量（还可以压缩的量）和压缩到的目标位置
            m_zstream.avail_out = m_buff_size - ivc->iov_len;
            m_zstream.next_out = (Bytef*)ivc->iov_base + ivc->iov_len;

            // 压缩处理
            ret = deflate(&m_zstream, flush);
            if(ret == Z_STREAM_ERROR) {
                return ret;
            }
            // ivc 的长度等于总量-还可以压缩的量
            ivc->iov_len = m_buff_size - m_zstream.avail_out;
        } while(m_zstream.avail_out == 0);
    }
    // 如果已经压缩完毕了，那么也释放 zstream 的资源
    if(flush == Z_FINISH) {
        deflateEnd(&m_zstream);
    }
    return Z_OK;
}

int ZlibStream::decode(iovec* v,const uint64_t& size,bool finish){
    int ret = 0;
    int flush = 0;
    for(uint64_t i = 0; i < size; ++i) {
        // 设置解压源和解压源的长度
        m_zstream.avail_in = v[i].iov_len;
        m_zstream.next_in = (Bytef*)v[i].iov_base;
        // 当前是否解压结束了
        flush = finish ? (i == size - 1 ? Z_FINISH : Z_NO_FLUSH) : Z_NO_FLUSH;
        
        
        iovec* ivc = nullptr; 
        do {
            // 获取解压结果的缓冲区和位置
            if(!m_buffs.empty() && m_buffs.back().iov_len != m_buff_size) {
                ivc = &m_buffs.back();
            } else {
                iovec vc;
                vc.iov_base = malloc(m_buff_size);
                vc.iov_len = 0;
                m_buffs.push_back(vc);
                ivc = &m_buffs.back();
            }

            // 本次解压到的位置和解压剩余的长度
            m_zstream.avail_out = m_buff_size - ivc->iov_len;
            m_zstream.next_out = (Bytef*)ivc->iov_base + ivc->iov_len;

            ret = inflate(&m_zstream, flush);
            if(ret == Z_STREAM_ERROR) {
                return ret;
            }
            
            // 接收解压结果的缓冲区已经占用的空间 = zongd - 剩余未解压的
            ivc->iov_len = m_buff_size - m_zstream.avail_out;
        } while(m_zstream.avail_out == 0);
    }

    if(flush == Z_FINISH) {
        inflateEnd(&m_zstream);
    }
    return Z_OK;
}


void ZlibStream::close() {
    flush();
}

int ZlibStream::init(Type type, int level, int window_bits,int memlevel,Strategy strategy){
    // ？？？ 这几个参数分别有什么用？
    SYLAR_ASSERT((level <= 9 && level >=0)  || level == DEFAULT_COMPRESSION);
    SYLAR_ASSERT((window_bits >= 8 && window_bits <= 15));
    SYLAR_ASSERT((memlevel >= 1 && memlevel <= 9));

    // 填充 m_zstream 所有设为 0 表示初始化
    memset(&m_zstream, 0, sizeof(m_zstream));

    // 
    m_zstream.zalloc = Z_NULL;
    m_zstream.zfree = Z_NULL;
    m_zstream.opaque = Z_NULL;

    switch(type) {
        case DEFLATE:
            window_bits = -window_bits;
            break;
        case GZIP:
            window_bits += 16;
            break;
        case ZLIB:
        default:
            break;
    }

    if(m_encode) {
        return deflateInit2(&m_zstream, level, Z_DEFLATED
                ,window_bits, memlevel, (int)strategy);
    } else {
        return inflateInit2(&m_zstream, window_bits);
    }
    

}
/// @brief 为了释放 zstream 资源
/// @return 
int ZlibStream::flush() {
    iovec ivc;
    ivc.iov_base = nullptr;
    ivc.iov_len = 0;
    if(m_encode){
        return encode(&ivc,1,true);
    } else{
        return decode(&ivc,1,true);
    }
}

std::string ZlibStream::getResult() const {
    std::string rt;
    for(auto& v : m_buffs) {
        rt.append((const char*)v.iov_base,v.iov_len);
    }
    return rt;
}

ByteArray::ptr ZlibStream::getByteArray(){
    ByteArray::ptr rt(new ByteArray());
    for(auto &i : m_buffs){
        rt->write(i.iov_base,i.iov_len);
    }
    // 为了可以读
    rt->setPosition(0);
    return rt;
}

}// namespace mysylar
