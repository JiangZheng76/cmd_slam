/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-02-28 22:46:29
 * @LastEditors: Johnathan 2440877322@qq.com
 * @LastEditTime: 2024-07-06 12:43:46
 * @FilePath: /mysylar/mysylar/address.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "address.h"
#include "string.h"
#include <sstream>
#include "endians.h"
#include "log.h"
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <ifaddrs.h>

namespace mysylar{

// static Logger::ptr g_logger_sys = SYLAR_LOG_NAME("sysytem");
static Logger::ptr g_logger_sys = SYLAR_LOG_ROOT();

template<class T>
T CreateMask(uint32_t bits){
    // ？？？ 假如bits为1，那么得到的为01111111么，我们想要的不是 11111110么？？？
    return (1 << (sizeof(T) - bits)) - 1;
}


int Address::getFamily() const{
    return getAddr()->sa_family;
}
std::string Address::toString(){
    std::stringstream ss;
    insert(ss);
    return ss.str();
}

std::ostream& operator<<(std::ostream& os,const Address* addr){
    return addr->insert(os);
}
/**
 * @brief 不知道地址类型的时候，进行创建地址，getaddrinfo直接返回地址sockaddr
 * @param {sockaddr*} addr
 * @description: 
 * @return {*}
 */
Address::ptr Address::Create(sockaddr* addr){
    if(!addr){
        return nullptr;
    }
    Address::ptr rt; 
    switch(addr->sa_family){
        case AF_INET:
            rt.reset((Address*)new IPv4Address(*(const sockaddr_in*)addr));
            break;
        case AF_INET6:
            rt.reset((Address*)new IPv6Address(*(const sockaddr_in6*)addr));
            break;
        default:
            rt.reset(new UnkonwAddress(*(const sockaddr*)addr));
            break;
    }
    
    return rt;
}

bool Address::Lookup(std::vector<Address::ptr>& result, const std::string& host,int family,int type,int protocol){
    addrinfo hints,* results, * next;
    hints.ai_flags = 0;
    hints.ai_family = 0;
    // ？？？ 下面这两个有什么用么？
    // 默认为0，表示tcp和udp，字节流，报文全部都支持
    hints.ai_socktype = type;
    hints.ai_protocol = protocol;
    
    hints.ai_addrlen = 0;
    hints.ai_canonname = NULL;
    hints.ai_addr = NULL;

    const char* service = NULL;
    std::string node;
    //检查 ipv6address serivce
    if(!host.empty() && host[0] == '[') {
        const char* endipv6 = (const char*)memchr(host.c_str() + 1, ']', host.size() - 1);
        if(endipv6) {
            //TODO check out of range
            // ipv6是 [xx:dfds:afsfd]:port类型的，所以这里在找port
            if(*(endipv6 + 1) == ':') {
                service = endipv6 + 2;
            }
            // node 就是address
            node = host.substr(1, endipv6 - host.c_str() - 1);
        }
    }

    //检查 node serivce
    // 如果不是ipv6的格式 255.224.243.54: 32的格式
    if(node.empty()) {
        service = (const char*)memchr(host.c_str(), ':', host.size());
        if(service) {
            if(!memchr(service + 1, ':', host.c_str() + host.size() - service - 1)) {
                node = host.substr(0, service - host.c_str());
                ++service;
            }
        }
    }

    if(node.empty()){
        node = host;
    }
    // 获取 hint 指示类型的地址，node 可以是域名，也可以是 ip 名
    // result 返回所有结果
    int error = getaddrinfo(node.c_str(),service,&hints,&results);
    if(error){
        SYLAR_LOG_ERROR(g_logger_sys) << "Address::Loopup getaddrinfo("<<node.c_str()  
            << "," << service << ",type= " << type << ",family = " << family << ") error="<< error
            << " errno=" << strerror(errno);
        return false;
    }
    // 创建并返回所有的address
    next = results;
    while(next){
        result.push_back(Address::Create(next->ai_addr));
        next = next->ai_next;
    }
    freeaddrinfo(results);
    return true;
}

std::shared_ptr<IPAddress> Address::LookupAnyIPAddress(const std::string& host,int family,int type,int protocol ){
    std::vector<Address::ptr>  result;
    bool rt = Lookup(result,host,family,type,protocol);
    if(rt){
        // 只返回第一个有用的address？？？ 其他都是没有用的么？
        for(auto& i : result){
            IPAddress::ptr v = std::dynamic_pointer_cast<IPAddress>(i);
            if(v){ return v;}
        }
    }
    return nullptr;
}
Address::ptr Address::LookupAny(const std::string& host,int family,int type,int protocol){
    std::vector<Address::ptr>  result;
    bool rt = Lookup(result,host,family,type,protocol);
    if(rt){
        // 只返回第一个address(ip和unknow)？？？ 其他都是没有用的么？
        return result[0];
    }
    return nullptr;
}
/**
 * @brief ？？？ 看不懂(计算 int下面有多少个1)
 * @param {uint32_t} netmask
 * @description: 
 * @return {*}
 */
template<class T>
static uint32_t CountBytes(T value) {
    uint32_t result = 0;
    for(; value; ++result) {
        // value -1之后就会有一个1变成了0
        // value &= value - 1; 就会去掉了value里面的一个1
        // 每去一次 result + 1
        value &= value - 1;
    }
    return result;
}

/**
 * @brief 获取网卡的地址 ？？？没看懂
 * @param {multimap<std::string,std::pair<Address::ptr,uint32_t>>} &
 * @param {int} family
 * @description: 
 * @return {*}
 */
bool Address::GetInterafceAddress(std::multimap<std::string,std::pair<Address::ptr,uint32_t>> & result
                                    ,int family){
    struct ifaddrs* next,*results;
    if(getifaddrs(&results) != 0){
        SYLAR_LOG_ERROR(g_logger_sys) << "Address::GetInterafceAddress getifaddrs erron=" 
                                        << std::strerror(errno);
        return false;
    }
    try{
        for(next = results;next;next = next->ifa_next){
            Address::ptr addr;
            uint32_t prefix_length = ~0u;
            if(!next->ifa_addr){
                continue;
            }
            if(family != AF_UNSPEC && family != next->ifa_addr->sa_family){
                // 不是我们要找的类型
                continue;
            }
            switch(next->ifa_addr->sa_family){
                case AF_INET:
                    {
                        addr = Create(next->ifa_addr);
                        uint32_t netmask = ((sockaddr_in*)next->ifa_netmask)->sin_addr.s_addr;
                        prefix_length = CountBytes(netmask);
                    }
                    break;
                case AF_INET6:
                    {
                        addr = Create(next->ifa_addr);
                        in6_addr& netmask = ((sockaddr_in6*)next->ifa_netmask)->sin6_addr;
                        prefix_length =0 ;
                        for(int i=0;i<16;++i){
                            prefix_length += CountBytes(netmask.s6_addr[i]);
                        }
                    }
                    break;
                default:
                    break;
            }
            if(addr){
                result.insert(std::make_pair(next->ifa_name,
                    std::make_pair(addr,prefix_length)));
            }
        }

    }catch(...){
        SYLAR_LOG_ERROR(g_logger_sys) << "Address::GetInterafceAddress exception.";
        freeifaddrs(results);
        return false;
    }
    freeifaddrs(results);
    return true;
}
/**
 * @brief 指定网卡名获取网络地址
 * @param {vector<std::pair<Address::ptr,uint32_t>>} & 返回的结果
 * @param {string&} iface 网卡名
 * @param {int} family 地址类型
 * @description: 
 * @return {*}
 */

bool Address::GetInterafceAddress(std::vector<std::pair<Address::ptr,uint32_t>> & result
                        ,const std::string& iface,int family){
    if(iface.empty() || iface == "*"){
        // ？？？不懂这里表示什么？？为什么网卡名为空就放两个空的地址进去
        // AF_UNSPEC 表示v4和v6
        if(family == AF_INET || family == AF_UNSPEC) {
            result.push_back(std::make_pair(Address::ptr((Address*)new IPv4Address()),(uint32_t)0u));
        }
        if(family == AF_INET6|| family == AF_UNSPEC){
            result.push_back(std::make_pair(Address::ptr((Address*)new IPv6Address()),(uint32_t)0u));
        }
        return true;
    }
    std::multimap<std::string,std::pair<Address::ptr,uint32_t>> results;
    // 没有找到网口
    if(!GetInterafceAddress(results,family)){
        return false;
    }

    auto its = results.equal_range(iface);
    // ？？？ 这里第一个不等于第二个是什么意思？
    for(;its.first != its.second;++its.first){
        result.push_back(its.first->second);
    }
    return true;
}
bool Address::operator<(const Address& rhs) const{
    socklen_t minlen = std::min(getAddrlen(),rhs.getAddrlen());
    int res = memcmp(getAddr(),rhs.getAddr(),minlen);
    // 先是数值比较
    if(res < 0){
        return true;
    }else if(res > 0){
        return false;
    }else if(getAddrlen() < rhs.getAddrlen()){
        // 数值比较相同就使用长度比较
        return true;
    }
    return false;
}
bool Address::operator==(const Address& rhs) const{
    return getAddrlen() == rhs.getAddrlen()
        && memcmp(getAddr(),rhs.getAddr(),getAddrlen());
}
bool Address::operator!=(const Address& rhs) const{
    return !(*this == rhs);
}
/**
 * @brief 把域名（www.baidu.com）的地址转化为address,只会返回第一个result
 * @param {char*} address 域名
 * @param {uint32_t} port
 * @description: 因为返回的域名可能为ipv4或者是ipv6
 * @return {*}
 */
IPAddress::ptr IPAddress::Create(const char* address,uint16_t port){
    struct addrinfo hints,* results;
    memset(&hints,0,sizeof(struct addrinfo));
    hints.ai_family = AF_UNSPEC;
    // If  hints.ai_flags contains the AI_NUMERICHOST flag, then node must be a numerical network address.
    // ？？？为什么要限制只能输入数字，那样的话创建这个函数的作用是什么？
    // hints.ai_flags = AI_NUMERICHOST;

    int error = getaddrinfo(address,NULL,&hints,&results);

    if(error){
        SYLAR_LOG_ERROR(g_logger_sys) << "IPAddress::Create(" << address 
            << ", " << port << ") error=" << error
            << "errno=" << strerror(errno);
        return nullptr;
    }
    try{
        /// Unkonw 类型就会转失败返回nullptr
        //  只接受ipv4或者ipv6的地址
        // 
        IPAddress::ptr rt = std::dynamic_pointer_cast<IPAddress>(Address::Create(results->ai_addr));
        if(rt){
            rt->setPort(port);
        }
        freeaddrinfo(results);
        return rt;
    }catch(...){
        freeaddrinfo(results);
        return nullptr;
    }
    


}
IPv4Address::IPv4Address(uint32_t address, uint16_t port){
    memset(&m_addr,0,sizeof(m_addr));
    m_addr.sin_family = AF_INET;
    // 网络字节序默认是大端的，所以我们只需要转化成大端就可以了？？？为什么sylar打的代码是转化成小端的
    m_addr.sin_port = swapToBigEndian(port);
    m_addr.sin_addr.s_addr = swapToBigEndian(address);
}
IPv4Address::IPv4Address(const sockaddr_in &addr){
    m_addr = addr;
}
/**
 * @brief ipv4 明文创建地址 
 * @param {char*} address
 * @param {uint32_t} port
 * @description: 
 * @return {*}
 */
IPv4Address::ptr IPv4Address::Create(const char* address, uint16_t port){
    IPv4Address::ptr rt(new IPv4Address);
    rt->m_addr.sin_port = swapToBigEndian(port);
    int result = inet_pton(AF_INET,address,&rt->m_addr.sin_addr);
    if(result <= 0){
        SYLAR_LOG_ERROR(g_logger_sys ) << "Create() inet_pton( AF_INET," << address << "," << &rt->m_addr.sin_addr 
            << ") rt="<< result << ", errno = " << strerror(errno);
        return nullptr;
    }
    return rt;
}
sockaddr* IPv4Address::getAddr() const {
    return  (sockaddr*)&m_addr;
}
socklen_t IPv4Address::getAddrlen() const{
    return sizeof(m_addr);
}
/**
 * @brief ipv4输出成我们看得懂的地址
 * @param {ostream&} os
 * @description: 
 * @return {*}
 */
std::ostream& IPv4Address::insert(std::ostream& os) const{
    uint32_t addr = swapToBigEndian(m_addr.sin_addr.s_addr);
    os  << ((addr >> 24) & 0xff) << "."
        << ((addr >> 16) & 0xff) << "."
        << ((addr >> 8) & 0xff) << "."
        << ((addr & 0xff ));
    os << ":" << swapToBigEndian(m_addr.sin_port);
    return os;
}

IPAddress::ptr IPv4Address::broadcastAddress(uint32_t prefix_len){
    if(prefix_len > 32){
        return nullptr;
    }

    sockaddr_in baddr(m_addr);
    // 这里不是很懂？？？0000111,在后面部分都加上1，就是广播地址
    baddr.sin_addr.s_addr |= swapToLittleEndian(CreateMask<uint32_t>(prefix_len));
    IPv4Address::ptr ipv4(new IPv4Address(baddr));
    return ipv4;

}
IPAddress::ptr IPv4Address::networkAddress(uint32_t prefix_len){
    if(prefix_len > 32){
        return nullptr;
    }

    sockaddr_in baddr(m_addr);
    // 这里不是很懂？？？0000111 后面部分都去掉1才是网络地址吧，不是应该&= ~Mask么？？？？
    baddr.sin_addr.s_addr &= swapToLittleEndian(CreateMask<uint32_t>(prefix_len));
    IPv4Address::ptr ipv4(new IPv4Address(baddr));
    return ipv4;
}
/**
 * @brief 本身之后的所有都可以作为子网？？？这里的写法和他的不一样
 * @param {uint32_t} prefix_len
 * @description: 
 * @return {*}
 */
IPAddress::ptr IPv4Address::subnetMask(uint32_t prefix_len){
    sockaddr_in sub_net(m_addr);
    // 这里不是很懂？？？0000111 后面部分都去掉1才是网络地址吧，不是应该&= ~Mask么？？？？
    sub_net.sin_addr.s_addr &= ~swapToLittleEndian(CreateMask<uint32_t>(prefix_len));
    IPv4Address::ptr ipv4(new IPv4Address(sub_net));
    return ipv4;
}
uint32_t IPv4Address::getPort() const{
    return m_addr.sin_addr.s_addr;
}
void IPv4Address::setPort(uint16_t v){
    m_addr.sin_port = swapToBigEndian(v);
}

/**
 * @brief
 * @param {char*} address 文本格式为：1110101010
 * @param {uint32_t} port
 * @description: 
 * @return {*}
 */
IPv6Address::IPv6Address(const char* address , uint16_t port){
    memset(&m_addr,0,sizeof(m_addr));
    m_addr.sin6_family = AF_INET6;
    // 网络字节序默认是大端的，所以我们只需要转化成大端就可以了？？？为什么sylar打的代码是转化成小端的
    m_addr.sin6_port = swapToBigEndian(port);
    // ？？？ 为神马止复制16个字节（16 * 8 = 128）
    memcpy(&m_addr.sin6_addr.s6_addr,address,16);
}
IPv6Address::IPv6Address(const sockaddr_in6& addr){
    m_addr = addr;
}
IPv6Address::IPv6Address(){
    memset(&m_addr,0,sizeof(m_addr));
    m_addr.sin6_family = AF_INET6;
}
IPv6Address::IPv6Address(const uint8_t address[16],uint16_t port){
    memset(&m_addr,0,sizeof(m_addr));
    m_addr.sin6_family = AF_INET6;
    m_addr.sin6_port = swapToBigEndian(port);
    memcpy(&m_addr.sin6_addr.s6_addr,address,16);
}
/**
 * @brief 从明文文本地址创建address
 * @param {char*} address 文本格式为 ea:12:34::34
 * @param {uint32_t} port
 * @description: 
 * @return {*}
 */
IPv6Address::ptr IPv6Address::Create(const char* address,uint16_t port){
    IPv6Address::ptr rt(new IPv6Address);
    rt->m_addr.sin6_port = swapToBigEndian(port);
    int result = inet_pton(AF_INET6,address,&rt->m_addr.sin6_addr);
    if(result <= 0){
        SYLAR_LOG_ERROR(g_logger_sys ) << "Create() inet_pton( AF_INET6," << address << "," << &rt->m_addr.sin6_addr 
            << ") rt="<< result << ", errno = " << strerror(errno);
        return nullptr;
    }
    return rt;
}
sockaddr* IPv6Address::getAddr() const {
    return (sockaddr*)&m_addr;
}
socklen_t IPv6Address::getAddrlen() const {
    return sizeof(m_addr);
}
std::ostream& IPv6Address::insert(std::ostream& os) const {
    os << "[";
    uint16_t* addr = (uint16_t*)m_addr.sin6_addr.s6_addr;
    bool used_zeros = false;
    for(size_t i = 0; i < 8; ++i) {
        if(addr[i] == 0 && !used_zeros) {
            continue;
        }
        if(i && addr[i - 1] == 0 && !used_zeros) {
            os << ":";
            used_zeros = true;
        }
        if(i) {
            os << ":";
        }
        os << std::hex << (int)swapToBigEndian(addr[i]) << std::dec;
    }

    if(!used_zeros && addr[7] == 0) {
        os << "::";
    }

    os << "]:" << swapToBigEndian(m_addr.sin6_port);
    return os;
}
/**
 * @brief ？？？没看懂原理
 * @param {uint32_t} prefix_len
 * @description: 
 * @return {*}
 */
IPAddress::ptr IPv6Address::broadcastAddress(uint32_t prefix_len) {
    sockaddr_in6 baddr(m_addr);
    baddr.sin6_addr.s6_addr[prefix_len / 8] &= 
        CreateMask<uint8_t>(prefix_len % 8);
    for(int i= prefix_len / 8 + 1;i<16;++i){
        baddr.sin6_addr.s6_addr[i] = 0xff;
    }
    return IPv6Address::ptr(new IPv6Address(baddr));

}
IPAddress::ptr IPv6Address::networkAddress(uint32_t prefix_len) {
    sockaddr_in6 baddr(m_addr);
    baddr.sin6_addr.s6_addr[prefix_len / 8] &= 
        CreateMask<uint8_t>(prefix_len % 8);
    // for(int i= prefix_len / 8 + 1;i<16;++i){
    //     baddr.sin6_addr.s6_addr[i] = 0xff;
    // }
    return IPv6Address::ptr(new IPv6Address(baddr));
}
IPAddress::ptr IPv6Address::subnetMask(uint32_t prefix_len) {
    sockaddr_in6 subnet;
    memset(&subnet,0,sizeof(subnet));
    subnet.sin6_family = AF_INET6;
    subnet.sin6_addr.s6_addr[prefix_len / 8] = 
        ~CreateMask<uint8_t>(prefix_len % 8);
    
    for(int i=0 ; i< (int)prefix_len / 8 ;++i){
        subnet.sin6_addr.s6_addr[i] = 0xff;
    }
    return IPv6Address::ptr(new IPv6Address(subnet));
}
uint32_t IPv6Address::getPort() const {
    return swapToBigEndian(m_addr.sin6_port);
}
/**
 * @brief 如果uint32_t的话会出现swap 之后 port 为 0 的情况。？？？？所以 port 必须使用 16 位的
 * @param {uint16_t} v
 * @description: 
 * @return {*}
 */
void IPv6Address::setPort(uint16_t v) {
    m_addr.sin6_port = swapToBigEndian(v);
}

static const size_t MAX_PATH_LEN = sizeof(((sockaddr_un*)0)->sun_path -1);
/**
 * @brief
 * @param {string&} path文件地址
 * @description: 
 * @return {*}
 */
UnixAdress::UnixAdress(){
    memset(&m_addr,0,sizeof(m_addr));
    m_addr.sun_family = AF_UNIX;
    m_length = offsetof(sockaddr_un ,sun_path) + MAX_PATH_LEN;
}
UnixAdress::UnixAdress(const std::string& path){
    memset(&m_addr,0,sizeof(m_addr));
    m_addr.sun_family = AF_UNIX;
    m_length = path.size() +1;
    // ？？？不懂
    if(!path.empty() && path[0] == '\0'){
        --m_length;
    }
    if(m_length > sizeof(m_addr.sun_path)){
        throw std::logic_error("path too long");
    }

    memcpy(m_addr.sun_path,path.c_str(),m_length);
    m_length += offsetof(sockaddr_un,sun_path);
}

sockaddr* UnixAdress::getAddr() const {
    return (sockaddr*)&m_addr;
}
socklen_t UnixAdress::getAddrlen() const {
    return m_length;
}
void UnixAdress::setAddrlen(socklen_t len){
    m_length = len;
}
std::ostream& UnixAdress::insert(std::ostream& os) const {
    if(m_length > offsetof(sockaddr_un,sun_path) 
        && m_addr.sun_path[0] == '\0'){
            // 将前面的\0去掉
            return os << "\\0 " << std::string(m_addr.sun_path + 1,
                m_length - offsetof(sockaddr_un, sun_path) -1);
        }
    return os << m_addr.sun_path;
}
UnkonwAddress::UnkonwAddress(int family){
    m_addr.sa_family = family;
}
UnkonwAddress::UnkonwAddress(const sockaddr& addr){
    m_addr = addr;
}
sockaddr* UnkonwAddress::getAddr() const {
    return (sockaddr*)&m_addr;
}
socklen_t UnkonwAddress::getAddrlen() const {
    return sizeof(m_addr);
}
std::ostream& UnkonwAddress::insert(std::ostream& os) const {
    os << "[ UnknowAddresss family:  "<< m_addr.sa_family << "]";
    return os;
}



}