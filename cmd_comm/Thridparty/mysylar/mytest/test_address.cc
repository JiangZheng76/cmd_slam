/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-03-02 16:38:46
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-02 17:23:36
 * @FilePath: /mysylar/mytest/test_address.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "address.h"
#include "log.h"

void test(){
    std::vector<mysylar::Address::ptr> addrs;
    // 由于protocol,family.type都没有限定所以会返回三个一样的ip
    bool v = mysylar::Address::Lookup(addrs,"www.sylar.top",AF_INET,SOCK_STREAM);
    if(!v){
        SYLAR_LOG_ERROR(SYLAR_LOG_ROOT())<< "Lookup fail.";
    }
    for(size_t i =0;i<addrs.size();i++){
        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << i << " - " << addrs[i]->toString();
    }
}
void test_iface(){
    std::multimap<std::string,std::pair<mysylar::Address::ptr,uint32_t>> results;
    bool rt = mysylar::Address::GetInterafceAddress(results,AF_UNSPEC);
    if(!rt){
        SYLAR_LOG_ERROR(SYLAR_LOG_ROOT())<< "GetInterafceAddress fail.";
    }
    int i=0;
    for(auto& m : results){
        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << i++ << " - " << m.first << " - " << m.second.first->toString() ;
    }
}
void test_ipv4(){
    mysylar::IPAddress::ptr addr =  mysylar::IPAddress::Create("www.baidu.com",0);
    if(addr){
        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << addr->toString();
    }
}

int main(){
    // test();
    // test_iface();
    test_ipv4();
    return 0;
}
