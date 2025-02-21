/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-05-23 11:07:41
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-12 11:31:55
 * @FilePath: /mysylar/mysylar/config.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "config.h"
#include <iostream>
namespace mysylar{
/**
 * @brief 从全局数据中查找 key 为 name 的 value，没有注册的返回 nullptr
 * @param {string&} name
 * @description: 
 * @return {*}
 */
ConfigVarBase::ptr Config::LookupBase(std::string& name){
    MutexType::ReadLock lock(GetMutex());
    auto it = GetSdatas().find(name);
    if(it == GetSdatas().end())return nullptr;
    return it->second;
}
/**
 * @brief 遍历整个node，并且返回一个拆分完 node 的list
 * @description: 
 * 拆分出来的 list，里面既包含 node 本身，也包含了 node 下面的子 node,所以会包含重复的内容，但不会有重复的名字
 * 比如说有 server.port 和 port，他们的值都是相同的
 * 每个节点包含 【node的 prefix(名称: tcp.server.ip) , node 本身】
 * @return {out} std::list<std::pair<std::string,const YAML::Node>>& output
 */
static void ListAllMember(const std::string& prefix,const YAML::Node& node
    ,std::list<std::pair<std::string,const YAML::Node>>& output){
    // 检查是否有不合法的字符
    if(prefix.find_first_not_of("abcdefghijklmnopqrstuvwxyz._/:1234567890") != std::string::npos){
        SYLAR_LOG_ERROR(SYLAR_LOG_ROOT()) << " config invalid name "<< prefix << " : "<< node ;
        return;
    }
    
    output.push_back(std::make_pair(prefix,node));
    // 本节点是一个键值对时
    if(node.IsMap()){
        for(auto it= node.begin();
            it!=node.end();it++){
            // 每一个node 都有一个 Scalar（）表示自身的值,然后他的下面可以有不同的类型
            // 如果 map 下面还有 map ，那么scalar可以是 map 的名字，也可以是 map 的 value。
            // 键值对类型的 node 【prefix : node】
            ListAllMember(prefix.empty()?
                it->first.Scalar():prefix+"." + it->first.Scalar()
                ,it->second,output);
        }
    }else if(node.IsSequence()){
        // 本节点是一个队列时
        for(size_t i = 0;i<node.size();i++){
            ListAllMember(prefix.empty()? 
                node[i].Scalar() + "_" + std::to_string(i) : prefix+node[i].Scalar()+"_"+ std::to_string(i),node[i],output);
        }
    }
}
/**
 * @brief 使用node修改全局配置中已有的值。约定优于配置，只会覆盖原有的约定，如果没有约定，纵使yaml中有也不会添加到里面
 * @param {Node &} node yaml config 读取出来的节点
 * @description: 
 * @return {*}
 */
void Config::LoadFileFromYaml(YAML::Node & node){
    std::list<std::pair<std::string,const YAML::Node>> all_node;
    // 拆分 node 成节点
    ListAllMember("",node,all_node);
    // showAllNode(all_node);
    for(auto & i: all_node){ 
        auto& key = i.first;
        if(key.empty()){
            continue;
        }
        // 统一将prefix变成 lower 小写
        std::transform(key.begin(),key.end(),key.begin(),::tolower);
        // 查找是否已经有约定（注册）
        // key 原本约定了是什么类型就出来什么类型 比如说 loggerDefined
        ConfigVarBase::ptr var =  LookupBase(key);
        
        // 如果存在约定就更新里面的值,那什么时候插入新的值呢？？？（约定优于配置）
        // 不插入新的，程序启动的时候有什么，就使用什么，
        if(var){
            // 表示是一个 value，不是一个 key
            if(i.second.IsScalar()){
                // 从 string状态的 scalar 转换成对应类型的 value
                var->fromString(i.second.Scalar());
            }else {
                std::stringstream ss;
                //  这个不是很懂，map传进去然后也可以恢复成 map是么？？？
                // 如果不是scalar 类型，那么就将这个 node 转换成字符串，再送进去转换成 value
                ss << i.second;
                var->fromString(ss.str());
            }
        }
    }
}
/**
 * @brief 遍历所有约定了的 configVar ，使用自己自定义的函数来查看
 * @param {function<void(ConfigVarBase::ptr)>} cb 查看 configvar 的自定义函数
 * @description: 
 * @return {*}
 */
void Config::visit(std::function<void(ConfigVarBase::ptr)> cb){
    MutexType::ReadLock lock(GetMutex());
    ConfigVarMap & m = GetSdatas();
    for(auto it = m.begin();
        it!=m.end();it++){
            // 自定义函数来访问 configVar 中的数据 
            cb(it->second);
        }
}

}