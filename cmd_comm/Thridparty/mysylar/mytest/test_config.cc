/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-05-23 11:06:21
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-13 17:54:45
 * @FilePath: /mysylar/mytest/test_config.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "../mysylar/config.hpp"
#include "../mysylar/util.h"
#include "../mysylar/log.h"
#include <stdlib.h>
#include <vector>
#include <map>
#include <list>
#include <unordered_map>
#include <yaml-cpp/yaml.h>


typename mysylar::ConfigVar<int>::ptr g_int_value_config = 
    mysylar::Config::Lookup("servers.keepalive",(int)8080,"servers keepalive");
// 测试 预设类型错误的情况
typename mysylar::ConfigVar<float>::ptr g_int_valuex_config = 
    mysylar::Config::Lookup("servers.keepalive",(float)8080,"servers keepalive");
typename mysylar::ConfigVar<float>::ptr  g_float_value_config =
    mysylar::Config::Lookup("servers.timeout",(float)8081.0,"servers timeout"); 
typename mysylar::ConfigVar< std::vector<int> >::ptr g_int_vec_value_config = 
    mysylar::Config::Lookup("servers.vec",std::vector<int>() = {1,2},"servers vec");
typename mysylar::ConfigVar< std::list<int> >::ptr g_int_list_value_config = 
    mysylar::Config::Lookup("serviers.list",std::list<int>() = {1,2},"server list");
typename mysylar::ConfigVar<std::set<int> >::ptr g_int_set_value_config = 
    mysylar::Config::Lookup("servers.set",std::set<int>() = {20,20,10},"server set");
typename mysylar::ConfigVar<std::unordered_set<int>>::ptr g_int_unset_value_config = 
    mysylar::Config::Lookup("servers.unordered_set",std::unordered_set<int>() = {10,10,20},"server unordered_set");
typename mysylar::ConfigVar<Person>::ptr g_person = 
    mysylar::Config::Lookup("class.person",Person(),"class person");
void print_yaml(const YAML::Node& node,int level){
    if(node.IsScalar()){
        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << std::string(level*4,' ')
            << node.Scalar() << " - " << node.Type() <<" " << level << "scalar";
    }else if(node.IsNull()){
        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << std::string(level*4,' ')
             << "NULL  - " << node.Type()<<" " << level << "null";
    }else if(node.IsMap()){
        for(auto it = node.begin();it != node.end() ;it++){
            SYLAR_LOG_INFO(SYLAR_LOG_ROOT())  << std::string(level*4,' ')
                << it->first << " - " << it->second.Type()<<" " << level <<" map ";
            print_yaml(it->second,level+1);
        }
    }else if(node.IsSequence()){
        for(size_t i =0;i<node.size();i++){
            SYLAR_LOG_INFO(SYLAR_LOG_ROOT())  << std::string(level*4,' ')
                << i<< " - " << node[i].Type()<<" " << level <<" sequence ";
            print_yaml(node[i],level+1);
        }
    }
}
void test_yaml(){
    YAML::Node root = YAML::LoadFile("/home/jiangz/CODE/mysylar/bin/conf/logs.yml");
    print_yaml(root,0);
}

void test_config(){
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "before: " <<  g_person->getValue().toString();
    g_person->addListener([](const Person& old_val,const Person& new_val) -> void {
        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "old_val " << old_val.toString() 
            << " new_val " << new_val.toString();
    });

#define XX(g_value,name,prefix) \
    {\
        for(auto &v :  g_value->getValue()){\
            SYLAR_LOG_INFO(SYLAR_LOG_ROOT() )<< #prefix << " " << #name <<" "<<  v ;\
        }\
    }

    // XX(g_int_value_config,int,before);
    // XX(g_float_value_config,float,before);
    XX(g_int_vec_value_config,int_vec,before);
    XX(g_int_list_value_config,int_list,before);
    XX(g_int_set_value_config,int_set,before);
    XX(g_int_unset_value_config,int_unset,before);
    // XX(g_vetor_person_value_config,person_vec,before); // 不知道为什么不可以
    YAML::Node root = YAML::LoadFile("/home/jiangz/CODE/mysylar/bin/conf/log.yml");
    mysylar::Config::LoadFileFromYaml(root);
    // XX(g_int_value_config,int,after);
    // XX(g_float_value_config,float,after);
    XX(g_int_vec_value_config,int_vec,after);
    XX(g_int_list_value_config,int_list,after);    
    XX(g_int_set_value_config,int_set,after);
    XX(g_int_unset_value_config,int_unset,after);
    // XX(g_vetor_person_value_config,person_vec,after); // 不知道为什么不可以
#undef XX
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "after" <<  g_person->getValue().toString();
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "before " <<   g_int_value_config->getValue();
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "before " <<   g_float_value_config->toString();
    // YAML::Node root = YAML::LoadFile("/home/jiangz/CODE/mysylar/bin/conf/server.yml");
    // mysylar::Config::LoadFileFromYaml(root);
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "after " <<   g_int_value_config->getValue();
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "after " <<   g_float_value_config->toString();

    // print_yaml(root,0);
}
void test_log(){
    std::cout << mysylar::LoggerMgr::GetInstance()->toYamlString() << std::endl;
    YAML::Node root = YAML::LoadFile("/home/jiangz/CODE/mysylar/config/log.yml");
    mysylar::Config::LoadFileFromYaml(root);
    // auto system = mysylar::LoggerMgr::GetInstance()->getLogger("system");
    // SYLAR_LOG_ROOT()->setFormatter(mysylar::LogFormatter::ptr(new mysylar::LogFormatter("%d%T - %m%n")));
    // std::cout << mysylar::LoggerMgr::GetInstance()->toYamlString() << std::endl;
    SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) <<"\n" << mysylar::LoggerMgr::GetInstance()->toYamlString();
}
int main(int argc,char ** agrv){
    // test_config();
    test_log();
    usleep(100);
    
    mysylar::Config::visit([](mysylar::ConfigVarBase::ptr var){
        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "name = "<< var->getName()
            <<"description=" << var->getDescription()
            <<"type= " << var->getType()
            <<"value=" << var->toString();
    });
    // SYLAR_LOG_INFO(system) <<"\n" << mysylar::LoggerMgr::GetInstance()->toYamlString();
    
    return 0;
}