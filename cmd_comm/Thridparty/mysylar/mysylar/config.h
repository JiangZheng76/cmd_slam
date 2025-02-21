/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-05-22 11:47:00
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-12 11:48:46
 * @FilePath: /mysylar/mysylar/config.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
// 头宏是为了保证这个.h文件只编译一次
#ifndef __MYSYLAR_CONFIG_HPP__
#define __MYSYLAR_CONFIG_HPP__
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>
#include <boost/lexical_cast.hpp> // 内存转化
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <type_traits>
#include <typeinfo>

#include "config.h"
#include "log.h"
#include "mutex.h"
#include "singleton.h"
#include "thread.h"
#include "sylar.h"

class Person {
public:
    Person() {};
    std::string m_name;
    int m_age = 0;
    bool m_sex = 0;

    std::string toString() const {
        std::stringstream ss;
        ss << "[Person name=" << m_name
           << " age=" << m_age
           << " sex=" << m_sex
           << "]";
        return ss.str();
    }

    bool operator==(const Person& oth) const {
        return m_name == oth.m_name
            && m_age == oth.m_age
            && m_sex == oth.m_sex;
    }
};
namespace mysylar{

/**
 * @brief 定义类型转换模板仿函数，为使用片特化 将stl库的各种类型之间的转化做准备
 * @description: 
 * @return {*}
 */
template<class F,class T>
class LexicalCast{
public:
    T operator()(const F& v){
        SYLAR_LOG_DEBUG(SYLAR_LOG_ROOT()) << "from "<< typeid(F).name() << " to " << typeid(T).name();
        return boost::lexical_cast<T>(v);
    }
};
/**
 * @brief person 类的偏特化
 * @description: 
 * 特定类的片特化，template 位置什么都不需要定义
 * 只需要的类那里定义
 * @return {*}
 */
template<>
class LexicalCast<std::string,Person>{
public:
    Person operator()(const std::string& v){
        // v 是 YAML 格式的 string
        YAML::Node node = YAML::Load(v);
        Person p;
        p.m_name = node["name"].Scalar();
        p.m_sex = LexicalCast<std::string,bool>()(node["sex"].Scalar());
        p.m_age = LexicalCast<std::string,int>()(node["age"].Scalar());

        return p;
    }
};
template<>
class LexicalCast<Person,std::string>{
public:
    std::string operator()(Person & v){
        YAML::Node node(YAML::NodeType::Map);
        node["name"] = v.m_name;
        node["sex"] = v.m_sex;
        node["age"] = v.m_age;
        std::stringstream ss;
        ss << node;
        return ss.str();
    }
};
// logAppender 的配置信息 结构体封装
struct LogAppenderDefine{
    LogLevel::Level level;
    std::string formatter;
    std::string file;
    int type = 0;                   // 1 stdout; 2 file
};
/**
 * @brief logger 配置信息 结构体封装
 * @description: 
 * @return {*}
 */
struct LoggerDefine{
    std::string name;                                   //  log名称
    LogLevel::Level level = LogLevel::Level::UNKNOW;    //  日志级别
    std::vector<LogAppenderDefine> appender_define;     //  Appender 集合
    std::string formatter;                              //  默认的format

    // 判断 是否和当前的 LoggerAppender 相同
    bool appenderIsEqual(const std::vector<LogAppenderDefine> & appenders) const {
        bool res = true;;
        for(size_t i=0;i<appenders.size();i++){
            size_t j=0;
            while(appender_define[j].type != appenders[i].type){
                j++;
            }
            if(j == appender_define.size() )return false;
            if(appenders[i].type == 1){
                res = (appender_define[j].level == appenders[j].level 
                    && appender_define[j].formatter == appenders[j].formatter);
            }
            else if (appenders[i].type == 2)
            {
                res = (appender_define[j].level == appenders[j].level 
                    && appender_define[j].formatter == appenders[j].formatter
                    && appender_define[j].file == appenders[j].file);
            }
            if(!res)return res;
        }
        return res;
    }
    // 排序运算 重写
    bool operator< (const LoggerDefine& log) const {
        // 用名字来排序
        return name < log.name;
    }
    bool operator== (const LoggerDefine& log) const {
        return name == log.name 
            && level == log.level
            && formatter == log.formatter
            && appenderIsEqual(log.appender_define);
    }
};
/**
 * @brief string 转换成  LoggerDefine 偏特化
 * @description: 
 * @return {*}
 */
template<>
class LexicalCast<std::string,LoggerDefine>{
public:
    LoggerDefine operator() (const std::string& v){
        // SYLAR_LOG_DEBUG(SYLAR_LOG_ROOT()) << "from std::string to LoggerDefine ";
        LoggerDefine ld;
        if(v.empty()){
            // std::cout << "log config string is null " << v << std::endl;
            throw std::logic_error("log config string is null");
        }
        YAML::Node node = YAML::Load(v);
        if(!node["name"].IsDefined()){
            // std::cout << "log config name is define error " << v << std::endl;
            throw std::logic_error("log config name is define error");
        }
        ld.name = node["name"].as<std::string>();
        if(node["formatter"].IsDefined()){
            ld.formatter = node["formatter"].as<std::string>();
        }
        if(node["level"].IsDefined()){
            ld.level = LogLevel::fromString(node["level"].as<std::string>());
        }            
        if(node["appenders"].IsDefined()){
            for(size_t i=0;i<node["appenders"].size();i++){
                auto a = node["appenders"][i];
                LogAppenderDefine lad;
                if(!a["type"].IsDefined()){
                    std::cout << "log config appender type is not define " << node << std::endl;
                    throw std::logic_error("log config appender type is not define");
                }
                if(a["type"].as<std::string>() == "StdoutLogAppender"){
                    if(a["formatter"].IsDefined())lad.formatter = a["formatter"].as<std::string>();
                    if(a["level"].IsDefined()) lad.level = LogLevel::fromString(a["level"].as<std::string>());
                    lad.type = 1;
                }else if (a["type"].as<std::string>() == "FileLogAppender"){
                    // if(a["formatter"].IsDefined())
                        lad.formatter = a["formatter"].as<std::string>();
                    // if(a["file"].IsDefined())
                        lad.file = a["file"].as<std::string>();
                    // if(a["level"].IsDefined()) 
                        lad.level = LogLevel::fromString(a["level"].as<std::string>());
                    lad.type = 2;
                }else {
                    std::cout << "log config appender type is define error " << node << std::endl;
                    throw std::logic_error("log config appender type is define error");
                }
                ld.appender_define.push_back(lad);
            }
        }
        // SYLAR_LOG_DEBUG(SYLAR_LOG_ROOT()) << ld;
        return ld;
    }
};
template<>
class LexicalCast<LoggerDefine,std::string>{
public:
    std::string operator()(const LoggerDefine& v){
        SYLAR_LOG_DEBUG(SYLAR_LOG_ROOT()) << "from LoggerDefine to std::string";
        // 创建一个 map 类型的 node
        YAML::Node node(YAML::NodeType::Map);
        node["name"] = v.name;
        node["level"] = LogLevel::ToString(v.level);
        for(auto& ap : v.appender_define){
            YAML::Node na(YAML::NodeType::Map);
            if(ap.type == 1){
                na["type"] = "StdoutLogAppender";
            }else if(ap.type == 2){
                na["type"] = "StdoutLogAppender";
                na["file"] = ap.file;
            }else {
                std::cout << "log define appender type is define error " << node << std::endl;
                throw std::logic_error("log define appender type is define error");
            }

            if(ap.level != LogLevel::Level::UNKNOW){
                na["level"] = LogLevel::ToString(ap.level);
            }
            if(!ap.formatter.empty()){
                na["formatter"] = ap.formatter;
            }

            node["appenders"].push_back(na);
        }
        std::stringstream ss;
        ss << node;
        return ss.str();
    }
};

// 偏特化 YAML--string 转化成 vector<T>
template<class T >
class LexicalCast<std::string,std::vector<T>>{
public:
    std::vector<T> operator()(const std::string& v){
        YAML::Node node = YAML::Load(v);
        std::vector<T> vec;
        std::stringstream ss;
        for(size_t i = 0;i<node.size();i++){
            ss.str("");
            ss << node[i]; // node 写好了通过字符流进行读取的格式，输入进去就变成在yaml文件中看到的格式
            // 调用偏特化将 string 转化成 T
            vec.push_back(LexicalCast<std::string,T>()(ss.str()));
        }
        return vec;
    }
};
// 片特化 将vector<T>转化为 yaml--string
template<class T>
class LexicalCast<std::vector<T>,std::string>{
public:
    std::string operator()(const std::vector<T>& vec){
        std::stringstream ss;
        // 声明node为sequence类型，然后通过 push_back添加到后面去。
        YAML::Node node(YAML::NodeType::Sequence);
        for(size_t i =0;i< vec.size();i++){
            // 通过vector 还原出node
            node.push_back(YAML::Load(LexicalCast<T,std::string>()(vec[i])));
        }
        ss << node;
        return ss.str();
    }
};
// 片特化 YAML--string 转化为 list<T>
template<class T>
class LexicalCast<std::string,std::list<T>>{
public:
    std::list<T> operator()(const std::string& v){
        YAML::Node node = YAML::Load(v);
        std::stringstream ss;
        std::list<T> li;
        for(size_t i = 0;i<node.size();i++){
            ss.str("");
            ss << node[i];
            li.push_back(LexicalCast<std::string,T>()(ss.str()));
        }
        return li;
    }
};

// 片特化 list<T> 转化为YAML--string
template<class T>
class LexicalCast<std::list<T>,std::string>{
public:
    std::string operator()(std::list<T>& v){
        YAML::Node node(YAML::NodeType::Sequence);
        std::stringstream ss;
        for(auto& l : v){
            node.push_back(YAML::Load(LexicalCast<T,std::string>()(l)));
        }
        ss << node;
        return ss.str();
    }
};
template<class T >
class LexicalCast<std::string,std::set<T>>{
public:
    std::set<T> operator()(const std::string& v){
        YAML::Node node = YAML::Load(v);
        std::set<T> set;
        std::stringstream ss;
        for(size_t i = 0;i< node.size();i++){
            ss.str("");
            ss << node[i];
            set.insert(LexicalCast<std::string,T>()(ss.str()));
        }
        return set;
    }
};
template<class T>
class LexicalCast<std::set<T>,std::string>{
public:
    std::string operator()(std::set<T>& v){
        YAML::Node node(YAML::NodeType::Sequence);
        std::stringstream ss;
        for(auto& l : v){
            node.push_back(YAML::Load(LexicalCast<T,std::string>()(l)));
        }
        ss << node;
        return ss.str();
    }
};
template<class T>
class LexicalCast<std::string,std::map<std::string,T>>{
public:
    std::map<std::string,T> operator()(const std::string& v){
        YAML::Node node = YAML::Load(v);
        std::map<std::string,T> mp;
        std::stringstream ss;
        for(auto it = node.begin();it != node.end();it++){
            ss.str("");
            ss<< it->second;
            mp.insert(make_pair(it->first.Scalar(),LexicalCast<std::string,T>()(ss.str())));
        }
        return mp;
    }
};
template<class T>
class LexicalCast<std::map<std::string,T>,std::string>{
public:
    std::string operator()(std::map<std::string,T>& v){
        YAML::Node node(YAML::NodeType::Map);
        std::stringstream ss;
        for(auto & m : v){
            node[m.first] = YAML::Load(LexicalCast<T,std::string>()(v.second));
        }
        ss << node;
        return ss.str();
    }
};
template<class T>
class LexicalCast<std::string,std::unordered_map<std::string,T>>{
public:
    std::unordered_map<std::string,T> operator()(const std::string& v){
        YAML::Node node = YAML::Load(v);
        std::unordered_map<std::string,T> mp;
        std::stringstream ss;
        for(auto it = node.begin();it != node.end();it++){
            ss.str("");
            ss<< it->second;
            mp.insert(make_pair(it->first.Scalar(),LexicalCast<std::string,T>()(ss.str())));
        }
        return mp;
    }
};
template<class T>
class LexicalCast<std::unordered_map<std::string,T>,std::string>{
public:
    std::string operator()(std::unordered_map<std::string,T>& v){
        YAML::Node node(YAML::NodeType::Map);
        std::stringstream ss;
        for(auto & m : v){
            node[m.first] = YAML::Load(LexicalCast<T,std::string>()(v.second));
        }
        ss << node;
        return ss.str();
    }
};
template<class T>
class LexicalCast<std::string,std::unordered_set<T>>{
public:
    std::unordered_set<T> operator()(const std::string& v){
        YAML::Node node = YAML::Load(v);
        std::unordered_set<int> set;
        std::stringstream ss;
        for(size_t i = 0;i < node.size();i++){
            ss.str("");
            ss << node[i];
            set.insert(LexicalCast<std::string,T>()(ss.str()));
        }
        return set;
    }
};
template<class T>
class LexicalCast<std::unordered_set<T>,std::string>{
public:
    std::string operator()(std::unordered_set<T>& v){
        std::stringstream ss;
        YAML::Node node(YAML::NodeType::Sequence);
        for(auto & s : v){
            node.push_back(YAML::Load(LexicalCast<T,std::string>()(s)));
        }
        ss << node;
        return ss.str();
    }
};
// 基类 主要用作提供基类指针
class ConfigVarBase{
public:
    typedef std::shared_ptr<ConfigVarBase> ptr;
    ConfigVarBase(const std::string& name, const std::string& description="")
        :m_name(name),
        m_description(description){
            // 名字都设置成小写 std::transform(name.begin(),name.end(),::tolower)
            std::transform(m_name.begin(),m_name.end(),m_name.begin(),::tolower);
        }

    const std::string& getName() const { return m_name; }
    const std::string& getDescription() const { return m_description; }

    virtual std::string toString() = 0;
    virtual bool fromString(const std::string& val) = 0;

    
    
    virtual std::string getType() = 0;
protected:
    std::string m_name;
    std::string m_description;
};
// config中的变量
// 模板中声明了一个 
template<class T,class FromStr = LexicalCast<std::string,T>
                ,class ToStr = LexicalCast<T,std::string>>
class ConfigVar : public ConfigVarBase {
public:
    typedef RWMutex MutexType;
    typedef std::shared_ptr<ConfigVar> ptr;
    // 回调函数有两个参数，一个旧值，一个新值
    typedef std::function<void (const T& old_val ,const T& new_val)> on_change_cb;

    ConfigVar(const std::string& name
        ,const T& default_val
        ,const std::string& description = "")
        :ConfigVarBase(name,description) // 初始化基类方式
        ,m_val(default_val){}

    std::string toString() override {
        MutexType::ReadLock lock(m_mutex);
        try{
            // boost::lexical_cast 是一个类型转换工具，它可以将各种基本类型和 STL 类型转换为另一种基本类型或 STL 类型。
            return ToStr()(m_val); 
        }catch (std::exception& e) {
            SYLAR_LOG_ERROR(SYLAR_LOG_ROOT())<<"Config::toString expection."
            << e.what() << "convert:" << typeid(m_val).name() << " to string";
        }
        return "";
    }
    bool fromString(const std::string& val) override {
        try{
            //  利用模板类实现
            setVal(FromStr()(val));
            return true;
        }catch(std::exception& e) {
            SYLAR_LOG_ERROR(SYLAR_LOG_ROOT()) << "Config::fromtString expection "
                << e.what() << " convert: string to "<< typeid(m_val).name();
        }
        return false;
    }
    const T getValue() { 
        MutexType::ReadLock lock(m_mutex);
        return m_val; 
    }
    std::string getType() override { return typeid(T).name(); }

    // 添加改变 val 时候调用的回调函数
    uint64_t addListener(on_change_cb cb ){
        // 内部确保了内部函数的key都是不一样的
        static uint64_t s_fun_id = 0;
        s_fun_id++;
        m_cbs[s_fun_id] = cb;
        return s_fun_id;
    }

    void delListener(uint64_t fun_id) {
        MutexType::WriteLock lock(m_mutex);
        m_cbs.erase(fun_id);
    }

    on_change_cb getListener(uint64_t key){
        if(m_cbs.count(key)!= m_cbs.end()){
            return m_cbs[key];
        }else {
            return nullptr;
        }
    }

    void clearListener(){
        m_cbs.clear();
    }

    // 设置 val，并调用设置的回调函数
    void setVal(const T& v){
        {
            // 先读验证所以需要用到读锁
            MutexType::ReadLock lk(m_mutex);
            if(m_val == v) {
                return ;
            }
            // 调用所有注册的回掉函数a
            for(auto & f : m_cbs){
                f.second(m_val,v);
            }
        }
        // 修改用到写锁
        MutexType::WriteLock lock(m_mutex);
        m_val = v;
    }

private:
    T m_val;
    // 可以挂多个回掉函数
    // 回调函数 id 和 std::function
    std::map<uint64_t,on_change_cb> m_cbs;
    MutexType m_mutex;
};

/**
 * @brief 一个封装的工具类，里面没有非静态成员函数，都是静态成员函数，可以全局随意调用。【获取，设置，添加参数。】
 * @description: 
 * @return {*}
 */
class Config{
public:
    friend ConfigVarBase;
    typedef std::map<std::string,ConfigVarBase::ptr> ConfigVarMap;
    typedef RWMutex MutexType;

    static void showConfigVar(){
        MutexType::ReadLock lk(GetMutex());
        std::cout << GetSdatas().size() <<std::endl;
        for(auto &s_data : GetSdatas()){
            SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "name = " << s_data.first << ", value = " << s_data.second;
        }
    }
    /**
     * @brief 纯查找不添加，通过名字（key）查找
     * @description: 
     * @return {*}
     */
    template<class T>
    static typename ConfigVar<T>::ptr Lookup(const std::string& name){
        MutexType::ReadLock lk(GetMutex());
        auto it = GetSdatas().find(name);
        if(it == GetSdatas().end()){
            return nullptr;
        }
        // 它的作用是将一个指向某个类的基类指针或智能指针转换成指向派生类的指针或智能指针。
        // it->second 是 ConfigVarBase::ptr智能指针，将它强制转换为 ConfigVar<T>类的指针
        // 相比于 C/C++ 中的强制类型转换，std::dynamic_pointer_cast 更加安全
        // 因为它会在运行时检查转换的合法性。
        // 如果转换不合法（例如将一个基类指针或智能指针转换成与之不兼容的派生类指针或智能指针），则会返回一个空指针。
        return std::dynamic_pointer_cast<ConfigVar<T>> (it->second);
    }
    /**
     * @brief 查找并添加
     * @description: 
     * 如果定义了class T 表示 T就是一个类，如果是 typename T 表示不确定T是否是一个类
     * @return {*}
     */    
    template<class T>
    static typename ConfigVar<T>::ptr Lookup(const std::string name //  typename 是用于声明 ptr 是一个类型，而不是一个变量 （必须的） 
        ,const T& default_value, const std::string& description = "") {
        MutexType::WriteLock lock(GetMutex());
        auto it = GetSdatas().find(name);
        typename ConfigVar<T>::ptr tmp = nullptr;
        if(it != GetSdatas().end()){ 
            tmp =  std::dynamic_pointer_cast<ConfigVar<T>>(it->second);
            if(tmp){
               SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "Lookup name = " << name << " exist.";
                return tmp;
            }else {
                SYLAR_LOG_ERROR(SYLAR_LOG_ROOT()) << "Lookup name = "<< name << " but the type is " << it->second->getType()
                    << " not "  << typeid(T).name();
            }
        }
        
        // 查找第一不属于下面字符串的下标，找不到的返回 string::npos
        if(name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ._0123456789") 
            != std::string::npos){
            SYLAR_LOG_ERROR(SYLAR_LOG_ROOT()) << "Lookup name invalid" << name;
            throw std::invalid_argument(name);// 抛出参数异常
        }
        typename ConfigVar<T>::ptr v(new ConfigVar<T>(name,default_value,description));
        GetSdatas()[name] = v;
        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "Lookup name = " << name << " create a new one.";
        return v;
    }
    // 这个与单例模式相类似，只会创建一次s_data


    static void LoadFileFromYaml(YAML::Node & node);
    static typename ConfigVarBase::ptr LookupBase(std::string& name);
    // 查看s_data数据的借口，自己定义函数来查看s_datas
    static void visit(std::function<void(ConfigVarBase::ptr)> cb);
private:


    /**
     * @brief 获取所有已约定的配置
     * @description: 
     * 声明手周期和程序一样长，不会随着对象的消亡而消亡
     * 静态函数没有隐式的this指针，因为它们并不属于任何对象。
     * 因此，它们不能访问非静态成员变量和非静态成员函数，因为这些变量和函数是与特定对象实例相关联的。？？？（因为静态成员函数编译器不会给他配置 this 指针，可以看作一个普通的静态函数）
     * 不知道为什么这个不行？？？static ConfigVarMap s_datas; （也是可以的，封装起来更好管理）
     * @return {*}
     */    
    static ConfigVarMap &GetSdatas(){
        static ConfigVarMap s_datas;
        return s_datas;
    }
    /**
     * @brief 利用静态函数来获取静态的锁的效果
     * @description: 
     * @return {*}
     */
    static MutexType& GetMutex() {
        static MutexType m_mutex;
        return m_mutex;
    }
};



}

#endif