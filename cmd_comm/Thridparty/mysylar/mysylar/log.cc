#include <log.h>
#include <functional>
namespace mysylar {
/**
 * @brief 定义各个 Item 的输出方式和输出结果
 * @description: 
 * @return {*}
 */
class MessageFormatItem : public LogFormatter::FormatItem {
public:
    // 继承之后都需要默认构造函数 
    MessageFormatItem(const std::string& str="hello mysylar!"){}
    // 因为Logger在后面定义 所以都不能使用ptr
    virtual void format(std::stringstream& os,std::shared_ptr<Logger> logger,LogLevel::Level level,LogEvent::ptr event) override{
        os << event->getContent();
    }
};
class LevelFormatItem : public LogFormatter::FormatItem {
public:
    LevelFormatItem(const std::string& str=""){}
    void format(std::stringstream& os,std::shared_ptr<Logger> logger,LogLevel::Level level,LogEvent::ptr event) override{
        if (level == LogLevel::ERROR){
            os << "\e[1;31m" <<  LogLevel::ToString(level) << "\e[0m";    
        }else if (level == LogLevel::DEBUG){
            os << "\e[1;33m" <<  LogLevel::ToString(level) << "\e[0m";    
        } else {
            os << LogLevel::ToString(level);
        }
        
    }
};
class ElapseFormatItem : public LogFormatter::FormatItem {
public:
    ElapseFormatItem(const std::string& str=""){}
    void format(std::stringstream& os,std::shared_ptr<Logger> logger,LogLevel::Level level,LogEvent::ptr event) override{
        os << event->getElapse();
    }
};
class NameFormatItem : public LogFormatter::FormatItem {
public:
    NameFormatItem(const std::string& str=""){}
    void format(std::stringstream& os,std::shared_ptr<Logger> logger,LogLevel::Level level,LogEvent::ptr event) override{
        os << logger->getName();
    }
};
class FiberIdFormatItem : public LogFormatter::FormatItem {
public:
    FiberIdFormatItem(const std::string& str=""){}
    void format(std::stringstream& os,std::shared_ptr<Logger> logger,LogLevel::Level level,LogEvent::ptr event) override{
        os << event->getFiberId();
    }
};
class DateTimeFormatItem : public LogFormatter::FormatItem {
public:
    DateTimeFormatItem(const std::string& format = "%Y-%m-%d %H:%M:%S")
        :m_format(format){
            if(m_format.empty()){
                m_format = "%Y-%m-%d %H:%M:%S";
            }
        }
        
    void format(std::stringstream& os,std::shared_ptr<Logger> logger,LogLevel::Level level,LogEvent::ptr event) override{
        struct tm tm;
        char buf[64];
        time_t time = (time_t)event->getTime();
        
        localtime_r(&time,&tm);
        strftime(buf,sizeof(buf),m_format.c_str(),&tm);

        os << buf;
    }
private:
    std::string m_format;
};
class FilenameFormatItem : public LogFormatter::FormatItem {
public:
    FilenameFormatItem(const std::string& str=""){}
    void format(std::stringstream& os,std::shared_ptr<Logger> logger,LogLevel::Level level,LogEvent::ptr event) override{
        os << event->getFile();
    }
};
class LineFormatItem : public LogFormatter::FormatItem {
public:
    LineFormatItem(const std::string& str=""){}
    void format(std::stringstream& os,std::shared_ptr<Logger> logger,LogLevel::Level level,LogEvent::ptr event) override{
        os << event->getLine();
    }
};
class NewLineFormatItem : public LogFormatter::FormatItem {
public:
    NewLineFormatItem(const std::string& str= ""){}
    void format(std::stringstream& os,std::shared_ptr<Logger> logger,LogLevel::Level level,LogEvent::ptr event) override{
        os << std::endl;
    }
};
class StringFormatItem : public LogFormatter::FormatItem {
public:
    StringFormatItem(const std::string& str="")
        :m_string(str){}
    void format(std::stringstream& os,std::shared_ptr<Logger> logger,LogLevel::Level level,LogEvent::ptr event) override{
        os << m_string;
    }
private:
    std::string m_string;
};

class TabFormatItem : public StringFormatItem {
public:
    TabFormatItem(const std::string& str= "\t"){}
    void format(std::stringstream& os,std::shared_ptr<Logger> logger,LogLevel::Level level,LogEvent::ptr event) override{
        os << "\t";
    }
};

class ThreadIdFormatItem : public LogFormatter::FormatItem {
public:
    ThreadIdFormatItem(const std::string& str=""){}
    void format(std::stringstream & os,std::shared_ptr<Logger> logger,LogLevel::Level level ,LogEvent::ptr event) override{
        os << event->getThreadId();
    }
};


// 宏定义函数写法 https://blog.csdn.net/Dontla/article/details/122526092
 // 定义不需要带 static
const char* LogLevel::ToString(LogLevel::Level level){
    switch(level){
#define XX(name) \
    case LogLevel::Level::name:\
        return #name;\
        break;

    XX(UNKNOW);
    XX(DEBUG);
    XX(INFO);
    XX(WARN);
    XX(ERROR);
    XX(FATAL);
#undef XX
    default:
        return "UNKNOW";
    }
    return "UNKNOW";
}
/**
 * @brief 使用宏定义的方式将 str 转成 string 
 * @param {string} str
 * @description: 
 * @return {*}
 */
LogLevel::Level LogLevel::fromString(std::string str){
    if(str!=""){
#define XX(level,v) \
    if(str == #v) \
        return LogLevel::Level::level;
    XX(DEBUG,debug);
    XX(INFO,info);
    XX(WARN,warn);
    XX(ERROR,error);
    XX(FATAL,fatal);
    XX(DEBUG,DEBUG);
    XX(INFO,INFO);
    XX(WARN,WARN);
    XX(ERROR,ERROR);
    XX(FATAL,FATAL);
#undef XX
    }
    return LogLevel::Level::UNKNOW;
}
/**
 * @brief  Logger 构造函数初始化输出格式
 * @param {string} name
 * @description: 
 * @return {*}
 */
Logger::Logger(const std::string name)
    :m_name(name)
    ,m_level(LogLevel::Level::DEBUG){
        m_formatter.reset(new LogFormatter("[%c]%T[%p]%T%d{%Y-%m-%d %H:%M:%S}%T[T:%t]%T[F:%F]%T%f:%l%T%T%m%T%n"));
}
/**
 * @brief Logger 的输出函数，顺序调用所有的 appender 进行输出
 * @param {Level} level
 * @param {ptr} event
 * @description: 
 * @return {*}
 */
void Logger::log(LogLevel::Level level,LogEvent::ptr event){
    MutexType::Lock lock(m_mutex);
    if(level >= m_level){// 为什么？
        // 为什么需要使用 shared_from_this()？？？
        // 1、当其他函数或者对象需要使用本对象，尽量不要使用this，不知道函数会不会调用 delete
        // 2、如果重新构建 shared_ptr ，多个 shared_ptr 的计数引用都是 1，会造成对象的多次释放。https://blog.csdn.net/weixin_41442027/article/details/105571428
        // 所以尽量继承 enable_shared_from_this 。
        auto self = shared_from_this();
        // MutexType::Lock lock)()
        for(auto &i : m_appenders){
            i->log(level,self,event);
        }
    }
}
void LogAppender::setLogFormatter(LogFormatter::ptr formatter) {
    MutexType::Lock lock(m_mutex);
    this->m_formatter = formatter;
}
void Logger::debug(LogEvent::ptr event){
    log(LogLevel::DEBUG,event);
}
void Logger::info(LogEvent::ptr event){
    log(LogLevel::INFO,event);
}
void Logger::warn(LogEvent::ptr event){
    log(LogLevel::WARN,event);
}
void Logger::error(LogEvent::ptr event){
    log(LogLevel::ERROR,event);
}
void Logger::fatal(LogEvent::ptr event){
    log(LogLevel::FATAL,event);
}
/**
 * @brief 将 logger 的信息保存下来，保存成 yaml 格式下次使用
 * @description: 
 * @return {*}
 */
std::string Logger::toYamlString() {
    MutexType::Lock lk(m_mutex);
    YAML::Node node;
    node["name"] = m_name;
    node["level"] = LogLevel::ToString(m_level);
    if(!(m_formatter->getPattern()).empty()){
        node["formatter"] = m_formatter->getPattern();
    }
    for(auto& ap : m_appenders){
        node["appenders"].push_back(YAML::Load(ap->toYamlString()));
    }
    std::stringstream ss;
    ss << node;
    return ss.str();
};
/**
 * @brief 添加输出器 appender ，如果 appender 没有自定义输出的格式，就使用 logger 的格式 
 * @param {ptr} appender
 * @description: 
 * @return {*}
 */
void Logger::addAppender(LogAppender::ptr appender){

    // std::cout << "进锁"<<std::endl;
    MutexType::Lock lk(m_mutex);
    if(!appender->getFormatter()){
        // MutexType::Lock lk(appender->m_mutex);
        appender->setLogFormatter(this->m_formatter);
    }
    m_appenders.push_back(appender);
    // std::cout << "解锁"<<std::endl;
}
void Logger::delAppender(LogAppender::ptr appender){
    MutexType::Lock lk(m_mutex);
    for(auto it = m_appenders.begin();
        it != m_appenders.end();it++){
            if(*it == appender){
                m_appenders.erase(it);
                break;
            }
        }
}
/**
 * @brief 设计 logger formatter，appender 如果没有就重新设置
 * @param {ptr} val
 * @description: 
 * @return {*}
 */
void Logger::setFormatter(LogFormatter::ptr val){
    MutexType::Lock lk(m_mutex);
    m_formatter = val;
    if(!(m_formatter->getPattern().empty())){
        for(auto& ap : m_appenders){
            if(!ap->getHasFormatter()){
                // 防止toString出错
                MutexType::Lock lk(ap->m_mutex);
                ap->setLogFormatter(m_formatter);
            }
        }
    }
        
    
};
FileLogAppender::FileLogAppender(const std::string& filename)
    :m_filename(filename){
        if(m_filename.empty()){
            m_filename = "../log/log_INFO.txt";
        }
        m_filestream.open(m_filename);
    }
/**
 * @brief 
 * @param {LogAppenderDefine} lad 里面包含format和  file 信息
 * @description: 
 * @return {*}
 */    
FileLogAppender::FileLogAppender(struct LogAppenderDefine lad){
    m_level = lad.level;
    auto fmt = LogFormatter::ptr( new LogFormatter(lad.formatter));
    if(!fmt->isError()){
        setLogFormatter(fmt);
        hasFormatter = true;
    }
    else {
        std::cout << "the formattrer = ' " << lad.formatter << "' is invalid. "<<std::endl;
    }
    // only have value, the value would be change 
    if(!lad.file.empty()){
        m_filename = lad.file;
        if(m_filename.empty()){
            m_filename = "../log/log_INFO.txt";
        }
    }
    m_filestream.open(m_filename);
}
/**
 * @brief 周期性的重启文件，防止文件被删除，并将 event 信息已既定格式输出到文件中 
 * @param {Level} level
 * @param {shared_ptr<Logger>} logger
 * @param {ptr} event
 * @description: 
 * @return {*}
 */
void FileLogAppender::log(LogLevel::Level level ,std::shared_ptr<Logger> logger,LogEvent::ptr event){
    MutexType::Lock lock(m_mutex);
    if(level >= m_level){
        // 写文件，周期性reopen。防止文件被删除
        uint64_t now = time(0);
        if(now != lastTime){
            reopen(1);
            lastTime = now;
        }
        m_filestream << m_formatter->format(level ,logger,event);
        if(m_filestream.fail()){
            std::cout << "file log error" <<std::endl;
        }
    }
}
/**
 * @brief 重启文件
 * @param {int} type
 * @description: 
 * @return {*}
 */
bool FileLogAppender::reopen(int type){
    if(m_filestream) {
        m_filestream.close();
    }
    if(type == 0)
        m_filestream.open(m_filename);
    else 
        m_filestream.open(m_filename,std::ios::app);
    return !!m_filestream;// 可能只对文件系统可以这样？(！->false 再！-> true)
}
/**
 * @brief 利用 yaml 输出文件 appender 
 * @description: 
 * @return {*}
 */
std::string FileLogAppender::toYamlString() {
    MutexType::Lock lock(m_mutex);
    YAML::Node node;
    node["type"] = "FileLogAppender";
    if(!(m_formatter->getPattern()).empty()){
        node["formatter"] = m_formatter->getPattern();
    }
    node["level"] = LogLevel::ToString(m_level);
    node["file"] = m_filename;
    std::stringstream ss;
    ss << node;
    return ss.str();
}
/**
 * @brief  初始化输出格式 
 * @param {LogAppenderDefine} lad
 * @description: 
 * @return {*}
 */
StdoutLogAppender::StdoutLogAppender(struct LogAppenderDefine lad){
    m_level = lad.level;
    auto fmt = LogFormatter::ptr( new LogFormatter(lad.formatter));
    if(!fmt->isError()){
        setLogFormatter(fmt);
    }
}
void StdoutLogAppender::log(LogLevel::Level level,std::shared_ptr<Logger> logger,LogEvent::ptr event){
    MutexType::Lock lock(m_mutex);
    if(level >= m_level){
        std::cout << m_formatter->format(level ,logger,event);
    }
}
std::string StdoutLogAppender::toYamlString(){
    MutexType::Lock lk(m_mutex);
    YAML::Node node;
    node["type"] = "StdoutLogAppender";
    if(!(m_formatter->getPattern()).empty()){
        node["formatter"] = m_formatter->getPattern();
    }
    node["level"] = LogLevel::ToString(m_level);
    std::stringstream ss;
    ss << node;
    return ss.str();
}
/**
 * @brief 通过 string 格式初始化输出的 format 
 * @param {string&} patterm
 * @description: 
 * @return {*}
 */
LogFormatter::LogFormatter(const std::string& patterm )
    :m_pattern(patterm){
        init();
    }
std::string LogFormatter::format(LogLevel::Level level,std::shared_ptr<Logger> logger,LogEvent::ptr event) {
    std::stringstream  ss;
    // 不是很懂
    for(auto &i : m_items){
        i->format(ss,logger,level,event);
    }
    return ss.str();
}
// %xxx %xxx{xxx} %%
/**
 * @brief 初始化 解析format格式，创建对应的m_item
 * @description: 
 * 日志格式器，执行日志格式化，负责日志格式的初始化。
 * 解析日志格式，将用户自定义的日志格式，解析为对应的FormatItem。
 * 日志格式举例：%d{%Y-%m-%d %H:%M:%S}%T%t%T%N%T%F%T[%p]%T[%c]%T%f:%l%T%m%n
 * 格式解析：
 * ？？？有哪些格式，用什么来表示的？
 * %d{%Y-%m-%d %H:%M:%S} : %d 标识输出的是时间 {%Y-%m-%d %H:%M:%S}为时间格式，可选 DateTimeFormatItem
 * %T : Tab[\t]            TabFormatItem
 * %t : 线程id             ThreadIdFormatItem
 * %N : 线程名称           ThreadNameFormatItem
 * %F : 协程id             FiberIdFormatItem
 * %p : 日志级别           LevelFormatItem       
 * %c : 日志名称           NameFormatItem
 * %f : 文件名             FilenameFormatItem
 * %l : 行号               LineFormatItem
 * %m : 日志内容           MessageFormatItem
 * %n : 换行符[\r\n]       NewLineFormatItem
 * @return {*}
 */
void LogFormatter::init(){
    //字符串解析部分，分成三块 1内容 2参数 3是否是命令
    // 存放处理结果的串，第一个是命令，第二个是参数，根据第三个判断是否字符串
    std::vector<std::tuple<std::string,std::string,int> >vec;
    std::string nstr;                               //  存放[ ] 和 %%
    for (size_t i = 0; i < m_pattern.size(); i++)
    {
        // 总过程
        // 1、读取奇特字符[ ]等
        // 2、读取 %% 字符
        // 3、读取 %d 类型字符，并查看后面是否有{}进一步描述字符
        // 4、将读取结果保存到vec 中后面通过宏定义解析

        if(m_pattern[i]!='%'){//不是%直接添加到结果字符串
            nstr.append(1,m_pattern[i]);
            continue;
        }
        if((i+1)<m_pattern.size()){//%% 时 表示需要添加一个%字符，保留后一个 %
            if(m_pattern[i+1]=='%'){
                nstr.append(1,'%');
                continue;
            }
        }
        //接下来是 %x 的情况
        size_t n = i + 1;       //n表示 % 后面的一个字符的序号
        int fmt_status = 0;     // 1：表示在解析{%Y-%m-%d %H:%M:%S}时间，0 表示解析其他
        size_t fmt_begin = 0;   //记录 { 的开始位置
        
        std::string str;        //内容字符串（%m中的m)
        std::string fmt;        //时间格式 {} 中的格式 %Y-%m-%d %H:%M:%S
        while(n<m_pattern.size()){
            if(!fmt_status&&(!isalpha(m_pattern[n])
                        &&m_pattern[n]!='{'
                        &&m_pattern[n]!='}')){              // 遇到非字母字符一般来说是% "%b%" 到后一个%退出
                str = m_pattern.substr(i + 1, n - i - 1);   // 返回 累计读到的字符 退出
                break;
            }
            if(fmt_status==0){                                  // 解析其他的状态
                if(m_pattern[n]=='{'){                          // 遇到解析时间{}
                    str = m_pattern.substr(i + 1, n - i - 1);   // 将前面解析到的字符先保存起来
//                    std::cout << "*" << str << std::endl;
                    fmt_status = 1;                             //切换到解析时间的状态
                    fmt_begin = n;                              //记录{位置
                    ++n;
                    continue;
                }
            }else if(fmt_status==1){                            // 处于解析时间{}状态
                if(m_pattern[n]=='}'){                          // 解析到}末尾
                    fmt = m_pattern.substr(fmt_begin + 1
                        , n - fmt_begin - 1);                   //将时间解析{}中的内容提取出来
//                    std::cout << "#" << fmt << std::endl;
                    fmt_status = 0;                             //找到截取字符后，状态切换回原来的
                    ++n;                                        //跳转到下一个字符的位置
                    break;                                      
                }
            }
            ++n;
            if(n==m_pattern.size()){                            //若到字符串最尾部
                if(str.empty())
                    str = m_pattern.substr(i + 1);              // 读完了保存剩下的字符退出
            }
        }
        if(fmt_status==0){
            if(!nstr.empty()){
                vec.push_back(std::make_tuple(nstr,std::string(), 0));      // 将[]等特殊字符保存
                nstr.clear();
            }
            vec.push_back(std::make_tuple(str, fmt, 1));                    //  保存类型字符和其格式，比如说d 和 {}中的元素
            i = n - 1;
        }
        else if (fmt_status == 1)
        {
            // std::cout << "pattern parse error " << m_pattern << " - " << m_pattern.substr(i) << std::endl;
            vec.push_back(std::make_tuple("<<pattern_error>>", fmt, 0));
        }
    }
    if(!nstr.empty()){
        vec.push_back(std::make_tuple(nstr, "", 0));
    }
    // 创建 map ，将类型字符和 封装之后Item 构造函数相连
    static std::map<std::string
        , std::function<FormatItem::ptr(const std::string &str)>> s_format_items = {
#define XX(str, C)    \
    {#str, [](const std::string &fmt) { return FormatItem::ptr(new C(fmt));}}
        
        XX(m, MessageFormatItem),
        XX(p, LevelFormatItem),
        XX(r, ElapseFormatItem),
        XX(c, NameFormatItem),
        XX(t, ThreadIdFormatItem),
        XX(n, NewLineFormatItem),
        XX(d, DateTimeFormatItem),
        XX(f, FilenameFormatItem),
        XX(l, LineFormatItem),
        XX(T, TabFormatItem),
        XX(F, FiberIdFormatItem)
#undef XX
    };
    for(auto& i:vec){
        if(std::get<2>(i)==0){
            // 不为类型字符，使用StringFormatItem最后直接输出 [ ，]，\,|等 
            m_items.push_back(FormatItem::ptr(new StringFormatItem(std::get<0>(i))));
        }else{
            // 从s_format_items找到类型字符对应的构造函数
            auto it = s_format_items.find(std::get<0>(i));
            // 找不到的一律按StringFormatItem处理
            if(it==s_format_items.end()){
                m_items.push_back(FormatItem::ptr(new StringFormatItem("<<error_format %" + std::get<0>(i) + ">>")));
            }else{
                // 找到将参数传进去 Item 构造函数创建对应的 FormatItem
                m_items.push_back(it->second(std::get<1>(i)));
            }
        }
    }
}
/**
 * @brief 当前logger 事件的构造函数
 * @description: 
 * @return {*}
 */
// LogEvent
LogEvent::LogEvent(const char* file,std::shared_ptr<Logger> logger,LogLevel::Level level,int32_t line,uint32_t elapse,uint32_t threadId
        ,uint32_t fiberId,uint32_t time,const std::string& content)
        :m_file(file)
        ,m_line(line)
        ,m_elapse(elapse)
        ,m_threadId(threadId)
        ,m_fiberId(fiberId)
        ,m_time(time)
        ,m_content(content)
        ,m_logger(logger)
        ,m_level(level){

        }
std::stringstream& LogEvent::getSS(){
    return m_ss;
}

//LogEventWrap
LogEventWrap::LogEventWrap(LogEvent::ptr e)
    :m_event(e){}
LogEventWrap::~LogEventWrap(){
    m_event->getLogger()->log(m_event->getLevel(),m_event);
}
std::stringstream& LogEventWrap::getSS(){
    
    return m_event->getSS();
}

// 全局的logger配置 
mysylar::ConfigVar<std::set<LoggerDefine> >::ptr g_loggers_define
    = mysylar::Config::Lookup("logs",std::set<LoggerDefine>(),"logs");
    
/**
 * @brief 主要为了给 logger配置 添加回调函数
 * @description: 
 * @return {*}
 */
struct LogIniter {
    LogIniter(){
        g_loggers_define->addListener([](const std::set<LoggerDefine>& old_value 
            , const std::set<LoggerDefine>& new_value)-> void{
                for(auto &n : new_value){
                    auto o = old_value.find(n);
                    Logger::ptr logger;
                    if(o == old_value.end()){
                        // 新增  
                        logger = SYLAR_LOG_NAME(n.name);
                        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "callback add new logger : " << n.name;
                    }else{
                        // 修改 
                        if(!(n == *o)){
                            logger = SYLAR_LOG_NAME(n.name);
                        }else {
                            continue;
                        }
                        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "callback modify logger : " << n.name;
                    }
                    logger->setLevel(n.level);
                    if(!n.formatter.empty()){
                        logger->setFormatter(LogFormatter::ptr(new LogFormatter(n.formatter)));
                    }
                    logger->clearAppender();
                    // 重新创建各种appender
                    for(auto& a : n.appender_define){
                        LogAppender::ptr appender;
                        if(a.type == 1){
                            appender.reset(new StdoutLogAppender(a));
                            
                        }else if(a.type == 2){
                            appender.reset(new FileLogAppender(a));
                        }
                        logger->setLevel(a.level);
                        logger->addAppender(appender);
                        
                    }           
                }
                // 删除 
                for(auto& o : old_value){
                    auto n = new_value.find(o);
                    // 不在新的里面需要删除掉
                    if(n == new_value.end()){
                        auto logger = SYLAR_LOG_NAME(o.name);
                        SYLAR_LOG_INFO(SYLAR_LOG_ROOT()) << "callback delete logger " << o.name;
                        // 将level设置得很大就不会被输出出来了
                        logger->setLevel((LogLevel::Level)100 );
                        logger->clearAppender();
                    }
                    
                }
            
        });
    }
};

static LogIniter __log_init; // 在main函数之前进行初始化
//LoggerManager
LoggerManager::LoggerManager(){
    m_root.reset(new Logger());

    m_root->addAppender(LogAppender::ptr(new StdoutLogAppender()));
    // m_root->addAppender(LogAppender::ptr(new FileLogAppender()));
    m_loggers[m_root->getName()] = m_root;

    init();

}
void LoggerManager::init(){

}
Logger::ptr LoggerManager::getLogger(const std::string& name)  {
    MutexType::Lock lk(m_mutex);
    auto it = m_loggers.find(name);
    if(it != m_loggers.end()){
        return it->second;
    }
    Logger::ptr logger(new Logger(name));
    logger->setRoot(m_root);
    m_loggers[name] = logger;
    return logger;
}
std::string LoggerManager::toYamlString() {
    MutexType::Lock lk(m_mutex);
    YAML::Node node;
    for(auto & i : m_loggers){
        node["loggers"].push_back(YAML::Load(i.second->toYamlString()));
    }
    std::stringstream ss;
    ss << node;
    return ss.str();
};
}

