/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-05-12 11:49:20
 * @LastEditors: Jiangzheng 2440877322@qq.com
 * @LastEditTime: 2024-03-10 12:32:09
 * @FilePath: /mysylar/mysylar/log.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "config.hpp"
#ifndef __MYSYLAR_LOG_H__
#define __MYSYLAR_LOG_H__

#include <string>
#include <stdint.h>
#include <list>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <map>
#include <memory>
#include <tuple>

#include "mutex.hpp"
#include "singleton.h"
#include "threads.h"

#define SYLAR_LOG_LEVEL(logger, level) \
    if(logger->getLevel() <= level)  \
        mysylar::LogEventWrap(mysylar::LogEvent::ptr(new mysylar::LogEvent(__FILE__,logger,level,__LINE__,0,mysylar::GetThreadId(),\
            mysylar::GetFiberId(),time(0)))).getSS()
#define SYLAR_LOG_ROOT() mysylar::LoggerMgr::GetInstance()->getRoot()
#define SYLAR_LOG_NAME(name) mysylar::LoggerMgr::GetInstance()->getLogger(name)

#define SYLAR_LOG_DEBUG(logger)  SYLAR_LOG_LEVEL(logger,mysylar::LogLevel::Level::DEBUG)
#define SYLAR_LOG_INFO(logger)  SYLAR_LOG_LEVEL(logger,mysylar::LogLevel::Level::INFO)
#define SYLAR_LOG_FATAL(logger)  SYLAR_LOG_LEVEL(logger,mysylar::LogLevel::Level::FATAL)
#define SYLAR_LOG_WARN(logger)  SYLAR_LOG_LEVEL(logger,mysylar::LogLevel::Level::WARN)
#define SYLAR_LOG_ERROR(logger)  SYLAR_LOG_LEVEL(logger,mysylar::LogLevel::Level::ERROR)
#define SYLAR_LOG_UNKNOW(logger)  SYLAR_LOG_LEVEL(logger,mysylar::LogLevel::Level::UNKNOW)


namespace mysylar{
    
class Logger;
class LogAppender;
class LoggerManager;
//  声明单例，保证全局仅有一个实例
typedef SingletonPtr<LoggerManager> LoggerMgr;


// 日志文件级别
class LogLevel {
public:
enum Level{
    UNKNOW = 0,
    DEBUG = 1,
    INFO = 2,
    WARN = 3,
    ERROR = 4,
    FATAL = 5
};
    static const char* ToString(LogLevel::Level level);
    static LogLevel::Level fromString(std::string str);
};

// 日志事件
class LogEvent {
public:
    typedef std::shared_ptr<LogEvent> ptr;
    LogEvent(const char* file,std::shared_ptr<Logger> logger , LogLevel::Level level,int32_t line,uint32_t elapse,uint32_t threadId
        ,uint32_t fiberId,uint32_t time,const std::string& content = "hello mysylar!");//  对于默认值，写在头文件申明中
    //  const 用法 https://blog.csdn.net/kaida1234/article/details/80403534 
    //  https://zhuanlan.zhihu.com/p/110159656
    const char * getFile() const {  return m_file;} 
    int32_t getLine() const { return m_line; }
    uint32_t getElapse() const { return m_elapse; } 
    uint32_t getThreadId() const { return m_threadId; }
    uint32_t getFiberId() const { return m_fiberId; }
    uint64_t getTime() const { return m_time; }
    std::string getContent() const { return m_ss.str();}// 字节流可以直接转化成 string 但是不可以有左值&
    std::shared_ptr<Logger> getLogger() const { return m_logger;}
    LogLevel::Level getLevel() const { return m_level; }
    std::stringstream& getSS();

private:
    const char* m_file = nullptr;       // 触发日志源码的文件名
    int32_t m_line = 0;                 // 行号  
    uint32_t m_elapse = 0;              //程序启动到现在的时间（毫秒）
    uint32_t m_threadId = 0;            // 线程号
    uint32_t m_fiberId = 0;             // 协程号
    uint64_t m_time = 0;                // 时间到毫秒级别 所以用64位
    std::string m_content;              // 内容
    std::stringstream m_ss;             // 内容字节流
    std::shared_ptr<Logger> m_logger;   // 负责输出的 logger
    LogLevel::Level m_level;            // 输出等级
};

// event warp 有什么用？？？
class LogEventWrap{
public:
    LogEventWrap(LogEvent::ptr e);
    ~LogEventWrap();
    std::stringstream& getSS();
public:
    LogEvent::ptr m_event;
};



// 日志输出格式
class LogFormatter {
public:
    typedef std::shared_ptr<LogFormatter> ptr;
    LogFormatter(const std::string & pattern);
    // 将event 修改成格式输出
    std::string format(LogLevel::Level level,std::shared_ptr<Logger> logger,LogEvent::ptr event);
    bool isError() { return m_error; }
    std::string getPattern() const { return m_pattern; };
    // 初始化 解析pattern模式
    void init();
public:
    // 虚拟内部类，用于LogFormatter内部使用
    class FormatItem {
    public:
        typedef std::shared_ptr<FormatItem> ptr;
        // 修改event格式 结果直接输出到流里面去
        // FormatItem(const std::string& fmt = "");//  这种用法 需要添加const 
        virtual void format(std::stringstream & os,std::shared_ptr<Logger> logger,LogLevel::Level level ,LogEvent::ptr event) = 0;
        // 因为有纯虚函数 所以需要一个虚的析构函数释放数据，一定要定义
        virtual ~FormatItem() {}
    };
private:
    std::string m_pattern;// 定义的数据格式
    std::vector<FormatItem::ptr> m_items;
    bool m_error = false;

};

//日志输出地
class LogAppender {
public:
    typedef std::shared_ptr<LogAppender> ptr;
    typedef SpinLock MutexType;
    friend class Logger;
    // LogAppender(); // 需要有默认构造函数
    virtual ~LogAppender() {}
    virtual void log(LogLevel::Level level,std::shared_ptr<Logger> logger,LogEvent::ptr event) = 0; // 纯虚函数，子类必须要实现

    void setLogFormatter(LogFormatter::ptr formatter);
    LogFormatter::ptr getFormatter() const { return m_formatter; }

    void setLevel(LogLevel::Level level){ m_level = level; } 
    LogLevel::Level getLevel() const { return m_level; }

    virtual std::string toYamlString() = 0;
    bool getHasFormatter() { return hasFormatter; };

protected:
    LogLevel::Level m_level;
    LogFormatter::ptr m_formatter;
    bool hasFormatter = false;
    MutexType m_mutex;                  // 防止formatter 突然改变造成 tostring错误
};


//  日志器 继承enable_shared_from_this 可以让他调用自己
class Logger : public std::enable_shared_from_this<Logger> {
public:
    typedef std::shared_ptr<Logger> ptr;
    typedef SpinLock MutexType;
    friend class LoggerManager;
    
    Logger(const std::string m_name="root");
    // 输出log
    void log(LogLevel::Level level,LogEvent::ptr event);

    // void debug(LogLevel::Level level,LogEvent::ptr event);
    void debug(LogEvent::ptr event);
    void info(LogEvent::ptr event);
    void warn(LogEvent::ptr event);
    void error(LogEvent::ptr event);
    void fatal(LogEvent::ptr event);

    void addAppender(LogAppender::ptr appender);
    void delAppender(LogAppender::ptr appender);
    void clearAppender(){ m_appenders.clear(); };
    
    void setLevel(LogLevel::Level val) { m_level = val;}
    void setRoot(Logger::ptr root) { m_root = root; }
    void setFormatter(LogFormatter::ptr val);

    const std::string& getName() const { return m_name;}
    LogLevel::Level getLevel() const { return m_level;}
    Logger::ptr getRoot() const { return m_root; }
    
    std::string toYamlString();
private:
    
    std::string m_name;                         //  log名称
    LogLevel::Level m_level;                    //  日志级别
    std::list<LogAppender::ptr> m_appenders;    //  Appender 集合
    LogFormatter::ptr m_formatter;              //  默认的format
    Logger::ptr m_root;                         //  每一个logger都有一个root
    MutexType m_mutex;                          //  防止信息修改 toString错误

};

// 输出到控制台的appender
class StdoutLogAppender : public LogAppender{
public:
    typedef std::shared_ptr<StdoutLogAppender> ptr;
    StdoutLogAppender(){};
    StdoutLogAppender(struct LogAppenderDefine lad);
    void log(LogLevel::Level level,std::shared_ptr<Logger> logger,LogEvent::ptr event) override; // 从基类中重载的实现

    std::string toYamlString() override;
private:
};
// 输出到file 的appender
class FileLogAppender : public LogAppender{
public:
    typedef std::shared_ptr<FileLogAppender> ptr;
    FileLogAppender(const std::string& filename="../log/log_INFO.txt");
    FileLogAppender(struct LogAppenderDefine lad);
    void log(LogLevel::Level level,std::shared_ptr<Logger> logger,LogEvent::ptr event) override;
    // 有问题重新打开文件
    // 默认参数表示清空里面的文件打开
    bool reopen(int type = 0);
    std::string toYamlString() override;
    
private:
    std::string m_filename;
    std::ofstream m_filestream;//#include <fstream>
    uint64_t lastTime = 0;

};

// 管理器 
// 没有Logger 从里面拿
class LoggerManager{
public:
    typedef SpinLock MutexType;
    LoggerManager();
    Logger::ptr getLogger(const std::string& name) ;
    void init();
    Logger::ptr getRoot() const { return m_root; }
    std::string toYamlString() ;
private:
    MutexType m_mutex;
    Logger::ptr m_root;
    std::map<std::string,Logger::ptr> m_loggers;
};

}


#endif