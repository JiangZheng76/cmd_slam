/*
 * @Author: Jiangzheng 2440877322@qq.com
 * @Date: 2024-03-19 18:36:55
 * @LastEditors: Johnathan 2440877322@qq.com
 * @LastEditTime: 2024-07-06 12:53:35
 * @FilePath: /mysylar/mysylar/servlet.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SYLAR_SERVLET_H__
#define __SYLAR_SERVLET_H__
#include <memory>
#include "http/http.h"
#include "http/http_session.h"
#include <map>
#include "thread.h"
namespace mysylar{

namespace http{

class Servlet{
public:
    typedef std::shared_ptr<Servlet> ptr;

    Servlet(const std::string& name):m_name(name){}
    virtual ~Servlet(){}
    virtual int32_t handle(HttpRequest::ptr req,
        HttpResponse::ptr rsp,
        HttpSession::ptr session) = 0;
    
    const std::string& getName()const {return m_name;}
    
protected:
    std::string m_name;

};

class FunctionServlet : public Servlet{
public:
    typedef std::shared_ptr<FunctionServlet> ptr;
    typedef std::function<int32_t (HttpRequest::ptr req,
        HttpResponse::ptr rsp,
        HttpSession::ptr session)> Callback;
    FunctionServlet(Callback cb)
        :Servlet("FunctionServlet")
        ,m_cb(cb){}

    virtual ~FunctionServlet(){}
    virtual int32_t handle(HttpRequest::ptr req,
        HttpResponse::ptr rsp,
        HttpSession::ptr session) override;
private:
    Callback m_cb;
};

class ServletDispatch : public Servlet{
public:
    typedef std::shared_ptr<ServletDispatch> ptr;
    typedef RWMutex RWMutexType;
    ServletDispatch();
    virtual ~ServletDispatch(){}
    virtual int32_t handle(HttpRequest::ptr req,
        HttpResponse::ptr rsp,
        HttpSession::ptr session) override;

    void addServlet(const std::string& uri,Servlet::ptr slt);
    void addServlet(const std::string& uri,FunctionServlet::Callback cb);
    void addGlobsServlet(const std::string& uri,Servlet::ptr slt);
    void addGlobsServlet(const std::string& uri,FunctionServlet::Callback cb);

    void delServlet(const std::string& uri);
    void delGlobServlet(const std::string& uri);

    Servlet::ptr getDefault()const{ return m_default;}
    void setDefault(Servlet::ptr v){ m_default = v;};

    Servlet::ptr getServlet(const std::string& uri);
    Servlet::ptr getGlobServlet(const std::string& uri);

    Servlet::ptr getMatchServlet(const std::string& uri);
private:
    RWMutexType m_mutex;
    //  uri -> servlet 精准匹配
    std::unordered_map<std::string,Servlet::ptr> m_datas;
    // uri -> /sylar/* 模糊匹配
    std::vector<std::pair<std::string,Servlet::ptr>> m_globs;
    // 默认的 servlet
    Servlet::ptr m_default;

};

class NotFoundServlet : public Servlet{
public:
    typedef std::shared_ptr<NotFoundServlet> ptr;
    NotFoundServlet();
    virtual ~NotFoundServlet(){}

    virtual int32_t handle(HttpRequest::ptr req,
        HttpResponse::ptr rsp,
        HttpSession::ptr session) override;
private:
    std::string m_content;

};
}


}



#endif
