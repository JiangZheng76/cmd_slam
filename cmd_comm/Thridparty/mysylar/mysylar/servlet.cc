#include "servlet.h"
#include <fnmatch.h>

namespace mysylar{

namespace http{


int32_t FunctionServlet::handle(HttpRequest::ptr req,
        HttpResponse::ptr rsp,HttpSession::ptr session) {
    return m_cb(req,rsp,session);
}
ServletDispatch::ServletDispatch()
    :Servlet("ServletDispatch"){
        m_default.reset(new NotFoundServlet());
}
int32_t ServletDispatch::handle(HttpRequest::ptr req,
        HttpResponse::ptr rsp,HttpSession::ptr session) {
    auto slt = getMatchServlet(req->getPath());
    if(slt){
        return slt->handle(req,rsp,session);
    }
    return m_default->handle(req,rsp,session);
}
void ServletDispatch::addServlet(const std::string& uri,Servlet::ptr slt){
    RWMutexType::WriteLock lk(m_mutex);
    m_datas[uri] = slt;
}
void ServletDispatch::addServlet(const std::string& uri,FunctionServlet::Callback cb){
    RWMutexType::WriteLock lk(m_mutex);
    m_datas[uri].reset(new FunctionServlet(cb));
}
void ServletDispatch::addGlobsServlet(const std::string& uri,Servlet::ptr slt){
    RWMutexType::WriteLock lf(m_mutex);
    for(auto it = m_globs.begin(); it != m_globs.end();++it){
        // fnmatch 模糊匹配
        if(!fnmatch(it->first.c_str(), uri.c_str(),0)){
            m_globs.erase(it);
            break;
        }
    }
    m_globs.push_back(std::make_pair(uri,slt));
}
void ServletDispatch::addGlobsServlet(const std::string& uri,FunctionServlet::Callback cb){
    addGlobsServlet(uri,Servlet::ptr(new FunctionServlet(cb)));
}
void ServletDispatch::delServlet(const std::string& uri){
    RWMutexType::WriteLock lk(m_mutex);
    m_datas.erase(uri);
}
void ServletDispatch::delGlobServlet(const std::string& uri){
    RWMutexType::WriteLock lk(m_mutex);
    for(auto it = m_globs.begin(); it != m_globs.end();++it){
        if(it->first == uri){
            m_globs.erase(it);
            break;
        }
    }
}
Servlet::ptr ServletDispatch::getServlet(const std::string& uri){
    RWMutexType::ReadLock lk(m_mutex);
    auto it = m_datas.find(uri);
    return it == m_datas.end() ? nullptr : it->second;
}
Servlet::ptr ServletDispatch::getGlobServlet(const std::string& uri){
    RWMutexType::ReadLock lk(m_mutex);
    for(auto it = m_globs.begin(); it != m_globs.end();++it){
        if(it->first == uri){
            return it->second;
        }
    }
    return nullptr;
}
/**
 * @brief 获得精准匹配或者是模糊匹配的 servlet
 * @param {string&} uri
 * @description: 
 * @return {*}
 */
Servlet::ptr ServletDispatch::getMatchServlet(const std::string& uri){
    RWMutexType::ReadLock lk(m_mutex);
    auto it = m_datas.find(uri);
    if(it != m_datas.end()) {
        return it->second;   
    }
    for(auto it = m_globs.begin(); it != m_globs.end();++it){
        if(!fnmatch(it->first.c_str(), uri.c_str(),0)){
            return it->second;
        }
    }
    return m_default;
}
NotFoundServlet::NotFoundServlet():Servlet("notFoundServlet"){
    m_content = "<html><head><title>404 Not Found"
        "</title></head><body><center><h1>404 Not Found</h1></center>"
        "<hr><center>" + m_name + "</center></body></html>";
}
int32_t NotFoundServlet::handle(HttpRequest::ptr req,
            HttpResponse::ptr rsp,HttpSession::ptr session) {
    rsp->setHeaders("Content-Type","text/html");
    rsp->setStatus(HttpStatus::NOT_FOUND);
    rsp->setBody(m_content);
    return 0;
}

}

}