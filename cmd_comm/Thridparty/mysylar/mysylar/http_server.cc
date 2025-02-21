#include "http_server.h"
#include "http_session.h"
namespace mysylar{
namespace http{


mysylar::Logger::ptr g_logger = SYLAR_LOG_ROOT();
HttpServer::HttpServer(bool keepalive,IOManager* worker
    ,IOManager* accept_worker
    )
    :TcpServer(worker,accept_worker)
    ,m_keep_alive(keepalive)
    ,m_dispatch(new ServletDispatch()){

}
void HttpServer::handleClient(Socket::ptr client) {
    // 会话管理 http 链接
    HttpSession::ptr session(new HttpSession(client));
    do{
        // 接收 http 请求
        auto req = session->recvRequest();
        if(!req){
            // 假的 返回 404
            HttpResponse::ptr rsp(new HttpResponse(0x11));
            m_dispatch->getDefault()->handle(req,rsp,session);
            break;
        } else {
            HttpResponse::ptr rsp(new HttpResponse(req->getVersion(),m_keep_alive || !req->isClose()));
            // servlet 分发处理 http 请求
            m_dispatch->handle(req,rsp,session) ;        
            // session 回复处理
            session->sendResponse(rsp);
            SYLAR_LOG_INFO(g_logger) << "RESPONSE \n" << rsp->toString();
        }
    }while(isKeepalive());
    session->close();
}



}
}