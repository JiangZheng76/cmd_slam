#include "backend.hpp"

#include "agent.hpp"
#include "loop_handler.hpp"
#include "map.hpp"
#include "visualization/pangolin_viewer.hpp"

namespace cmd {
static LoggerPtr g_logger_backend = SYLAR_LOG_NAME("Backend");
CmdBackend::CmdBackend() {
  m_viewer = std::make_shared<PangolinViewer>(VIEWER_WIDTH, VIEWER_HIGH);
  m_mapmanager = std::make_shared<Mapmanager>(m_viewer);
  // 开启等待任务线程
  m_backend_thread.reset(
      new Thread(std::bind(&CmdBackend::Run, this), "backend thread."));
}
void CmdBackend::Run() {
  SYLAR_LOG_DEBUG(g_logger_backend) << "--> START cmd-slam backend.";
  acceptAgent();
  connectSocket();
  SYLAR_LOG_DEBUG(g_logger_backend) << "<-- END cmd-slam backend.";
}
void CmdBackend::acceptAgent() {
  std::stringstream ss;
  ss << COMM_IP << ":" << COMM_PORT;
  std::string host = ss.str();
  IPv4AddressPtr addr = Address::LookupAnyIPAddress(host);
  if (!addr) {
    SYLAR_ASSERT2(false, "找不到 host:" + host);
  }
  m_sock = Socket::CreateTCP(addr);
  bool rt = m_sock->bind(addr);
  if (!rt) {
    SYLAR_ASSERT2(false, "bind 失败.");
  }

  m_sock->listen();
  std::stringstream ss_sock;
  m_sock->dump(ss_sock);
  SYLAR_LOG_INFO(g_logger_backend) << "创建监听 socket " << ss_sock.str();
}
int CmdBackend::get_counter() {
  static int client_id = 0;
  client_id++;
  return client_id;
}
void CmdBackend::connectSocket() {
  while (true) {
    SocketPtr comm_sock = m_sock->accept();
    if (!comm_sock) {
      SYLAR_LOG_ERROR(g_logger_backend) << "accept socket error.";
      // comm_sock->dump(std::cout);
      continue;
    }
    AgentHandlerPtr agent(
        new AgentHandler(get_counter(), comm_sock, m_mapmanager));
    m_agents.push_back(agent);

    SYLAR_LOG_INFO(g_logger_backend)
        << "ACCEPT NEW Agent_" << agent->m_client_id;
  }
}
}  // namespace cmd