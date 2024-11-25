#include "agent.hpp"

#include "communicator.hpp"

namespace cmd {

AgentHandler::AgentHandler(size_t client_id, SocketPtr sock,
                           MapmanagerPtr mapMgr, LoopHandlerPtr loop)
    : m_client_id(client_id), m_mapMgr(mapMgr) {
  m_comm.reset(new Communicator(client_id, sock, loop, mapMgr));
  // 启动 comm
  m_comm->action();
}

}  // namespace cmd