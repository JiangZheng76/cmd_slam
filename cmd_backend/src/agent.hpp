#pragma once
#include "typedefs_backend.hpp"

namespace cmd {

class AgentHandler {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AgentHandler(size_t client_id, SocketPtr sock, MapmanagerPtr mapMgr,
               LoopHandlerPtr loop);

  int_t m_client_id;
  CommPtr m_comm;
  MapmanagerPtr m_mapMgr;
};
}  // namespace cmd
