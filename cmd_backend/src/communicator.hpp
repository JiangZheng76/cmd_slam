#pragma once
#include "comm_base.hpp"
#include "typedefs_backend.hpp"

namespace cmd {
class Communicator : public CommunicatorBase {
 public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Communicator(int client_id, SocketPtr sock, LoopHandlerPtr loop_handler,
               MapmanagerPtr mapMgr);

  virtual void Run() override;
  virtual void processRecvMsgLoopframe() override;
  virtual void processSendMsgLoopframe() override;

 public:
  // PangolinLoopViewerPtr m_viewer;
  LoopHandlerPtr m_loophander;
  MapmanagerPtr m_mapmanager;
};

}  // namespace cmd
