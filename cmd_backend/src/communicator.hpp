#pragma once
#include "comm_base.hpp"
#include "typedefs_backend.hpp"

namespace cmd {
class Communicator : public CommunicatorBase {
 public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Communicator(int client_id, SocketPtr sock,
               MapmanagerPtr mapMgr);

  virtual void Run() override;
  virtual void processRecvMsgLoopframe() override;
  virtual void processSendMsgLoopframe() override;

 public:
  MapmanagerPtr m_mapmanager;
};

}  // namespace cmd
