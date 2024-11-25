#pragma once

// C++
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

// Socket Programming
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <atomic>

#include "typedefs_backend.hpp"

namespace cmd {

class CmdBackend {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CmdBackend();
  void Run();

 protected:
  void acceptAgent();
  void connectSocket();

  int get_counter();

  // Infrastructure
  std::vector<AgentHandlerPtr> m_agents;
  MapmanagerPtr m_mapmanager;
  LoopHandlerPtr m_loop;
  PangolinViewerPtr m_viewer;

  int m_agent_next_id = 0;

  ros::NodeHandle m_nh;
  SocketPtr m_sock;
  ThreadPtr m_backend_thread;

  // Device Counter
  std::atomic<int> m_counter, m_overall_counter;

  // Sync
  std::mutex mtx_num_agents_;
};

}  // namespace cmd
