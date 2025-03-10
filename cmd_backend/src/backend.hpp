#pragma once

// C++
#include <memory>
#include <mutex>
#include <vector>
#include <thread>

// Socket Programming
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <atomic>

#include <ros/ros.h>

#include "typedefs_backend.hpp"

namespace cmd
{

    class CmdBackend
    {
    public:
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
        PangolinLoopViewerPtr m_viewer;

        int m_agent_next_id = 0;

        ros::NodeHandle m_nh;
        SocketPtr m_sock;
        ThreadPtr m_backend_thread;

        // Device Counter
        std::atomic<int> m_counter, m_overall_counter;

        // Sync
        std::mutex mtx_num_agents_;
    };

} // end ns
