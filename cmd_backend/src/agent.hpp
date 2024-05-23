#pragma once
#include "typedefs_backend.hpp"

namespace cmd
{

    class AgentHandler
    {
    public:
        AgentHandler(size_t client_id, SocketPtr sock, MapmanagerPtr mapMgr,LoopHandlerPtr loop);

        int_t m_client_id;
        CommPtr m_comm;
        MapmanagerPtr m_mapMgr;
    };
}
