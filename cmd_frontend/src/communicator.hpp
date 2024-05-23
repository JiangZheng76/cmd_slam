#pragma once

// CODSV
#include "cmd_comm.hpp"

#define NO_LOOP_FINDER
#define NO_RELOC
#define CODSV_MOD

namespace cmd {


class FrontEndComm : public CommunicatorBase
    {
    public:
        FrontEndComm(std::string ip,std::string port);
        virtual ~FrontEndComm(){}
        
        virtual void Run() override;
        virtual void processRecvMsgLoopframe() override;

        virtual void processSendMsgLoopframe() override;

        void publishMsg(MsgLoopframePtr msg);

    public:
        std::list<MsgLoopframePtr> m_process_lfs;
        size_t m_ref_num = 4;
};

} //end ns
