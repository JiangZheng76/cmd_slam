#include "communicator.hpp"

namespace cmd
{
    static LoggerPtr g_logger_sys = SYLAR_LOG_NAME("system");

    FrontEndComm::FrontEndComm(std::string ip, std::string port)
        :CommunicatorBase()
    {
        std::stringstream ss;
        ss << ip << ":" << port;
        IPv4AddressPtr addr = Address::LookupAnyIPAddress(ss.str());
        if (!addr)
        {
            SYLAR_LOG_ERROR(g_logger_sys) << "ip:" << ip << ":" << port << " error!";
            SYLAR_ASSERT(false);
        }
        SocketPtr sock = Socket::CreateTCP(addr);
        bool rt = sock->connect(addr);
        if (!rt)
        {
            SYLAR_LOG_ERROR(g_logger_sys) << "connect error";
            sock->dump(std::cout);
            SYLAR_ASSERT(false);
        }
        // 重新绑定 socket_stream
        reset(sock, true);
        SYLAR_LOG_ERROR(g_logger_sys) << "connect to backend success";
        sock->dump(std::cout);
    }
    void FrontEndComm::Run()
    {
        Thread thread(std::bind(&FrontEndComm::recvMsg, this), "recv msg thread");
        thread.detach();

        SYLAR_LOG_INFO(g_logger_sys) << "Agent_" << m_client_id << " START communicator.";
        // 修改验证通信
        while (true)
        {
            int check_num_map;

            this->processSendMsgLoopframe();
            this->processBufferIn();
            this->processBufferOut();

            if (this->tryLock())
            {
                this->processRecvMsgLoopframe();
                this->unLock();
            }

            if (this->shallFinish())
            {
                showResult();
                break;
            }
            usleep(500);
        }
        std::unique_lock<std::mutex> lock(m_mtx_finish);
        m_is_finished = true;

        SYLAR_LOG_INFO(g_logger_sys) << "Agent_" << m_client_id << " End communicator.";
    }
    /// @brief 处理接收到的 msg
    void FrontEndComm::processRecvMsgLoopframe()
    {
        std::unique_lock<std::mutex> lk(m_mtx_in);
        while (!m_buf_msg_in.empty())
        {
            // SYLAR_LOG_WARN(g_logger_sys) << "接收到新帧!";
            SYLAR_ASSERT2(false, "接收到新帧")
            m_buf_msg_in.clear();
        }
    }

    /// @brief 处理准备发送的 msg,仅负责打包，ref 的处理不在这里
    void FrontEndComm::processSendMsgLoopframe()
    {
        // * step 1 关键帧队列上锁
        std::unique_lock<std::mutex>(m_mtx_out);
        int cnt = 0;
        // * step 2 循环处理关键帧buf
        while (!m_buf_msg_out.empty())
        {
            auto msg = m_buf_msg_out.front();
            m_buf_msg_out.pop_front();

            DataBundlePtr map_chunk(new DataBundle());
            map_chunk->m_lfs.push_back(msg);

            this->passDataBundle(map_chunk);
        }
    }
    /// @brief 获取m_ref_num 帧的 ref
    /// @param msg
    void FrontEndComm::publishMsg(MsgLoopframePtr msg)
    {
        if (m_process_lfs.size() != 0)
        {
            SYLAR_ASSERT(m_process_lfs.size() <= m_ref_num);
            for (auto it = m_process_lfs.begin(); it != m_process_lfs.end(); it++)
            {
                MsgLoopframePtr ref = *it;
                TransMatrixType Tcf = msg->m_twc.inverse() * ref->m_twc;
                msg->m_ref_id.push_back(ref->m_lf_id);
                msg->m_ref_cf.push_back(Tcf);
            }

            m_process_lfs.push_front(msg);
            while(m_process_lfs.size() > m_ref_num){
                m_process_lfs.pop_back();
            }
        }
        
        std::unique_lock<std::mutex>(m_mtx_out);
        m_buf_msg_out.push_back(msg);
    }
}