#include "loopframe.hpp"
#include "sophus/sim3.hpp"

#define POSE_R_WEIGHT 100

namespace cmd
{
    static LoggerPtr g_logger_sys = SYLAR_LOG_NAME("system");

    Point::Point(const MsgPoint &msg)
        : m_u(msg.m_u), m_v(msg.m_v), m_idepth_scaled(msg.m_idepth_scaled), m_maxRelBaseline(msg.m_maxRelBaseline), m_idepth_hessian(msg.m_idepth_hessian)
    {
    }
    LoopEdge::LoopEdge(LoopframePtr from, LoopframePtr to, const TransMatrixType &t_tf, precision_t icp_score, precision_t sc_score)
        : m_from_lf(from), m_to_lf(to), m_t_tf(t_tf), m_is_added(false), m_icp_score(icp_score), m_sc_score(sc_score)
    {
        precision_t dso_error = to->m_dso_error;
        precision_t scale_error = to->m_scale_error;

        m_info.setIdentity();
        m_info *= (1.0 / dso_error);
        m_info.topLeftCorner<3, 3>() *=
            scale_error > 0 ? (1.0 / scale_error) : 1e-9;
        // 由于dso对于旋转的估计比位移准确，所以将尺度控制权重先设置为与位移相同
        m_info.bottomRightCorner<3, 3>() *= POSE_R_WEIGHT;
    }

    Loopframe::Loopframe(MsgLoopframePtr msg, bool first_loopframe)
        : m_lf_id(msg->m_lf_id), m_incoming_id(msg->m_incoming_id), m_client_id(msg->m_client_id), m_timestamp(msg->m_timestamp), m_calib(msg->m_calib), m_twc(msg->m_twc), m_dso_error(msg->m_dso_error), m_scale_error(msg->m_scale_error), m_ab_exposure(msg->m_ab_exposure), m_is_first(first_loopframe), m_graph_added(false), m_is_display(false), m_pose_optimized(false), m_is_need_to_update_viewer(false), m_optimizing(false)
    {

        uint32_t size = msg->m_msg_points.size();
        m_points.reserve(size);
        for (int i = 0; i < size; i++)
        {
            Point p(msg->m_msg_points[i]);
            m_points.push_back(p);
        }
        m_ref_id = msg->m_ref_id;
        m_ref_cf = msg->m_ref_cf;
    }

    bool Loopframe::isFirstFrame()
    {
        if (m_edges.empty())
        {
            return true;
        }
        return false;
    }

    LoopframePtr Loopframe::getReference()
    {
        if (isFirstFrame())
        {
            return nullptr;
        }
        return m_edges[0]->m_from_lf;
    }
    void Loopframe::updateFromMsg(MsgLoopframePtr msg)
    {
        m_twc = msg->m_twc;
    }
    bool Loopframe::getRef2Cur(TransMatrixType &tcr)
    {
        if (isFirstFrame())
        {
            return false;
        }
        tcr = m_edges[0]->m_t_tf;
        return true;
    }
    bool Loopframe::getCur2Ref(TransMatrixType &trc)
    {
        if (!getRef2Cur(trc))
        {
            return false;
        }
        trc = trc.inverse();
        return true;
    }
    uint64_t Loopframe::getClientMerageKfId(){
        uint64_t res;
        res |= m_client_id;
        res <<= 32;
        res |= m_lf_id;
        return res;
    }
    bool Loopframe::addConstrant(LoopEdgePtr le)
    {
        m_edges.push_back(le);
        return true;
    }
    bool Loopframe::addReference(LoopframePtr ref, const TransMatrixType &t_tf, precision_t icp_score, precision_t sc_score)
    {

        for (auto it = m_edges.begin(); it != m_edges.end(); it++)
        {
            if ((*it)->m_from_lf == ref)
            {
                (*it)->m_t_tf = t_tf;
                (*it)->m_icp_score = icp_score;
                (*it)->m_sc_score = sc_score;
                return true;
            }
        }
        LoopEdgePtr le(new LoopEdge(ref, shared_from_this(), t_tf, icp_score, sc_score));
        m_edges.push_back(le);
        return true;
    }
    void Loopframe::updateFromCeres()
    {
        Sophus::Vector7d sim_vec7;
        for (int i = 0; i < 7; i++)
        {
            sim_vec7[i] = m_ceres_pose[i];
        }
        Sophus::Sim3d sim = Sophus::Sim3d::exp(sim_vec7);
        m_twc = TransMatrixType(sim.matrix());
    }
    std::string Loopframe::dump()
    {
        SYLAR_LOG_INFO(g_logger_sys) << "未定义 loopframe dump()";
    }

}