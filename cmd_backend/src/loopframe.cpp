#include "loopframe.hpp"

#include "sophus/sim3.hpp"

#define POSE_R_WEIGHT 100

namespace cmd {
static LoggerPtr g_logger_sys = SYLAR_LOG_NAME("CMD-SLAM");

Point2::Point2(const MsgPoint &msg)
    : m_u(msg.m_u),
      m_v(msg.m_v),
      m_idepth_scaled(msg.m_idepth_scaled),
      m_maxRelBaseline(msg.m_maxRelBaseline),
      m_idepth_hessian(msg.m_idepth_hessian) {}
LoopEdge::LoopEdge(LoopframePtr from, LoopframePtr to,
                   const TransMatrixType &t_tf, precision_t icp_score,
                   precision_t sc_score, EdgeType type)
    : m_from_lf(from),
      m_to_lf(to),
      m_t_tf(t_tf),
      m_icp_score(icp_score),
      m_sc_score(sc_score),
      m_type(type) {
  precision_t dso_error = to->m_dso_error;
  precision_t scale_error = to->m_scale_error;

  m_info.setIdentity();
  m_info *= (1.0 / dso_error);
  m_info.topLeftCorner<3, 3>() *= scale_error > 0 ? (1.0 / scale_error) : 1e-9;
  // 由于dso对于旋转的估计比位移准确，所以将尺度控制权重先设置为与位移相同
  m_info.bottomRightCorner<3, 3>() *= POSE_R_WEIGHT;
}

Loopframe::Loopframe(MsgLoopframePtr msg, bool first_loopframe)
    : m_lf_id(msg->m_lf_id),
      m_incoming_id(msg->m_incoming_id),
      m_client_id(msg->m_client_id),
      m_timestamp(msg->m_timestamp),
      m_calib(msg->m_calib),
      m_twc(msg->m_twc),
      m_dso_error(msg->m_dso_error),
      m_scale_error(msg->m_scale_error),
      m_ab_exposure(msg->m_ab_exposure),
      m_is_first(first_loopframe),
      m_graph_added(false),
      m_is_display(false),
      m_pose_optimized(false),
      m_is_need_to_update_viewer(false),
      m_optimizing(false) {
  uint32_t size = msg->m_msg_points.size();
  m_points.reserve(size);
  for (int i = 0; i < size; i++) {
    Point2 p(msg->m_msg_points[i]);
    m_points.push_back(p);
  }
  m_ref_id = msg->m_ref_id;
  m_ref_cf.clear();
  for (auto &tcf : msg->m_ref_cf) {
    m_ref_cf.push_back(TransMatrixType(tcf));
  }

  m_dso_error = msg->m_dso_error * DSO_ERROR_SCALE;
  m_scale_error = msg->m_scale_error * SCALE_ERROR_SCALE;
  m_ab_exposure = msg->m_ab_exposure;
}

bool Loopframe::isFirstFrame() {
  if (m_edges.empty()) {
    return true;
  }
  return false;
}

LoopframePtr Loopframe::getReference() {
  if (isFirstFrame()) {
    return nullptr;
  }
  return m_edges[0]->m_from_lf;
}
void Loopframe::updateFromMsg(MsgLoopframePtr msg) {
  m_twc = ToOrthogonalTrans(msg->m_twc);
}
bool Loopframe::getRef2Cur(TransMatrixType &tcr) {
  if (isFirstFrame()) {
    return false;
  }
  tcr = m_edges[0]->m_t_tf;
  return true;
}
bool Loopframe::getCur2Ref(TransMatrixType &trc) {
  if (!getRef2Cur(trc)) {
    return false;
  }
  trc = trc.inverse();
  return true;
}
uint64_t Loopframe::getClientMerageKfId() {
  uint64_t res;
  res |= m_client_id;
  res <<= 32;
  res |= m_lf_id;
  return res;
}
bool Loopframe::addConstrant(LoopEdgePtr le) {
  m_edges.push_back(le);
  return true;
}
bool Loopframe::addReference(LoopframePtr ref, const TransMatrixType &t_tf,
                             precision_t icp_score, precision_t sc_score) {
  for (auto it = m_edges.begin(); it != m_edges.end(); it++) {
    if ((*it)->m_from_lf == ref) {
      (*it)->m_t_tf = t_tf;
      (*it)->m_icp_score = icp_score;
      (*it)->m_sc_score = sc_score;
      return true;
    }
  }
  LoopEdgePtr factor = std::make_shared<LoopEdge>(
      ref, shared_from_this(), t_tf, icp_score, sc_score, EdgeType::ODOMETRY);
  m_edges.push_back(factor);
  return true;
}
void Loopframe::updateFromCeres() {
  EigenMatrix sim_matrix = Sophus::Sim3d::exp(m_ceres_pose).matrix().inverse();
  m_twc = ToOrthogonalTrans(sim_matrix);
}
std::string Loopframe::dump() {
  std::stringstream ss;
  ss << "Loopframe INFO:\n"
     << "[lf id: " << m_lf_id << ",client id: " << m_client_id
     << ",incoming id: " << m_incoming_id << ",timestamp: " << m_timestamp
     << ",point nums:" << m_points.size() << "]";
  return ss.str();
}

}  // namespace cmd