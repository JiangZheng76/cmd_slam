#include "msg_loopframe.hpp"

namespace cmd {
static LoggerPtr g_logger_sys = SYLAR_LOG_NAME("CMD_SLAM");
MsgLoopframe::MsgLoopframe() : m_is_update_msg(false), m_msgtype(5, 0) {}
MsgLoopframe::MsgLoopframe(MsgType msgtype)
    : m_is_update_msg(false), m_msgtype(msgtype), m_ref_cf(0) {
  SYLAR_ASSERT2(m_msgtype.size() == 5,
                "msgtype 长度出错 m_msgtype length :" + m_msgtype.size());
}
MsgLoopframe::~MsgLoopframe() {
  // SYLAR_LOG_DEBUG(g_logger_sys) << dump();
}
void MsgLoopframe::setMsgType(int msg_size) {
  m_msgtype[0] = msg_size;
  m_msgtype[1] = (int)m_is_update_msg;
  m_msgtype[2] = m_lf_id;
  m_msgtype[3] = m_client_id;
  m_msgtype[4] = 0;
}
void MsgLoopframe::setMsgType(MsgType msgtype) { m_msgtype = msgtype; }
std::string MsgLoopframe::dump() {
  std::stringstream ss;
  ss << "MsgLoopframe INFO: "
     << "[lf id: " << m_lf_id << ",client id: " << m_client_id
     << ",incoming id: " << m_incoming_id << ",timestamp: " << m_timestamp
     << ",point nums : " << m_msg_points.size() << "]";
  return ss.str();
}
Calibration::Calibration()
    : m_k(KType::Identity()), m_img_dims(), m_dist_coeffs(), m_intrinsics() {}

}  // namespace cmd