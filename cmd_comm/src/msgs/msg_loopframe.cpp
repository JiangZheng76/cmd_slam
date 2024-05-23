#include "msg_loopframe.hpp"

namespace cmd{

MsgLoopframe::MsgLoopframe()
    :m_is_update_msg(false)
    ,m_msgtype(5,0){

}
MsgLoopframe::MsgLoopframe(MsgType msgtype)
    :m_is_update_msg(false)
    ,m_msgtype(msgtype){
    SYLAR_ASSERT2(m_msgtype.size() == 5,"msgtype 长度出错 m_msgtype length :" + m_msgtype.size());
}
void MsgLoopframe::setMsgType(int msg_size){
    m_msgtype[0] = msg_size;
    m_msgtype[1] = (int)m_is_update_msg;
    m_msgtype[2] = m_lf_id;
    m_msgtype[3] = m_client_id;
    m_msgtype[4] = 0;
}
void MsgLoopframe::setMsgType(MsgType msgtype){
    m_msgtype = msgtype;
}
std::string MsgLoopframe::dump(){
    std::stringstream ss;
    ss << "MsgLoopframe INFO:\n" 
        << "[lf id: " << m_lf_id
        << ",client id: " << m_client_id
        << ",incoming id: " << m_incoming_id
        << ",timestamp: " << m_timestamp 
        << "]";
    return ss.str();
}

}