#include "msg_loopframe.hpp"

namespace cmd{

MsgLoopframe::MsgLoopframe()
    :m_is_update_msg(false)
    ,m_msgtype(5,0){

}
void MsgLoopframe::setMsgType(int msg_size){
    m_msgtype[0] = msg_size;
    m_msgtype[1] = (int)m_is_update_msg;
    m_msgtype[2] = m_kf_id;
    m_msgtype[3] = m_client_id;
    m_msgtype[4] = 0;
}
void MsgLoopframe::setMsgType(MsgType msgtype){
    m_msgtype = msgtype;
}
std::string MsgLoopframe::dump(){
    std::stringstream ss;
    ss << "MsgLoopframe::dump()" 
        << "还没写";
    return ss.str();
}

}