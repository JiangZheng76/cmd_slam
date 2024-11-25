#include "communicator.hpp"

#include "loop_handler.hpp"
#include "loopframe.hpp"
#include "map.hpp"

namespace cmd {
static LoggerPtr g_logger_sys = SYLAR_LOG_NAME("CMD-SLAM");
Communicator::Communicator(int client_id, SocketPtr sock,
                           LoopHandlerPtr loop_handler, MapmanagerPtr mapMgr)
    : CommunicatorBase(client_id, sock),
      m_loophander(loop_handler),
      m_mapmanager(mapMgr) {
  MessageContainer out_container;
  // 构建返回的设置client_id的msg
  out_container.msg_info.push_back(1);
  out_container.msg_info.push_back(m_client_id);
  // 填充到像 5个msg_type那么大
  while (out_container.msg_info.size() != ContainerSize * 5) {
    out_container.msg_info.push_back(0);
  }

  sendMsgContainer(out_container);
  SYLAR_LOG_INFO(g_logger_sys) << "Pass new ID " << m_client_id << " to client";
}

void Communicator::Run() {
  Thread thread(std::bind(&Communicator::recvMsg, this), "recv msg thread");
  thread.detach();

  SYLAR_LOG_INFO(g_logger_sys)
      << "Agent_" << m_client_id << " START communicator.";
  // 修改验证通信
  while (true) {
    int check_num_map;

    this->processBufferIn();
    this->processBufferOut();

    if (this->tryLock()) {
      this->processRecvMsgLoopframe();
      this->unLock();
    }
    // mapmanager_->ReturnMap(client_id_,check_num_map);
    // map_ = nullptr; //make sure the FrameManager is used correctly - this
    // will cause SEGFAULT if accessed

    if (this->shallFinish()) {
      showResult();
      break;
    }
    usleep(500);
  }
  std::unique_lock<std::mutex> lock(m_mtx_finish);
  m_is_finished = true;

  SYLAR_LOG_INFO(g_logger_sys)
      << "Agent_" << m_client_id << " End communicator.";
}
/// @brief 将接收到的 msg 转化成 lf，放入到待处理的队列中
void Communicator::processRecvMsgLoopframe() {
  std::unique_lock<std::mutex> lk(m_mtx_msg_in);
  // 遍历接受的loopframe，每次最多放5帧新帧 【cnt】
  size_t cnt = 0;
  while (cnt < 5 && !m_buf_msg_in.empty()) {
    MsgLoopframePtr msg = m_buf_msg_in.front();
    m_buf_msg_in.pop_front();
    if (!msg->m_is_update_msg) {  // 新帧
      LoopframePtr lf(new Loopframe(msg));
      // 是最新的帧
      m_mapmanager->addLoopframe(lf);
      m_loophander->pushLoopframe2Buf(lf);
    }
    cnt++;
  }
}
void Communicator::processSendMsgLoopframe() {
  while (!m_buf_msg_out.empty()) {
    SYLAR_ASSERT2(false, "backend 不发送数据！");
  }
}

}  // namespace cmd