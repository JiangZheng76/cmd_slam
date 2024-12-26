#include "comm_base.hpp"

namespace cmd {
static LoggerPtr g_logger_sys = SYLAR_LOG_NAME("CMD-SLAM");

CommunicatorBase::CommunicatorBase() {}
CommunicatorBase::CommunicatorBase(int client_id, SocketPtr sock)
    : SocketStream(sock, true), m_client_id(client_id), m_sock(sock) {}
void CommunicatorBase::action() {
  InitAndGetDuringTime(true);
  m_main_thread.reset(new Thread(std::bind(&CommunicatorBase::Run, this),
                                 "Communicator thread"));
}
void CommunicatorBase::passDataBundle(DataBundlePtr data) {
  std::unique_lock<std::mutex> lock(m_mtx_datas_out);
  m_buf_datas_out.push_back(data);
}
void CommunicatorBase::lock() { m_mtx_comm.lock(); }
void CommunicatorBase::unLock() { m_mtx_comm.unlock(); }
bool CommunicatorBase::tryLock() { return m_mtx_comm.try_lock(); }
int CommunicatorBase::getClientId() { return m_client_id; }

/// @brief 序列化 loopframe
/// @param lf
void CommunicatorBase::serialize(MsgLoopframePtr lf) {
  m_send_ser.str("");
  m_send_ser.clear();
  cereal::BinaryOutputArchive oarchive(m_send_ser);  // 设置序列化目的地
  oarchive(*lf);
  m_package_size_send = (int)m_send_ser.str().length();

  lf->setMsgType(m_package_size_send);  // 设置 lf 准备发送的 package 大小
}

// Message passing
static auto GetInAddr(struct sockaddr *sa) -> void * {
  return nullptr;
}  // get sockaddr, IPv4 or IPv6: 暂时不需要

/// @brief 客户端连接服务器
/// @param node ip地址
/// @param port 端口
/// @return
SocketPtr CommunicatorBase::connectToServer(const char *node,
                                            std::string port) {
  SYLAR_LOG_INFO(g_logger_sys)
      << "connnect to server [node:" << node << ", port:" << port << "]";

  std::vector<IPv4AddressPtr> res;
  IPv4AddressPtr address = IPv4Address::LookupAnyIPAddress(
      node, Socket::Family::IPv4, Socket::Type::TCP);
  SocketPtr sock(new Socket(Socket::Family::IPv4, Socket::Type::TCP));
  if (!sock->connect(address, 5000)) {
    return nullptr;
  }
  return sock;
}
/// @brief 此函数调用必须在序列化之后，先发送 MsgType 信息，再发送 msg package
/// @param msg_send MsgType 信息
/// @return 发送的大小
int CommunicatorBase::sendAll(MsgType &msg_send) {
  int len = msg_send.size();
  size_t byte_left = len * sizeof(msg_send[0]);
  int rt = write(&msg_send[0], byte_left);
  collectTotalSendByte(rt);
  return rt;
}
/// @brief 先发送 MsgType 信息，再发送 msg package
/// @param msg
/// @return
int CommunicatorBase::sendAll(std::stringstream &msg) {
  // SocketStream ss(m_sock,false);
  std::string tmp_msg = msg.str();
  int rt = write(&tmp_msg[0], tmp_msg.size());
  collectTotalSendByte(tmp_msg.size());
  return rt;
}
size_t CommunicatorBase::collectTotalSendByte(int bytes) {
  if (bytes != -1) {
    totalSendBytes_ += bytes;
  }
  return totalSendBytes_;
}
size_t CommunicatorBase::collectTotalRecvByte(int bytes) {
  if (bytes != -1) {
    totalRecvBytes_ += bytes;
  }
  return totalRecvBytes_;
}
long CommunicatorBase::InitAndGetDuringTime(bool reset) {
  if (reset) {
    start_time_ = std::chrono::steady_clock::now();
  }
  return std::chrono::duration_cast<std::chrono::seconds>(
             std::chrono::steady_clock::now() - start_time_)
      .count();
}
inline double calculateRate(double bytes, double time) { return bytes / time; }
int CommunicatorBase::recvAll(unsigned int sz, std::vector<char> &buffer) {
  size_t tot_bytes = 0;
  size_t n_bytes;
  // SocketStream ss(m_sock,false);
  buffer.resize(sz, 0);
  while (tot_bytes < sz) {
    n_bytes = read(&buffer[tot_bytes], sz - tot_bytes);
    if (n_bytes <= 0) {
      if (n_bytes == 0) {
        SYLAR_LOG_INFO(g_logger_sys)
            << "------- selectserver: [client:" << m_client_id
            << "] hung up. -------\n"
            << "during time: " << InitAndGetDuringTime(false) << "s\n"
            << "send bytes: " << collectTotalSendByte(0) / 1024 << "KB\n"
            << "send rate: "
            << calculateRate(collectTotalSendByte(0) / 1024,
                             InitAndGetDuringTime(false))
            << "KB/s\n"
            << "recv bytes: " << collectTotalRecvByte(0) / 1024 << "KB\n"
            << "recv rate: "
            << calculateRate(collectTotalRecvByte(0) / 1024,
                             InitAndGetDuringTime(false))
            << "KB/s\n";
      } else {
        perror("recv");
        SYLAR_LOG_ERROR(g_logger_sys)
            << "------- selectserver: [client:" << m_client_id
            << "] socket ERROR! -------\n"
            << "during time: " << InitAndGetDuringTime(false) << "s\n"
            << "send bytes: " << collectTotalSendByte(0) / 1024 << "KB\n"
            << "send rate: "
            << calculateRate(collectTotalSendByte(0) / 1024,
                             InitAndGetDuringTime(false))
            << "KB/s\n"
            << "recv bytes: " << collectTotalRecvByte(0) / 1024 << "KB\n"
            << "recv rate: "
            << calculateRate(collectTotalRecvByte(0) / 1024,
                             InitAndGetDuringTime(false))
            << "KB/s\n";
      }
      m_sock->dump(std::cout) << std::endl;
      return -1;
    }
    tot_bytes += n_bytes;
  }
  collectTotalRecvByte(tot_bytes);
  // 表示成功
  return 0;
}
/// @brief
/// @param sz
/// @param buffer
/// @return success 0，fail -1
int CommunicatorBase::recvAll(unsigned int sz, MsgType &buffer) {
  size_t tot_bytes = 0;
  size_t n_bytes;
  ByteArrayPtr ba(new ByteArray(sz));
  // SocketStream ss(m_sock,false);
  while (tot_bytes < sz) {
    // buffer[tot_bytes] 数组的偏移量
    n_bytes = read(&buffer[tot_bytes], sz - tot_bytes);
    if (n_bytes <= 0) {
      if (n_bytes == 0) {
        SYLAR_LOG_INFO(g_logger_sys)
            << "------- selectserver: [client:" << m_client_id
            << "] hung up. -------\n"
            << "during time: " << InitAndGetDuringTime(false) << "s\n"
            << "send bytes: " << collectTotalSendByte(0) / 1024 << "KB\n"
            << "send rate: "
            << calculateRate(collectTotalSendByte(0) / 1024,
                             InitAndGetDuringTime(false))
            << "KB/s\n"
            << "recv bytes: " << collectTotalRecvByte(0) / 1024 << "KB\n"
            << "recv rate: "
            << calculateRate(collectTotalRecvByte(0) / 1024,
                             InitAndGetDuringTime(false))
            << "KB/s\n";
      } else {
        perror("recv");
        SYLAR_LOG_ERROR(g_logger_sys)
            << "------- selectserver: [client:" << m_client_id
            << "] socket ERROR! -------\n"
            << "during time: " << InitAndGetDuringTime(false) << "s\n"
            << "send bytes: " << collectTotalSendByte(0) / 1024 << "KB\n"
            << "send rate: "
            << calculateRate(collectTotalSendByte(0) / 1024,
                             InitAndGetDuringTime(false))
            << "KB/s\n"
            << "recv bytes: " << collectTotalRecvByte(0) / 1024 << "KB\n"
            << "recv rate: "
            << calculateRate(collectTotalRecvByte(0) / 1024,
                             InitAndGetDuringTime(false))
            << "KB/s\n";
      }
      m_sock->dump(std::cout) << std::endl;
      return -1;
    }
    tot_bytes += n_bytes;
  }
  collectTotalRecvByte(tot_bytes);
  return 0;
}
/// @brief 接收数据的线程
void CommunicatorBase::recvMsg() {
  SYLAR_LOG_INFO(g_logger_sys) << "--> Start Recv Thread";
  size_t total_msg = 0;
  m_msgtype_container.resize(ContainerSize * 5);
  size_t type_size = sizeof(m_msgtype_container[0]) * ContainerSize * 5;
  while (true) {
    // ？？？ 不是很懂，一次性收了这么多不会污染后面的 msgloopframe
    // 么？（发送的时候也是一次性发送 10 个 msgtype，不够补 0）
    if (recvAll(type_size, m_msgtype_container)) {
      SYLAR_LOG_INFO(g_logger_sys) << "--> Exit Recv Thread";
      setFinish();
      break;
    }
    // 大小为 1 ，接受到 client id
    if (m_msgtype_container[0] == 1) {
      m_client_id = m_msgtype_container[1];
      SYLAR_LOG_INFO(g_logger_sys) << "--> Set Client ID: " << m_client_id;
      m_msgtype_container[0] = 0;  // 长度为 0 ，后面不再处理
    }

    // 接收 msgloopframe
    total_msg = 0;
    for (int i = 0; i < ContainerSize; i++) {
      total_msg += m_msgtype_container[i * 5];
    }
    std::unique_lock<std::mutex> lk(m_mtx_recv_buf);
    m_recv_buf.clear();  // 每一次接收前都要重置
    if (recvAll(total_msg, m_recv_buf)) {
      SYLAR_LOG_INFO(g_logger_sys) << "----> Exit Recv Thread\n";
      setFinish();
      break;
    }
    writeToBuffer();

    if (shallFinish()) {
      SYLAR_LOG_INFO(g_logger_sys) << "----> Exit Recv Thread\n";
      break;
    }
  }
}
/// @brief 将主线程中接收的 msgloopframe 字节流序列化
void CommunicatorBase::writeToBuffer() {
  MsgType tmp_msgtype(5);

  // 读取 msgtype
  std::unique_lock<std::mutex> lk(m_mtx_recv_data_info);
  for (int i = 0; i < ContainerSize; i++) {
    if (m_msgtype_container[i * 5] == 0) break;  //

    for (int j = 0; j < 5; j++) {
      tmp_msgtype[j] = m_msgtype_container[i * 5 + j];
    }
    m_buf_recv_info.push_back(tmp_msgtype);
  }

  // 读取二进制 msgloopframe 字节流
  size_t to_bytes = 0;
  for (int i = 0; i < ContainerSize; i++) {
    if (m_msgtype_container[i * 5] == 0) break;

    // 将接收到的数据，分离开来
    std::vector<char> tmp_data(m_msgtype_container[i * 5], 0);
    memcpy(&tmp_data[0], &m_recv_buf[to_bytes], m_msgtype_container[i * 5]);
    to_bytes += m_msgtype_container[i * 5];
    m_buf_recv_data.push_back(tmp_data);
  }
}
/// @brief 检查并取出数据到 m_tmp_recv_xxxx
/// @return
bool CommunicatorBase::checkBufferAndPop() {
  std::unique_lock<std::mutex> lk(m_mtx_recv_data_info);

  if (!m_buf_recv_data.empty()) {
    // m_tmp_recv_data = m_buf_recv_data.front();
    // m_buf_recv_data.pop_front();

    // m_tmp_recv_info = m_buf_recv_info.front();
    // m_buf_recv_info.pop_front();
    return true;
  }
  return false;
}
/// @brief 发送容器数据
/// @param msg
/// @return 返回发送的二进制 msgloopframe 数量
size_t CommunicatorBase::sendMsgContainer(MessageContainer &msg) {
  size_t info_bytes = sendAll(msg.msg_info);
  size_t data_bytes = sendAll(msg.msg_data);
  // SYLAR_LOG_DEBUG(g_logger_sys) << "sendMsgContainer() send data_bytes: "
  //                               << data_bytes << " bytes and info_bytes "
  //                               << info_bytes << " bytes";
  return data_bytes;
}

/// @brief 序列化需要发送的数据，并全部发送完毕
void CommunicatorBase::processBufferOut() {
  size_t cnt = 0;  // 限定每次只发送 5 帧
  MessageContainerList mcts;
  {
    std::unique_lock<std::mutex> lock(m_mtx_datas_out);
    while (cnt < 5 && !m_buf_datas_out.empty()) {
      DataBundlePtr db = m_buf_datas_out.front();
      m_buf_datas_out.pop_front();

      // 序列化 db 中的 msgloopframe,并发送
      while (!db->m_lfs.empty()) {
        MessageContainerPtr mct(new MessageContainer());
        MsgLoopframePtr lf = db->m_lfs.front();
        db->m_lfs.pop_front();
        serialize(lf);

        mct->msg_data << m_send_ser.str();
        mct->msg_info.insert(mct->msg_info.end(), lf->m_msgtype.begin(),
                             lf->m_msgtype.end());

        mct->msg_info.reserve(ContainerSize * 5);
        while (mct->msg_info.size() != ContainerSize * 5) {
          mct->msg_info.push_back(0);
        }
        mcts.push_back(mct);
      }
      cnt++;
    }
  }
  // 将发送任务放到外面处理
  while (!mcts.empty()) {
    size_t rt = sendMsgContainer(*mcts.front());
    mcts.pop_front();
    if (!rt) {
      SYLAR_LOG_WARN(g_logger_sys) << "send loopframe msg is empty!";
    }
  }
}

/// @brief 处理接收到的数据
void CommunicatorBase::processBufferIn() {
  size_t cnt = 0;  // 限定每次只接收 5 帧
  while (cnt < 5 && checkBufferAndPop()) {
    std::stringstream tmp_data;
    MsgType tmp_info;
    {
      std::unique_lock<std::mutex> lk(m_mtx_recv_data_info);
      // 将接收到的写到字节流中
      size_t len = m_buf_recv_data.front().size();
      tmp_data.write(&m_buf_recv_data.front()[0], len);
      m_buf_recv_data.pop_front();
      // SYLAR_LOG_DEBUG(g_logger_sys) << "recv " << len << " bytes loopframe
      // msg.";

      // 获取 msginfo
      tmp_info = m_buf_recv_info.front();
      m_buf_recv_info.pop_front();
    }
    cereal::BinaryInputArchive iarchive(tmp_data);
    MsgLoopframePtr msg(new MsgLoopframe(tmp_info));
    // 判断 SentOnce 是否已经发送过
    if (tmp_info[1] == 0) {
      // 新数据
      if (tmp_info[4] == 0) {
        // loopframe
        std::unique_lock<std::mutex> lk(m_mtx_msg_in);
        iarchive(*msg);
        // 保证 id 顺序增长
        m_buf_msg_in.push_back(msg);
        // SYLAR_LOG_DEBUG(g_logger_sys) << "接收新 message\n"<< msg->dump();
      } else {
        SYLAR_LOG_ERROR(g_logger_sys) << "新数据类型错误 type:" << tmp_info[4];
        SYLAR_ASSERT(false);
      }
    } else if (tmp_info[1] == 1) {
      // 更新数据
      if (tmp_info[4] == 0) {
        // loopframe
        iarchive(*msg);
        SYLAR_LOG_INFO(g_logger_sys) << "接收到更新帧 \n" << msg->dump();
        // TODO 编写更新帧逻辑
      } else {
        SYLAR_LOG_ERROR(g_logger_sys) << "新数据类型错误 type:" << tmp_info[4];
        SYLAR_ASSERT(false);
      }
    } else {
      SYLAR_LOG_ERROR(g_logger_sys) << "SentOnce 类型错误:" << tmp_info[1];
      SYLAR_ASSERT(false);
    }
    cnt++;
  }
}

void CommunicatorBase::showResult() {
  SYLAR_LOG_INFO(g_logger_sys) << "CommunicatorBase::showResult() 未实现.";
}

}  // namespace cmd