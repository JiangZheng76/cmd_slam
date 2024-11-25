#pragma once

// C++
#include <netinet/in.h>
#include <unistd.h>

#include <Eigen/Core>
#include <list>
#include <mutex>
#include <thread>

// cmd
#include "msgs/msg_loopframe.hpp"
#include "typedefs.hpp"

#define ContainerSize 10

namespace cmd {

// 待发送lf集
class DataBundle {
 public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // ！！！ EIGEN_MAKE_ALIGNED_OPERATOR_NEW也会导致释放智能指针时候的double free
  // 的问题
  MsgLoopframeList m_lfs;
};

/**
 * @brief 对象字节流 与 消息类型
 * @description:
 * @return {*}
 */
struct MessageContainer {
 public:
  std::stringstream msg_data;   // 序列化msg
  std::vector<int_t> msg_info;  // 消息类型 unit32
};

class CommunicatorBase : public SocketStream {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CommunicatorBase();
  CommunicatorBase(int client_id, SocketPtr sock);

  // Main
  virtual void action();
  virtual auto Run() -> void = 0;

  // Interfaces
  virtual void passDataBundle(DataBundlePtr data);
  virtual void lock();
  virtual void unLock();
  virtual bool tryLock();
  virtual int getClientId();

  // Message handling
  virtual void serialize(MsgLoopframePtr lf);

  // Message passing
  static auto GetInAddr(struct sockaddr *sa)
      -> void *;  // get sockaddr, IPv4 or IPv6:
  virtual SocketPtr connectToServer(const char *node, std::string port);

  // Synchronization
  virtual void setFinish() {
    std::unique_lock<std::mutex> lock(m_mtx_finish);
    m_finish = true;
  }
  virtual bool shallFinish() {
    std::unique_lock<std::mutex> lock(m_mtx_finish);
    return m_finish;
  }
  virtual bool isFinished() {
    std::unique_lock<std::mutex> lock(m_mtx_finish);
    return m_is_finished;
  }

 protected:
  // Message passing
  virtual int sendAll(MsgType &msg_send);
  virtual int sendAll(std::stringstream &msg);
  virtual int recvAll(unsigned int sz, std::vector<char> &buffer);
  virtual int recvAll(unsigned int sz, MsgType &buffer);
  virtual void recvMsg();
  virtual void writeToBuffer();
  virtual bool checkBufferAndPop();  // Attention: Removes data from buffer, do
                                     // not use if for mere empty/non-ems
  virtual size_t sendMsgContainer(MessageContainer &msg);

  // msg-(processSendMsgLoopframe)->
  // db -(processBufferOut)->
  // binary-(processBufferIn)->
  // msg -(processRecvMsgLoopframe)-> lf

  // Data handling
  virtual void processBufferOut();
  virtual void processBufferIn();

  /// @brief 接受端的 msg 处理
  virtual void processRecvMsgLoopframe() = 0;
  /// @brief 发送端的 msg 处理
  virtual void processSendMsgLoopframe() = 0;

 public:
  // recored bandwidth
  double bytes_sent = 0;
  double bytes_recv = 0;
  std::chrono::_V2::steady_clock::time_point start_time;
  std::chrono::_V2::steady_clock::time_point end_time;

  void showResult();

 protected:
  // Infrastructure
  int m_client_id = -1;
  // Data
  std::mutex m_mtx_recv_data_info;
  std::list<std::vector<char>> m_buf_recv_data;  // 保存接受的对象字节流信息
  std::list<std::vector<uint32_t>>
      m_buf_recv_info;  // 保存接收 msg 对象的信息，和 buffer_recv_data_
                        // 一一对应

  std::mutex m_mtx_datas_out;
  DataBundleList m_buf_datas_out;  // 点云 和 loopframe打包好的 待发送 buffer

  std::mutex m_mtx_msg_out;        // 准备发送 msg buf 锁
  MsgLoopframeList m_buf_msg_out;  // 准备发送的 lf msg

  std::mutex m_mtx_msg_in;        // 准备接收 msg buf 锁
  MsgLoopframeList m_buf_msg_in;  // 接收到的 lf msg

  // Sync
  std::mutex m_mtx_comm;  // comm 执行线程锁
  std::mutex m_mtx_finish;

  bool m_finish = false;
  bool m_is_finished = false;

  // message passing
  std::mutex m_mtx_recv_buf;
  std::vector<char> m_recv_buf;  // 主线程中接收的二进制 msgloopframe 数据
  int m_package_size_send;
  int m_newfd;
  SocketPtr m_sock;
  ThreadPtr m_main_thread;

  std::stringstream m_send_ser;  // 序列化结果
  MsgType m_msgtype_buf =
      MsgType(5);               // msg size, SentOnce, id.first, id.second
  MsgType m_msgtype_container;  // 主线程中接收的二进制 msgtype 数据 msg size,
                                // is_update_msg, id.first, id.second,
                                // 会进行扩容接收多个消息
  MsgType m_msgtype_deserialize =
      MsgType(5);  // 接收到的一个 MsgType【msg size, SentOnce, id.first,
                   // id.second，type】
};

}  // namespace cmd
