#pragma once
#include "map.hpp"
#include "typedefs_backend.hpp"

namespace cmd {

class LoopHandler {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LoopHandler(float lidar_range, float scan_context_thres,
              Mapmanager* mapMgr);
  ~LoopHandler();

  void Run();
  void join();
  void close();

  void pushLoopframe2Buf(LoopframePtr lf);

 private:
  bool m_running = true;
  ThreadPtr m_main_thread;

  // buff
  MutexType m_mtx_buf_lf;
  bool buf_or_requery_ = true; // 交替选择
  LoopframeList m_buf_lfs;  // LoopHandler 中新加入待处理的 LF
  LoopframeList m_requery_lfs_;  // LoopHandler 中新加入重检测待处理的 LF

  LoopframeVector
      m_preocessed_lf;  // 所有回环处理过的帧，所有帧，包括不是顶点的帧
                        // 下标和 scancontext 相同

  // loop detection by ScanContext
  float m_lidar_range;
  float m_scan_context_thres;
  flann::Index<flann::L2<float>> *m_ringkeys;
  ScanContextPtr m_sc;  // scanContext的处理器

  Mapmanager*
      m_mapMgr;  // 用来存储所有帧，并处理帧合并的问题，知道那几个帧是在同一个地图上的
};

}  // namespace cmd
