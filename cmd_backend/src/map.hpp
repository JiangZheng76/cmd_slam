#pragma once
#include "cmd_comm.hpp"
#include "optimization/pcm_pgo/pcm_solver.hpp"
#include "typedefs_backend.hpp"
#include "loopframe.hpp"
namespace cmd {
enum OptimizationMode { CERES_SIM3 = 0, PCM_OUTLIER = 1 };
class Framemanager {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int_t m_clientId;
  std::map<int_t, LoopframePtr, std::less<int_t>,
           Eigen::aligned_allocator<std::pair<const int_t, LoopframePtr>>>
      m_lfs;                       // 已经添加到 fmgr 中的帧
  LoopframeList m_optimizing_buf;  // 优化的过程中添加的帧

  bool m_optimizing = false;  // 是否优化中

  LoopframePtr m_first_lf = nullptr;

  int_t m_prev_lfId = 0;  // 上一帧LF id

  double m_lidar_range = 40.0;  // 保留 Lidar scan 范围
  IDPose3Vector m_pts_nearby;  // 上一次处理完的附近点云 【所属LFid，xyz】
                               // 用于生成类 Lidar scan
  IDTransMatrixMap
      m_id_pose_wc;  // 所有帧的李代数位姿【LFid，se(3)】 用于生成类 Lidar scan

 public:
  Framemanager(int client_id);  // 只有Map才可以创建
  ~Framemanager() {}

  // 重置新来帧的 tfm
  // void resetTFMOptimized() { tfm_optimized_cur_ = g2o::SE3Quat(); }

  void addLoopframe(LoopframePtr lf);
  void transfromFrammanager(const TransMatrixType &Ttc);  // cut->target

  LoopframePtr getPrevLoopframe();
  LoopframePtr getLoopframeByKFId(int_t kf_id);

  void updateFramesFromCeres();

  void dump();
  void showResult();

 private:
  void genImiLidarScan(LoopframePtr lf);
};

class Mapmanager {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::set<MapPtr> m_maps;
  MutexType lc_buf_mutex_;  // from -> to
  LoopEdgeList m_lc_buf;    // from -> to
  ThreadPtr main_thread_;
  ThreadPtr optimized_thread_;

  Mapmanager(PangolinViewerPtr viewer = nullptr);
  virtual ~Mapmanager();

  void Run();
  void OptimizeRun();

  bool addLoopframeWithLock(LoopframePtr lf);
  void insertLoopframeToMap(LoopframePtr loopframe);
  bool checkNewFrameBuf(LoopframeList &lfs);
  bool checkLoopclosureBuf();
  void checkOptimizeAndViewUpdate();
  void processLoopClosures();
  void createConstrant(LoopframePtr from, LoopframePtr to, TransMatrixType t_tf,
                       precision_t icp_score, precision_t sc_score);

 private:
  MutexType mutex_;
  MutexType newframe_buf_mutex_;
  std::queue<LoopframePtr> newframe_buf_;
  std::unordered_map<int_t, FramemanagerPtr> fmgrs_;

  bool optimize_running_ = true;
  bool m_runing = true;
  PangolinViewerPtr viewer_;
  std::unique_ptr<PcmSolver> solver_;
  std::unique_ptr<LoopHandler> handler_;
};
}  // namespace cmd
