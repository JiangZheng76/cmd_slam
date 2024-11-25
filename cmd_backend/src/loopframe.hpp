#pragma once

// C++
#include <Eigen/Core>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <set>
#include <vector>

#include "cmd_comm.hpp"

// loop closure
#include "loop_closure/scancontext.hpp"
#include "typedefs_backend.hpp"

// #define FMT_HEADER_ONLY
// #include "fmt/format.h"

namespace cmd {

class Point2 {
 public:
  Point2() {}
  Point2(const MsgPoint &msg);

  precision_t m_u;
  precision_t m_v;
  precision_t m_idepth_scaled;
  precision_t m_maxRelBaseline;
  precision_t m_idepth_hessian;
};

enum EdgeType {
  ODOMETRY,
  LOOPCLOSURE,
  UNCLASSIFIED,
};
struct LoopEdge {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // 持有该 edge 的 lf 为入度端
  LoopframePtr m_from_lf;  // ref ｜ query
  LoopframePtr m_to_lf;    // cur ｜ match

  InformationMat m_info;   // 信息矩阵
  TransMatrixType m_t_tf;  // from -> to ref -> to query->match

  precision_t m_icp_score;  // icp 匹配得分
  precision_t m_sc_score;   // sc 匹配得分

  EdgeType m_type;

  LoopEdge(LoopframePtr from, LoopframePtr to, const TransMatrixType &t_tf,
           precision_t icp_error, precision_t sc_error, EdgeType type);
};

class Loopframe : public std::enable_shared_from_this<Loopframe> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint32_t m_lf_id;        // kF id, for pose graph and visualization
  uint32_t m_incoming_id;  // increasing id, for ground truth
  uint32_t m_client_id;
  uint64_t m_timestamp;

  // loop detection
  SigType m_signature;             // place signature
  flann::Matrix<float> m_ringkey;  // fast detect loop
  TransMatrixType m_t_pca_rig;     // transformation from rig to pca frame

  Calibration m_calib;    // calib for slam
  TransMatrixType m_twc;  // coordinate in pose graph

  // ceres optimization
  VecSim3 m_ceres_pose;  // tcw

  // heuristics for setting edge information
  precision_t m_dso_error;  // 前后帧之间的残差函数的结果值 乘上了一个系数
  precision_t m_scale_error;  // 优化尺度的擦差函数的结果值乘上了一个系数
  precision_t m_ab_exposure;

  // loop edge
  Point2Vector m_points;
  Point3Vector m_pts_spherical;  // loop correction by icp

  std::vector<int_t> m_ref_id;  // 从 msg 来的reference 帧 id
  TransMatrixVector m_ref_cf;   // 从 msg 来的reference 帧 相对位姿
  LoopEdgeVector m_edges;

  // control default is false
  bool m_is_first;
  bool m_graph_added;
  bool m_is_display;
  bool m_pose_optimized;
  bool m_is_need_to_update_viewer;

  bool m_optimizing;  // 优化状态符
  RWMutexType m_mtx;  // 访问的读写互斥锁

 public:
  Loopframe(MsgLoopframePtr msg, bool first_loopframe = false);
  virtual ~Loopframe() {}

  bool isFirstFrame();

  LoopframePtr getReference();
  void updateFromMsg(MsgLoopframePtr msg);
  bool getRef2Cur(TransMatrixType &tcr);
  bool getCur2Ref(TransMatrixType &trc);

  bool addConstrant(LoopEdgePtr le);
  bool addReference(LoopframePtr lf, const TransMatrixType &t_tf,
                    precision_t icp_score, precision_t sc_score);

  uint64_t getClientMerageKfId();

  void updateFromCeres();
  std::string dump();
};
inline std::ostream &operator<<(std::ostream &out, LoopframePtr lf) {
  return out << lf->dump();
};

}  // namespace cmd
