#pragma once

#undef Success
#include <pangolin/pangolin.h>

#include <Eigen/Core>
#include <fstream>
#include <sstream>

#include "typedefs_backend.hpp"

namespace cmd {
struct MyVertex {
  float point[3];
  unsigned char color[4];
};

// stores a pointcloud associated to a Keyframe.
class LoopframeDisplay {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LoopframeDisplay(LoopframePtr lf);
  ~LoopframeDisplay();

  void refreshPC();
  void drawPC(float pointSize);
  void drawPose(float poseSize);

  void setFromLF(LoopframePtr lf);
  void initColor(std::vector<float> color);

 public:
  // id 设置为uint64 位
  bool m_active;
  bool m_need_refresh;
  uint64_t m_lf_id;
  TransMatrixType m_Twc;

  MutexType m_muetx;

  LoopframePtr m_lf;

 private:
  int m_num_sparse_points;       // point 个数
  int m_num_sparse_buffer_size;  // 符合条件的可以输出点个数

  bool m_buffer_valid;              // gl buf 是否合理的标识
  int m_num_gl_buffer_points;       // gl buf 中点的数目
  int m_num_gl_buffer_good_points;  // 最新的 gl 可以用到好点

  std::vector<Point2> m_original_point2s;  // 当前帧的点
  std::vector<float> m_color;              // 存储的颜色
  pangolin::GlBuffer m_vertex_buffer;      // 最终显示的点
  pangolin::GlBuffer m_color_buffer;  // 最终显示的颜色，下标和点对应
};

}  // namespace cmd
