#pragma once

// C++
#include <Eigen/Core>

#include "typedefs_backend.hpp"
namespace cmd {
// 辅助 map 更改优化状态
class MapOptimizationWrap {
 public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MapOptimizationWrap(MapPtr map);
  ~MapOptimizationWrap();

  MapPtr m_map;
};

class Optimization {
 public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Optimization() = delete;
  static void Sim3PoseGraphOptimization(MapPtr map);

  static void PCMPoseGraphOptimization(MapPtr map, LoopEdgeVector les);
};

}  // namespace cmd
