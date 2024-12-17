#pragma once
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <cereal/cereal.hpp>
#include <flann/flann.hpp>
#include <iostream>
#include <memory>
#include <vector>

#include "typedefs_backend.hpp"

typedef std::vector<std::pair<int, double>> SigType;
namespace cmd {
class ScanContext {
 public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<ScanContext> Ptr;
  ScanContext();
  ScanContext(int s, int r);

  unsigned int getHeight();
  unsigned int getWidth();

  void generate(const Point3Vector &pts_spherical,
                flann::Matrix<float> &ringkey, SigType &signature,
                double lidar_range, Eigen::Matrix4d &tfm_pca_rig);

 private:
  int num_s_;
  int num_r_;
};

}  // namespace cmd
