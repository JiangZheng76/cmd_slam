#pragma once
#include <fstream>

#include "typedefs_backend.hpp"

namespace cmd {

inline TransMatrixType ToOrthogonalTrans(EigenMatrix matrix) {
  Eigen::Matrix3d R = matrix.block<3, 3>(0, 0);
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      R, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  Eigen::Matrix3d S = Eigen::Matrix3d::Identity();

  if (U.determinant() * V.determinant() < 0) {
    S(2, 2) = -1;
  }
  R = U * S * V.transpose();
  matrix.block<3, 3>(0, 0) = R;
  return TransMatrixType(matrix);
}

inline void saveKeyTimeResult(const std::string& result,
                              const std::string& path) {
  char buff[FILENAME_MAX];
  getcwd(buff, sizeof(buff));
  std::string save_path(buff);
  save_path += "/";
  save_path += path;
  std::ofstream f;
  f.open(save_path);
  f << result << std::endl;
  f.close();
  std::cout << "save key time result to " << save_path << std::endl;
}

}  // namespace cmd