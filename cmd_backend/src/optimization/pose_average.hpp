#include <ceres/ceres.h>

#include <Eigen/Dense>

using Eigen::Matrix4d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

// 定义代价函数
struct SingleVertexError {
  SingleVertexError(const Matrix4d& pose, const double& rot_sigma,
                    const double& trans_sigma)
      : pose_(pose), rot_sigma_(rot_sigma), trans_sigma_(trans_sigma) {}

  template <typename T>
  bool operator()(const T* const edge_pose, T* residuals) const {
    // 将输入的 pose 转换为 Eigen::Matrix4d
    const Matrix4d pose_matrix(edge_pose);

    // 计算旋转和平移误差
    Quaterniond q1(pose_matrix.block<3, 3>(0, 0));
    Vector3d t1 = pose_matrix.block<3, 1>(0, 3);
    Quaterniond q2(pose_.block<3, 3>(0, 0));
    Vector3d t2 = pose_.block<3, 1>(0, 3);

    Vector3d rot_error = (q1.inverse() * q2).coeffs().head<3>();
    Vector3d trans_error = t1 - t2;

    // 将误差存储在 residuals 中
    residuals[0] = rot_error[0];
    residuals[1] = rot_error[1];
    residuals[2] = rot_error[2];
    residuals[3] = trans_error[0];
    residuals[4] = trans_error[1];
    residuals[5] = trans_error[2];

    return true;
  }

  static ceres::CostFunction* Create(const Matrix4d& pose,
                                     const double& rot_sigma,
                                     const double& trans_sigma) {
    return new ceres::AutoDiffCostFunction<SingleVertexError, 6, 6>(
        new SingleVertexError(pose, rot_sigma, trans_sigma));
  }

  Matrix4d pose_;
  double rot_sigma_;
  double trans_sigma_;
};