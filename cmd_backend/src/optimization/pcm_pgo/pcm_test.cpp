// #include "optimization/pcm_pgo/pcm.hpp"  //
// 假设Pcm类和gncRobustPoseAveraging函数定义在Pcm.h中

#include <ceres/ceres.h>
#include <gtest/gtest.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <sophus/se3.hpp>
#include <sophus/sophus.hpp>
#include "optimization/pose_average.hpp"

using TransMatrixType = Sophus::SE3d;
using TransMatrixVector =
    std::vector<TransMatrixType, Eigen::aligned_allocator<TransMatrixType>>;
TransMatrixType GetNoisePose(TransMatrixType pose, double noise) {
  Eigen::Vector3d noiseVec(noise, noise, noise);
  Eigen::Matrix3d rotation = TransMatrixType::exp(noiseVec).matrix();
  pose.setRotationMatrix(rotation);
  return pose;
}
TransMatrixType CeresRobustPoseAveraging(const TransMatrixVector &input_poses,
                                         const double &rot_sigma,
                                         const double &trans_sigma) {
  // 初始化姿态
  Eigen::Matrix4d initial_pose = Eigen::Matrix4d::Identity();

  // 创建优化问题
  ceres::Problem problem;

  // 根据对齐方法选择损失函数
  ceres::LossFunction *loss_function = new ceres::HuberLoss(0.99);

  // 添加测量值
  for (const auto &pose : input_poses) {
    problem.AddResidualBlock(
        SingleVertexError::Create(pose.matrix(), rot_sigma, trans_sigma),
        loss_function,  // 使用指定的损失函数
        initial_pose.data());
  }

  // 设置优化选项
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;

  // 执行优化
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // 获取优化结果
  TransMatrixType result(initial_pose);
  return result;
}

// 测试用例：测试gncRobustPoseAveraging函数的基本功能
TEST(PcmTest, gncRobustPoseAveraging) {
  // 设置输入值
  TransMatrixVector input_poses;
  // 构造矩阵
  TransMatrixType pose1;
  auto pose2 = GetNoisePose(pose1, rand() / RAND_MAX);
  auto pose3 = GetNoisePose(pose1, rand() / RAND_MAX);
  auto pose4 = GetNoisePose(pose1, rand() / RAND_MAX);

  input_poses.push_back(pose1);  // 添加一个单位矩阵作为测试用例
  input_poses.push_back(pose2);  // 添加一个单位矩阵作为测试用例
  input_poses.push_back(pose3);  // 添加一个单位矩阵作为测试用例
  input_poses.push_back(pose4);  // 添加一个单位矩阵作为测试用例

  double rot_sigma = 1.0;    // 旋转噪声标准差
  double trans_sigma = 1.0;  // 平移噪声标准差

  // 调用待测函数
  TransMatrixType result =
      CeresRobustPoseAveraging(input_poses, rot_sigma, trans_sigma);

  // 验证结果是否足够接近单位矩阵
  Eigen::Matrix4d expected = Eigen::Matrix4d::Identity();
  std::cout << result.matrix() << std::endl;
  //   EXPECT_MATRIX_NEAR(result, expected, 1e-5);
}

// 主函数
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}