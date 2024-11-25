#include "scancontext.hpp"

#include <cmath>

#include "typedefs_backend.hpp"
namespace cmd {
/// @brief pca对齐，因为没有imu的重力方向
/// @param pts_clr_in
/// @param pts_clr_out
/// @param tfm_pca_rig
inline void align_points_PCA(const Point3Vector &pts_clr_in,
                             Point3Vector &pts_clr_out,
                             Eigen::Matrix4d &tfm_pca_rig) {
  double mx(0), my(0), mz(0);
  for (auto &pc : pts_clr_in) {
    mx += pc(0);
    my += pc(1);
    mz += pc(2);
  }
  mx /= pts_clr_in.size();
  my /= pts_clr_in.size();
  mz /= pts_clr_in.size();

  // normalize pts
  Eigen::MatrixXd pts_mat(pts_clr_in.size(), 3);
  for (size_t i = 0; i < pts_clr_in.size(); i++) {
    pts_mat(i, 0) = pts_clr_in[i](0) - mx;
    pts_mat(i, 1) = pts_clr_in[i](1) - my;
    pts_mat(i, 2) = pts_clr_in[i](2) - mz;
  }

  // PCA
  auto cov = pts_mat.transpose() * pts_mat;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(cov);
  auto v0 = es.eigenvectors().col(0);
  auto v1 = es.eigenvectors().col(1);
  auto v2 = es.eigenvectors().col(2);

  // rotate pts
  auto nx = (pts_mat * v0).eval();
  auto ny = (pts_mat * v1).eval();
  auto nz = (pts_mat * v2).eval();

  pts_clr_out.clear();
  for (size_t i = 0; i < pts_clr_in.size(); i++) {
    pts_clr_out.emplace_back(Eigen::Vector3d(nx(i), ny(i), nz(i)));
  }

  // Transformation
  tfm_pca_rig.setIdentity();
  tfm_pca_rig.block<1, 3>(0, 0) = v0.transpose();
  tfm_pca_rig.block<1, 3>(1, 0) = v1.transpose();
  tfm_pca_rig.block<1, 3>(2, 0) = v2.transpose();
  Eigen::Vector3d t;
  t << mx, my, mz;
  t = -tfm_pca_rig.topLeftCorner<3, 3>() * t;
  tfm_pca_rig.block<3, 1>(0, 3) = t;
}
/// @brief 设置ring和scale的格子数
ScanContext::ScanContext() {
  num_s_ = 60;
  num_r_ = 20;
}

ScanContext::ScanContext(int s, int r) : num_s_(s), num_r_(r) {}

unsigned int ScanContext::getHeight() { return num_r_; }
unsigned int ScanContext::getWidth() { return num_s_; }
/// @brief
/// 根据传进来某一帧的点云pts_spherical，创建ringkey和signature用于后面的回环检测
/// @param pts_spherical
/// @param ringkey {out}
/// @param signature {out}
/// @param lidar_range
/// @param tfm_pca_rig
void ScanContext::generate(const Point3Vector &pts_spherical,
                           flann::Matrix<float> &ringkey, SigType &signature,
                           double lidar_range, Eigen::Matrix4d &tfm_pca_rig) {
  // align spherical points by PCA
  Point3Vector pts_aligned;
  align_points_PCA(pts_spherical, pts_aligned, tfm_pca_rig);

  // ringkey 保存ring中的点数
  ringkey = flann::Matrix<float>(new float[num_r_], 1, num_r_);
  for (int i = 0; i < num_r_; i++) {
    ringkey[0][i] = 0.0;
  }

  // signature matrix A
  Eigen::VectorXd max_height =
      (-lidar_range - 1.0) * Eigen::VectorXd::Ones(num_s_ * num_r_);

  // 计算ring 和 scale 每个格子中的最高点
  for (size_t i = 0; i < pts_aligned.size(); i++)  // loop on pts
  {
    // projection to polar coordinate
    // After PCA, x: up; y: left; z:back
    double yp = pts_aligned[i](1);
    double zp = pts_aligned[i](2);

    double rho = std::sqrt(yp * yp + zp * zp);
    double theta = std::atan2(zp, yp);
    while (theta < 0) theta += 2.0 * M_PI;
    while (theta >= 2.0 * M_PI) theta -= 2.0 * M_PI;

    // get projection bin w.r.t. theta and
    // rho,计算pca对齐之后的某一个点属于哪一个格子
    int si = theta / (2.0 * M_PI) * num_s_;
    assert(si < num_s_);
    int ri = rho / lidar_range * num_r_;
    if (ri >= num_r_)  // happens because PCA translated the points
      continue;

    max_height(si * num_r_ + ri) =
        std::max(max_height(si * num_r_ + ri), pts_aligned[i](0));
  }

  // calculate ringkey and signature
  Eigen::VectorXd sig_norm_si = Eigen::VectorXd::Zero(num_s_);
  for (int i = 0; i < num_s_ * num_r_; i++) {
    if (max_height(i) >= (-lidar_range)) {
      // ringkey记录每个ring格子里面的点总数
      ringkey[0][i % num_r_]++;
      // 将最高点的下标和高度都放入到signature当中去
      signature.push_back({i, max_height(i)});
      sig_norm_si(i / num_r_) +=
          max_height(i) * max_height(i);  // 记录每一个sig的最大高度平方和
    }
  }

  // normalize ringkey
  // 进行归一化处理，防止某个数据过大，对ringkey进行归一化处理
  for (int i = 0; i < num_r_; i++) {
    ringkey[0][i] /= num_s_;
  }

  // normalize signature 对签名也进行归一化处理
  sig_norm_si = sig_norm_si.cwiseSqrt();
  for (size_t i = 0; i < signature.size(); i++) {
    assert(sig_norm_si(signature[i].first / num_r_) > 0);
    // 每一个最大高度也会除以 sig方向上的最大高度平方和
    signature[i].second /= sig_norm_si(signature[i].first / num_r_);
  }
}
}  // namespace cmd
