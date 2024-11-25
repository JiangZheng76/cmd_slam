// #pragma once
#ifndef __GENERATE_SPHERICAL_POINTS_H__
#define __GENERATE_SPHERICAL_POINTS_H__
#include "typedefs_backend.hpp"
// #include <typedefs>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <unordered_map>

#define RES_X 1.0
#define RES_Y 0.5
#define RES_Z 1.0

// #define RES_X 2
// #define RES_Y 0.75
// #define RES_Z 2
namespace cmd {

void generate_spherical_points(IDPose3Vector &pts_nearby,
                               IDTransMatrixMap &id_pose_wc,
                               const TransMatrixType &cur_cw,
                               double lidar_range,
                               Point3Vector &pts_spherical) {
  // if the oriention difference between a historical keyframe and current
  // keyframe is large, trim all associated points
  for (auto &ip : id_pose_wc) {
    auto pose_diff_se3 = (cur_cw * TransMatrixType::exp(ip.second)).log();
    auto rotation_norm = pose_diff_se3.tail(3).norm();
    if (rotation_norm > 0.5) {
      int id = ip.first;
      id_pose_wc.erase(id);
    }
  }

  // get/filter spherical points
  std::vector<double> steps{1.0 / RES_X, 1.0 / RES_Y, 1.0 / RES_Z};
  std::vector<int> voxel_size{
      static_cast<int>(floor(2 * lidar_range * steps[0]) + 1),
      static_cast<int>(floor(2 * lidar_range * steps[1]) + 1),
      static_cast<int>(floor(2 * lidar_range * steps[2]) + 1)};
  std::vector<int> loc_step{1, voxel_size[0], voxel_size[0] * voxel_size[1]};
  std::unordered_map<int, std::pair<int, Eigen::Vector3d>> loc2idx_pt;
  // output useful points
  IDPose3Vector new_pts_nearby;
  for (size_t i = 0; i < pts_nearby.size(); i++) {
    int id = pts_nearby[i].first;
    if (id_pose_wc.find(id) == id_pose_wc.end()) {
      continue;  // orientation changed too much
    }

    Eigen::Vector4d p_g(pts_nearby[i].second(0), pts_nearby[i].second(1),
                        pts_nearby[i].second(2), 1.0);
    Eigen::Vector3d p_l = cur_cw.matrix3x4() * p_g;  // 相机坐标下的点

    if (p_l.norm() >= lidar_range) {
      continue;  // out of range
    }

    // voxel indices
    int xi = static_cast<int>(floor((p_l(0) + lidar_range) * steps[0]));
    int yi = static_cast<int>(floor((p_l(1) + lidar_range) * steps[1]));
    int zi = static_cast<int>(floor((p_l(2) + lidar_range) * steps[2]));
    int loc = xi * loc_step[0] + yi * loc_step[1] + zi * loc_step[2];

    // store the highest points
    if (loc2idx_pt.find(loc) == loc2idx_pt.end() ||
        -loc2idx_pt[loc].second(1) < -p_l(1)) {
      loc2idx_pt[loc] = {i, p_l};
    }
  }

  pts_spherical.reserve(5000);
  new_pts_nearby.reserve(10000);
  for (auto &l_ip : loc2idx_pt) {
    // pts_spherical 放入经过雷达网格过滤之后的点
    pts_spherical.push_back(l_ip.second.second);
    // pts_nearby 将雷达网格中的点放入到最新的临近点数组中，等待下一次使用
    new_pts_nearby.push_back(pts_nearby[l_ip.second.first]);
  }

  // update nearby pts
  pts_nearby = new_pts_nearby;
}

}  // namespace cmd
#endif