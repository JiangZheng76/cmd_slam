#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include "typedefs_backend.hpp"
// #define ICP_THRES 1.5
using namespace cmd;
inline pcl::PointCloud<pcl::PointXYZ> transformPoints(
    const Point3Vector &pts_input, const Eigen::Matrix4d T) {
  pcl::PointCloud<pcl::PointXYZ> pc_output;
  pc_output.width = pts_input.size();
  pc_output.height = 1;
  pc_output.is_dense = false;
  pc_output.points.resize(pts_input.size());

  Eigen::Matrix<double, 4, 1> pt_source, pt_transferred;
  for (size_t i = 0; i < pts_input.size(); i++) {
    pt_source.head(3) = pts_input[i];
    pt_source(3) = 1.0;
    pt_transferred = T * pt_source;
    pc_output.points[i].x = pt_transferred[0];
    pc_output.points[i].y = pt_transferred[1];
    pc_output.points[i].z = pt_transferred[2];
  }

  return pc_output;
}

inline bool icp(const Point3Vector &pts_source, const Point3Vector &pts_target,
                Eigen::Matrix4d &tfm_target_source, float &icp_score) {
  Eigen::Matrix4d I4;
  I4.setIdentity();

  auto pc_target_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>(transformPoints(pts_target, I4)));
  auto pc_target_source_ptr =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(
          transformPoints(pts_source, tfm_target_source)));
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations(5);
  icp.setTransformationEpsilon(0.01);
  icp.setMaxCorrespondenceDistance(2);
  icp.setRANSACOutlierRejectionThreshold(0.5);

  // icp.setMaximumIterations(ICP_ITERA_TIME);
  // icp.setTransformationEpsilon(ICP_TRANS_EPSILON); // orginal 0.01
  // icp.setEuclideanFitnessEpsilon(ICP_EUC_EPSILON);
  // // 设置最大匹配距离阈值
  // icp.setMaxCorrespondenceDistance(2);

  // icp.setRANSACOutlierRejectionThreshold(0.3);

  icp.setInputSource(pc_target_source_ptr);
  icp.setInputTarget(pc_target_ptr);
  pcl::PointCloud<pcl::PointXYZ> pc_icp;
  icp.align(pc_icp);
  tfm_target_source =
      icp.getFinalTransformation().cast<double>() * tfm_target_source;

  icp_score = icp.getFitnessScore();
  // printf("icp: %5.2f ", icp_score);
  return icp_score < ICP_THRES;
}