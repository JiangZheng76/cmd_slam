/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-09-08 10:43:50
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-12-20 13:23:27
 * @FilePath: /covins_ws/src/direct_stereo_slam/src/loop_closure/pose_estimation/icp.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
// Copyright (C) <2020> <Jiawei Mo, Junaed Sattar>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include "typedefs_backend.hpp"
// #define ICP_THRES 1.5

inline pcl::PointCloud<pcl::PointXYZ>
transformPoints(const std::vector<Eigen::Vector3d> &pts_input,
                const Eigen::Matrix4d T)
{
    pcl::PointCloud<pcl::PointXYZ> pc_output;
    pc_output.width = pts_input.size();
    pc_output.height = 1;
    pc_output.is_dense = false;
    pc_output.points.resize(pts_input.size());

    Eigen::Matrix<double, 4, 1> pt_source, pt_transferred;
    for (size_t i = 0; i < pts_input.size(); i++)
    {
        pt_source.head(3) = pts_input[i];
        pt_source(3) = 1.0;
        pt_transferred = T * pt_source;
        pc_output.points[i].x = pt_transferred[0];
        pc_output.points[i].y = pt_transferred[1];
        pc_output.points[i].z = pt_transferred[2];
    }

    return pc_output;
}

inline bool icp(const std::vector<Eigen::Vector3d> &pts_source,
                const std::vector<Eigen::Vector3d> &pts_target,
                Eigen::Matrix4d &tfm_target_source, float &icp_score)
{

    Eigen::Matrix4d I4;
    I4.setIdentity();

    // auto pc_target_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(
    //     transformPoints(pts_target, I4));
    // auto pc_target_source_ptr =
    //     boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(
    //         transformPoints(pts_source, tfm_target_source));
    auto pc_target_ptr =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(
            transformPoints(pts_target, I4)));
    auto pc_target_source_ptr =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(
            transformPoints(pts_source, tfm_target_source)));
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(ICP_ITERA_TIME);
    icp.setTransformationEpsilon(ICP_TRANS_EPSILON); // orginal 0.01
    icp.setEuclideanFitnessEpsilon(ICP_EUC_EPSILON);

    // 设置最大匹配距离阈值
    icp.setMaxCorrespondenceDistance(2);

    icp.setRANSACOutlierRejectionThreshold(0.3);

    icp.setInputSource(pc_target_source_ptr);
    icp.setInputTarget(pc_target_ptr);
    pcl::PointCloud<pcl::PointXYZ> pc_icp;
    icp.align(pc_icp);
    tfm_target_source =
        icp.getFinalTransformation().cast<double>() * tfm_target_source;

    icp_score = icp.getFitnessScore();
    printf("icp: %5.2f ", icp_score);
    return icp_score < ICP_THRES;
}