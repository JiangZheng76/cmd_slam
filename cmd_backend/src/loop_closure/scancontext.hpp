/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-09-22 21:01:20
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-10-08 11:57:45
 * @FilePath: /codsv/codsv_backend/include/codsv/loop_closure/loop_detection/ScanContext.h
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
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <vector>
#include <memory>
#include <flann/flann.hpp>

typedef std::vector<std::pair<int, double>> SigType;
namespace cmd
{
  class ScanContext
  {
  public:
    typedef std::shared_ptr<ScanContext> Ptr;
    ScanContext();
    ScanContext(int s, int r);

    unsigned int getHeight();
    unsigned int getWidth();

    void generate(const std::vector<Eigen::Vector3d> &pts_spherical,
                  flann::Matrix<float> &ringkey, SigType &signature,
                  double lidar_range, Eigen::Matrix4d &tfm_pca_rig);

  private:
    int num_s_;
    int num_r_;
  };

}
