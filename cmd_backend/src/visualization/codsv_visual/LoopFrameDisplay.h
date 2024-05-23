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

// This file is modified from <https://github.com/JakobEngel/dso>

#pragma once

// #undef Success
// #include <util/NumType.h>
#include <Eigen/Core>
#include <pangolin/pangolin.h>

#include <fstream>
#include <sstream>

namespace cmd
{
  template <int ppp>
  struct InputPointSparse
  {
    typedef std::shared_ptr<InputPointSparse> Ptr;
    float u;
    float v;
    float idpeth;
    float idepth_hessian;
    float relObsBaseline;
    int numGoodRes;
    unsigned char color[ppp];
    unsigned char status;
  };

  struct MyVertex
  {
    float point[3];
    unsigned char color[4];
  };

  // stores a pointcloud associated to a Keyframe.
  class LoopFrameDisplay
  {

  public:
    typedef std::shared_ptr<LoopFrameDisplay> Ptr;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LoopFrameDisplay(LoopframePtr lf);
    ~LoopFrameDisplay();

    // copies points from KF over to internal buffer,
    // keeping some additional information so we can render it differently.
    // void setFromKF(FrameHessian *fh, CalibHessian *HCalib);
    void setFromKF(LoopframePtr lf);

    // copies points from KF over to internal buffer,
    // keeping some additional information so we can render it differently.
    // void setFromF(FrameShell *fs, CalibHessian *HCalib);
    void setCalib(LoopframePtr lf, const Calibration &cabli);
    // copies & filters internal data to GL buffer for rendering. if nothing to
    // do: does nothing.
    void refreshPC();

    // renders cam & pointcloud.
    void drawPC(float pointSize);

    void drawLidar();

    void setColor(std::vector<float> color);

    // id 设置为uint64 位
    uint64_t client_kf_id_;
    bool active_;
    TransMatrixType tfm_w_c_;
    bool need_refresh_;
    std::mutex update_mtx_;
    inline bool operator<(const LoopFrameDisplay &other) const
    {
      return (client_kf_id_ < other.client_kf_id_);
    }

    LoopframePtr lf_;

  private:
    float fx_, fy_, cx_, cy_;
    float fxi_, fyi_, cxi_, cyi_;
    int width_, height_;

    float my_scaled_th_, my_abs_th_, my_scale_;
    int my_sparsify_factor_;
    int my_displayMode_;
    float my_min_rel_bs_;

    int num_sparse_points_;
    int num_sparse_buffer_size_;
    std::vector<InputPointSparse<MAX_RES_PER_POINT>> original_input_sparse_;

    bool buffer_valid_;
    int num_gl_buffer_points_;
    int num_gl_buffer_good_points_;
    std::vector<float> color_;
    pangolin::GlBuffer vertex_buffer_;
    pangolin::GlBuffer color_buffer_;
  };

} // namespace dso
