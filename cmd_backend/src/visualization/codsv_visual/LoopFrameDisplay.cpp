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

#include <stdio.h>

// #include <GL/glx.h>
// #include <GL/gl.h>
// #include <GL/glu.h>
#include <pangolin/pangolin.h>

#include "loopframe.hpp"
#include "LoopFrameDisplay.h"
namespace cmd
{
  static LoggerPtr g_logger_sys = SYLAR_LOG_NAME("system");

  LoopFrameDisplay::LoopFrameDisplay(LoopframePtr lf)
  {
    // assert(!lf->is_display_);
    original_input_sparse_.clear();
    num_sparse_buffer_size_ = 0;
    num_sparse_points_ = 0;

    client_kf_id_ = 0;
    active_ = true;
    tfm_w_c_ = TransMatrixType();

    // need_refresh_ = true;

    my_scaled_th_ = 0.001;
    my_abs_th_ = 0.001;
    my_displayMode_ = 1;
    my_min_rel_bs_ = 0.1;
    my_sparsify_factor_ = 1;

    num_gl_buffer_points_ = 0;
    buffer_valid_ = false;

    setFromKF(lf);
    lf_ = lf;

    // lf->is_display_ = true;
  }
  void LoopFrameDisplay::setCalib(LoopframePtr lf, const Calibration &cabli)
  {
    client_kf_id_ = lf->getClientMerageKfId();
    fx_ = cabli.m_intrinsics[0];
    fy_ = cabli.m_intrinsics[1];
    cx_ = cabli.m_intrinsics[2];
    cy_ = cabli.m_intrinsics[3];
    width_ = cabli.m_img_dims[0];
    height_ = cabli.m_img_dims[1];
    fxi_ = 1 / fx_;
    fyi_ = 1 / fy_;
    cxi_ = -cx_ / fx_;
    cyi_ = -cy_ / fy_;
    tfm_w_c_ = lf->m_twc;
    need_refresh_ = true;
  }
  /// @brief 设置lf的边缘化的点，但是还是有不懂的？？？
  /// @param lf
  void LoopFrameDisplay::setFromKF(LoopframePtr lf)
  {
    std::unique_lock<std::mutex> lk(update_mtx_);
    setCalib(lf, lf->m_calib);

    int npoints = lf->m_points.size();

    if (original_input_sparse_.size() < npoints)
    {
      original_input_sparse_.resize(npoints + 100);
    }
    std::vector<InputPointSparse<MAX_RES_PER_POINT>> &pc = original_input_sparse_;
    num_sparse_points_ = 0;
    for (auto p : lf->m_points)
    {
      if (p.m_idepth_scaled <= 0)
        continue;
      for (int i = 0; i < patternNum; i++)
      {
        // 暂时不知道这个color有什么作用？？？
        pc[num_sparse_points_].color[i] = i;
      }
      pc[num_sparse_points_].u = p.m_u;
      pc[num_sparse_points_].v = p.m_v;
      pc[num_sparse_points_].idpeth = p.m_idepth_scaled;
      // 表示已经被边缘化的点
      pc[num_sparse_points_].status = 1;
      // 下面的作用未知？？？
      pc[num_sparse_points_].idepth_hessian = p.m_idepth_hessian;
      pc[num_sparse_points_].relObsBaseline = p.m_maxRelBaseline;
      pc[num_sparse_points_].numGoodRes = 0;
      num_sparse_points_++;
    }
    tfm_w_c_ = lf->m_twc;
    SYLAR_ASSERT(num_sparse_points_ <= npoints);

    need_refresh_ = true;
  }

  LoopFrameDisplay::~LoopFrameDisplay()
  {
    // if (original_input_sparse_ != 0)
      // delete[] original_input_sparse_;
  }
  void LoopFrameDisplay::setColor(std::vector<float> color)
  {
    assert(color.size() == 3);
    color_ = color;
  }
  /// @brief 只有在创建或者是更新之后会刷新点云信息
  void LoopFrameDisplay::refreshPC()
  {
    if (!need_refresh_)
        {
            return;
        }
        std::unique_lock<std::mutex> lk(update_mtx_);
        need_refresh_ = false;
        // if there are no vertices, done!
        if (num_sparse_points_ == 0)
        {
            return;
        }

        // make data
        std::vector<Vec3Type> tempVertexBuf(num_sparse_points_ * patternNum);
        std::vector<Vec3bType> tempColorBuf(num_sparse_points_ * patternNum);
        int vertexBufferNumPoints = 0;

        for (int i = 0; i < num_sparse_points_; i++)
        {
            /* display modes:
             * my_displayMode_==0 - all pts, color-coded
             * my_displayMode_==1 - normal points
             * my_displayMode_==2 - active only
             * my_displayMode_==3 - nothing
             */
            // 显示状态为1 同时点状态为 1和 2 的点可以显示出来
            if (my_displayMode_ == 1 && original_input_sparse_[i].status != 1 &&
                original_input_sparse_[i].status != 2)
                continue;
            if (my_displayMode_ == 2 && original_input_sparse_[i].status != 1)
                continue;
            if (my_displayMode_ > 2)
                continue;

            if (original_input_sparse_[i].idpeth < 0)
                continue;

            float depth = 1.0f / original_input_sparse_[i].idpeth;
            float depth4 = depth * depth;
            depth4 *= depth4;
            float var = (1.0f / (original_input_sparse_[i].idepth_hessian + 0.01));

            if (var * depth4 > my_scaled_th_)
                continue;

            if (var > my_abs_th_)
                continue;

            if (original_input_sparse_[i].relObsBaseline < my_min_rel_bs_)
                continue;

            for (int pnt = 0; pnt < patternNum; pnt++)
            {

                if (my_sparsify_factor_ > 1 && rand() % my_sparsify_factor_ != 0)
                    continue;
                int dx = patternP[pnt][0];
                int dy = patternP[pnt][1];

                tempVertexBuf[vertexBufferNumPoints][0] =
                    ((original_input_sparse_[i].u + dx) * fxi_ + cxi_) * depth;
                tempVertexBuf[vertexBufferNumPoints][1] =
                    ((original_input_sparse_[i].v + dy) * fxi_ + cxi_) * depth;
                tempVertexBuf[vertexBufferNumPoints][2] =
                    depth * (1 + 2 * fxi_ * (rand() / (float)RAND_MAX - 0.5f));

                tempColorBuf[vertexBufferNumPoints][0] = color_[0];
                tempColorBuf[vertexBufferNumPoints][1] = color_[1];
                tempColorBuf[vertexBufferNumPoints][2] = color_[2];

                vertexBufferNumPoints++;

                SYLAR_ASSERT(vertexBufferNumPoints <= num_sparse_points_ * patternNum);
            }
        }

        if (vertexBufferNumPoints == 0)
        {
            // std::cout << "refresh loopframe pointNum " << vertexBufferNumPoints << std::endl;
            SYLAR_LOG_INFO(g_logger_sys) << client_kf_id_ << "没有刷新点.";
            return;
        }

        num_gl_buffer_good_points_ = vertexBufferNumPoints;
        if (num_gl_buffer_good_points_ > num_gl_buffer_points_)
        {
            // 扩充 pangolin buf 的空间
            num_gl_buffer_points_ = vertexBufferNumPoints * 1.3;
            vertex_buffer_.Reinitialise(pangolin::GlArrayBuffer, num_gl_buffer_points_,
                                         GL_FLOAT, 3, GL_DYNAMIC_DRAW);
            color_buffer_.Reinitialise(pangolin::GlArrayBuffer, num_gl_buffer_points_,
                                        GL_UNSIGNED_BYTE, 3, GL_DYNAMIC_DRAW);
        }
        // 点和 color 内容写到 glbuf 中
        vertex_buffer_.Upload(&tempVertexBuf[0],
                               sizeof(float) * 3 * num_gl_buffer_good_points_, 0);
        color_buffer_.Upload(&tempColorBuf[0],
                              sizeof(unsigned char) * 3 * num_gl_buffer_good_points_,
                              0);

        buffer_valid_ = true;
  }

  void LoopFrameDisplay::drawPC(float pointSize)
  {

    if (!buffer_valid_ || num_gl_buffer_good_points_ == 0)
      return;

    glDisable(GL_LIGHTING);
    glColor3f(color_[0], color_[1], color_[2]);
    glPushMatrix();

    // Sophus::Matrix4f m = tfm_w_c_.to_homogeneous_matrix().cast<float>();
    glMultMatrixd((GLdouble *)tfm_w_c_.matrix().data());
    // 画出来点的大小
    glPointSize(pointSize);
    // 这个绑定颜色不行，使用glColor来对点进行染色
    //   color_buffer_.Bind();
    // // glColorPointer()函数
    // // size：指定每个颜色的分量数量。通常为 3（表示 RGB 颜色）或 4（表示 RGBA 颜色）。
    // // type：指定颜色数据的类型。通常使用 GL_UNSIGNED_BYTE（每个分量为无符号字节），GL_FLOAT（每个分量为浮点数）等。
    // // stride：指定相邻颜色之间的字节偏移量。通常可以设置为 0，表示颜色数据是紧密排列的。
    // // pointer：指向颜色数据的指针。这可以是一个数组、缓冲区对象或其他内存位置。
    //   glColorPointer(color_buffer_.count_per_element, color_buffer_.datatype, 0, 0);
    //   glEnableClientState(GL_COLOR_ARRAY);

    vertex_buffer_.Bind();
    glVertexPointer(vertex_buffer_.count_per_element, vertex_buffer_.datatype, 0,
                    0);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_POINTS, 0, num_gl_buffer_good_points_);
    glDisableClientState(GL_VERTEX_ARRAY);
    vertex_buffer_.Unbind();

    // glDisableClientState(GL_COLOR_ARRAY);
    // color_buffer_.Unbind();

    glPopMatrix();
  }
}; // namespace dso
