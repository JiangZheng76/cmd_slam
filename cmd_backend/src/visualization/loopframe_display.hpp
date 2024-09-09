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

#undef Success
#include "typedefs_backend.hpp"

#include <Eigen/Core>
#include <pangolin/pangolin.h>

#include <fstream>
#include <sstream>

namespace cmd
{
    struct MyVertex
    {
        float point[3];
        unsigned char color[4];
    };

    // stores a pointcloud associated to a Keyframe.
    class LoopframeDisplay
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        LoopframeDisplay(LoopframePtr lf);
        ~LoopframeDisplay();

        void refreshPC();
        void drawPC(float pointSize);

        void setFromLF(LoopframePtr lf);
        void initColor(std::vector<float> color);

    public:
        // id 设置为uint64 位
        bool m_active;
        bool m_need_refresh;
        uint64_t m_lf_id;
        TransMatrixType m_Twc;

        MutextType m_muetx;

        LoopframePtr m_lf;

    private:
        int m_num_sparse_points;      // point 个数
        int m_num_sparse_buffer_size; // 符合条件的可以输出点个数

        bool m_buffer_valid;             // gl buf 是否合理的标识
        int m_num_gl_buffer_points;      // gl buf 中点的数目
        int m_num_gl_buffer_good_points; // 最新的 gl 可以用到好点

        std::vector<Point2> m_original_point2s; // 当前帧的点
        std::vector<float> m_color;             // 存储的颜色
        pangolin::GlBuffer m_vertex_buffer;     // 最终显示的点
        pangolin::GlBuffer m_color_buffer;      // 最终显示的颜色，下标和点对应
    };

} // namespace dso
