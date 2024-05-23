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

#include "typedefs_backend.hpp"
#include "pangolin_viewer.hpp"
#include "agent_display.hpp"

namespace cmd
{
    static LoggerPtr g_logger_sys = SYLAR_LOG_NAME("system");
    void PangolinViewer::Run()
    {
        SYLAR_LOG_INFO(g_logger_sys) << "--> START pangolin viewer thread.";

        pangolin::CreateWindowAndBind("cmd-slam", 2560, 1600);
        const float ratio = w_ / float(h_);
        // zFar ： 可见的距离
        auto proj_mat = pangolin::ProjectionMatrix(w_, h_, 200, 200, w_ / 2, h_ / 2, 0.1, 10000);
        // 相机看的位置
        auto model_view = pangolin::ModelViewLookAt(0, -2000, -1, 0, 0, 0, pangolin::AxisNegY);
        glEnable(GL_DEPTH_TEST);
        // 设置背景为黑色
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        // 主界面 显示所有的点路径建图效果
        pangolin::OpenGlRenderState Visualization3D_camera(proj_mat, model_view);

        Visualization3D_display =
            pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.0, 1.0, -ratio)
                .SetHandler(new pangolin::Handler3D(Visualization3D_camera));
        while (!pangolin::ShouldQuit() && running_)
        {
            // Clear entire screen
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

            updateDisplay();

            // Activate efficiently by object
            // 在主界面中画出 全局 点云和路径
            Visualization3D_display.Activate(Visualization3D_camera);
            boost::unique_lock<boost::mutex> lk3d(model_3d_mutex_);

            for (auto &ag : agdmap_)
            {
                auto &ag_ptr = ag.second;
                // 不知道这里这么画行不行，因为里面用了glend
                ag_ptr->refreshLoopFrames();
                ag_ptr->drawLoopFrames();
            }
            // 画完递交画面
            pangolin::FinishFrame();
        }
        SYLAR_LOG_INFO(g_logger_sys) << "<-- JOIN pangolin viewer thread.";
        exit(1);
    }

}