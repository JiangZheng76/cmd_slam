/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-09-22 21:01:20
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-11-09 17:47:12
 * @FilePath: /codsv_backend/include/codsv/loop_closure/pangolin_viewer/PangolinViewer.h
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

// This file is modified from <https://github.com/JakobEngel/dso>

#pragma once
#include <map>
#include <queue>
#include <deque>
#include <pangolin/pangolin.h>

#include "agent_display.hpp"
#include "loopframe_display.hpp"

namespace cmd
{

    // class LoopframeDisplay;
    struct VisColorRGB
    {
    public:
        VisColorRGB(float fR, float fG, float fB)
            : mfR(fR), mfG(fG), mfB(fB),
              mu8R((u_int8_t)(fR * 255)), mu8G((u_int8_t)(fG * 255)), mu8B((u_int8_t)(fB * 255))
        {
        }

        const float mfR, mfG, mfB;
        const u_int8_t mu8R, mu8G, mu8B;
    };

    class PangolinViewer
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        PangolinViewer(int w, int h, bool startRunThread = true);
        virtual ~PangolinViewer();

        void Run();
        void close();
        // ==================== Output3DWrapper Functionality ======================

        void showLoopframes(LoopframePtr lf);
        void showLoopframes(const LoopframeVector &lfs);

        void updateDisplay();
        void getColors(std::vector<float>& color);
    private:
        int colors_index_ = 0;

        ThreadPtr m_view_thread;
        bool m_running;
        int m_w, m_h;

        // 3D model rendering
        std::mutex m_model_3d_mutex;
        std::unordered_map<uint32_t, AgentDisplayPtr> m_agent_displays;

        // 更新buf
        std::mutex m_update_loopframe_buf_mtx;
        std::queue<LoopframePtr> m_update_loopframe_buf; // 【锁】

        // colors
        std::vector<VisColorRGB> m_pgl_colors;

        // main view
        pangolin::View m_view;
    };

} // namespace dso
