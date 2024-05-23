/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-09-22 21:01:20
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-11-09 17:47:12
 * @FilePath: /codsv_backend/include/codsv/loop_closure/pangolin_viewer/PangolinLoopViewer.h
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
#include "typedefs_backend.hpp"
#include "boost/thread.hpp"
#include <deque>
#include <map>
#include <queue>
#include <pangolin/pangolin.h>

#include "AgentDisplay.h"
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
  class LoopFrameDisplay;

  class PangolinLoopViewer
  {
  public:
    using AgentDisplayMap = std::map<uint32_t, AgentDisplayPtr>;
    typedef std::shared_ptr<PangolinLoopViewer> Ptr;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PangolinLoopViewer(int w, int h, bool startRunThread = true);
    virtual ~PangolinLoopViewer();

    void Run();
    void close();

    void clearAllImagesToDisplay();
    void createNewAgentDisplay(uint32_t client_id);
    void removeAgentDisplay(uint32_t client_id);
    // ==================== Output3DWrapper Functionality ======================

    void publishKeyframes(LoopframePtr lf, bool final = true);
    void publishKeyframes(const std::vector<LoopframePtr> &lfs, bool final = true);

    void refreshLidarData(LoopframePtr current_lf);

    virtual void join();

    void resetDisplayBounds();

    void updateDisplay();
    std::vector<float> getColors();

  private:
    void modifyKeyframePoseByKFID(uint32_t client_id, uint32_t kf_id,
                                  const TransMatrixType &tfm_w_c);
    void passToUpdateBuf(LoopFrameDisplayPtr lf);
    void passToUpdateBuf(const std::vector<LoopFrameDisplayPtr> &lfs);

    void set_map_manager(MapmanagerPtr mapMgr);

  public:
    int colors_index_ = 0;
    // void drawConstraints();
    // void drawLidar();

    boost::thread run_thread_;
    bool running_;
    int w_, h_;
    size_t agents_num;

    // 3D model rendering
    boost::mutex model_3d_mutex_;
    std::vector<LoopFrameDisplayPtr> keyframes_;

    // 是否需要更新agent信息的标识
    bool need_to_update_bound_ = true;
    // 记录已经发送的帧ID
    std::map<uint64_t, LoopFrameDisplayPtr> keyframes_by_client_and_kid_;

    AgentDisplayMap agdmap_;
    MapmanagerPtr mapMgr_;

    // 更新buf
    std::mutex update_loopframe_buf_mtx_;
    std::queue<LoopFrameDisplayPtr> update_loopframe_buf_; // 【锁】

    // lidar rendering
    boost::mutex model_lidar_mutex_;
    std::vector<Eigen::Vector3d> lidar_pts_;
    size_t lidar_cur_sz_;

    // images rendering
    boost::mutex open_images_mutex_;
    // MinimalImageB3 *internal_kf_img_;
    bool kf_img_changed_;

    // colors
    std::vector<VisColorRGB> pgl_colors_;

    // main view
    pangolin::View Visualization3D_display;

    uint32_t agent_id = 0;
  };

} // namespace dso
