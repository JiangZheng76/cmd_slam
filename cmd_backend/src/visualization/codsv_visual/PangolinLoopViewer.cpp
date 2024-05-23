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

#include "PangolinLoopViewer.h"
#include "LoopFrameDisplay.h"
#include "AgentDisplay.h"

#include "typedefs_backend.hpp"

#include "loopframe.hpp"

namespace cmd
{

  PangolinLoopViewer::PangolinLoopViewer(int w, int h, bool startRunThread)
  {
    for(auto color : col_vec){
      VisColorRGB vcr(color[0],color[1],color[2]);
      pgl_colors_.push_back(vcr);
    }

    w_ = w;
    h_ = h;
    running_ = true;

    boost::unique_lock<boost::mutex> lk(open_images_mutex_);

    if (startRunThread)
      run_thread_ = boost::thread(&PangolinLoopViewer::Run, this);

    lidar_cur_sz_ = 0;
  }

  PangolinLoopViewer::~PangolinLoopViewer()
  {
    close();
    run_thread_.join();
  }

  /// @brief 没有搞好，调试时候再调试,根据agent的数量来控制显示的布局？？？?
  void PangolinLoopViewer::resetDisplayBounds()
  {
    int ag_nums = agdmap_.size();
    int i = 0;
    for (auto &ag : agdmap_)
    {
      auto &ag_ptr = ag.second;
      ag_ptr->setDisplayBounds(0.2 * i, 1.0, 0.0, 1.0);
    }
    std::cout << "add new agent resetDisplayBounds()." << std::endl;
    need_to_update_bound_ = false;
  }

  void PangolinLoopViewer::passToUpdateBuf(LoopFrameDisplayPtr lf)
  {
    std::unique_lock<std::mutex> lk(update_loopframe_buf_mtx_);
    update_loopframe_buf_.push(lf);
  }
  void PangolinLoopViewer::passToUpdateBuf(const std::vector<LoopFrameDisplayPtr> &lfs)
  {
    std::unique_lock<std::mutex> lk(update_loopframe_buf_mtx_);
    for (auto &lf : lfs)
      update_loopframe_buf_.push(lf);
  }
  /// @brief 将发送过来的loopframe进行更新，无论是已经存在或者是不存在,不存在的就创建新的
  void PangolinLoopViewer::updateDisplay()
  {
    std::unique_lock<std::mutex> lf(update_loopframe_buf_mtx_);
    int cnt = 0;
    while (!update_loopframe_buf_.empty())
    {
      auto lfd = update_loopframe_buf_.front();
      update_loopframe_buf_.pop();
      uint32_t client_id = lfd->lf_->m_client_id;
      uint32_t kf_id = lfd->lf_->m_lf_id;

      if (agdmap_.find(client_id) == agdmap_.end())
      {
        createNewAgentDisplay(client_id);
        // 把agentdisplay 给到 FrameManager中
        need_to_update_bound_ = true;
      }
      AgentDisplayPtr agd = agdmap_[client_id];
      assert(agd);
      if (!lfd->lf_->m_is_display)
      {
        agd->addNewLoopFrame(kf_id, lfd);
        refreshLidarData(lfd->lf_);
        lfd->lf_->m_is_display = true;
      }
      else
      {
        // Sophus::SE3d经常会出错？？？?
        // g2o::SE3Quat g2o_tfm_w_c(lf->get_tfm_w_c());
        // assert(Sophus::isOrthogonal(g2o_tfm_w_c.to_homogeneous_matrix()));
        // Sophus::SE3d tfm_w_c (g2o_tfm_w_c.rotation(),g2o_tfm_w_c.translation());
        // 这个修改还没有改
        modifyKeyframePoseByKFID(client_id, kf_id, lfd->tfm_w_c_);
        cnt++;
      }
    }
    // if(cnt){
    //   std::cout << "modifyKeyframePoseByKFID " << cnt  << " loopframes." << std::endl;
    // }
  }
  std::vector<float> PangolinLoopViewer::getColors()
  {
    auto tmp = pgl_colors_[(colors_index_++) % pgl_colors_.size()];
    return std::vector<float>({tmp.mfR, tmp.mfG, tmp.mfB});
  }
  void PangolinLoopViewer::set_map_manager(MapmanagerPtr mapMgr)
  {
    mapMgr_ = mapMgr;
  }
  /// @brief 可视化执行主函数，设置成可以显示多个agent的界面，建图是一开始就一起的，参考DCL-SLAM的画法
  void PangolinLoopViewer::Run()
  {
    std::cout << "+++ start pangolin loop viewer thread. +++ " << std::endl;
    pangolin::CreateWindowAndBind("CODSV-SLAM", w_, h_);
    const float ratio = w_ / float(h_);
    // zFar ： 可见的距离
    auto proj_mat =
        pangolin::ProjectionMatrix(w_, h_, 200, 200, w_ / 2, h_ / 2, 0.1, 10000);
    // pangolin::GLprecision x, pangolin::GLprecision y, pangolin::GLprecision z 相机初始的位置
    // 相机看的位置
    auto model_view =
        pangolin::ModelViewLookAt(0, -2000, -1, 0, 0, 0, pangolin::AxisNegY);
    glEnable(GL_DEPTH_TEST);
    // 设置背景为黑色
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    // 主界面 显示所有的点路径建图效果
    pangolin::OpenGlRenderState Visualization3D_camera(proj_mat, model_view);

    Visualization3D_display =
        pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -ratio)
            .SetHandler(new pangolin::Handler3D(Visualization3D_camera));
    // 初始创建 4 个loop display
    pangolin::OpenGlRenderState Visualization_lidar_camera(proj_mat, model_view);
    for (int i = 0; i < 4; i++)
    {
      createNewAgentDisplay(i);
      AgentDisplayPtr agd = agdmap_[i];
      agd->createLidarDisplay(ratio, Visualization_lidar_camera);
    }
    // Default hooks for exiting (Esc) and fullscreen (tab).
    while (!pangolin::ShouldQuit() && running_)
    {
      // Clear entire screen
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

      // ？？？检查界面的存在情况，然后重新reset画面
      // 如果没有agent的话，就停止运动
      // 更新画面信息

      updateDisplay();

      // 更新点云信息

      // Activate efficiently by object
      // 在主界面中画出 全局 点云和路径
      Visualization3D_display.Activate(Visualization3D_camera);
      boost::unique_lock<boost::mutex> lk3d(model_3d_mutex_);
      // pangolin::glDrawColouredCube();
      for (auto &ag : agdmap_)
      {
        auto &ag_ptr = ag.second;
        // 不知道这里这么画行不行，因为里面用了glend
        ag_ptr->refreshLoopFrames();
        ag_ptr->drawLoopFrames();
      }
      lk3d.unlock();
      // 更新画点云信息
      for (auto agd : agdmap_)
      {
        auto &agentDisplay = agd.second;
        agentDisplay->activeAndDrawDisplay();
      }
      // 画照片暂时没有进行保存照片，以后再说？？？
      // open_images_mutex_.lock();
      // if (kf_img_changed_)
      //   texKFDepth.Upload(internal_kf_img_->data, GL_BGR, GL_UNSIGNED_BYTE);
      // kf_img_changed_ = false;
      // open_images_mutex_.unlock();
      // d_kfDepth.Activate();
      // glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
      // texKFDepth.RenderToViewportFlipY();

      // 暂时先不画agent的，先话主界面的？？？
      // for(auto& ag : agdmap_){
      //   auto& ag_ptr = ag.second;
      //   ag_ptr->activeAndDrawDisplay();
      // }

      // Swap frames and Process Events
      pangolin::FinishFrame();
    }
    std::cout << "--- join pangolin loop viewer thread. --- " << std::endl;
    exit(1);
  }

  void PangolinLoopViewer::close() { running_ = false; }

  void PangolinLoopViewer::join()
  {
    run_thread_.join();
    printf("JOINED Pangolin thread!\n");
  }
  void PangolinLoopViewer::createNewAgentDisplay(uint32_t client_id)
  {
    if (client_id < agent_id)
    {
      std::cerr << "the agent " << client_id << " is less than the prev agent's client_id." << std::endl;
      assert(false);
    }

    if (agdmap_.find(client_id) != agdmap_.end())
    {
      std::cerr << "the agent " << client_id << " already exist." << std::endl;
      assert(false);
    }
    AgentDisplayPtr agd(new AgentDisplay(client_id, w_, h_, getColors()));
    agdmap_[client_id] = agd;
  }
  void PangolinLoopViewer::removeAgentDisplay(uint32_t client_id)
  {
    if (agdmap_.find(client_id) == agdmap_.end())
    {
      std::cerr << "the agent " << client_id << " dose not exist." << std::endl;
      assert(false);
    }
    auto agd = agdmap_.find(client_id);
    agdmap_.erase(agd);
  }
  /// @brief 发送 loopframe到pangolin里面去
  /// @param frames
  /// @param final
  /// @param HCalib
  void PangolinLoopViewer::publishKeyframes(LoopframePtr lf,
                                            bool final)
  {
    // only work on marginalized frame
    if (!final)
      return;
    LoopFrameDisplayPtr lfd(new LoopFrameDisplay(lf));
    passToUpdateBuf(lfd);
  }
  void PangolinLoopViewer::publishKeyframes(const std::vector<LoopframePtr> &lfs,
                                            bool final)
  {
    // only work on marginalized frame
    if (!final)
      return;
    std::vector<LoopFrameDisplayPtr> lfds;
    lfds.reserve(lfs.size());
    for (auto &lf : lfs)
    {
      lfds.push_back(LoopFrameDisplayPtr(new LoopFrameDisplay(lf)));
    }
    passToUpdateBuf(lfds);
  }

  void PangolinLoopViewer::modifyKeyframePoseByKFID(uint32_t client_id, uint32_t kf_id,
                                                    const TransMatrixType &tfm_w_c)
  {
    boost::unique_lock<boost::mutex> lk3d(model_3d_mutex_);
    // auto agd_it = agdmap_.find(client_id);
    // if(agd_it == agdmap_.end()){
    //   std::cerr << "modifyKeyframePoseByKFID() agent : " << client_id << " is not exist." <<std::endl;
    //   return ;
    // }
    auto agd = agdmap_[client_id];
    assert(agd);
    agd->getLF(kf_id)->tfm_w_c_ = tfm_w_c;
    agd->getLF(kf_id)->need_refresh_ = true;
  }
  /// @brief 需要将雷达放入到对应的agent里面去
  /// @param pts
  /// @param cur_sz
  void PangolinLoopViewer::refreshLidarData(LoopframePtr current_lf)
  {
    boost::unique_lock<boost::mutex> lk(model_lidar_mutex_);
    // assert(cur_sz <= pts.size());
    AgentDisplayPtr agent = agdmap_[current_lf->m_client_id];
    assert(agent);
    agent->refreshCurlidar(current_lf->m_pts_spherical, current_lf->m_pts_spherical.size());
    // lidar_pts_ = pts;
    // lidar_cur_sz_ = cur_sz;
  }

} // namespace dso
