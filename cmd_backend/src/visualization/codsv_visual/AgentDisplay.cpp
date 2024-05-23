/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-09-25 17:38:57
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-12-25 16:15:06
 * @FilePath: /codsv_backend/src/loop_closure/pangolin_viewer/AgentDisplay.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "typedefs_backend.hpp"
#include "AgentDisplay.h"

namespace cmd{
/// @brief 是否需要加锁，或者是换一种方式进行。暂时不用，因为获取都是在pangolin里面，暂时不会出现冲突问题
/// @param kf_id 
/// @return 
LoopFrameDisplayPtr AgentDisplay::getLF(uint32_t kf_id){
    auto lfd = lfdmap_.find(kf_id);
    if(lfd == lfdmap_.end()){
        return nullptr;
    }
    return lfd->second;
}
void AgentDisplay::addNewLoopFrame(uint32_t kf_id, LoopFrameDisplayPtr lfd){
    // if(kf_id < s_kf_id){
    //     std::cerr << "In the agent addNewLoopFrame() " <<  client_id_
    //         << " the loopframe " << kf_id << " is less than the prev loopframe " << std::endl;
    //     assert(false);
    // }
    std::unique_lock<std::mutex> lk(loopframes_mtx_);
    lfdmap_[kf_id] = lfd;
    loopframes_.push_back(lfd);
    s_kf_id = kf_id;

    lfd->setColor(color_);
}
AgentDisplay::AgentDisplay(uint32_t client_id,float w,float h,std::vector<float> color)
    : client_id_(client_id),ratio_(w / h),w_(w),h_(h),color_(color){
    auto proj_mat =
      pangolin::ProjectionMatrix(w_, h_, 200, 200, w_ / 2, h_ / 2, 0.1, 1000);
    // 它用于控制相机的位置和观察方向
    auto model_view =
        pangolin::ModelViewLookAt(-0, -5, -10, 0, 0, 0, pangolin::AxisNegY);;
    agent_view_camera_ = pangolin::OpenGlRenderState(proj_mat,model_view);
      pangolin::ModelViewLookAt(-0, -5, -10, 0, 0, 0, pangolin::AxisNegY);

    agentView_ = pangolin::Display("Agent_" + client_id_)
        .SetAspect(ratio_)
        .SetHandler(new pangolin::Handler3D(agent_view_camera_));

    agent_lidar_dispaly_ =
        pangolin::Display("lidarDisplay")
          .SetAspect(ratio_)
          .SetHandler(new pangolin::Handler3D(agent_view_camera_));
    agent_trace_display_ = 
        pangolin::Display("traceDisplay")
            .SetAspect(ratio_)
            .SetHandler(new pangolin::Handler3D(agent_view_camera_));
    agentView_
        .SetLayout(pangolin::LayoutEqualHorizontal)
        .AddDisplay(agent_lidar_dispaly_)
        .AddDisplay(agent_trace_display_);
}
void AgentDisplay::setDisplayBounds(float l,float r,float b,float t){
    agentView_.SetBounds(l,r,b,t,-ratio_);
    
}
void AgentDisplay::refreshLoopFrames(){
    std::unique_lock<std::mutex> lk(loopframes_mtx_);
    for(auto& lf : loopframes_){
        lf->refreshPC();
    }
}
/// @brief 画出位姿之间的连接线
/// @param colors 线的颜色
void AgentDisplay::drawConstraints(std::vector<float> colors){
  glColor3f(colors[0], colors[1], colors[2]);
  glLineWidth(3);

  glBegin(GL_LINE_STRIP);
  for(auto& lf : loopframes_){
    glVertex3d(lf->tfm_w_c_.translation()[0],
                lf->tfm_w_c_.translation()[1],
                lf->tfm_w_c_.translation()[2]);
  }
  glEnd();
}
/// @brief 画出点云和路径
void AgentDisplay::drawLoopFrames(){
    std::unique_lock<std::mutex> lk(loopframes_mtx_);
    for(auto& lf : loopframes_){
        lf->refreshPC();
        lf->drawPC(1);
    }
    drawConstraints(color_);
    // drawConstraints({1,0,0});

}

/// @brief 在loophandler里面进行更新，所以需要加锁？？？
/// @param pts 
/// @param cur_sz 
void AgentDisplay::refreshCurlidar(const std::vector<Eigen::Vector3d>& pts,size_t cur_sz){
    assert(cur_sz <= pts.size());
    // 雷达只有在画完之后才可以进行处理
    std::unique_lock<std::mutex> lk(pts_cur_lidar_mtx_);
    pts_cur_lidar_ = pts;
    lidar_cur_sz_ = cur_sz;
}
void AgentDisplay::drawCurlidar(){
    std::unique_lock<std::mutex> lk(pts_cur_lidar_mtx_);
    glPointSize(3.0);
    glBegin(GL_POINTS);
    for (size_t i = 0; i < pts_cur_lidar_.size(); i++) {
    if (i < lidar_cur_sz_) {
    //   glColor3ub(0, 255, 0);
      glColor3ub(255*color_[0],255*color_[1],255*color_[2]);
    } else {
      glColor3ub(255, 0, 0);
    }
    glVertex3f(pts_cur_lidar_[i](0), pts_cur_lidar_[i](1), pts_cur_lidar_[i](2));
    }
    glEnd();
}
/// @brief 激活view并且画出里面的图案，帧图案和lidar图案
void AgentDisplay::activeAndDrawDisplay(){
    // agent_trace_display_.Activate(agent_view_camera_);
    // drawLoopFrames();
    agent_lidar_dispaly_.Activate(agent_view_camera_);
    drawCurlidar();
}
void AgentDisplay::createLidarDisplay(float ratio,pangolin::OpenGlRenderState& Visualization_lidar_camera){
    static float bottom = 0.0,top=0.3,left = 0.0,right = 1.0;    
    agent_lidar_dispaly_ =
      pangolin::Display("lidarDisplay_"+client_id_)
          .SetAspect(ratio)
          .SetBounds(bottom,top,left,right)
          .SetHandler(new pangolin::Handler3D(Visualization_lidar_camera));
}

};
