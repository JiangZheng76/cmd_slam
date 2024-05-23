/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-09-25 17:03:12
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-10-17 17:08:44
 * @FilePath: /codsv_backend/include/codsv/loop_closure/pangolin_viewer/AgentDisplay.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __AGENTDISPLAY_H__
#define __AGENTDISPLAY_H__

#include "typedefs_backend.hpp"
#include "pangolin/pangolin.h"
namespace cmd
{
        class AgentDisplay
        {
        public:
            AgentDisplay(uint32_t client_id, float w, float h, std::vector<float> color);

            using LoopFrameDisplayMap = std::map<uint32_t, LoopframeDisplayPtr>;

            void insertLoopFrame(LoopframeDisplayPtr lfd);

            void updateLoopFrame(LoopframePtr lf);

            // LoopframeDisplayPtr getLF(uint32_t kf_id);

            void addNewLoopFrame(uint32_t kf_id, LoopframeDisplayPtr lfd);

            void activeAndDrawDisplay();

            void setDisplayBounds(float l, float r, float b, float t);

            void refreshLoopFrames();

            void drawLoopFrames();

            void drawConstraints(std::vector<float> colors);

            void drawCurlidar();

            void refreshCurlidar(const std::vector<Eigen::Vector3d> &pts, size_t cur_sz);
            //
            void createLidarDisplay(float ratio, pangolin::OpenGlRenderState &Visualization_lidar_camera);

        private:
            uint32_t client_id_;

            // 这个用于display
            std::mutex loopframes_mtx_;
            std::list<LoopframeDisplayPtr> loopframes_;
            // 这个用于修改
            LoopFrameDisplayMap lfdmap_;

            float ratio_;
            float w_;
            float h_;
            std::mutex pts_cur_lidar_mtx_;
            std::vector<Eigen::Vector3d> pts_cur_lidar_;
            size_t lidar_cur_sz_;

            // 显示模块
            pangolin::View agentView_;
            pangolin::View agent_lidar_dispaly_;
            pangolin::View agent_trace_display_;

            pangolin::OpenGlRenderState agent_view_camera_;

            std::vector<float> color_;

            uint32_t s_kf_id = 0;
        };

}
#endif