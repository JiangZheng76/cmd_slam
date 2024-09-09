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
#include "loopframe_display.hpp"
namespace cmd
{
    class AgentDisplay
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        AgentDisplay(uint32_t client_id, std::vector<float> color);


        void drawLidar();
        void drawLoopFrames();
        void drawConstraints();
        void addLoopframe(LoopframePtr lf);
        
    public:
        uint32_t m_client_id;

        std::vector<float> m_color;
        std::unordered_map<int_t, LoopframeDisplayPtr> m_displays;
        std::list<LoopframeDisplayPtr> m_list_displays;
        
        Point3Vector m_lidar_pts;

        std::mutex m_mtx_display;
    };

}
#endif