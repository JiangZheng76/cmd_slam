#ifndef __AGENTDISPLAY_H__
#define __AGENTDISPLAY_H__
#include "loopframe_display.hpp"
#include "pangolin/pangolin.h"
#include "typedefs_backend.hpp"
namespace cmd {
class AgentDisplay {
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

}  // namespace cmd
#endif