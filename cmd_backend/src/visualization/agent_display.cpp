#include "agent_display.hpp"

#include "loopframe.hpp"
namespace cmd {
AgentDisplay::AgentDisplay(uint32_t client_id, std::vector<float> color)
    : m_client_id(client_id), m_color(color) {}
void AgentDisplay::addLoopframe(LoopframePtr lf) {
  std::unique_lock<std::mutex> lk(m_mtx_display);
  LoopframeDisplayPtr lf_display = nullptr;
  if (m_displays.find(lf->m_lf_id) == m_displays.end()) {
    lf_display.reset(new LoopframeDisplay(lf));
    lf_display->initColor(m_color);
    m_displays.insert(std::make_pair(lf->m_lf_id, lf_display));
    m_list_displays.push_back(lf_display);
    m_lidar_pts = lf->m_pts_spherical;
  } else {
    m_displays[lf->m_lf_id]->setFromLF(lf);
  }
  return;
}
void AgentDisplay::drawLoopFrames() {
  std::unique_lock<std::mutex> lk(m_mtx_display);
  for (auto lf_display : m_list_displays) {
    lf_display->refreshPC();
    lf_display->drawPC(1.0);
    lf_display->drawPose(0.4);
  }
  drawConstraints();
}
void AgentDisplay::drawConstraints() {
  glColor3f(m_color[0], m_color[1], m_color[2]);
  glLineWidth(3);

  glBegin(GL_LINE_STRIP);
  for (auto& lf_display : m_list_displays) {
    Point3 translatioin = lf_display->m_Twc.translation();
    glVertex3d(translatioin[0], translatioin[1], translatioin[2]);
  }
  glEnd();
}
void AgentDisplay::drawLidar() {
  glPointSize(1.0);

  glBegin(GL_POINTS);
  for (size_t i = 0; i < m_lidar_pts.size(); i++) {
    glColor3ub(0, 255, 0);
    glVertex3f(m_lidar_pts[i](0), m_lidar_pts[i](1), m_lidar_pts[i](2));
  }
  glEnd();
}
}  // namespace cmd