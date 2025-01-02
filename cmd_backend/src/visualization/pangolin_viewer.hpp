#pragma once
#include <pangolin/pangolin.h>

#include <deque>
#include <map>
#include <queue>

#include "agent_display.hpp"
#include "loopframe_display.hpp"
#include "optimization/pcm_pgo/utils/TypeUtils.h"

namespace cmd {

// class LoopframeDisplay;
struct VisColorRGB {
 public:
  VisColorRGB(float fR, float fG, float fB)
      : mfR(fR),
        mfG(fG),
        mfB(fB),
        mu8R((u_int8_t)(fR * 255)),
        mu8G((u_int8_t)(fG * 255)),
        mu8B((u_int8_t)(fB * 255)) {}

  const float mfR, mfG, mfB;
  const u_int8_t mu8R, mu8G, mu8B;
};
struct FactorDisplay {
  TransMatrixType from_wc;
  TransMatrixType to_wc;
  FactorDisplay(const LoopEdge& factor);
};
class FactorGraphDisplay : public std::vector<FactorDisplay> {
 public:
  FactorGraphDisplay(const FactorGraph& fg);
  FactorGraphDisplay(){}
};
class PangolinViewer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PangolinViewer(int w, int h, bool startRunThread = true);
  virtual ~PangolinViewer();

  void Run();
  void close();
  // ==================== Output3DWrapper Functionality ======================

  void showLoopframes(LoopframePtr lf);
  void showLoopClosures(LoopEdge& le);
  void show(const LoopframeList& lfs, const FactorGraph& factors);

  bool checkLoopframeBuf();
  bool checkFactorGraphBuf(FactorGraphDisplay& fg);

  AgentDisplayPtr getAgentsDisplay(int client);

  void updateDisplay();
  void drawLoopClosureFactor();
  void getColors(std::vector<float>& color);
  void saveTrajectory(const std::string& filename);

 private:
  int colors_index_ = 0;

  ThreadPtr m_view_thread;
  bool m_running;
  int m_w, m_h;

  // 3D model rendering
  std::mutex m_model_3d_mutex;
  std::unordered_map<uint32_t, AgentDisplayPtr> m_agent_displays;
  FactorGraphDisplay loopclosure_factors_;

  // 更新buf
  std::mutex update_mtx_;
  std::queue<LoopframePtr> update_loopframe_buf_;  // 【锁】
  std::list<FactorGraph> update_fg_buf_;

  // colors
  std::vector<VisColorRGB> m_pgl_colors;

  // main view
  pangolin::View m_view;
};

}  // namespace cmd
