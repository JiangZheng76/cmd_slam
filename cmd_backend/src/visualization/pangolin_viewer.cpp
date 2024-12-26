#include "pangolin_viewer.hpp"

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>

#include "agent_display.hpp"
#include "loopframe.hpp"
#include "typedefs_backend.hpp"

namespace cmd {
static LoggerPtr g_logger_sys = SYLAR_LOG_NAME("CMD-VIEW");

PangolinViewer::PangolinViewer(int w, int h, bool startRunThread)
    : m_w(w), m_h(h) {
  m_running = true;

  for (auto color : col_vec) {
    VisColorRGB vcr(color[0], color[1], color[2]);
    m_pgl_colors.push_back(vcr);
  }

  if (startRunThread) {
    m_view_thread.reset(
        new Thread(std::bind(&PangolinViewer::Run, this), "cmd_viewer thread"));
  }
}
PangolinViewer::~PangolinViewer() {
  m_running = false;
  m_view_thread->join();
}
void PangolinViewer::showLoopframes(LoopframePtr lf) {
  std::unique_lock<std::mutex> lk(m_update_loopframe_buf_mtx);
  m_update_loopframe_buf.push(lf);
}
void PangolinViewer::showLoopframes(const LoopframeVector &lfs) {
  std::unique_lock<std::mutex> lk(m_update_loopframe_buf_mtx);
  for (auto lf : lfs) {
    m_update_loopframe_buf.push(lf);
  }
}
void PangolinViewer::updateDisplay() {
  std::unique_lock<std::mutex> lk(m_update_loopframe_buf_mtx);
  while (!m_update_loopframe_buf.empty()) {
    LoopframePtr lf = m_update_loopframe_buf.front();
    m_update_loopframe_buf.pop();

    auto client_id = lf->m_client_id;
    AgentDisplayPtr agent_display = nullptr;
    if (m_agent_displays.find(client_id) == m_agent_displays.end()) {
      std::vector<float> colors;
      getColors(colors);
      agent_display.reset(new AgentDisplay(client_id, colors));
      m_agent_displays.insert(std::make_pair(client_id, agent_display));
    } else {
      agent_display = m_agent_displays[client_id];
    }

    agent_display->addLoopframe(lf);
  }
}
void PangolinViewer::getColors(std::vector<float> &color) {
  static int s_color_id = 0;
  ++s_color_id;

  int color_nums = m_pgl_colors.size();
  s_color_id %= color_nums;

  color.resize(3);
  color[0] = m_pgl_colors[s_color_id].mfR;
  color[1] = m_pgl_colors[s_color_id].mfG;
  color[2] = m_pgl_colors[s_color_id].mfB;
}
void PangolinViewer::Run() {
  pangolin::CreateWindowAndBind("CMD-SLAM", 960, 1080);
  const float ratio = m_w / m_h;

  // 透视投影矩阵
  // left、right、bottom、top：定义了视锥体的近裁剪面的左、右、底部和顶部坐标。
  // near 和 far：定义了视锥体的近裁剪面和远裁剪面的距离。
  // right_handed：一个布尔值，表示是否使用右手坐标系。如果为true，则使用右手坐标系；如果为false，则使用左手坐标系。
  auto proj_mat = pangolin::ProjectionMatrix(m_w, m_h, 200, 200, m_w / 2,
                                             m_h / 2, 0.1, 1000);
  // 它用于控制相机的位置和观察方向
  auto model_view =
      pangolin::ModelViewLookAt(-0, -5, -10, 0, 0, 0, pangolin::AxisNegY);

  glEnable(GL_DEPTH_TEST);

  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);  // 设置背景为黑色
  pangolin::OpenGlRenderState camera(
      proj_mat, model_view);  // 主界面 显示所有的点路径建图效果
  m_view = pangolin::CreateDisplay()
               .SetBounds(0.0, 1.0, 0.0, 1.0, -ratio)
               .SetHandler(new pangolin::Handler3D(camera));

  // TODO 测试雷达
  // lidar visualization
  // 创建一个点云的小显示界面，并且绑定鼠标的指针控制器
  pangolin::OpenGlRenderState Visualization_lidar_camera(proj_mat, model_view);
  pangolin::View &Visualization_lidar_display =
      pangolin::Display("lidarDisplay")
          .SetAspect(ratio)
          .SetHandler(new pangolin::Handler3D(Visualization_lidar_camera));
  pangolin::CreateDisplay()
      // .SetBounds() 方法来设置显示对象的位置和大小
      .SetBounds(0.0, 0.3, 0.0, 1.0)
      // 设置布局选项，例如水平布局、垂直布局、网格布局等。
      .SetLayout(pangolin::LayoutEqualHorizontal)
      // 添加子view
      .AddDisplay(Visualization_lidar_display);

  while (!pangolin::ShouldQuit() && m_running) {
    // Clear entire screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    updateDisplay();

    // Activate efficiently by object
    // 在主界面中画出 全局 点云和路径
    m_view.Activate(camera);
    std::unique_lock<std::mutex> lk3d(m_model_3d_mutex);

    for (auto &p : m_agent_displays) {
      auto agent_display = p.second;
      agent_display->drawLoopFrames();
    }

    Visualization_lidar_display.Activate(Visualization_lidar_camera);
    // boost::unique_lock<boost::mutex> lklidar(model_lidar_mutex_);
    for (auto &p : m_agent_displays) {
      auto agent_display = p.second;
      agent_display->drawLidar();
    }

    // 画完递交画面
    pangolin::FinishFrame();
  }
  // 保存微位姿信息
  saveTrajectory("Trajectory");
  exit(1);
}
void PangolinViewer::saveTrajectory(const std::string &filename) {
  char buff[FILENAME_MAX];
  getcwd(buff, sizeof(buff));
  std::string save_path(buff);
  save_path += "/" + filename;

  struct stat st;
  if (!(stat(save_path.c_str(), &st) == 0 && S_ISDIR(st.st_mode))) {
    int result = mkdir(save_path.c_str(), 0777);
    if (result == -1) {
      SYLAR_LOG_INFO(g_logger_sys) << "mkdir " << save_path << " failed";
      return;
    }
  }
  for (auto &[client, agent] : m_agent_displays) {
    std::string client_filename =
        save_path + "/" + std::to_string(client) + ".txt";
    std::ofstream file_stream(client_filename);
    if (!file_stream.is_open()) {
      std::cerr << "Failed to open the file: " << client_filename << std::endl;
      SYLAR_LOG_ERROR(g_logger_sys)
          << "Failed to open the file: " << client_filename;
      break;
    }
    for (const auto &lf : agent->m_list_displays) {
      auto Twc = lf->m_Twc.matrix().transpose();
      int rows = Twc.rows(), cols = Twc.cols();
      for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
          file_stream << Twc(j, i) << " ";
        }
      }
      file_stream << std::endl;
    }
    file_stream.close();
    SYLAR_LOG_INFO(g_logger_sys) << "save trajectory to " << client_filename;
  }
}

}  // namespace cmd