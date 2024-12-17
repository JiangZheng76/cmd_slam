#include "map.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include "loop_closure/generate_spherical_points.h"
#include "loopframe.hpp"
#include "typedefs_backend.hpp"
#include "visualization/pangolin_viewer.hpp"

namespace cmd {

static LoggerPtr g_logger_sys = SYLAR_LOG_NAME("CMD-SLAM");

Framemanager::Framemanager(int client_id)
    : m_clientId(client_id), m_optimizing(false) {}
void Framemanager::updateFramesFromCeres() {
  SYLAR_ASSERT2(m_optimizing, "framemanager 不是处于优化状态!");
  // 不需要ceres 更新
  for (auto lf : m_lfs) {
    lf.second->updateFromCeres();
  }
}
LoopframeVector Framemanager::updateInsertFrameWhileOptimize() {
  SYLAR_ASSERT2(m_optimizing, "framemanager 不是处于优化状态!");
  LoopframeVector result;
  result.reserve(m_optimizing_buf.size());
  // 更新未入档的 lf
  while (!m_optimizing_buf.empty()) {
    auto lf = m_optimizing_buf.front();
    m_optimizing_buf.pop_front();

    size_t len = lf->m_ref_id.size();
    // 构建 reference
    for (int i = 0; i < len; i++) {
      auto tmp_id = lf->m_ref_id[i];
      auto tmp_T_tf = lf->m_ref_cf[i];
      if (m_lfs.find(tmp_id) == m_lfs.end()) {
        SYLAR_ASSERT2(false, "出现错误 ref 帧 id.");
      }
      auto ref_lf = m_lfs[tmp_id];
      lf->addReference(ref_lf, tmp_T_tf, 0, 0);
      lf->m_twc = ref_lf->m_twc * tmp_T_tf;
    }
    m_lfs.insert(std::make_pair(lf->m_lf_id, lf));
    result.push_back(lf);
  }

  m_optimizing =
      true;  // 更新完之后就重新开锁，防止后面的 addLoopframe 还放进 buf 中
  return result;
}
/// @brief 添加新帧，生成点云信息
/// @param lf
void Framemanager::addLoopframe(LoopframePtr lf) {
  // 因为优化会导致生成的位置发生问题，所以必须要利用原来的位姿信息来生成点云
  genImiLidarScan(lf);

  if (m_optimizing) {
    m_optimizing_buf.push_back(lf);
    return;
  }
  size_t len = lf->m_ref_id.size();
  // 构建 reference
  for (int i = 0; i < len; i++) {
    auto tmp_id = lf->m_ref_id[i];
    auto tmp_T_tf = lf->m_ref_cf[i];
    if (m_lfs.find(tmp_id) == m_lfs.end()) {
      SYLAR_ASSERT2(false, "出现错误 ref 帧 id.");
    }
    auto ref_lf = m_lfs[tmp_id];
    lf->addReference(ref_lf, tmp_T_tf, 0, 0);
    lf->m_twc = ref_lf->m_twc * tmp_T_tf.inverse();
  }
  m_lfs.insert(std::make_pair(lf->m_lf_id, lf));
}
void Framemanager::transfromFrammanager(const TransMatrixType &Ttc) {
  for (auto it = m_lfs.begin(); it != m_lfs.end(); it++) {
    LoopframePtr lf = it->second;
    lf->m_twc = lf->m_twc * Ttc.inverse();
  }
}
LoopframePtr Framemanager::getPrevLoopframe() {
  if (m_lfs.size() != 0) {
    return m_lfs.rbegin()->second;
  }
  return nullptr;
}
LoopframePtr Framemanager::getLoopframeByKFId(int_t kf_id) {
  if (m_lfs.find(kf_id) != m_lfs.end()) {
    return m_lfs[kf_id];
  }
  return nullptr;
}
void Framemanager::genImiLidarScan(LoopframePtr lf) {
  Point3Vector pts_spherical;
  if (m_lidar_range > 0) {
    /* ====================== Extract points ================================ */
    float cx = lf->m_calib.getCx();
    float cy = lf->m_calib.getCy();
    float fx = lf->m_calib.getFx();
    float fy = lf->m_calib.getFy();
    TransMatrixType Twc = lf->m_twc;
    int cur_id = lf->m_lf_id;
    for (const auto &p : lf->m_points) {
      Eigen::Vector4d p_c((p.m_u - cx) / fx / p.m_idepth_scaled,
                          (p.m_v - cy) / fy / p.m_idepth_scaled,
                          1 / p.m_idepth_scaled, 1);
      // 世界坐标的点
      Eigen::Vector3d p_g = Twc.matrix3x4() * p_c;
      m_pts_nearby.emplace_back(std::pair<int, Eigen::Vector3d>(cur_id, p_g));
    }

    /* ============= Preprocess points to have sphereical shape ============= */
    m_id_pose_wc[cur_id] = Twc.log();
    // auto t0 = std::chrono::high_resolution_clock::now();
    generate_spherical_points(m_pts_nearby, m_id_pose_wc, Twc.inverse(),
                              m_lidar_range, pts_spherical);
    // auto t1 = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t1
    // - t0);
    // pts_generation_time_.emplace_back(std::chrono::duration_cast<std::chrono::microseconds>(t1
    // - t0));
  }
  lf->m_pts_spherical = pts_spherical;
}
void Framemanager::dump() {
  // TODO
}
void Framemanager::showResult() {
  // TODO
}
void Framemanager::saveTrajectory(const std::string &save_path) {
  // TODO
}

Mapmanager::Mapmanager(PangolinViewerPtr viewer) : viewer_(viewer) {
  RobustSolverParams params;
  solver_ = std::make_unique<PcmSolver>(params);
  m_thread.reset(new Thread(std::bind(&Mapmanager::Run, this), "map manager"));
}
Mapmanager::~Mapmanager() {
  m_runing = false;
  m_thread->join();
}
void Mapmanager::Run() {
  SYLAR_LOG_INFO(g_logger_sys) << "--> START map manager ";
  while (m_runing) {
    checkOptimizeAndViewUpdate();
    if (checkLoopclosureBuf()) {
      processLoopClosures();
    }
    usleep(1000);
  }
  SYLAR_LOG_INFO(g_logger_sys) << "<-- END map manager ";
}
void Mapmanager::checkOptimizeAndViewUpdate() {
  std::vector<LoopframeValue> full_values;
  std::vector<LoopframePtr> update_lfs;
  if (solver_->checkIsOptimized(full_values)) {
    update_lfs.reserve(full_values.front().size() * 10);
    for (const auto &values : full_values) {
      for (const auto &[key, pose] : values) {
        auto client = GetKeyClientID(key);
        auto id = GetKeyLoopframeID(key);
        if (fmgrs_.find(client) == fmgrs_.end()) {
          std::vector<int> clients;
          std::stringstream ss;
          ss << "client:" << client << " | fmgrs client:[ ";
          for (auto &fmgr : fmgrs_) {
            ss << fmgr.first << " ";
          }
          ss << "]";
          SYLAR_LOG_DEBUG(g_logger_sys) << ss.str();
          SYLAR_ASSERT(false);
        }
        auto lf = fmgrs_[client]->getLoopframeByKFId(id);
        update_lfs.push_back(lf);
      }
    }
    if (debug_)
      SYLAR_LOG_DEBUG(g_logger_sys)
          << "update Loopframe viewer size:" << update_lfs.size();
    viewer_->showLoopframes(update_lfs);
  }
  return;
}

/// @brief 负责创建新的 framemanage
/// @param lf
/// @return
bool Mapmanager::addLoopframe(LoopframePtr lf) {
  // SYLAR_LOG_DEBUG(g_logger_sys) << "接收到新 Loopframe\n"<< lf->dump();
  auto client = lf->m_client_id;
  if (fmgrs_.find(client) == fmgrs_.end()) {
    FramemanagerPtr fmgr = std::make_shared<Framemanager>(client);
    fmgrs_.insert({fmgr->m_clientId, fmgr});
  }
  fmgrs_[client]->addLoopframe(lf);

  // 添加到 pcm 里面
  LoopEdgeVector pcm_les;
  for (auto le : lf->m_edges) {
    pcm_les.push_back(le);
  }
  // pcm_les 这里的 le 是纯里程计约束
  solver_->insertLoopEdgeAndUpdate(pcm_les, true);
  viewer_->showLoopframes(lf);
  return true;
}
bool Mapmanager::checkLoopclosureBuf() { return !m_lc_buf.empty(); }

void Mapmanager::processLoopClosures() {
  LoopEdgeVector factors;
  std::unique_lock<std::mutex> lk(m_mtx_lc_buf);
  while (checkLoopclosureBuf()) {
    auto factor = m_lc_buf.front();
    factors.push_back(factor);
    m_lc_buf.pop_front();
  }
  solver_->insertLoopEdgeAndUpdate(factors, true);
  if (debug_)
    SYLAR_LOG_DEBUG(g_logger_sys)
        << "process " << factors.size() << " loopclosures";
}
void Mapmanager::createConstrant(LoopframePtr from, LoopframePtr to,
                                 TransMatrixType t_tf, precision_t icp_score,
                                 precision_t sc_score) {
  LoopEdgePtr factor = std::make_shared<LoopEdge>(
      from, to, t_tf, icp_score, sc_score, EdgeType::LOOPCLOSURE);
  std::unique_lock<std::mutex> lk(m_mtx_lc_buf);
  m_lc_buf.push_back(factor);
}

}  // namespace cmd