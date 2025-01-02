#include "map.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include "loop_closure/generate_spherical_points.h"
#include "loop_handler.hpp"
#include "loopframe.hpp"
#include "typedefs_backend.hpp"
#include "visualization/pangolin_viewer.hpp"

namespace cmd {
static LoggerPtr g_logger_map = SYLAR_LOG_NAME("Map");
Framemanager::Framemanager(int client_id)
    : m_clientId(client_id), m_optimizing(false) {}
void Framemanager::updateFramesFromCeres() {
  SYLAR_ASSERT2(m_optimizing, "framemanager 不是处于优化状态!");
  // 不需要ceres 更新
  for (auto lf : m_lfs) {
    lf.second->updateFromCeres();
  }
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
    // lf->m_twc = ref_lf->m_twc * tmp_T_tf.inverse();
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
    TransMatrixType Twc = lf->m_original_twc;
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
  if (!viewer_) {
    viewer_ = std::make_shared<PangolinViewer>(VIEWER_WIDTH, VIEWER_HIGH);
  }
  handler_ =
      std::make_unique<LoopHandler>(LIDAR_RANGE, SCANCONTEXT_THRES, this);
  main_thread_ = std::make_shared<Thread>(std::bind(&Mapmanager::Run, this),
                                          "map manager");
  optimized_thread_ = std::make_shared<Thread>(
      std::bind(&Mapmanager::OptimizeRun, this), "optimized thread");
}
Mapmanager::~Mapmanager() {
  m_runing = false;
  optimize_running_ = false;
  main_thread_->join();
}
void Mapmanager::insertLoopframeToMap(LoopframePtr loopframe) {
  MutexType::Lock lock(newframe_buf_mutex_);
  newframe_buf_.push(loopframe);
}
bool Mapmanager::checkNewFrameBuf(LoopframeList &lfs) {
  if (newframe_buf_.empty()) {
    return false;
  }
  MutexType::Lock lock(newframe_buf_mutex_);
  while (!newframe_buf_.empty()) {
    lfs.push_back(newframe_buf_.front());
    newframe_buf_.pop();
  }
  return true;
}
void Mapmanager::Run() {
  SYLAR_LOG_INFO(g_logger_map) << "--> START map manager ";
  while (m_runing) {
    LoopframeList lfs;
    if (checkNewFrameBuf(lfs)) {
      while (!lfs.empty()) {
        auto loopframe = lfs.front();
        lfs.pop_front();
        addLoopframeWithLock(loopframe);
      }
    }
    usleep(50);
  }
  SYLAR_LOG_INFO(g_logger_map) << "<-- END map manager ";
}
void Mapmanager::OptimizeRun() {
  SYLAR_LOG_INFO(g_logger_map) << "--> OPT START";
  while (optimize_running_) {
    FactorGraph new_factors;
    LoopframeValue new_vals;  // [key,twc]
    LoopEdgeVector les;
    {
      MutexType::Lock lk(mutex_);
      if (solver_->checkFactorBufferWithLock(les)) {
        solver_->convertFactorAndValue(les, new_factors, new_vals);
        if (new_factors.size() >= 1 &&
            new_factors[0].m_type == EdgeType::LOOPCLOSURE) {
          SYLAR_LOG_DEBUG(g_logger_map)
              << "PcmSolver process " << new_factors.size() << " factors.";
        }
      }
    }
    if (new_factors.size() != 0) {
      std::vector<bool> need_optimize_idx;
      std::vector<cmd::Sim3LoopframeValue> optimized_values;
      // 执行优化操作
      MutexType::Lock lk(mutex_);
      if (solver_->removeOutlierAndOptimize(
              new_factors, new_vals, need_optimize_idx, optimized_values)) {
        // 更新数据
        solver_->updateDataAfterOptimizeWithLock(need_optimize_idx,
                                                 optimized_values);
        checkOptimizeAndViewUpdate();  // 更新 viewer
      }
    };
    usleep(100);
  }
  SYLAR_LOG_INFO(g_logger_map) << "<-- OPT thread is end.";
}
void Mapmanager::checkOptimizeAndViewUpdate() {
  std::vector<LoopframeValue> full_values;
  std::vector<FactorGraph> full_fgs;
  std::list<LoopframePtr> update_loopframes;
  FactorGraph update_loopclosures;
  auto &frame_mgrs = fmgrs_;
  if (solver_->checkIsOptimized(full_values, full_fgs)) {
    // 获取所有的
    int map_id = 0;
    for (const auto &values : full_values) {
      for (const auto &[key, pose] : values) {
        auto client = GetKeyClientID(key);
        auto id = GetKeyLoopframeID(key);
        if (frame_mgrs.find(client) == frame_mgrs.end()) {
          std::vector<int> clients;
          std::stringstream ss;
          ss << "client:" << client << " | fmgrs client:[ ";
          for (const auto &fmgr : frame_mgrs) {
            ss << fmgr.first << " ";
          }
          ss << "]";
          SYLAR_LOG_DEBUG(g_logger_map) << ss.str();
          SYLAR_ASSERT(false);
        }
        auto lf = frame_mgrs[client]->getLoopframeByKFId(id);
        lf->m_twc = pose;
        update_loopframes.push_back(lf);
      }
    }
    update_loopclosures.reserve(full_fgs.size() * 100);
    for (const auto &fgs : full_fgs) {
      for (const auto &factor : fgs) {
        if (factor.m_type == LOOPCLOSURE) {
          update_loopclosures.push_back(factor);
        }
      }
    }
    if (debug()) {
      SYLAR_LOG_DEBUG(g_logger_map)
          << "update loopframes size:" << update_loopframes.size();
    }
    viewer_->show(update_loopframes, update_loopclosures);
  } else {
    if (debug()) {
      SYLAR_LOG_WARN(g_logger_map) << "pcm solver is not optimized";
    }
  }
  return;
}

/// @brief 负责创建新的 framemanage
/// @param lf
/// @return
bool Mapmanager::addLoopframeWithLock(LoopframePtr lf) {
  MutexType::Lock lock(mutex_);
  auto client = lf->m_client_id;
  if (fmgrs_.find(client) == fmgrs_.end()) {
    FramemanagerPtr fmgr = std::make_shared<Framemanager>(client);
    fmgrs_.insert({fmgr->m_clientId, fmgr});
  }
  // 生成 lidar scan
  auto &fmgr = fmgrs_[client];
  fmgr->addLoopframe(lf);
  // 添加到 pcm 里面
  LoopEdgeVector pcm_les;
  for (auto le : lf->m_edges) {
    pcm_les.push_back(le);
  }
  // pcm_les 这里的 le 是纯里程计约束
  solver_->initFrameAndUpdateOdom(lf, pcm_les);
  viewer_->showLoopframes(lf);
  handler_->pushLoopframe2Buf(lf);
  return true;
}
bool Mapmanager::checkLoopclosureBuf() { return !m_lc_buf.empty(); }

void Mapmanager::processLoopClosures() {
  LoopEdgeVector factors;
  MutexType::Lock lock(lc_buf_mutex_);
  while (checkLoopclosureBuf()) {
    auto factor = m_lc_buf.front();
    factors.push_back(factor);
    m_lc_buf.pop_front();
  }
  solver_->insertLoopEdgeAndUpdate(factors, true);
  // if (debug()) {
  //   SYLAR_LOG_DEBUG(g_logger_map)
  //       << "process " << factors.size() << " loopclosures";
  // }
}
void Mapmanager::createConstrant(LoopframePtr from, LoopframePtr to,
                                 TransMatrixType t_tf, precision_t icp_score,
                                 precision_t sc_score) {
  LoopEdgePtr factor = std::make_shared<LoopEdge>(
      from, to, t_tf, icp_score, sc_score, EdgeType::LOOPCLOSURE);
  MutexType::Lock lock(lc_buf_mutex_);
  // m_lc_buf.push_back(factor);
  solver_->insertLoopEdgeAndUpdate({factor}, true);
}

}  // namespace cmd