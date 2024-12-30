#pragma once

#include <sophus/sim3.hpp>

#include "optimization/pcm_pgo/pcm.hpp"
#include "optimization/pcm_pgo/solver_params.hpp"
#include "optimization/pcm_pgo/utils/TypeUtils.h"
#include "typedefs_backend.hpp"

namespace cmd {
// 维护所有的 factor 和 一个 Loopframe副本
// 原有的LoopframePtr 保持不变，用来可视化
class PcmSolver {
 public:
  PcmSolver(RobustSolverParams params);

  virtual ~PcmSolver();

  // 插入 factor 并更新
  void insertLoopEdgeAndUpdate(const LoopEdgeVector& les, bool is_optimize);

  void initFrameAndUpdateOdom(LoopframePtr loopframe,
                              const LoopEdgeVector& les);

  bool removeOutlierAndOptimize(
      FactorGraph& factors, LoopframeValue& values,
      std::vector<bool>& need_optimize_idx,
      std::vector<cmd::Sim3LoopframeValue>& optimized_values);

  void convertFactorAndValue(const LoopEdgeVector& les, FactorGraph& factors,
                             LoopframeValue& value);

  void getAllValueToUpdateMap(LoopframeValue& vals);

  void optimize(std::vector<bool>& need_optimize_idx,
                std::vector<Sim3LoopframeValue>& optimized_values);

  bool checkNeedOptimize();

  void Stop();

//   void Run();

  void ceresSim3Optimize(std::vector<FactorGraph>& full_fgs,
                         std::vector<Sim3LoopframeValue>& full_values,
                         const std::vector<bool>& need_optimize_idx);

  void updateDataAfterOptimizeWithLock(
      const std::vector<bool>& need_optimize_idx,
      const std::vector<Sim3LoopframeValue>& sim3_values);

  bool checkIsOptimized(std::vector<cmd::LoopframeValue>& values);

  bool checkFactorBufferWithLock(LoopEdgeVector& les);

  MapPtr getMap();

 private:
  std::unique_ptr<Pcm> outlier_removal_;
  std::vector<FactorGraph> nfg_;  // 包括里程计和回环的合法 factor
  std::vector<LoopframeValue> values_;  // 所有顶点记录的是 t_cw 的位姿

  bool optimizing_;
  bool need_optimize_;
  bool is_optimized_;  // view 更新
  std::unordered_set<int_t>
      need_optimize_clients_;  // 需要优化的client，会累计，重复的会合并
  std::unordered_map<int_t, std::pair<LoopframeKey, TransMatrixType>>
      last_client_value_;  // 保存每一个 client 当前最新的位姿

  bool solver_running_;  // 优化线程
  MutexType mutex_;      // 读写里面数据的互斥变量
  std::queue<LoopEdgePtr> factors_buf_;
  MutexType optimize_mutex_;
};
}  // namespace cmd