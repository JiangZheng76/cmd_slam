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

  void update(FactorGraph& factors, LoopframeValue& value,
              bool is_optimize = true);

  void convertFactorAndValue(const LoopEdgeVector& les, FactorGraph& factors,
                             LoopframeValue& value);

  void getAllValueToUpdateMap(LoopframeValue& vals);

  void optimize(std::vector<bool>& need_optimize_idx);

  bool checkNeedOptimize();

  void Stop();

  void Run();

  void ceresSim3Optimize(std::vector<FactorGraph>& full_fgs,
                         std::vector<Sim3LoopframeValue>& full_values,
                         std::vector<bool>& need_optimize_idx);

  void updateDataAfterOptimize(std::vector<bool>& need_optimize_idx,
                               std::vector<Sim3LoopframeValue>& sim3_values);

  bool checkIsOptimized(std::vector<LoopframeValue>& values);

  MapPtr getMap();

 private:
  std::unique_ptr<OutlierRemoval> outlier_removal_;
  std::vector<FactorGraph> nfg_;  // 包括里程计和回环的合法 factor
  std::vector<LoopframeValue> values_;  // 所有顶点记录的是 t_cw 的位姿

  bool optimizing_;
  bool need_optimize_;
  bool is_optimized_;  // view 更新
  std::unordered_set<int_t>
      need_optimize_clients_;  // 需要优化的client，会累计，重复的会合并

  bool solver_running_;  // 优化线程
  RWMutexType mutex_;    // 读写里面数据的互斥变量
  std::queue<LoopEdgePtr> factors_buf_;
  MutexType optimize_mutex_;
};
}  // namespace cmd