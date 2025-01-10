#include "optimization/pcm_pgo/pcm_solver.hpp"

#include <ceres/ceres.h>
#include "loopframe.hpp"
#include "optimization/cmd_sim3.hpp"

namespace cmd {
static LoggerPtr g_logger_solver = SYLAR_LOG_NAME("Solver");
TimeCosters pcm_costs_("pcm_cost");
TimeCosters pgo_costs_("pgo_cost");
PcmSolver::PcmSolver(RobustSolverParams params) {
  solver_running_ = true;
  optimizing_ = false;
  need_optimize_ = false;
  is_optimized_ = false;
  PcmParams pcm_params;
  outlier_removal_ = std::make_unique<Pcm>(pcm_params);
  // 启动优化线程
  // Thread(std::bind(&PcmSolver::Run, this), "pcm_solver_thread");
}
PcmSolver::~PcmSolver() { Stop(); }
bool PcmSolver::checkFactorBufferWithLock(LoopEdgeVector &les) {
  MutexType::TryLock lk(mutex_);
  if (lk.trylock()) {
    les.reserve(factors_buf_.size());
    while (!factors_buf_.empty()) {
      auto le = factors_buf_.front();
      factors_buf_.pop();
      les.push_back(le);
    }
    return true;
  }
  return false;
}
void PcmSolver::convertFactorAndValue(const LoopEdgeVector &les,
                                      FactorGraph &factors,
                                      LoopframeValue &values) {
  factors.reserve(les.size());
  for (auto ptr : les) {
    factors.push_back(*ptr);
  }
  for (auto fac : factors) {
    auto from = fac.m_from_lf;
    auto to = fac.m_to_lf;

    auto from_key = GetKey(from->m_client_id, from->m_lf_id);
    auto to_key = GetKey(to->m_client_id, to->m_lf_id);

    // 保持和外面 framemanager 保持一致
    // ???
    // 一次错误的优化，会不会导致整个崩掉，从而导致后面正确的优化也会失败呢？（不会，pcm工具里面的
    // odom 不会更新）
    // 第一帧才会放 twd，其他都是放 ref，用于 pcm 里面更新
    values.insert(
        {from_key, from->m_is_first ? from->m_twc : from->m_ref_cf[0]});
    values.insert({to_key, to->m_is_first ? to->m_twc : to->m_ref_cf[0]});
  }
}
inline void initFrame(LoopframePtr prev_frame, LoopframePtr cur_frame,
                      TransMatrixType Tpc) {
  auto twp = prev_frame->m_twc;  // first_frame的帧未初始化时候是真值
  auto twc = twp * Tpc;
  cur_frame->m_twc = twc;
  cur_frame->m_is_init = true;
  // if (debug()) {
  //   SYLAR_LOG_DEBUG(g_logger_solver)
  //       << "init frame "
  //       << DumpKey(GetKey(cur_frame->m_client_id, cur_frame->m_lf_id));
  // }
}
// 将新帧初始化到 pcm 中，并且更新 lf 的值
void PcmSolver::initFrameAndUpdateOdom(LoopframePtr loopframe,
                                       const LoopEdgeVector &les) {
  auto client = loopframe->m_client_id;
  auto id = loopframe->m_lf_id;
  auto key = GetKey(client, id);
  if (loopframe->m_is_first) {
    return;  // 跳过第一帧,第二帧的时候会处理第一帧
  }
  // 初始化位姿
  if (loopframe->m_is_init) {
    SYLAR_LOG_ERROR(g_logger_solver)
        << "frame is already init " << DumpKey(key);
    SYLAR_ASSERT(false);
  }
  // 初始化 loopframe 数据,基于 values 里面找 values->loopframe
  // 判断是否第二帧，如果第二帧需要初始化
  auto odom_factor = les[0];
  LoopframePtr prev_frame = odom_factor->m_from_lf;  // from 是上一帧
  TransMatrixType Tpc = odom_factor->m_t_tf.inverse();
  initFrame(prev_frame, loopframe, Tpc);
  FactorGraph new_factors;
  LoopframeValue new_vals;  // [key,twc]
  if (les.size() != 0) {
    convertFactorAndValue(les, new_factors, new_vals);
    if (new_factors.size() >= 1 &&
        new_factors[0].m_type == EdgeType::LOOPCLOSURE) {
      SYLAR_LOG_DEBUG(g_logger_solver)
          << "PcmSolver process " << new_factors.size() << " factors.";
    }
    // 更新 values_ 数据 loopfram->new_vals->values
    MutexType::Lock lk(mutex_);
    outlier_removal_->classifyNewLoopframeToMap(new_vals, &values_, &nfg_);
    // 更新 pcm 中odom
    for (auto factor : les) {
      EdgeType type = factor->m_type;
      if (type == EdgeType::ODOMETRY) {
        auto from_client = factor->m_from_lf->m_client_id;
        auto from_id = factor->m_from_lf->m_lf_id;
        int_t client = from_client;
        // values size -> updateOdom
        outlier_removal_->updateOdom(from_client, *factor);
      } else {
        SYLAR_LOG_ERROR(g_logger_solver) << "type is not odom " << DumpKey(key);
        SYLAR_ASSERT(false)
      }
    }
  }
}
void PcmSolver::insertLoopEdgeAndUpdate(const LoopEdgeVector &les,
                                        bool is_optimize) {
  MutexType::Lock lk(mutex_);
  for (auto le : les) factors_buf_.push(le);
}
void PcmSolver::getAllValueToUpdateMap(LoopframeValue &vals) {
  // TODO 获取所有的优化结果
}
bool PcmSolver::checkIsOptimized(std::vector<cmd::LoopframeValue> &values,
                                 std::vector<FactorGraph> &fgs) {
  MutexType::Lock lk(mutex_);
  if (is_optimized_) {
    values = values_;
    fgs = nfg_;
    is_optimized_ = false;
    return true;
  }
  return false;
}

bool PcmSolver::removeOutlierAndOptimize(
    FactorGraph &factors, LoopframeValue &values,
    std::vector<bool> &need_optimize_idx,
    std::vector<cmd::Sim3LoopframeValue> &optimized_values) {
  bool do_optimize;

  {
    MutexType::Lock lk(mutex_);
    auto t0 = TimeCosters::GetNow();
    do_optimize = outlier_removal_->removeOutliers(factors, values, &nfg_,
                                                   &values_, last_client_value_,
                                                   &need_optimize_idx);
    auto t1 = TimeCosters::GetNow();
    GetPcmCosts().addCost(t0, t1);
  }

  // 唤醒优化
  if (do_optimize) {
    SYLAR_LOG_INFO(g_logger_solver) << "=== RPGO" << "\e[1;32m START \e[0m ===";
    optimize(need_optimize_idx, optimized_values);
    SYLAR_LOG_INFO(g_logger_solver) << "=== RPGO" << "\e[1;33m END \e[0m ===";
  }
  return do_optimize;
}
void PcmSolver::updateDataAfterOptimizeWithLock(
    const std::vector<bool> &need_optimize_idx,
    const std::vector<Sim3LoopframeValue> &full_sim3_values) {
  MutexType::Lock lk(mutex_);
  int len = need_optimize_idx.size();
  int same_value_nums = 0, diff_value_nums = 0;
  for (int i = 0; i < len; i++) {
    if (!need_optimize_idx[i]) {
      if (debug()) {
        SYLAR_LOG_DEBUG(g_logger_solver) << "map " << i << "without optimized ";
      }
      continue;
    }
    const auto &sim3_values = full_sim3_values[i];
    auto &values = values_[i];
    // 记录原有的帧
    std::unordered_map<int_t, std::pair<int_t, TransMatrixType>> record;
    for (const auto &sval : sim3_values) {
      auto key = sval.first;
      auto client = GetKeyClientID(key);
      auto id = GetKeyLoopframeID(key);

      // ceres 优化时候需要 tcw 优化结束之后转回 twc
      auto sim3_tcw = Sophus::Sim3d::exp(sval.second);
      auto twc =
          ToOrthogonalTrans(Sim3LoopframeValue::ToTwcSE3(sim3_tcw).matrix());

      // 记录id最大一帧的位姿
      if (record.find(client) == record.end() || record[client].first < id) {
        record[client] = {id, values[key]};  // 记录更新前的位姿
        last_client_value_[client] = {key, values[key]};
      }
      same_value_nums += (values[key].matrix() == twc.matrix());
      diff_value_nums += (values[key].matrix() != twc.matrix());
      values[key] = twc;
    }
    // 更新优化过程中新进来的帧
    for (auto &client_id_pose : record) {
      auto robot = client_id_pose.first;
      auto id = client_id_pose.second.first;
      auto original_id = id;
      auto old_t_w_prev = client_id_pose.second.second;
      int update_nums = 0;
      while (values.find(GetKey(robot, id + 1)) != values.end()) {
        id++;
        update_nums++;
        auto old_t_wc = values[GetKey(robot, id)];
        auto cur_t_w_prev = values[GetKey(robot, id - 1)];
        auto t_prev_c = old_t_w_prev.inverse() * old_t_wc;
        auto cur_t_wc = cur_t_w_prev * t_prev_c;
        values[GetKey(robot, id)] = cur_t_wc;
        old_t_w_prev = old_t_wc;  // 循环用作更新下一个
      }
      if (debug()) {
        SYLAR_LOG_DEBUG(g_logger_solver)
            << DumpKey(GetKey(robot, original_id)) << " update " << update_nums
            << " frames";
      }
    }
    is_optimized_ = true;
  }
}
void PcmSolver::ceresSim3Optimize(std::vector<FactorGraph> &full_fgs,
                                  std::vector<Sim3LoopframeValue> &full_values,
                                  const std::vector<bool> &need_optimize_idx) {
  int len = need_optimize_idx.size();
  for (int i = 0; i < len; i++) {
    if (!need_optimize_idx[i]) {
      continue;
    }
    auto &full_fg = full_fgs[i];
    auto &full_value = full_values[i];
    SYLAR_ASSERT2(full_fg.size() != 0 && full_value.size() != 0,
                  "factor 和 values 数值出现异常");

    ceres::Problem::Options problem_options;
    problem_options.enable_fast_removal = true;
    ceres::Problem problem(problem_options);

    auto fix_key = full_value.getFixKey();
    if (full_value.find(fix_key) == full_value.end()) {
      SYLAR_LOG_FATAL(g_logger_solver)
          << "fix key not found [client:" << GetKeyClientID(fix_key) << ","
          << GetKeyLoopframeID(fix_key) << "]";
      SYLAR_ASSERT(false);
    }
    auto fix_pose = full_value[fix_key];
    // 添加边
    ceres::LossFunctionWrapper *loss_function = new ceres::LossFunctionWrapper(
        new ceres::CauchyLoss(
            OPT_ROBUST_LOSS),  // CauchyLoss参数越小对于约束越严格
        ceres::
            TAKE_OWNERSHIP);  // 核函数
                              // TAKE_OWNERSHIP是否自动释放CauchyLoss等这些资源

    for (auto &factor : full_fg) {
      auto from = factor.m_from_lf;
      auto to = factor.m_to_lf;
      auto from_key = GetKey(from->m_client_id, from->m_lf_id);
      auto to_key = GetKey(to->m_client_id, to->m_lf_id);
      Sophus::Sim3d sim3_tf(factor.m_t_tf.matrix());
      ceres::CostFunction *cost_function =
          PoseGraphError::Create(sim3_tf, factor.m_info);
      // ??? 这里也是先后顺序的问题，在获取 full_value 的时候，没有和原来的一样
      // inverse
      problem.AddResidualBlock(cost_function, loss_function,
                               full_value[to_key].data(),
                               full_value[from_key].data());
    }
    // 添加所有的顶点
    for (auto &key_pose : full_value) {
      // problem.AddParameterBlock(key_pose.second.data(),
      //                           Sim3Parameterization::Size(),
      //                           new Sim3Parameterization());
      problem.SetParameterization(key_pose.second.data(),
                                  new Sim3Parameterization());
      if (key_pose.first == fix_key) {
        SYLAR_LOG_DEBUG(g_logger_solver)
            << "set fix frame: [client:" << GetKeyClientID(fix_key)
            << ",id:" << GetKeyLoopframeID(fix_key) << "]";
        problem.SetParameterBlockConstant(key_pose.second.data());
      }
    }
    // 迭代求解
    ceres::Solver::Options solver_options;
    solver_options.max_num_iterations = OPT_ITER;
    solver_options.minimizer_progress_to_stdout = false;  // 是否输出迭代信息
    solver_options.function_tolerance = 1e-16;            // 收敛的阈值
    solver_options.linear_solver_type = ceres::SPARSE_SCHUR;

    ceres::Solver::Summary summary;
    ceres::Solve(solver_options, &problem, &summary);
    if (debug()) {
      // SYLAR_LOG_DEBUG(g_logger_solver) << summary.FullReport();
    }
  }
}

void PGO(std::vector<FactorGraph> &full_fgs,
         std::vector<Sim3LoopframeValue> &full_values,
         const std::vector<bool> &need_optimize_idx) {}
/// @brief 执行优化主函数
void PcmSolver::optimize(std::vector<bool> &need_optimize_idx,
                         std::vector<Sim3LoopframeValue> &optimized_values) {
  std::vector<FactorGraph> full_fgs;
  {
    MutexType::Lock lk(mutex_);
    need_optimize_ = false;
    optimizing_ = true;

    int len = need_optimize_idx.size();
    full_fgs.resize(len);
    optimized_values.resize(len);
    for (int i = 0; i < len; i++) {
      if (need_optimize_idx[i]) {
        full_fgs[i] = nfg_[i];
        optimized_values[i].insertByLoopframeValue(values_[i]);
      }
    }
  }
  auto t0 = TimeCosters::GetNow();
  ceresSim3Optimize(full_fgs, optimized_values, need_optimize_idx);
  auto t1 = TimeCosters::GetNow();
  GetPgoCosts().addCost(t0, t1);
}
/// @brief 检查是否启动优化
/// @return
bool PcmSolver::checkNeedOptimize() {
  if (need_optimize_) {
    return true;
  }
  return false;
}
void PcmSolver::Stop() { solver_running_ = false; }
}  // namespace cmd