#include "optimization/pcm_pgo/pcm_solver.hpp"
#include "loopframe.hpp"
#include "ceres/ceres.h"
#include "optimization/cmd_sim3.hpp"
#include "map.hpp"

namespace cmd
{
    static LoggerPtr g_logger_sys = SYLAR_LOG_NAME("PCM-PGO");

    PcmSolver::PcmSolver(const RobustSolverParams &params)
    {
        solver_running_ = true;
        optimizing_ = false;
        need_optimize_ = false;
        is_optimized_ = false;
        outlier_removal_ = std::make_unique<Pcm>(params);
        // 启动优化线程
        Thread(std::bind(Run,this),"pcm_solver_thread");
    }
    PcmSolver::~PcmSolver()
    {
        Stop();
    }
    void PcmSolver::convertFactorAndValue(const LoopEdgeVector &les, FactorGraph &factors, LoopframeValue &values)
    {
        factors.reserve(les.size());
        for (auto ptr : les)
        {
            factors.push_back(*ptr);
        }
        for (auto fac : factors)
        {
            auto from = fac.m_from_lf;
            auto to = fac.m_to_lf;

            auto from_key = GetKey(from->m_client_id, from->m_lf_id);
            auto to_key = GetKey(to->m_client_id, to->m_lf_id);

            // 保持和外面 framemanager 保持一致
            // ??? 一次错误的优化，会不会导致整个崩掉，从而导致后面正确的优化也会失败呢？（不会，pcm工具里面的 odom 不会更新）
            values.insert({from_key, from->m_twc});
            values.insert({to_key, to->m_twc});
        }
    }
    void PcmSolver::insertLoopEdgeAndUpdate(const LoopEdgeVector &les, bool is_optimize)
    {
        RWMutexType::WriteLock lk(mutex_);
        for (auto le : les)
            factors_buf_.push(le);
    }
    void PcmSolver::getAllValueToUpdateMap(LoopframeValue &vals)
    {
        // TODO 获取所有的优化结果
    }
    bool PcmSolver::checkIsOptimized(std::vector<LoopframeValue>& values){
        RWMutexType::ReadLock lk(mutex_);
        if(is_optimized_){
            values = values_;
            is_optimized_ = false;
            return true;
        }
        return false;
        
    }

    void PcmSolver::update(FactorGraph &factors, LoopframeValue &values, bool optimize_graph)
    {
        bool do_optimize;
        std::vector<bool> need_optimize_idx;
        {
            RWMutexType::WriteLock lk(mutex_);
            do_optimize =
                outlier_removal_->removeOutliers(factors, values, &nfg_, &values_, &need_optimize_idx);
        }

        // 唤醒优化
        if (do_optimize && optimize_graph){
            SYLAR_LOG_INFO(g_logger_sys) << "--> RPGO START ";
            optimize(need_optimize_idx);
            // 更新 view

            SYLAR_LOG_INFO(g_logger_sys) << "<-- RPGO END ";
        }
    }
    void PcmSolver::updateDataAfterOptimize(std::vector<bool>& need_optimize_idx,std::vector<Sim3LoopframeValue>& full_sim3_values)
    {
        int len = need_optimize_idx.size();
        for(int i=0;i<len;i++){
            if(!need_optimize_idx[i]){
                continue;
            }
            auto& sim3_values = full_sim3_values[i];
            auto& values = values_[i];
            // 记录原有的帧
            std::unordered_map<int_t, std::pair<int_t, TransMatrixType>> record;
            for (auto &sval : sim3_values)
            {
                auto key = sval.first;
                auto client = GetKeyClientID(key);
                auto id = GetKeyLoopframeID(key);

                EigenMatrix sim_matrix = Sophus::Sim3d::exp(sval.second).matrix();
                auto twc = ToOrthogonalTrans(sim_matrix);

                // 记录id最大一帧的位姿
                if (record.find(client) == record.end() ||
                    record[client].first < id)
                {
                    record[client] = {id, values[key]};
                }
                values.insert({key, twc});
            }
            // 更新优化过程中新进来的帧
            for (auto &client_id_pose : record)
            {
                auto robot = client_id_pose.first;
                auto id = client_id_pose.second.first;
                auto t_w_prev = client_id_pose.second.second;

                while (values.find(GetKey(robot, id + 1)) != values.end())
                {
                    id++;
                    auto old_t_wc = values[GetKey(robot, id)];
                    auto new_t_w_prev = values[GetKey(robot, id - 1)];
                    auto t_prev_c = t_w_prev.inverse() * old_t_wc;
                    auto new_t_wc = new_t_w_prev * t_prev_c;
                    values[GetKey(robot, id)] = new_t_wc;
                    t_w_prev = old_t_wc; // 循环用作更新下一个
                }
            }
        }
    }
    /// @brief 执行优化主函数
    void PcmSolver::optimize(std::vector<bool>& need_optimize_idx)
    {
        std::vector<FactorGraph> full_fgs;
        std::vector<Sim3LoopframeValue> full_values;
        {
            RWMutexType::WriteLock lk(mutex_);
            need_optimize_ = false;
            optimizing_ = true;

            int len = need_optimize_idx.size();
            full_fgs.resize(len);
            full_values.resize(len);
            for(int i=0;i<len;i++){
                if(need_optimize_idx[i]){
                    full_fgs[i] = nfg_[i];
                    full_values[i].insertByLoopframeValue(values_[i]);
                }
            }
        }
        int len = need_optimize_idx.size();
        for (int i = 0; i < len; i++)
        {
            if(!need_optimize_idx[i]){
                continue;
            }
            auto &full_fg = full_fgs[i];
            auto &full_value = full_values[i];
            SYLAR_ASSERT2(full_fg.size() != 0 && full_value.size() != 0,"factor 和 values 数值出现异常");

            ceres::Problem::Options problem_options;
            problem_options.enable_fast_removal = true;
            ceres::Problem problem(problem_options);

            ceres::LocalParameterization *local_pose_param = new Sim3Parameterization();

            auto fix_key = full_value.getFixKey();
            auto fix_pose = full_value[fix_key];

            // 添加所有的顶点
            for (auto &key_pose : full_value)
            {
                problem.AddParameterBlock(key_pose.second.data(), Sim3Parameterization::Size(), local_pose_param);
                if (key_pose.first == fix_key)
                {
                    SYLAR_LOG_DEBUG(g_logger_sys) << "set fix frame: ["
                                                  << GetKeyClientID(fix_key) << ","
                                                  << GetKeyLoopframeID(fix_key) << "]";
                    problem.SetParameterBlockConstant(key_pose.second.data());
                }
            }
            // 添加边
            ceres::LossFunctionWrapper *loss_function = new ceres::LossFunctionWrapper(
                new ceres::CauchyLoss(OPT_ROBUST_LOSS), ceres::TAKE_OWNERSHIP); // 核函数

            for (auto &factor : full_fg)
            {
                auto from = factor.m_from_lf;
                auto to = factor.m_to_lf;
                auto from_key = GetKey(from->m_client_id, from->m_lf_id);
                auto to_key = GetKey(to->m_client_id, to->m_lf_id);
                Sophus::Sim3d sim3_tf(factor.m_t_tf.matrix());
                ceres::CostFunction *cost_function = PoseGraphError::Create(sim3_tf, factor.m_info);
                // ??? 这里也是先后顺序的问题，在获取 full_value 的时候，没有和原来的一样 inverse
                problem.AddResidualBlock(cost_function, nullptr, full_value[to_key].data(), full_value[from_key].data());
            }
            // 迭代求解
            ceres::Solver::Options solver_options;
            solver_options.max_num_iterations = OPT_ITER;
            solver_options.minimizer_progress_to_stdout = false; // 是否输出迭代信息
            solver_options.function_tolerance = 1e-16;           // 收敛的阈值
            solver_options.linear_solver_type = ceres::SPARSE_SCHUR;

            ceres::Solver::Summary summary;
            ceres::Solve(solver_options, &problem, &summary);
        }
        {
            RWMutexType::WriteLock lk(mutex_);
            // 更新数据
            updateDataAfterOptimize(need_optimize_idx,full_values);
            is_optimized_ = true;
        }
    }
    /// @brief 检查是否启动优化
    /// @return
    bool PcmSolver::checkNeedOptimize()
    {
        if (need_optimize_)
        {
            return true;
        }
        return false;
    }
    /// @brief 优化线程
    void PcmSolver::Run()
    {
        while (solver_running_)
        {
            FactorGraph new_factors;
            LoopframeValue new_vals; // [key,ref_pose]
            LoopEdgeVector les;
            {
                RWMutexType::WriteLock lk(mutex_);
                les.reserve(factors_buf_.size());
                while (!factors_buf_.empty())
                {
                    auto &le = factors_buf_.front();
                    factors_buf_.pop();
                    les.push_back(le);
                }
            }
            convertFactorAndValue(les, new_factors, new_vals);
            update(new_factors, new_vals);
            usleep(500);
        }
        SYLAR_LOG_INFO(g_logger_sys) << "pcm solver optimize thread is end.";
    }
    void PcmSolver::Stop()
    {
        solver_running_ = false;
    }
}