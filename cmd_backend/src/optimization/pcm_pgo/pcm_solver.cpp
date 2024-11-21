#include "optimization/pcm_pgo/pcm_solver.hpp"
#include "loopframe.hpp"
#include "ceres/ceres.h"
#include "cmd_sim3.hpp"

namespace cmd
{
    static LoggerPtr g_logger_sys = SYLAR_LOG_NAME("PCM-PGO");

    PcmSolver::PcmSolver(const RobustSolverParams &params)
    {
        solver_running_ = true;
        optimizing_ = false;
        need_optimize_ = false;
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
    void PcmSolver::insertLoopEdgeAndUpdate(LoopEdgeVector &les, bool is_optimize)
    {
        //!!! 因为优化和outlier removal是两个不同的缓存位置，所以暂时不需要考虑这个优化时候的并行问题
        // 创建factorGraph
        FactorGraph new_factors;
        LoopframeValue new_vals; // [key,ref_pose]
        convertFactorAndValue(les, new_factors, new_vals);
        update(new_factors, new_vals, is_optimize);
    }
    void PcmSolver::getAllValueToUpdateMap(LoopframeValue &vals)
    {
        // TODO 获取所有的优化结果
    }

    void PcmSolver::update(FactorGraph &factors, LoopframeValue &values, bool optimize_graph)
    {
        bool do_optimize;
        if (outlier_removal_)
        {
            do_optimize =
                outlier_removal_->removeOutliers(factors, values, &nfg_, &values_);
        }
        else
        {
            do_optimize = addAndCheckIfOptimize(factors, values);
        };

        // 唤醒优化
        if (do_optimize && optimize_graph)
            callOptimize(); // optimize once after loading
    }
    void PcmSolver::callOptimize()
    {
        RWMutextType::WriteLock lk(mutex_);
        need_optimize_ = true;
    }
    /// @brief 不进行outlier处理，直接判断是否可以优化
    /// @param factors
    /// @param values
    bool PcmSolver::addAndCheckIfOptimize(FactorGraph &factors, LoopframeValue &values)
    {
        nfg_.add(factors);
        values_.add(values);
        // 判断是否优化
        if (nfg_.size() == 0 || values_.size() == 0)
        {
            return false;
        }

        // 判断是否具有回环边
        bool just_odom = true;
        for (auto &le : factors)
        {
            if (le.m_from_lf->m_client_id != le.m_to_lf->m_client_id)
            {
                just_odom = false;
                break;
            }
            auto from_key = le.m_from_lf->m_lf_id;
            auto to_key = le.m_to_lf->m_lf_id;
            if ((from_key - to_key) != 1)
            {
                just_odom = false;
                break;
            }
        }
        if (just_odom)
        {
            return false;
        }

        return false;
    }
    void PcmSolver::updateDataAfterOptimize(Sim3LoopframeValue& sim3_values)
    {   
        // 记录原有的帧
        std::unordered_map<int_t,std::pair<int_t,TransMatrixType>> record;
        for(auto& sval : sim3_values){
            auto key = sval.first;
            auto client = GetKeyClientID(key);
            auto id = GetKeyLoopframeID(key);

            EigenMatrix sim_matrix = Sophus::Sim3d::exp(sval.second).matrix();
            auto twc = ToOrthogonalTrans(sim_matrix);

            // 记录id最大一帧的位姿
            if(record.find(client) == record.end() ||
                record[client].first < id){
                record[client] = {id,values_[key]}; 
            }
            values_.insert({key,twc});
        }
        // 更新优化过程中新进来的帧
        for(auto& client_id_pose : record){
            auto robot = client_id_pose.first;
            auto id = client_id_pose.second.first;
            auto t_w_prev = client_id_pose.second.second;
            
            while(values_.find(GetKey(robot,id+1)) != values_.end())
            {   
                id++;
                auto old_t_wc = values_[GetKey(robot,id)];
                auto new_t_w_prev = values_[GetKey(robot,id-1)];
                auto t_prev_c = t_w_prev.inverse() * old_t_wc;
                auto new_t_wc = new_t_w_prev * t_prev_c;
                values_[GetKey(robot,id)] = new_t_wc;
                t_w_prev = old_t_wc; // 循环用作更新下一个
            }
        }
        is_update_ = true;
    }
    const LoopframeValue& PcmSolver::getValue(){
        return values_;
    }
    /// @brief 执行优化主函数
    void PcmSolver::optimize()
    {
        FactorGraph full_fg;
        Sim3LoopframeValue full_values;
        {
            RWMutextType::WriteLock lk(mutex_);
            need_optimize_ = false;
            optimizing_ = true;
            full_fg = nfg_;
            full_values.insertByLoopframeValue(values_);
        }
        ceres::Problem::Options problem_options;
        problem_options.enable_fast_removal = true;
        ceres::Problem problem(problem_options);

        ceres::LocalParameterization *local_pose_param = new Sim3Parameterization();

        auto fix_key = full_values.getFixKey();
        auto fix_pose = full_values[fix_key];

        // 添加所有的顶点
        for (auto &key_pose : full_values)
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
            problem.AddResidualBlock(cost_function, nullptr, full_values[to_key].data(), full_values[from_key].data());
        }
        // 迭代求解
        ceres::Solver::Options solver_options;
        solver_options.max_num_iterations = OPT_ITER;
        solver_options.minimizer_progress_to_stdout = false; // 是否输出迭代信息
        solver_options.function_tolerance = 1e-16;           // 收敛的阈值
        solver_options.linear_solver_type = ceres::SPARSE_SCHUR;

        ceres::Solver::Summary summary;
        ceres::Solve(solver_options, &problem, &summary);
        // 结束位置
        {
            RWMutextType::WriteLock lk(mutex_);
            // 更新数据
            updateDataAfterOptimize(full_values);
            optimizing_ = false;
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
            if (checkNeedOptimize())
            {
                optimize();
            }
            usleep(1000);
        }
    }
    void PcmSolver::Stop()
    {
        solver_running_ = false;
    }

}