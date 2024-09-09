#include "pcm_solver.hpp"
#include "loopframe.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

namespace cmd
{
    static LoggerPtr g_logger_sys = SYLAR_LOG_NAME("PCM-PGO");

    SE3PcmSolver::SE3PcmSolver(const KimeraRPGO::RobustSolverParams &params)
        : KimeraRPGO::RobustSolver(params) {}

    /// @brief 将 loopedge 转化成 lf，并加入到 value 和 Factor 中
    /// @param le
    /// @return 返回是否优化 is_optimize
    void SE3PcmSolver::convertLoopEdge2Factor(LoopEdgeVector &les, gtsam::NonlinearFactorGraph &tmp_nfg, gtsam::Values &tmp_est)
    {
        // 参数噪声
        static const gtsam::SharedNoiseModel &noise =
            gtsam::noiseModel::Isotropic::Variance(6, 0.1);

        for (auto le : les)
        {
            LoopframePtr from = le->m_from_lf;
            LoopframePtr to = le->m_to_lf;

            if (le->m_type == EdgeType::REFERENCE)
            {
                // 注意不同 le 之间的转化，lc 和 ref！！！
                // 因为在pcm 中是根据id 来判断是否是 odom 的
                SYLAR_ASSERT2(to->m_lf_id - from->m_lf_id == 1, "只支持一个ref边");
            }
            unsigned char name[] = {'a','b','c','d','e'};

            gtsam::Key sym_from = gtsam::Symbol(name[from->m_client_id], from->m_lf_id);
            gtsam::Key sym_to = gtsam::Symbol(name[to->m_client_id], to->m_lf_id);

            gtsam::Pose3 from_pos = gtsam::Pose3(from->m_twc.matrix());
            gtsam::Pose3 to_pos = gtsam::Pose3(to->m_twc.matrix());

            tmp_est.insert(sym_from, from_pos);
            tmp_est.insert(sym_to, to_pos);
            // TODO 为什么需要先验噪声？ 设置优化边的可信度
            gtsam::Pose3 T_tf_pos = gtsam::Pose3(le->m_t_tf.matrix());
            tmp_nfg.add(gtsam::BetweenFactor<gtsam::Pose3>(sym_from, sym_to, T_tf_pos, noise));
        }
    }

    void SE3PcmSolver::updateByLoopEdge(LoopEdgeVector &les,bool is_optimize)
    {
        gtsam::NonlinearFactorGraph new_factors;
        gtsam::Values new_vals;
        convertLoopEdge2Factor(les,new_factors,new_vals);
        for(auto val : new_vals){
            gtsam::Pose3 optimized_pose = new_vals.at<gtsam::Pose3>(val.key);
            SYLAR_LOG_DEBUG(g_logger_sys) << val.key << " " << optimized_pose;
        }
        size_t size = new_factors.size();
        for(size_t i=0;i<size ;i++){
            SYLAR_LOG_DEBUG(g_logger_sys) << new_factors[i];
        }
        // 更新回环边,is_optimize判断是否启动更新
        update(new_factors,new_vals,is_optimize);
    }
    void SE3PcmSolver::getAllVals(gtsam::Values& vals){
        auto keys = nfg_.keys();
        for(auto key : keys){
            auto tmp_val =  values_.at<gtsam::Pose3>(key);
            vals.insert(key,tmp_val);
        }
        return ; 
    }

}