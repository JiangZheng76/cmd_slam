#include "optimization.hpp"
#include "map.hpp"

#include "ceres/ceres.h"
#include "cmd_sim3.hpp"

#include "loopframe.hpp"

namespace cmd
{
    static LoggerPtr g_logger_sys = SYLAR_LOG_NAME("system");

    MapOptimizationWrap::MapOptimizationWrap(MapPtr map)
        : m_map(map)
    {
        map->setOptimizingMode();
    }
    MapOptimizationWrap::~MapOptimizationWrap()
    {
        m_map->setOptimizedMode();
    }

    void Optimization::Sim3PoseGraphOptimization(MapPtr map)
    {
        // 更改 map下面的 frammanager 优化状态
        MapOptimizationWrap map_wrap(map);
        SYLAR_LOG_INFO(g_logger_sys) << "<-- PGO START ";
        // 创建问题
        ceres::Problem::Options problem_options;
        problem_options.enable_fast_removal = true;
        ceres::Problem problem(problem_options);

        // 参数类型
        ceres::LocalParameterization *local_pose_param = new Sim3Parameterization();

        std::vector<LoopEdgePtr> loopedges = map->getAllLoopEdge();
        std::vector<LoopframePtr> loopframes = map->getAllLoopframe();
        LoopframePtr fix_frame = map->m_fix_lf;

        // 添加所有顶点
        size_t lf_len = loopframes.size();
        for (size_t i = 0; i < lf_len; i++)
        {
            LoopframePtr lf = loopframes[i];
            Sophus::Sim3d sim3(lf->m_twc.matrix());
            auto sim3_vec7 = sim3.log();
            lf->m_ceres_pose[0] = sim3_vec7[0];
            lf->m_ceres_pose[1] = sim3_vec7[1];
            lf->m_ceres_pose[2] = sim3_vec7[2];
            lf->m_ceres_pose[3] = sim3_vec7[3];
            lf->m_ceres_pose[4] = sim3_vec7[4];
            lf->m_ceres_pose[5] = sim3_vec7[5];
            lf->m_ceres_pose[6] = sim3_vec7[6];

            problem.AddParameterBlock(lf->m_ceres_pose, Sim3Parameterization::Size(), local_pose_param);
            if (lf == fix_frame)
            {
                problem.SetParameterBlockConstant(lf->m_ceres_pose);
            }
        }

        // 添加边
        ceres::LossFunctionWrapper *loss_function = new ceres::LossFunctionWrapper(
            new ceres::CauchyLoss(OPT_ROBUST_LOSS), ceres::TAKE_OWNERSHIP); // 核函数
        size_t le_len = loopedges.size();
        for (size_t i = 0; i < le_len; i++)
        {
            LoopEdgePtr le = loopedges[i];
            LoopframePtr from = le->m_from_lf;
            LoopframePtr to = le->m_to_lf;
            Sophus::Sim3d sim3_tf(le->m_t_tf.matrix());
            // TODO 对于不同类型的边优化信息矩阵可以优化一下
            ceres::CostFunction *cost_function = PoseGraphError::Create(sim3_tf, le->m_info);
            problem.AddResidualBlock(cost_function, loss_function, to->m_ceres_pose, from->m_ceres_pose);
        }

        // 迭代求解
        ceres::Solver::Options solver_options;
        solver_options.max_num_iterations = OPT_ITER;
        solver_options.minimizer_progress_to_stdout = false; // 是否输出迭代信息
        solver_options.function_tolerance = 1e-16;           // 收敛的阈值
        solver_options.linear_solver_type = ceres::SPARSE_SCHUR;

        ceres::Solver::Summary summary;
        ceres::Solve(solver_options, &problem, &summary);

        // 更新数据
        map->updateMapAfterOptimize();

        SYLAR_LOG_INFO(g_logger_sys) << "--> PGO END ";
    }

}