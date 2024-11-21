#include "optimization/pcm_pgo/pcm.hpp"
#include "optimization/pcm_pgo/utils/GraphUtils.h"
#include "optimization/pcm_pgo/utils/GeometryUtils.h"
#include "loopframe.hpp"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h> 

namespace cmd
{
    static mysylar::Logger::ptr g_logger = SYLAR_LOG_NAME("CMD_BACKEND");
    Pcm::Pcm(PcmParams params,
             MultiRobotAlignMethod align_method,
             double align_gnc_probability)
        : OutlierRemoval(),
          params_(params),
          total_lc_(0),
          total_good_lc_(0),
          multirobot_align_method_(align_method),
          multirobot_gnc_align_probability_(align_gnc_probability),
          odom_check_(true),
          loop_consistency_check_(true)
    {
    }

    bool Pcm::removeOutliers(const FactorGraph &new_factors,
                             const LoopframeValue &new_loopframes,
                             FactorGraph *output_nfg,
                             LoopframeValue *output_values)
    {
        // 1、保存所有的loopframe数据
        output_values->add(new_loopframes);

        if (new_factors.size() == 0)
        {
            return false;
        }
        bool do_optimize = false;
        FactorGraph loop_closure_factors;
        for (size_t i = 0; i < new_factors.size(); i++)
        {
            EdgeType type = new_factors[i].m_type;
            auto from_client = new_factors[i].m_from_lf->m_client_id;
            auto from_id = new_factors[i].m_from_lf->m_lf_id;
            auto to_client = new_factors[i].m_to_lf->m_client_id;
            auto to_id = new_factors[i].m_to_lf->m_lf_id;

            auto from_key = GetKey(from_client, from_id);
            auto to_key = GetKey(to_client, to_id);

            switch (type)
            {
            case EdgeType::ODOMETRY:
                updateOdom(new_factors[i], *output_values);
                break;
            case EdgeType::LOOPCLOSURE:
                // 后面处理
                loop_closure_factors.add(new_factors[i]);
                break;
            case EdgeType::UNCLASSIFIED:
                SYLAR_ASSERT2(false, "出现异常factor");
            default:
                break;
            }
        }
        // 处理回环帧
        if (loop_closure_factors.size() > 0)
        {
            std::unordered_map<ObservationId, size_t> num_new_loopclosures; // 用id记录新生成的回环边
            // 构造增量矩阵
            parseAndIncrementAdjMatrix(loop_closure_factors, *output_values, num_new_loopclosures);
            if (params_.incremental)
            {
                // 从增量里面判断是否有新的 consistent_factor 集
                findInliersIncremental(num_new_loopclosures);
            }
            else
            {
                // 判断所有 adjmatrix ，找新的 consistent_factor 集
                findInliers();
            }
            do_optimize = true;
        }
        *output_nfg = buildGraphToOptimize();
        if (multirobot_align_method_ != MultiRobotAlignMethod::NONE &&
            robot_order_.size() > 1)
        {
            *output_values = multirobotValueInitialization(*output_values);
        }
        if (debug_ && do_optimize)
            SYLAR_LOG_INFO(g_logger)
                << " milliseconds. Detected " << total_lc_
                << " total loop closures with " << total_good_lc_
                << " inliers.";
        return do_optimize;
    }

    FactorGraph Pcm::buildGraphToOptimize()
    {
        FactorGraph output_nfg;
        output_nfg.add(nfg_odom_);
        std::unordered_map<ObservationId, Measurements>::iterator it =
            loop_closures_.begin();
        while (it != loop_closures_.end())
        {
            if (std::find(ignored_prefixes_.begin(),
                          ignored_prefixes_.end(),
                          it->first.client_a) == ignored_prefixes_.end() &&
                std::find(ignored_prefixes_.begin(),
                          ignored_prefixes_.end(),
                          it->first.client_b) == ignored_prefixes_.end())
                output_nfg.add(it->second.consistent_factors);
            it++;
        }
        return output_nfg;
    }
    LoopframeValue Pcm::getRobotOdomValues(const int_t &client_id,
                                           const TransMatrixType &T_wb_wc)
    {
        LoopframeValue robot_values;
        for (const auto &key_pose : odom_trajectories_[client_id])
        {
            auto new_pose = T_wb_wc * key_pose.second;
            robot_values.insert({key_pose.first, new_pose});
        }
        return robot_values;
    }
    LoopframeValue Pcm::multirobotValueInitialization(LoopframeValue &input_value)
    {
        LoopframeValue initialized_values = input_value;
        if (robot_order_.size() == 0)
        {
            SYLAR_LOG_INFO(g_logger) << "No robot poses received. ";
            return initialized_values;
        }
        // Sort robot order from smallest prefix to larges
        std::sort(robot_order_.begin(), robot_order_.end());
        // 1、不优化第一个机器人的 value
        initialized_values.add(getRobotOdomValues(robot_order_[0]));

        // 2、Start estimating the frame-to-frame transforms between robots
        // ??？ 如果是在同一个 map 中，但是 a 和 0 之间没有 factor 应该怎么办？
        // 解决方法：小的向大的看齐，先是以 0 为基准进行调整，然后以 1 为基准调整没有调整的，以此类推，可以达到所有都向 0 看齐
        std::unordered_set<int_t> record_robot;
        record_robot.insert(0);
        for (int_t base_robot = 0; base_robot < robot_order_.size(); base_robot++)
        {
            // 只有已经位姿变化你的可以作为 base robot
            if (record_robot.find(base_robot) == record_robot.end())
            {
                continue;
            }
            for (int_t i = 0; i < robot_order_.size(); i++)
            {
                if (base_robot == i)
                {
                    continue;
                }
                const int_t &r_base = robot_order_[base_robot];
                const int_t &ri = robot_order_[i];
                ObservationId obs_id(r_base, ri);

                FactorGraph lc_factors =
                    loop_closures_.at(obs_id).consistent_factors;
                // 只有和 base robot 有联系且没有调整位姿的才可以调整位姿
                // ??? 是否可以拓展成和所有base 的平均 trans 呢？
                if (lc_factors.size() == 0 ||
                    record_robot.find(i) != record_robot.end())
                {
                    continue;
                }
                record_robot.insert(i);
                TransMatrixVector T_wb_wi_measured;
                for (auto &factor : lc_factors)
                {
                    auto from = factor.m_from_lf;
                    auto to = factor.m_to_lf;
                    auto T_tf = factor.m_t_tf;
                    // 保证 from 是 base
                    if (from->m_client_id != r_base)
                    {
                        auto tmp = from;
                        from = to;
                        to = tmp;
                        T_tf = T_tf.inverse();
                    }
                    auto from_key = GetKey(from->m_client_id, from->m_lf_id);
                    auto to_key = GetKey(to->m_client_id, to->m_lf_id);

                    auto T_wb_from = odom_trajectories_[r_base][from_key];
                    auto T_wi_to = odom_trajectories_[ri][to_key];

                    auto T_wb_wi = T_wb_from * T_tf.inverse() * T_wi_to.inverse();
                    T_wb_wi_measured.push_back(T_wb_wi);
                }
                TransMatrixType T_wb_wi_avg = gncRobustPoseAveraging(T_wb_wi_measured);
                initialized_values.add(getRobotOdomValues(robot_order_[i],T_wb_wi_avg));
            }
        }
        // 初始化 fix_key 
        auto fix_key = GetKey(robot_order_[0],0);
        initialized_values.setFixKey(fix_key);
        return initialized_values;
    }
    /// @brief 使用一元边的方式计算平均变换矩阵
    /// @param input_poses 
    /// @param rot_sigma 
    /// @param trans_sigma 
    /// @return 
    TransMatrixType Pcm::gncRobustPoseAveraging(const TransMatrixVector &input_poses,
                                           const double &rot_sigma = 0.1,
                                           const double &trans_sigma = 0.5)
    {
        gtsam::Values initial;
        initial.insert(0, gtsam::Pose3()); // identity pose as initialization

        gtsam::NonlinearFactorGraph graph;
        size_t dim = getDim<gtsam::Pose3>();
        size_t r_dim = getRotationDim<gtsam::Pose3>();
        size_t t_dim = getTranslationDim<gtsam::Pose3>();
        gtsam::Vector sigmas;
        sigmas.resize(dim);
        sigmas.head(r_dim).setConstant(rot_sigma);
        sigmas.tail(t_dim).setConstant(trans_sigma);
        const gtsam::noiseModel::Diagonal::shared_ptr noise =
            gtsam::noiseModel::Diagonal::Sigmas(sigmas);
        // add measurements
        for (auto pose : input_poses)
        {
            gtsam::Pose3 gtsam_pose(pose.matrix());
            graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam_pose, noise));
        }

        gtsam::GncParams<gtsam::LevenbergMarquardtParams> gncParams;
        auto gnc =
            gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>(
                graph, initial, gncParams);

        if (multirobot_align_method_ == MultiRobotAlignMethod::L2)
        {
            gnc.setInlierCostThresholds(std::numeric_limits<double>::max());
        }
        else if (multirobot_align_method_ == MultiRobotAlignMethod::GNC)
        {
            gnc.setInlierCostThresholdsAtProbability(
                multirobot_gnc_align_probability_);
        }
        else
        {
            SYLAR_LOG_WARN(g_logger)<<
                "Invalid multirobot alignment method in gncRobustPoseAveraging!";
        }

        gtsam::Values estimate = gnc.optimize();
        TransMatrixType result(estimate.at<gtsam::Pose3>(0).matrix());
        return result;
    }

    void Pcm::parseAndIncrementAdjMatrix(FactorGraph &factors,
                                         LoopframeValue &output_value,
                                         std::unordered_map<ObservationId, size_t> &num_new_loopclosure)
    {
        for (size_t i = 0; i < factors.size(); i++)
        {
            auto &new_factor = factors[i];
            auto from = new_factor.m_from_lf;
            auto to = new_factor.m_to_lf;

            auto from_key = GetKey(from->m_client_id, from->m_lf_id);
            auto to_key = GetKey(to->m_client_id, to->m_lf_id);

            //  相关帧找不到的不处理
            if (!output_value.exist(from_key) ||
                !output_value.exist(to_key))
            {
                continue;
            }

            bool is_valid = false;
            double dist;
            if (from->m_client_id == to->m_client_id)
            {
                // 检查新的同一robot回环是否与先验位姿具有一致性
                is_valid = isOdomConsistent(new_factor, dist);
            }
            else
            {
                is_valid = true;
            }

            if (is_valid)
            {
                ObservationId obs_id(GetKeyClientID(from_key), GetKeyClientID(to_key));
                if (num_new_loopclosure.find(obs_id) == num_new_loopclosure.end())
                {
                    num_new_loopclosure.insert({obs_id, 1});
                }
                else
                {
                    num_new_loopclosure[obs_id]++;
                }
                loop_closures_[obs_id].factors.add(new_factor);
                loop_closures_in_order_.push_back(obs_id);
                total_lc_++;
                // 更新adj矩阵
                incrementAdjMatrix(obs_id, new_factor);
            }
        }
    }

    void Pcm::findInliersIncremental(const std::unordered_map<ObservationId, size_t> &num_new_loopclosures)
    {
        for (const auto &num_lc : num_new_loopclosures)
        {
            ObservationId robot_pair = num_lc.first;
            std::vector<int> inliers_idx;
            total_good_lc_ = 0;
            size_t prev_maxclique_size =
                loop_closures_[robot_pair].consistent_factors.size(); // 原有回环边的数量
            // !!!找到最大的一致回环
            size_t num_inliers = findMaxCliqueHeuIncremental(loop_closures_[robot_pair].adj_matrix,
                                                             num_lc.second,
                                                             prev_maxclique_size,
                                                             &inliers_idx);
            //  3、提取对应的一致 factor 对
            if (num_inliers > 0)
            {
                loop_closures_[robot_pair].consistent_factors =
                    FactorGraph(); // reset
                for (size_t i = 0; i < num_inliers; i++)
                {
                    loop_closures_[robot_pair].consistent_factors.add(
                        loop_closures_[robot_pair].factors[inliers_idx[i]]);
                }
            }
            else
            {
                // Set of inliers not modified. Don't reset consistent_factors
                num_inliers = prev_maxclique_size;
            }
        }
        // 4、 更新总的回环边数目
        for (auto robot_pair_lc : loop_closures_)
        {
            total_good_lc_ =
                total_good_lc_ + robot_pair_lc.second.consistent_factors.size();
        }
    }

    void Pcm::findInliers()
    {
        total_good_lc_ = 0;
        // iterate through loop closures and find inliers
        std::unordered_map<ObservationId, Measurements>::iterator it =
            loop_closures_.begin();
        // 检查更新连续 factor
        while (it != loop_closures_.end())
        {
            size_t num_inliers;
            if (loop_consistency_check_)
            {
                std::vector<int> inliers_idx;
                it->second.consistent_factors = FactorGraph(); // reset
                // find max clique
                num_inliers = findMaxCliqueHeu(it->second.adj_matrix, &inliers_idx);
                // update inliers, or consistent factors, according to max clique result
                for (size_t i = 0; i < num_inliers; i++)
                {
                    it->second.consistent_factors.add(it->second.factors[inliers_idx[i]]);
                }
            }
            else
            {
                it->second.consistent_factors = it->second.factors;
                num_inliers = it->second.factors.size();
            }
            it++;
            total_good_lc_ = total_good_lc_ + num_inliers;
        }
    }

    /// 每插入一个lc factor 都会调用一次
    void Pcm::incrementAdjMatrix(const ObservationId &id, const LoopEdge &factor)
    {
        size_t num_lc = loop_closures_[id].factors.size();

        Matrix new_adj_matrix = Matrix::Zero(num_lc, num_lc);
        Matrix new_dst_matrix = Matrix::Zero(num_lc, num_lc);

        if (num_lc > 1)
        {
            new_adj_matrix.topLeftCorner(num_lc - 1, num_lc - 1) = loop_closures_[id].adj_matrix;
            new_dst_matrix.topLeftCorner(num_lc - 1, num_lc - 1) = loop_closures_[id].dist_matrix;
            for (size_t i = 0; i < num_lc - 1; i++)
            {
                double mah_distance = 0.0;
                const auto &factors = loop_closures_[id].factors;
                bool consistent = areLoopsConsistent(factor, factors[i], mah_distance);
                new_dst_matrix(num_lc - 1, i) = mah_distance;
                new_dst_matrix(i, num_lc - 1) = mah_distance;
                if (consistent)
                {
                    new_adj_matrix(num_lc - 1, i) = 1;
                    new_adj_matrix(i, num_lc - 1) = 1;
                }
            }
        }
        loop_closures_[id].adj_matrix = new_adj_matrix;
        loop_closures_[id].dist_matrix = new_dst_matrix;
    }
    bool Pcm::areLoopsConsistent(const LoopEdge &lc_a2b,
                                 const LoopEdge &lc_c2d,
                                 double &dist)
    {
        const auto a = lc_a2b.m_from_lf;
        auto a_key = GetKey(a->m_client_id, a->m_lf_id);

        const auto b = lc_a2b.m_to_lf;
        auto b_key = GetKey(b->m_client_id, b->m_lf_id);

        const auto c = lc_c2d.m_from_lf;
        auto c_key = GetKey(c->m_client_id, c->m_lf_id);

        const auto d = lc_c2d.m_to_lf;
        auto d_key = GetKey(d->m_client_id, d->m_lf_id);

        TransMatrixType t_ba, t_cd;
        t_ba = lc_a2b.m_t_tf;
        t_cd = lc_c2d.m_t_tf;

        // 确保ac是同一个robot，方便计算
        if (GetKeyClientID(a_key) != GetKeyClientID(c_key))
        {
            auto tmp = d_key;
            d_key = c_key;
            c_key = tmp;
            t_cd = t_cd.inverse();
        }
        SYLAR_ASSERT2(GetKeyClientID(a_key) == GetKeyClientID(c_key) && GetKeyClientID(b_key) == GetKeyClientID(d_key), "a,b,c,d 四个顶点位置不一致")

        auto robot_ac = GetKeyClientID(a_key);
        auto robot_bd = GetKeyClientID(b_key);
        TransMatrixType t_ac = odom_trajectories_[robot_ac][a_key].inverse() * odom_trajectories_[robot_ac][c_key];
        TransMatrixType t_db = odom_trajectories_[robot_bd][d_key].inverse() * odom_trajectories_[robot_bd][b_key];

        TransMatrixType result = t_ac * t_cd * t_db * t_ba;

        return checkLoopConsistent(result, dist);
    }
    bool Pcm::checkLoopConsistent(TransMatrixType &result, double &dist)
    {
        dist = result.translation().log().norm();
        double rot_dist = result.rotationMatrix().log().norm();
        if (dist < params_.dist_trans_threshold &&
            rot_dist < params_.dist_rot_threshold)
        {
            return true;
        }
        return false;
    }

    bool Pcm::isOdomConsistent(LoopEdge &factor, double &dist)
    {
        auto prev = factor.m_from_lf;
        auto cur = factor.m_to_lf;

        auto prev_key = GetKey(prev->m_client_id, prev->m_lf_id);
        auto cur_key = GetKey(cur->m_client_id, cur->m_lf_id);

        TransMatrixType T_cur_prev = factor.m_t_tf;

        if (prev->m_client_id != cur->m_client_id)
        {
            SYLAR_LOG_WARN(g_logger) << "Only check for odmetry consistency for intrarobot loop closures";
        }
        // TODO 应该是使用 solver里面的位姿还是全局的位姿呢？
        TransMatrixType result = prev->m_twc.inverse() * cur->m_twc * T_cur_prev;
        // 检查连续性是否在阈值里面
        return checkOdomConsistent(result, dist);
    }

    bool Pcm::checkOdomConsistent(TransMatrixType &trans, double &dist)
    {
        double rot_dist = trans.rotationMatrix().log().norm();
        dist = trans.translation().norm();

        if (dist < params_.odom_trans_threshold && rot_dist < params_.odom_rot_threshold)
        {
            return true;
        }
        return false;
    }

    void Pcm::updateOdom(const LoopEdge &factor, LoopframeValue &output_values)
    {
        nfg_odom_.add(factor);

        LoopframeKey prev_key = GetKey(factor.m_from_lf->m_client_id, factor.m_from_lf->m_lf_id);
        LoopframeKey cur_key = GetKey(factor.m_to_lf->m_client_id, factor.m_from_lf->m_lf_id);

        LoopframePtr prev = factor.m_from_lf;
        LoopframePtr cur = factor.m_to_lf;
        // 记录里程计位姿
        auto client = GetKeyClientID(prev_key);
        if (odom_trajectories_.find(client) == odom_trajectories_.end())
        {
            auto initial_pose = prev->m_twc;
            odom_trajectories_[client][prev_key] = initial_pose;
            robot_order_.push_back(client);
        }
        TransMatrixType prev_pose;
        if (odom_trajectories_[client].exist(prev_key))
        {
            prev_pose = odom_trajectories_[client][prev_key]; // twc
        }
        else
        {
            SYLAR_ASSERT2(false, "Attempted to add odom to non-existing key. ");
        }
        odom_trajectories_[client][cur_key] = prev_pose * factor.m_t_tf.inverse(); // twc * t_from_to
    }
}