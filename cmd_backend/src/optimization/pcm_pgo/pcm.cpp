#include "optimization/pcm_pgo/pcm.hpp"
#define SLOW_BUT_CORRECT_BETWEENFACTOR
#include <ceres/ceres.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "config.hpp"
#include "loopframe.hpp"
#include "optimization/pcm_pgo/utils/GeometryUtils.h"
#include "optimization/pcm_pgo/utils/GraphUtils.h"
// #include "optimization/pose_average.hpp"

namespace cmd {
static mysylar::Logger::ptr g_logger = SYLAR_LOG_NAME("CMD_BACKEND");
Pcm::Pcm(PcmParams params, MultiRobotAlignMethod align_method,
         double align_gnc_probability)
    : OutlierRemoval(),
      params_(params),
      total_lc_(0),
      total_good_lc_(0),
      multirobot_align_method_(align_method),
      multirobot_gnc_align_probability_(align_gnc_probability),
      odom_check_(true),
      loop_consistency_check_(true) {}
int findAndInsertClient(std::vector<ClientSet> &map_client, int_t client) {
  if (client == 0 || client == 32) {
    SYLAR_LOG_ERROR(g_logger) << "invaild client num.";
    SYLAR_ASSERT(false);
  }
  int len = map_client.size();
  for (int i = 0; i < len; i++) {
    auto &client_set = map_client[i];
    if (client_set.find(client) != client_set.end()) {
      return i;
    }
  }
  map_client.push_back(ClientSet());
  map_client.back().insert(client);
  return len;
}
/// 分类到 output_value 中
void Pcm::classifyNewLoopframeToMap(
    const LoopframeValue &new_loopframes,
    std::unordered_map<int_t, std::pair<LoopframeKey, TransMatrixType>>
        &last_client_key_pose,
    std::vector<LoopframeValue> *output_values,
    std::vector<FactorGraph> *output_nfg) {
  if (!(map_clients_.size() == output_values->size() &&
        map_clients_.size() == output_nfg->size()) &&
      map_clients_.size() == nfg_odom_.size()) {
    SYLAR_LOG_ERROR(g_logger)
        << "map 的四个成员长度不一致"
        << "[map_client:" << map_clients_.size() << "]"
        << "[output_values:" << output_values->size() << "]"
        << "[output_nfg:" << output_nfg->size() << "]"
        << "[nfg_odom:" << nfg_odom_.size() << "]";
    SYLAR_ASSERT(false);
  }
  // 获取所有的 client
  std::set<int> clients;
  std::unordered_map<int, std::vector<std::pair<int, TransMatrixType>>>
      new_loopframes_ordered;
  for (const auto &[key, pose] : new_loopframes) {
    auto client = GetKeyClientID(key);
    auto id = GetKeyLoopframeID(key);
    clients.insert(client);
    new_loopframes_ordered[client].push_back({id, pose});
  }
  // id 升序
  for (auto &[client, tmp] : new_loopframes_ordered) {
    sort(tmp.begin(), tmp.end(),
         [](const auto &a, const auto &b) { return a.first < b.first; });
  }
  for (auto client : clients) {
    findAndInsertClient(map_clients_, client);  // 返回client所在的 id
  }

  // 判断是否有扩充了 map 的数量
  // 扩充
  if (map_clients_.size() > output_values->size()) {
    int target_size = map_clients_.size();
    output_values->resize(target_size);
    output_nfg->resize(target_size);
    nfg_odom_.resize(target_size);
  }

  for (auto &[client, id_pose] : new_loopframes_ordered) {
    for (auto &[id, pose] : id_pose) {
      // 插入到对应的 values 中
      // 创建 map_clients_成员
      int map_id = findAndInsertClient(map_clients_, client);
      auto key = GetKey(client, id);
      if ((*output_values)[map_id].find(key) !=
          (*output_values)[map_id].end()) {
        continue;
      }
      if (last_client_key_pose.find(client) != last_client_key_pose.end()) {
        auto twp = last_client_key_pose[client].second;  // twp
        auto prev_key = last_client_key_pose[client].first;
        auto prev_id = GetKeyLoopframeID(prev_key);
        auto tpc = pose.inverse();
        if (prev_id + 1 != id) {
          SYLAR_LOG_ERROR(g_logger)
              << "id error prev_id:" << prev_id << ", id:" << id;
          SYLAR_ASSERT(prev_id + 1 == id);
        }
        auto twc = twp * tpc;  // twp * tpc
        last_client_key_pose[client] = {key, twc};
        (*output_values)[map_id][key] = twc;
      } else {
        last_client_key_pose[client] = {key, pose};  // first frame twc
        (*output_values)[map_id].insert({key, pose});
      }
    }
  }
}
void Pcm::mergeCheckAndPreform(
    std::unordered_map<ObservationId, size_t> &new_num_loopclosures,
    std::vector<FactorGraph> &output_nfg,
    std::vector<LoopframeValue> &output_values) {
  if (!(map_clients_.size() == output_values.size() &&
        map_clients_.size() == output_nfg.size()) &&
      map_clients_.size() == nfg_odom_.size()) {
    SYLAR_LOG_ERROR(g_logger)
        << "map 的四个成员长度不一致"
        << "[map_client:" << map_clients_.size() << "]"
        << "[output_values:" << output_values.size() << "]"
        << "[output_nfg:" << output_nfg.size() << "]"
        << "[nfg_odom:" << nfg_odom_.size() << "]";
    SYLAR_ASSERT(false);
  }
  // 检查是否有新的约束成立
  for (auto [obs_id, num] : new_num_loopclosures) {
    auto map_a = findAndInsertClient(map_clients_, obs_id.client_a);
    auto map_b = findAndInsertClient(map_clients_, obs_id.client_b);

    if (map_a != map_b) {
      // a是在前面的,大 map 的向小 map 合并
      if (map_a > map_b) std::swap(map_a, map_b);

      auto &a_value = output_values[map_a];
      auto &a_nfg = output_nfg[map_a];
      auto &a_nfg_odom = nfg_odom_[map_a];
      auto &a_map_clients = map_clients_[map_a];

      auto &b_value = output_values[map_b];
      auto &b_nfg = output_nfg[map_b];
      auto &b_nfg_odom = nfg_odom_[map_b];
      auto &b_map_clients = map_clients_[map_b];

      // 合并并删掉
      a_value.add(b_value);
      a_nfg.add(b_nfg);
      a_nfg_odom.add(b_nfg_odom);
      for (auto client : b_map_clients) {
        a_map_clients.insert(client);
      }

      // 删除掉 b
      auto it_output_values = output_values.begin();
      auto it_output_nfg = output_nfg.begin();
      auto it_nfg_odom_ = nfg_odom_.begin();
      auto it_map_clients_ = map_clients_.begin();
      int i = 0;
      while (i < map_b) {
        it_output_values++;
        it_output_nfg++;
        it_nfg_odom_++;
        it_map_clients_++;
        i++;
      }
      output_values.erase(it_output_values);
      output_nfg.erase(it_output_nfg);
      nfg_odom_.erase(it_nfg_odom_);
      map_clients_.erase(it_map_clients_);
      if (debug()) {
        std::stringstream ss;
        for (int i = 0; i < map_clients_.size(); i++) {
          ss << " map:" << i << "[ ";
          for (auto &client : map_clients_[i]) {
            ss << client << " ";
          }
          ss << "]";
        }
        SYLAR_LOG_DEBUG(g_logger)
            << "merge preform: map[" << map_a << "<-" << map_b << "]"
            << "after merge size is " << output_values.size() << ","
            << "map_client info => " << ss.str();
      }
    }
  }
}
/// @brief
/// @param new_factors
/// @param new_loopframes [key,tcf]
/// @param output_nfg
/// @param output_values
/// @param last_client_key_pose
/// @param need_optimized_map
/// @return
bool Pcm::removeOutliers(
    const FactorGraph &new_factors, const LoopframeValue &new_loopframes,
    std::vector<FactorGraph> *output_nfg,
    std::vector<LoopframeValue> *output_values,
    std::unordered_map<int_t, std::pair<LoopframeKey, TransMatrixType>>
        &last_client_key_pose,
    std::vector<bool> *need_optimized_map) {
  // 1、将新帧插分类到不同的位置中
  classifyNewLoopframeToMap(new_loopframes, last_client_key_pose, output_values,
                            output_nfg);

  if (new_factors.size() == 0) {
    return false;
  }
  bool do_optimize = false;
  FactorGraph loop_closure_factors;
  for (size_t i = 0; i < new_factors.size(); i++) {
    EdgeType type = new_factors[i].m_type;
    auto from_client = new_factors[i].m_from_lf->m_client_id;
    auto from_id = new_factors[i].m_from_lf->m_lf_id;
    auto to_client = new_factors[i].m_to_lf->m_client_id;
    auto to_id = new_factors[i].m_to_lf->m_lf_id;

    auto from_key = GetKey(from_client, from_id);
    auto to_key = GetKey(to_client, to_id);

    switch (type) {
      case EdgeType::ODOMETRY: {
        int_t client = from_client;
        int map_id = findAndInsertClient(map_clients_, client);
        updateOdom(map_id, new_factors[i], *output_values);
      } break;
      case EdgeType::LOOPCLOSURE:
        // 后面处理
        loop_closure_factors.add(new_factors[i]);
        break;
      case EdgeType::UNCLASSIFIED:
        SYLAR_ASSERT2(false, "出现异常factor");
        break;
      default:
        break;
    }
  }
  // 处理回环帧
  if (loop_closure_factors.size() > 0) {
    std::unordered_map<ObservationId, size_t>
        num_new_loopclosures;  // 用id记录新生成的回环边
    // 构造增量矩阵
    parseAndIncrementAdjMatrix(loop_closure_factors, *output_values,
                               num_new_loopclosures);
    if (params_.incremental) {
      // 从增量里面判断是否有新的 consistent_factor 集
      findInliersIncremental(num_new_loopclosures);
    } else {
      // 判断所有 adjmatrix ，找新的 consistent_factor 集
      findInliers();
    }
    // 检查合并处理
    mergeCheckAndPreform(num_new_loopclosures, *output_nfg, *output_values);
    extractNeedOptimizeMap(num_new_loopclosures,
                           *need_optimized_map);  // 提取需要优化的 mapid
    do_optimize = true;
  }
  if (robot_order_.size() > 1) {
    // 给新合并的 robot 位姿进行初始化处理
    multirobotValueInitialization(*output_values, *output_values);
  }
  buildGraphToOptimize(*output_nfg);
  if (debug() && do_optimize)
    SYLAR_LOG_INFO(g_logger)
        << "Detected " << total_lc_ << " total loop closures with "
        << total_good_lc_ << " inliers.";
  return do_optimize;
}
void Pcm::extractNeedOptimizeMap(
    std::unordered_map<ObservationId, size_t> &num_new_loopclosures,
    std::vector<bool> &output_client) {
  int len = map_clients_.size();
  output_client = std::vector<bool>(len, false);
  for (const auto &[obs_id, num] : num_new_loopclosures) {
    auto robot_a = obs_id.client_a;
    auto robot_b = obs_id.client_b;

    int a_map = findAndInsertClient(map_clients_, robot_a);
    int b_map = findAndInsertClient(map_clients_, robot_b);

    SYLAR_ASSERT2(a_map == b_map,
                  "extractNeedOptimizeMap() a_map与 b_map 不相等");
    output_client[a_map] = true;
  }
}

void Pcm::buildGraphToOptimize(std::vector<FactorGraph> &output_nfg) {
  int robot_num = nfg_odom_.size();
  output_nfg.resize(robot_num);
  for (int i = 0; i < robot_num; i++) {
    output_nfg[i] = nfg_odom_[i];
  }
  std::unordered_map<ObservationId, Measurements>::iterator it =
      loop_closures_.begin();
  while (it != loop_closures_.end()) {
    auto client_a = (*it).first.client_a;
    auto client_b = (*it).first.client_b;
    auto map_a = findAndInsertClient(map_clients_, client_a);
    auto map_b = findAndInsertClient(map_clients_, client_b);
    SYLAR_ASSERT2(map_a == map_b, "两个 robot 不在同一个 map 中，不合法!");
    output_nfg[map_a].add(it->second.consistent_factors);
    it++;
  }
}
LoopframeValue Pcm::getRobotOdomValues(const int_t &client_id,
                                       const TransMatrixType &T_wb_wc) {
  LoopframeValue robot_values;
  for (const auto &key_pose : odom_trajectories_[client_id]) {
    auto new_pose = T_wb_wc * key_pose.second;  // T_wb_wc * T_wc_c = T_wb_c
    // !!! unordered_map insert 如果存在不会改变原有的，而是返回原来的成员迭代器
    // robot_values.insert({key_pose.first, new_pose});
    robot_values[key_pose.first] = new_pose;
  }
  return robot_values;
}
/// 纠正两帧之间的位姿并且更新 fix_key
std::vector<LoopframeValue> Pcm::multirobotValueInitialization(
    std::vector<LoopframeValue> &input_value,
    std::vector<LoopframeValue> &output_value) {
  int len = input_value.size();
  std::vector<LoopframeValue> result;
  for (int i = 0; i < len; i++) {
    LoopframeValue initialized_values = input_value[i];
    auto clients = map_clients_[i];
    if (clients.size() == 0) {
      SYLAR_LOG_WARN(g_logger) << "Map:" << i << ",No robot poses received. ";
      result.push_back(initialized_values);
      continue;
    }
    auto first_robot = *(clients.begin());
    // 1、不优化第一个机器人的 value
    initialized_values.add(getRobotOdomValues(first_robot));

    // 2、Start estimating the frame-to-frame transforms between robots
    // ??？ 如果是在同一个 map 中，但是 a 和 0 之间没有 factor 应该怎么办？
    // 解决方法：小的向大的看齐，先是以 0 为基准进行调整，然后以 1
    // 为基准调整没有调整的，以此类推，可以达到所有都向 0 看齐
    std::unordered_set<int_t> record_robot;
    std::unordered_map<int_t, TransMatrixType>
        client_Tfc;  // 到 first 的变换矩阵，叠加变换

    record_robot.insert(first_robot);
    for (auto base_robot_it = clients.begin(); base_robot_it != clients.end();
         base_robot_it++) {
      auto base_robot = *base_robot_it;
      // 只有已经位姿变化你的可以作为 base robot
      if (record_robot.find(base_robot) == record_robot.end()) {
        continue;
      }
      for (auto i_robot_it = clients.begin(); i_robot_it != clients.end();
           i_robot_it++) {
        auto client_i = *i_robot_it;
        if (base_robot == client_i) {
          continue;
        }
        const int_t &r_base = base_robot;
        const int_t &ri = client_i;
        ObservationId obs_id(r_base, ri);

        // 只有和 base robot 有联系且没有调整位姿的才可以调整位姿
        // ??? 是否可以拓展成和所有base 的平均 trans 呢？
        if (loop_closures_.find(obs_id) == loop_closures_.end() ||
            record_robot.find(client_i) != record_robot.end()) {
          continue;
        }
        FactorGraph lc_factors = loop_closures_.at(obs_id).consistent_factors;
        record_robot.insert(client_i);
        TransMatrixVector T_wb_wi_measured;
        for (auto &factor : lc_factors) {
          auto from = factor.m_from_lf;
          auto to = factor.m_to_lf;
          auto T_tf = factor.m_t_tf;
          // 保证 from 是 base
          if (from->m_client_id != r_base) {
            auto tmp = from;
            from = to;
            to = tmp;
            T_tf = T_tf.inverse();
          }
          SYLAR_ASSERT2(from->m_client_id == r_base, "from 不是 base!");
          auto from_key = GetKey(from->m_client_id, from->m_lf_id);
          auto to_key = GetKey(to->m_client_id, to->m_lf_id);

          auto T_wb_from = odom_trajectories_[r_base][from_key];
          auto T_wi_to = odom_trajectories_[ri][to_key];

          auto T_wb_wi = T_wb_from * T_tf.inverse() *
                         T_wi_to.inverse();  // Tbf * Tft * Tti = Tbi
          T_wb_wi_measured.push_back(T_wb_wi);
        }
        TransMatrixType T_wb_wi_avg = gncRobustPoseAveraging(T_wb_wi_measured);
        client_Tfc[client_i] =
            client_Tfc[base_robot] * T_wb_wi_avg;  // 转换到 first 的坐标下
        initialized_values.add(
            getRobotOdomValues(client_i, client_Tfc[client_i]));
      }
    }
    // 初始化 fix_key
    auto fix_key = GetKey(*(clients.begin()), 0);
    SYLAR_ASSERT(GetKeyClientID(fix_key) != 0);
    initialized_values.setFixKey(fix_key);
    result.push_back(std::move(initialized_values));
  }
  output_value = std::move(result);
  return output_value;
}
/// @brief 使用一元边的方式计算平均变换矩阵
/// @param input_poses
/// @param rot_sigma
/// @param trans_sigma
/// @return
TransMatrixType Pcm::gncRobustPoseAveraging(
    const TransMatrixVector &input_poses, const double &rot_sigma,
    const double &trans_sigma) {
  gtsam::Values initial;
  initial.insert(0, gtsam::Pose3());  // identity pose as initialization

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
  for (auto pose : input_poses) {
    gtsam::Pose3 gtsam_pose(pose.matrix());
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam_pose, noise));
  }

  gtsam::GncParams<gtsam::LevenbergMarquardtParams> gncParams;
  auto gnc =
      gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>(
          graph, initial, gncParams);

  if (multirobot_align_method_ == MultiRobotAlignMethod::L2) {
    gnc.setInlierCostThresholds(std::numeric_limits<double>::max());
  } else if (multirobot_align_method_ == MultiRobotAlignMethod::GNC) {
    gnc.setInlierCostThresholdsAtProbability(multirobot_gnc_align_probability_);
  } else {
    SYLAR_LOG_WARN(g_logger)
        << "Invalid multirobot alignment method in gncRobustPoseAveraging!";
  }

  gtsam::Values estimate = gnc.optimize();
  TransMatrixType result(estimate.at<gtsam::Pose3>(0).matrix());
  return result;
}

void Pcm::parseAndIncrementAdjMatrix(
    FactorGraph &factors, std::vector<LoopframeValue> &output_value,
    std::unordered_map<ObservationId, size_t> &num_new_loopclosure) {
  for (size_t i = 0; i < factors.size(); i++) {
    auto &new_factor = factors[i];
    auto from = new_factor.m_from_lf;
    auto to = new_factor.m_to_lf;

    auto from_key = GetKey(from->m_client_id, from->m_lf_id);
    auto to_key = GetKey(to->m_client_id, to->m_lf_id);
    auto from_client = GetKeyClientID(from_key);
    auto to_client = GetKeyClientID(to_key);
    int map_from = findAndInsertClient(map_clients_, from_client);
    int map_to = findAndInsertClient(map_clients_, to_client);

    //  相关帧找不到的不处理
    if (!output_value[map_from].exist(from_key) ||
        !output_value[map_to].exist(to_key)) {
      continue;
    }

    bool is_valid = false;
    double dist;
    if (from_client == to_client) {
      // 检查新的同一robot回环是否与先验位姿具有一致性
      is_valid = isOdomConsistent(new_factor, dist);
    } else {
      is_valid = true;
    }

    if (is_valid) {
      ObservationId obs_id(GetKeyClientID(from_key), GetKeyClientID(to_key));
      if (num_new_loopclosure.find(obs_id) == num_new_loopclosure.end()) {
        num_new_loopclosure.insert({obs_id, 1});
      } else {
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

void Pcm::findInliersIncremental(
    const std::unordered_map<ObservationId, size_t> &num_new_loopclosures) {
  for (const auto &num_lc : num_new_loopclosures) {
    std::stringstream ss;
    ObservationId robot_pair = num_lc.first;
    std::vector<int> inliers_idx;
    total_good_lc_ = 0;
    size_t prev_maxclique_size =
        loop_closures_[robot_pair]
            .consistent_factors.size();  // 原有回环边的数量
    // !!!找到最大的一致回环
    size_t num_inliers = findMaxCliqueHeuIncremental(
        loop_closures_[robot_pair].adj_matrix, num_lc.second,
        prev_maxclique_size, &inliers_idx);
    //  3、提取对应的一致 factor 对
    if (num_inliers > 0) {
      loop_closures_[robot_pair].consistent_factors = FactorGraph();  // reset
      for (size_t i = 0; i < num_inliers; i++) {
        loop_closures_[robot_pair].consistent_factors.add(
            loop_closures_[robot_pair].factors[inliers_idx[i]]);
        ss << " " << inliers_idx[i];
      }
    } else {
      // Set of inliers not modified. Don't reset consistent_factors
      num_inliers = prev_maxclique_size;
    }
    if (debug() && num_inliers != 0) {
      auto client_a = robot_pair.client_a;
      auto client_b = robot_pair.client_b;
      SYLAR_LOG_DEBUG(g_logger)
          << "Robot pair: [" << client_a << "," << client_b
          << "] total factors:" << loop_closures_[robot_pair].factors.size()
          << " inliers num: " << num_inliers << " inliers ids :[" << ss.str()
          << "]";
    }
  }
  // 4、 更新总的回环边数目
  for (auto robot_pair_lc : loop_closures_) {
    total_good_lc_ =
        total_good_lc_ + robot_pair_lc.second.consistent_factors.size();
  }
}

void Pcm::findInliers() {
  total_good_lc_ = 0;
  // iterate through loop closures and find inliers
  std::unordered_map<ObservationId, Measurements>::iterator it =
      loop_closures_.begin();
  // 检查更新连续 factor
  while (it != loop_closures_.end()) {
    size_t num_inliers;
    std::stringstream ss;
    if (loop_consistency_check_) {
      std::vector<int> inliers_idx;
      it->second.consistent_factors = FactorGraph();  // reset
      // find max clique
      num_inliers = findMaxCliqueHeu(it->second.adj_matrix, &inliers_idx);
      // update inliers, or consistent factors, according to max clique result
      for (size_t i = 0; i < num_inliers; i++) {
        it->second.consistent_factors.add(it->second.factors[inliers_idx[i]]);
        ss << " " << inliers_idx[i];
      }
    } else {
      it->second.consistent_factors = it->second.factors;
      num_inliers = it->second.factors.size();
    }
    if (debug()) {
      auto client_a = it->first.client_a;
      auto client_b = it->first.client_b;
      SYLAR_LOG_DEBUG(g_logger)
          << "Robot pair: [" << client_a << "," << client_b
          << "] num_inliers: " << num_inliers << "\n adj matrix: \n"
          << it->second.adj_matrix.bottomRightCorner<7, 7>()
          << "\n mah distance matrix: \n"
          << it->second.dist_matrix.bottomRightCorner<7, 7>()
          << "\n rot distance matrix: \n"
          << it->second.rot_matrix.bottomRightCorner<7, 7>()
          << "\n inliers ids :[" << ss.str() << "]";
    }
    it++;
    total_good_lc_ = total_good_lc_ + num_inliers;
  }
}

/// 每插入一个lc factor 都会调用一次
void Pcm::incrementAdjMatrix(const ObservationId &id, const LoopEdge &factor) {
  size_t num_lc = loop_closures_[id].factors.size();

  Matrix new_adj_matrix = Matrix::Zero(num_lc, num_lc);
  Matrix new_dist_matrix = Matrix::Zero(num_lc, num_lc);
  Matrix new_rot_matrix = Matrix::Zero(num_lc, num_lc);

  if (num_lc > 1) {
    new_adj_matrix.topLeftCorner(num_lc - 1, num_lc - 1) =
        loop_closures_[id].adj_matrix;
    new_dist_matrix.topLeftCorner(num_lc - 1, num_lc - 1) =
        loop_closures_[id].dist_matrix;
    new_rot_matrix.topLeftCorner(num_lc - 1, num_lc - 1) =
        loop_closures_[id].rot_matrix;
    for (size_t i = 0; i < num_lc - 1; i++) {
      double mah_distance = 0.0, rot_distance = 0.0;
      const auto &factors = loop_closures_[id].factors;
      bool consistent =
          areLoopsConsistent(factor, factors[i], mah_distance, rot_distance);
      new_dist_matrix(num_lc - 1, i) = mah_distance;
      new_dist_matrix(i, num_lc - 1) = mah_distance;
      new_rot_matrix(num_lc - 1, i) = rot_distance;
      new_rot_matrix(i, num_lc - 1) = rot_distance;
      if (consistent) {
        new_adj_matrix(num_lc - 1, i) = 1;
        new_adj_matrix(i, num_lc - 1) = 1;
      }
    }
  }
  loop_closures_[id].adj_matrix = new_adj_matrix;
  loop_closures_[id].dist_matrix = new_dist_matrix;
  loop_closures_[id].rot_matrix = new_rot_matrix;
}
bool Pcm::areLoopsConsistent(const LoopEdge &lc_a2b, const LoopEdge &lc_c2d,
                             double &dist, double &rot_dist) {
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
  if (GetKeyClientID(a_key) != GetKeyClientID(c_key)) {
    auto tmp = d_key;
    d_key = c_key;
    c_key = tmp;
    t_cd = t_cd.inverse();
  }
  SYLAR_ASSERT2(GetKeyClientID(a_key) == GetKeyClientID(c_key) &&
                    GetKeyClientID(b_key) == GetKeyClientID(d_key),
                "a,b,c,d 四个顶点位置不一致")

  auto robot_ac = GetKeyClientID(a_key);
  auto robot_bd = GetKeyClientID(b_key);
  TransMatrixType t_ac = odom_trajectories_[robot_ac][a_key].inverse() *
                         odom_trajectories_[robot_ac][c_key];  // taw * twc
  TransMatrixType t_db = odom_trajectories_[robot_bd][d_key].inverse() *
                         odom_trajectories_[robot_bd][b_key];  // tdw * twb

  TransMatrixType result = t_ac * t_cd * t_db * t_ba;

  return checkLoopConsistent(result, dist, rot_dist);
}
bool Pcm::checkLoopConsistent(TransMatrixType &result, double &dist,
                              double &rot_dist) {
  dist = result.translation().norm();  // 二范数
  dist = sqrt(dist);                   // 实际距离
  rot_dist = result.rotationMatrix().norm();
  if (dist < params_.dist_trans_threshold &&
      rot_dist < params_.dist_rot_threshold) {
    return true;
  }
  return false;
}

bool Pcm::isOdomConsistent(LoopEdge &factor, double &dist) {
  auto prev = factor.m_from_lf;
  auto cur = factor.m_to_lf;

  auto prev_key = GetKey(prev->m_client_id, prev->m_lf_id);
  auto cur_key = GetKey(cur->m_client_id, cur->m_lf_id);

  TransMatrixType T_cur_prev = factor.m_t_tf;

  if (prev->m_client_id != cur->m_client_id) {
    SYLAR_LOG_WARN(g_logger)
        << "Only check for odmetry consistency for intrarobot loop closures";
  }
  // TODO 应该是使用 solver里面的位姿还是全局的位姿呢？
  TransMatrixType result = prev->m_twc.inverse() * cur->m_twc * T_cur_prev;
  // 检查连续性是否在阈值里面
  return checkOdomConsistent(result, dist);
}

bool Pcm::checkOdomConsistent(TransMatrixType &trans, double &dist) {
  double rot_dist = trans.rotationMatrix().norm();
  dist = trans.translation().norm();

  if (dist < params_.odom_trans_threshold &&
      rot_dist < params_.odom_rot_threshold) {
    return true;
  }
  return false;
}

void Pcm::updateOdom(int map_id, const LoopEdge &factor,
                     std::vector<LoopframeValue> &output_values) {
  nfg_odom_[map_id].add(factor);

  LoopframeKey prev_key =
      GetKey(factor.m_from_lf->m_client_id, factor.m_from_lf->m_lf_id);
  LoopframeKey cur_key =
      GetKey(factor.m_to_lf->m_client_id, factor.m_to_lf->m_lf_id);

  LoopframePtr prev = factor.m_from_lf;
  LoopframePtr cur = factor.m_to_lf;
  // 记录里程计位姿
  auto client = GetKeyClientID(prev_key);
  if (odom_trajectories_.find(client) == odom_trajectories_.end()) {
    auto initial_pose = prev->m_twc;
    odom_trajectories_[client][prev_key] = initial_pose;
    robot_order_.push_back(client);
    if (debug())
      SYLAR_LOG_DEBUG(g_logger) << "Create new robot trajectories [client:"
                                << GetKeyClientID(prev_key)
                                << ",id:" << GetKeyLoopframeID(prev_key) << "]";
  }
  TransMatrixType prev_pose;
  if (odom_trajectories_[client].exist(prev_key)) {
    prev_pose = odom_trajectories_[client][prev_key];  // twc
  } else {
    SYLAR_LOG_ERROR(g_logger) << "Attempted to add odom to non-existing key. "
                              << "prev:[client:" << GetKeyClientID(prev_key)
                              << ",id : " << GetKeyLoopframeID(prev_key) << "]"
                              << "cur:[client:" << GetKeyClientID(cur_key)
                              << ",id : " << GetKeyLoopframeID(cur_key) << "]";
    SYLAR_ASSERT2(false, "Attempted to add odom to non-existing key. ");
  }
  odom_trajectories_[client][cur_key] =
      prev_pose * factor.m_t_tf.inverse();  // twc * t_from_to
}
}  // namespace cmd