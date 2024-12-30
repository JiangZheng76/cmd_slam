#pragma once
#include <unordered_set>

#include "optimization/pcm_pgo/outlier_removal.hpp"
#include "optimization/pcm_pgo/solver_params.hpp"
namespace cmd {

class Pcm : public OutlierRemoval {
 public:
  Pcm(PcmParams params,
      MultiRobotAlignMethod align_method = MultiRobotAlignMethod::GNC,
      double align_gnc_probability = 0.99);
  virtual size_t getNumLC() override { return total_lc_; }
  virtual size_t getNumLCInliers() override { return total_good_lc_; }
  virtual size_t getNumOdomFactors() override { return nfg_odom_.size(); }
  virtual size_t getNumSpecialFactors() override { return 0; }

  virtual bool removeOutliers(
      const FactorGraph &new_factors, const LoopframeValue &new_loopframes,
      std::vector<FactorGraph> *output_nfg,
      std::vector<LoopframeValue> *output_values,
      std::unordered_map<int_t, std::pair<LoopframeKey, TransMatrixType>>
          &last_client_key_pose,
      std::vector<bool> *need_optimized_map) override;

  void updateOdom(int client, const LoopEdge &factor);

  void parseAndIncrementAdjMatrix(
      FactorGraph &factors, std::vector<LoopframeValue> &output_value,
      std::unordered_map<ObservationId, size_t> &num_new_loopclosure);

  bool isOdomConsistent(LoopEdge &factor, double &dist);

  bool checkOdomConsistent(TransMatrixType &trans, double &dist);

  void incrementAdjMatrix(const ObservationId &id, const LoopEdge &factor);

  bool areLoopsConsistent(const LoopEdge &lc_a2b, const LoopEdge &lc_c2d,
                          double &dist, double &rot_dist);

  bool checkLoopConsistent(TransMatrixType &result, double &dist,
                           double &rot_dist);

  TransMatrixType getCurrentKeyPose(LoopframeKey prev_key);

  void findInliersIncremental(
      const std::unordered_map<ObservationId, size_t> &num_new_loopclosures);

  void findInliers();

  void buildGraphToOptimize(std::vector<FactorGraph> &output_nfg);

  std::vector<LoopframeValue> multirobotValueInitialization(
      std::vector<LoopframeValue> &input_value,
      std::vector<LoopframeValue> &output_value);

  LoopframeValue getRobotOdomValues(
      const int_t &client_id,
      const TransMatrixType &T_wb_wc = TransMatrixType());

  TransMatrixType CeresRobustPoseAveraging(const TransMatrixVector &input_poses,
                                           const double &rot_sigma,
                                           const double &trans_sigma);

  TransMatrixType gncRobustPoseAveraging(const TransMatrixVector &input_poses,
                                         const double &rot_sigma = 0.1,
                                         const double &trans_sigma = 0.5);

  void extractNeedOptimizeMap(
      std::unordered_map<ObservationId, size_t> &num_new_loopclosures,
      std::vector<bool> &output_client);

  int getKeyMap(LoopframeKey key);

  void classifyNewLoopframeToMap(const LoopframeValue &new_loopframes,
                                 std::vector<LoopframeValue> *output_values,
                                 std::vector<FactorGraph> *output_nfg);

  void mergeCheckAndPreform(
      std::unordered_map<ObservationId, size_t> &new_num_loopclosures,
      std::vector<FactorGraph> &output_nfg,
      std::vector<LoopframeValue> &output_values);

 private:
  PcmParams params_;
  std::vector<FactorGraph> nfg_odom_;
  std::vector<ClientSet> map_clients_;  // 每一个需要优化的 map 的 client
  std::unordered_map<ObservationId, Measurements> loop_closures_;
  std::unordered_map<int_t, LoopframeValue> odom_trajectories_;  // twc
  std::vector<ObservationId> loop_closures_in_order_;

  size_t total_lc_, total_good_lc_;
  std::vector<char> ignored_prefixes_;
  FactorGraph last_ouput_nfg_;
  FactorGraph odom_inconsistent_factors_;
  FactorGraph pairwise_inconsistent_factors_;

  MultiRobotAlignMethod multirobot_align_method_;
  double multirobot_gnc_align_probability_;
  std::vector<int_t> robot_order_;  // 进来的机器人顺序

  // Toggle odom and loop consistency check
  bool odom_check_;
  bool loop_consistency_check_;
};
}  // namespace cmd