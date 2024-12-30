#pragma once
#include <fstream>
#include <string>

#include "optimization/pcm_pgo/solver_params.hpp"
#include "optimization/pcm_pgo/utils/TypeUtils.h"
#include "typedefs_backend.hpp"

namespace cmd {
class OutlierRemoval {
 public:
  OutlierRemoval() = default;
  virtual ~OutlierRemoval() = default;

  virtual size_t getNumLC() = 0;
  virtual size_t getNumLCInliers() = 0;
  virtual size_t getNumOdomFactors() = 0;
  virtual size_t getNumSpecialFactors() = 0;

  virtual bool removeOutliers(
      const FactorGraph &new_factors, const LoopframeValue &new_loopframes,
      std::vector<FactorGraph> *output_nfg,
      std::vector<LoopframeValue> *output_values,
      std::unordered_map<int_t, std::pair<LoopframeKey, TransMatrixType>>
          &last_client_key_pose,
      std::vector<bool> *need_optimized_map) = 0;
      
  virtual void classifyNewLoopframeToMap(
      const LoopframeValue &new_loopframes,
      std::unordered_map<int_t, std::pair<LoopframeKey, TransMatrixType>>
          &last_client_key_pose,
      std::vector<LoopframeValue> *output_values,
      std::vector<FactorGraph> *output_nfg) {};
  virtual void saveData(std::string folder_path) {}

  void logOutput(const std::string &output_folder) {
    log_output_ = true;
    log_folder_ = output_folder;
    std::string filename = output_folder + "/outlier_rejection_status.txt";
    std::ofstream outfile;
    outfile.open(filename);
    outfile << "total inliers spin-time mc-time\n";
    outfile.close();
  }

  virtual LoopEdgePtr removeLastLoopClosure(ObservationId id,
                                            FactorGraph *updated_factors) {
    return nullptr;
  }

  virtual LoopEdgePtr removeLastLoopClosure(FactorGraph *updated_factors) {
    return nullptr;
  }

  virtual void ignoreLoopClosureWithPrefix(char prefix,
                                           FactorGraph *updated_factors) {}

  virtual void reviveLoopClosureWithPrefix(char prefix,
                                           FactorGraph *updated_factors) {}

  virtual inline std::vector<char> getIgnoredPrefixes() { return {}; }

  virtual void removePriorFactorsWithPrefix(const char &prefix,
                                            FactorGraph *updated_factors) {}

 protected:
  // bool debug() = true;
  bool log_output_ = false;
  std::string log_folder_;
};

}  // namespace cmd
