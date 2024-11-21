#include "optimization/pcm_pgo/outlier_removal.hpp"
#include "optimization/pcm_pgo/solver_params.hpp"
#include <unordered_set>
namespace cmd
{

    class Pcm : public OutlierRemoval
    {
    public:
        Pcm(PcmParams params,
            MultiRobotAlignMethod align_method = MultiRobotAlignMethod::NONE,
            double align_gnc_probability = 0.99);
        virtual size_t getNumLC() override { return total_lc_; }
        virtual size_t getNumLCInliers() override { return total_good_lc_; }
        virtual size_t getNumOdomFactors() override { return nfg_odom_.size(); }
        virtual size_t getNumSpecialFactors() override { return 0; }

        virtual bool removeOutliers(const FactorGraph &new_factors,
                                    const LoopframeValue &new_loopframes,
                                    FactorGraph *nfg,
                                    LoopframeValue *values) override;

        void updateOdom(const LoopEdge &factor,
                        LoopframeValue &output_nfg);

        void parseAndIncrementAdjMatrix(FactorGraph &factors,
                                        LoopframeValue &output_value,
                                        std::unordered_map<ObservationId, size_t> &num_new_loopclosure);

        bool isOdomConsistent(LoopEdge &factor, double &dist);

        bool checkOdomConsistent(TransMatrixType &trans, double &dist);

        void incrementAdjMatrix(const ObservationId &id,
                                const LoopEdge &factor);

        bool areLoopsConsistent(const LoopEdge &a_lcBetween_b,
                                const LoopEdge &c_lcBetween_d,
                                double &dist);

        bool checkLoopConsistent(TransMatrixType &result, double &dist);

        void findInliersIncremental(const std::unordered_map<ObservationId, size_t> &num_new_loopclosures);

        void findInliers();

        FactorGraph buildGraphToOptimize();

        LoopframeValue multirobotValueInitialization(LoopframeValue &output_value);

        LoopframeValue getRobotOdomValues(const int_t &client_id,
                                         const TransMatrixType &T_wb_wc = TransMatrixType());

        TransMatrixType gncRobustPoseAveraging(const TransMatrixVector& input_poses,
                               const double& rot_sigma = 0.1,
                               const double& trans_sigma = 0.5);
        
        private:
            PcmParams params_;
            FactorGraph nfg_odom_;
            std::unordered_map<ObservationId, Measurements> loop_closures_;
            std::unordered_map<int_t, LoopframeValue> odom_trajectories_;
            std::vector<ObservationId> loop_closures_in_order_;

            size_t total_lc_, total_good_lc_;
            std::vector<char> ignored_prefixes_;
            FactorGraph last_ouput_nfg_;
            FactorGraph odom_inconsistent_factors_;
            FactorGraph pairwise_inconsistent_factors_;

            MultiRobotAlignMethod multirobot_align_method_;
            double multirobot_gnc_align_probability_;
            std::vector<int_t> robot_order_; // 进来的机器人顺序

            // Toggle odom and loop consistency check
            bool odom_check_;
            bool loop_consistency_check_;
        };
    }