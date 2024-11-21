#pragma once

#include "typedefs_backend.hpp"

#include "loopframe.hpp"
#include "optimization/pcm_pgo/utils/TypeUtils.h"
#include "optimization/pcm_pgo/solver_params.hpp"
#include "optimization/pcm_pgo/pcm.hpp"

namespace cmd
{
    // 维护所有的 factor 和 一个 Loopframe副本
    // 原有的LoopframePtr 保持不变，用来可视化
    class PcmSolver
    {
    public:
        PcmSolver(const RobustSolverParams &params);

        // 插入 factor 并更新
        void insertLoopEdgeAndUpdate(LoopEdgeVector &les, bool is_optimize);

        void update(FactorGraph &factors, LoopframeValue &value, bool is_optimize);

        void convertFactorAndValue(const LoopEdgeVector &les,FactorGraph& factors,LoopframeValue& value);
        
        bool addAndCheckIfOptimize(FactorGraph &factors, LoopframeValue &values);

        void getAllValueToUpdateMap(LoopframeValue& vals);

        void callOptimize();

        void optimize();

        bool checkNeedOptimize();

        void Run();
        
        void Stop();

        void updateDataAfterOptimize(Sim3LoopframeValue& sim3_values);

        const LoopframeValue& getValue();

    private:
        std::unique_ptr<OutlierRemoval> outlier_removal_;
        FactorGraph nfg_;       // 包括里程计和回环的合法 factor
        LoopframeValue values_; // 所有顶点记录的是 t_cw 的位姿
        LoopframeKey fix_key_;
        // Factors and values subjected to change
        FactorGraph temp_nfg_; 
        LoopframeValue temp_values_; // 记录的是 ref 的位姿

        bool optimizing_;
        bool need_optimize_;
        bool solver_running_; // 优化线程
        bool is_update-;
        RWMutextType mutex_; // 读写里面数据的互斥变量
    };
}