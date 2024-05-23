#pragma once

// C++
#include <eigen3/Eigen/Eigen>
#include "typedefs_backend.hpp"
namespace cmd
{
    // 辅助 map 更改状态
    class MapOptimizationWrap
    {
    public:
        MapOptimizationWrap(MapPtr map);
        ~MapOptimizationWrap();

        MapPtr m_map;
    };

    class Optimization
    {
    public:
        Optimization() = delete;
        static void Sim3PoseGraphOptimization(MapPtr map);
    };

} // end ns
