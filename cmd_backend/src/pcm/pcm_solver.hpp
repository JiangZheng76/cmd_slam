#pragma once 

#include "typedefs_backend.hpp"

#include "KimeraRPGO/RobustSolver.h"
#include "KimeraRPGO/SolverParams.h"

// using namespace KimeraRPGO;

namespace cmd{

class SE3PcmSolver : private KimeraRPGO::RobustSolver{
public:
    SE3PcmSolver(const KimeraRPGO::RobustSolverParams& params);

    void convertLoopEdge2Factor(LoopEdgeVector &les,gtsam::NonlinearFactorGraph& tmp_nfg,gtsam::Values&  tmp_est);

    void updateByLoopEdge(LoopEdgeVector &les,bool is_optimize);

    void getAllVals(gtsam::Values& vals);

};


}