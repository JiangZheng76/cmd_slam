#ifndef __TYPEDEFS_BACKEND_HPP__
#define __TYPEDEFS_BACKEND_HPP__
// #define EIGEN_DONT_ALIGN
#include <Eigen/Core>
#include <flann/flann.hpp>
#include <memory>
#include <queue>
#include <sophus/sim3.hpp>

#include "cmd_comm.hpp"
#include "utils.hpp"

#define DISTANCE_THRES 35.0 
#define ID_DISTANCE_THRES 80
// loop closure
#define LIDAR_RANGE 40.0
#define SCANCONTEXT_THRES 0.33
// #define SCANCONTEXT_THRES 0.40

// icp
#define ICP_ITERA_TIME 50
#define ICP_TRANS_EPSILON 0.01
#define ICP_EUC_EPSILON 0.001
// #define ICP_THRES 1.5
#define ICP_THRES 1.7

// optimization
#define OPT_ROBUST_LOSS 1.0
#define OPT_DIM 7
#define OPT_ITER 50  // 迭代次数

// Comm
#define COMM_IP "10.0.0.10"
#define COMM_PORT "8888"

// viewer
#define MAX_RES_PER_POINT 8
#define patternNum 8
#define VIEWER_WIDTH 640
#define VIEWER_HIGH 640

// normalize dso errors to roughly around 1.0
#define DSO_ERROR_SCALE 5.0
#define SCALE_ERROR_SCALE 0.1
#define DIRECT_ERROR_SCALE 0.1
#define ICP_ERROR_SCALE 1.0

// the rotation estimated by DSO is much more accurate than translation
#define POSE_R_WEIGHT 1e4

// loop handler
#define LOOPHANDLER_THRES 70

namespace cmd {
inline bool debug() {
  static bool debug_ = true;
  return debug_;
}
}  // namespace cmd

namespace cmd {

class TimeCosters : std::list<double> {
 private:
  std::string name_ = "";
  std::string unit_ = "ms";

 public:
  using Timepoint = std::chrono::_V2::steady_clock::time_point;
  TimeCosters(const std::string& name) { name_ = name; }
  void addCost(double cost) { push_back(cost); }
  void addCost(Timepoint start, Timepoint end) {
    /**
     * s : second
     * ms : millisecond
     * us : microsecond
     * ns : nanosecond
     */
    unit_ = "ms";
    using second_type = std::chrono::milliseconds;
    double duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    duration /= 1000;
    addCost(duration);
  }
  std::string Dump() {
    double len = size();
    double max = 0, min = INT_MAX, sum = 0;
    double average;
    for (auto cost : *this) {
      sum += cost;
      if (cost > max) max = cost;
      if (cost < min) min = cost;
    }
    average = sum / len;
    std::stringstream ss;
    ss << name_ << " size:" << len << "[max:" << max << unit_ << ", min:" << min
       << unit_ << ", sum:" << sum << unit_ << ", average:" << average << unit_
       << "] ";
    return ss.str();
  }
  static Timepoint GetNow() { return std::chrono::steady_clock::now(); }
};
extern TimeCosters pp_costs_;  // point prepare
inline TimeCosters& GetPpCosts() { return pp_costs_; }
extern TimeCosters sc_costs_;
inline TimeCosters& GetScCosts() { return sc_costs_; }
extern TimeCosters icp_costs_;
inline TimeCosters& GetIcpCosts() { return icp_costs_; }
extern TimeCosters rk_costs_;
inline TimeCosters& GetRkCosts() { return rk_costs_; }
extern TimeCosters pgo_costs_;
inline TimeCosters& GetPgoCosts() { return pgo_costs_; }
extern TimeCosters pcm_costs_;
inline TimeCosters& GetPcmCosts() { return pcm_costs_; }

}  // namespace cmd

namespace cmd {

class ScanContext;
using ScanContextPtr = std::shared_ptr<ScanContext>;
class LoopEdge;
using LoopEdgePtr = std::shared_ptr<LoopEdge>;
class Loopframe;
using LoopframePtr = std::shared_ptr<Loopframe>;

class Map;
using MapPtr = std::shared_ptr<Map>;
class Framemanager;
using FramemanagerPtr = std::shared_ptr<Framemanager>;

class PangolinViewer;
using PangolinViewerPtr = std::shared_ptr<PangolinViewer>;

class Optimizer;
using OptimizerPtr = std::shared_ptr<Optimizer>;

class Communicator;
using CommPtr = std::shared_ptr<Communicator>;

class AgentHandler;
using AgentHandlerPtr = std::shared_ptr<AgentHandler>;

class Mapmanager;
using MapmanagerPtr = std::shared_ptr<Mapmanager>;

class LoopHandler;
using LoopHandlerPtr = std::shared_ptr<LoopHandler>;

// display
class LoopframeDisplay;
using LoopframeDisplayPtr = std::shared_ptr<LoopframeDisplay>;

class AgentDisplay;
using AgentDisplayPtr = std::shared_ptr<AgentDisplay>;

class PangolinViewer;
using PangolinViewerPtr = std::shared_ptr<PangolinViewer>;

// codsv pangolin
class LoopFrameDisplay;
using LoopFrameDisplayPtr = std::shared_ptr<LoopFrameDisplay>;

class CmdBackend;
using CmdBackendPtr = std::shared_ptr<CmdBackend>;

class Point2;
}  // namespace cmd

namespace cmd {

using SigType = std::vector<std::pair<int, double>>;

using Point3 = Eigen::Vector3d;

using Matrix = Eigen::MatrixXd;

using InformationMat = Matrix7Type;

using VecSim3 = Eigen::Matrix<double, 7, 1>;

}  // namespace cmd

namespace cmd {
// display color
const std::vector<std::vector<precision_t>> col_vec = {
    {0.678, 0.847, 0.9},   {0.902, 0.902, 0.98},  {0.941, 0.502, 0.502},
    {0.941, 0.902, 0.549}, {0.827, 0.709, 0.878}, {0.647, 0.165, 0.165},
    {0.70, 0.87, 0.41}};
}  // namespace cmd

namespace cmd {

using Point2Vector = std::vector<Point2, Eigen::aligned_allocator<Point2>>;
using Point3Vector = std::vector<Point3, Eigen::aligned_allocator<Point3>>;
using LoopEdgeVector = std::vector<LoopEdgePtr>;
using LoopframeVector = std::vector<LoopframePtr>;
using TransMatrixVector =
    std::vector<TransMatrixType, Eigen::aligned_allocator<TransMatrixType>>;
using IDPose3Vector =
    std::vector<std::pair<int, Eigen::Vector3d>,
                Eigen::aligned_allocator<std::pair<int, Eigen::Vector3d>>>;

using LoopframeList = std::list<LoopframePtr>;
using LoopEdgeList = std::list<LoopEdgePtr>;

template <typename T>
using aligned_unordered_map =
    std::unordered_map<int, T, std::hash<int>, std::equal_to<int>,
                       Eigen::aligned_allocator<std::pair<const int, T>>>;

using IDTransMatrixMap = aligned_unordered_map<Eigen::Matrix<double, 6, 1>>;

};  // namespace cmd

namespace cmd {
const int staticPattern[10][40][2] = {
    {{0, 0},       {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},  // .
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}},

    {{0, -1},      {-1, 0},      {0, 0},       {1, 0},       {0, 1},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},  // +
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}},

    {{-1, -1},     {1, 1},       {0, 0},       {-1, 1},      {1, -1},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},  // x
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}},

    {{-1, -1},     {-1, 0},      {-1, 1},      {-1, 0},
     {0, 0},       {0, 1},       {1, -1},      {1, 0},
     {1, 1},       {-100, -100},  // full-tight
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}},

    {{0, -2},      {-1, -1},     {1, -1},      {-2, 0},
     {0, 0},       {2, 0},       {-1, 1},      {1, 1},
     {0, 2},       {-100, -100},  // full-spread-9
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}},

    {{0, -2},      {-1, -1},     {1, -1},      {-2, 0},
     {0, 0},       {2, 0},       {-1, 1},      {1, 1},
     {0, 2},       {-2, -2},  // full-spread-13
     {-2, 2},      {2, -2},      {2, 2},       {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}},

    {{-2, -2},     {-2, -1},     {-2, -0},     {-2, 1},
     {-2, 2},      {-1, -2},     {-1, -1},     {-1, -0},
     {-1, 1},      {-1, 2},  // full-25
     {-0, -2},     {-0, -1},     {-0, -0},     {-0, 1},
     {-0, 2},      {+1, -2},     {+1, -1},     {+1, -0},
     {+1, 1},      {+1, 2},      {+2, -2},     {+2, -1},
     {+2, -0},     {+2, 1},      {+2, 2},      {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}},

    {{0, -2},      {-1, -1},     {1, -1},      {-2, 0},
     {0, 0},       {2, 0},       {-1, 1},      {1, 1},
     {0, 2},       {-2, -2},  // full-spread-21
     {-2, 2},      {2, -2},      {2, 2},       {-3, -1},
     {-3, 1},      {3, -1},      {3, 1},       {1, -3},
     {-1, -3},     {1, 3},       {-1, 3},      {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}},

    {{0, -2},      {-1, -1},     {1, -1},      {-2, 0},
     {0, 0},       {2, 0},       {-1, 1},      {0, 2},
     {-100, -100}, {-100, -100},  // 8 for SSE efficiency
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100},
     {-100, -100}, {-100, -100}},

    {{-4, -4},     {-4, -2},     {-4, -0},     {-4, 2},
     {-4, 4},      {-2, -4},     {-2, -2},     {-2, -0},
     {-2, 2},      {-2, 4},  // full-45-SPREAD
     {-0, -4},     {-0, -2},     {-0, -0},     {-0, 2},
     {-0, 4},      {+2, -4},     {+2, -2},     {+2, -0},
     {+2, 2},      {+2, 4},      {+4, -4},     {+4, -2},
     {+4, -0},     {+4, 2},      {+4, 4},      {-200, -200},
     {-200, -200}, {-200, -200}, {-200, -200}, {-200, -200},
     {-200, -200}, {-200, -200}, {-200, -200}, {-200, -200},
     {-200, -200}, {-200, -200}, {-200, -200}, {-200, -200},
     {-200, -200}, {-200, -200}},
#define patternP staticPattern[8]
};

}  // namespace cmd
#endif