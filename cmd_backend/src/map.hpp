#pragma once

#include "cmd_comm.hpp"
#include "typedefs_backend.hpp"

namespace cmd
{

    class Map
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        int m_mapId;
        int m_mainId; //  map 中的主 framemanager id

        bool m_changed = false; // map是否得到了更新
        bool m_optimizing = false;
        LoopframePtr m_fix_lf = nullptr; // 优化固定帧

        std::set<LoopEdgePtr> m_les;
        std::unordered_map<uint32_t, FramemanagerPtr> m_fmgrs; // 当前map的 所有framemanager

        RWMutextType m_mutex;

    public:
        Map(size_t id);
        // static MapPtr CreateMap(size_t client_id, PangolinLoopViewerPtr pgllv, CommPtr comm);
        void setOptimizedMode();
        void setOptimizingMode();

        bool hasAgent(int client_id);
        LoopframePtr getLoopframe(size_t client_id, size_t kf_id, bool expect_null = false);

        void addLoopEdge(LoopEdgePtr le);
        void updateLoopframeFromMsg(MsgLoopframePtr msg);
        void addLoopframe(LoopframePtr lf);
        void transformMap(const TransMatrixType &Ttc);
        void mergeMap(MapPtr fuse, LoopEdgePtr le, const TransMatrixType &Tcf);

        bool updateMapAfterOptimize(); // 更新优化之后的效果

        std::vector<LoopframePtr> getAllLoopframe();
        std::vector<LoopEdgePtr> getAllLoopEdge();

        void saveMap();
        std::string dump();
    };

    class Framemanager
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        int_t m_clientId;
        std::map<int_t, LoopframePtr> m_lfs;      // 已经添加到 fmgr 中的帧
        std::list<LoopframePtr> m_optimizing_buf; // 优化的过程中添加的帧

        bool m_optimizing = false; // 是否优化中

        LoopframePtr m_first_lf = nullptr;

        int_t m_prev_lfId = 0; // 上一帧LF id

        double m_lidar_range = 40.0;                                       // 保留 Lidar scan 范围
        std::vector<std::pair<int, Eigen::Vector3d>> m_pts_nearby;         // 上一次处理完的附近点云 【所属LFid，xyz】 用于生成类 Lidar scan
        std::unordered_map<int, Eigen::Matrix<double, 6, 1>> m_id_pose_wc; // 所有帧的李代数位姿【LFid，se(3)】 用于生成类 Lidar scan

    public:
        Framemanager(int client_id); // 只有Map才可以创建
        ~Framemanager();

        // 重置新来帧的 tfm
        // void resetTFMOptimized() { tfm_optimized_cur_ = g2o::SE3Quat(); }

        void addLoopframe(LoopframePtr lf);
        void transfromFrammanager(const TransMatrixType &Ttc); // cut->target

        LoopframePtr getPrevLoopframe();
        LoopframePtr getLoopframeByKFId(int_t kf_id);

        void updateAfterOptimize();

        void dump();
        void showResult();
        void saveTrajectory(const std::string &save_path);

    private:
        void genImiLidarScan(LoopframePtr lf);
    };

    class Mapmanager
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::set<MapPtr> m_maps;
        std::list<LoopEdgePtr> m_merge_buf; // from -> to
        bool m_runing = true;
        ThreadPtr m_thread;
        PangolinLoopViewerPtr m_viewer; // TODO 还没有发送帧

        Mapmanager(PangolinLoopViewerPtr viewer = nullptr);
        virtual ~Mapmanager();

        void Run();

        MapPtr getMap(int_t clientId);
        bool addLoopframe(LoopframePtr lf);

        bool checkMerageBuf();
        void performMerge();
        void createConstrant(LoopframePtr from, LoopframePtr to, TransMatrixType t_tf, precision_t icp_score, precision_t sc_score);
    };
}
