#include "map.hpp"
#include "loopframe.hpp"
#include "typedefs_backend.hpp"
#include "visualization/pangolin_viewer.hpp"

#include "loop_closure/generate_spherical_points.h"

#include "optimization/optimization.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

namespace cmd
{

    static LoggerPtr g_logger_sys = SYLAR_LOG_NAME("CMD-SLAM");

    Map::Map(size_t id, PangolinViewerPtr viewer)
        : m_mapId(id), m_mainId(-1)
    {
        m_viewer = viewer;
        /**
         * PCM_MODE
         */
        RobustSolverParams params;
        params.setPcm3DParams(3.0, 0.05, Verbosity::QUIET);
        params.setMultiRobotAlignMethod(MultiRobotAlignMethod::GNC);
        params.outlierRemovalMethod = OutlierRemovalMethod::PCM3D;
        solver_ = std::make_unique<PcmSolver>(params);
        // 启动线程之后，detch 线程
        Thread(std::bind(PcmSolver::Run,solver_.get()),"pcm_solver_thread");
    }
    RWMutexType& Map::getMutext(){
        return m_mutex;
    }
    bool Map::checkIsNeedOptimize(LoopEdgePtr le){
        RWMutexType::ReadLock lk(m_mutex);
        std::pair<int_t,int_t> toID_formID = std::make_pair(le->m_to_lf->m_client_id,le->m_from_lf->m_client_id);
        if(m_last_opt.find(toID_formID) == m_last_opt.end()){
            m_last_opt.insert(std::make_pair(toID_formID,le->m_to_lf->m_lf_id));
            return true;
        }
        if(m_last_opt[toID_formID] + 30 < le->m_to_lf->m_lf_id ){
            m_last_opt[toID_formID] = le->m_to_lf->m_lf_id;
            return true;
        }
        return false;
    }
    bool Map::hasAgent(int client_id)
    {
        RWMutexType::ReadLock lk(m_mutex);
        return m_fmgrs.find(client_id) != m_fmgrs.end();
    }
    LoopframePtr Map::getLoopframe(size_t client_id, size_t kf_id, bool expect_null)
    {
        RWMutexType::ReadLock lk(m_mutex);
        if (m_fmgrs.find(client_id) != m_fmgrs.end())
        {
            return m_fmgrs[client_id]->getLoopframeByKFId(kf_id);
        }
        return nullptr;
    }
    /// @brief 在 merge 时候才放进去 Map 中
    /// @param 
    void Map::addLoopEdge(LoopEdgePtr le)
    {
        RWMutexType::WriteLock lk(m_mutex);
        m_les.insert(le);
        solver_->insertLoopEdgeAndUpdate({le},true);
    }

    /// @brief odom 添加过程中添加新帧
    /// @param lf
    void Map::addLoopframe(LoopframePtr lf)
    {
        RWMutexType::WriteLock lk(m_mutex);
        if (!m_fix_lf)
        {
            m_fix_lf = lf;
        }
        if (m_fmgrs.find(lf->m_client_id) != m_fmgrs.end())
        {
            m_fmgrs[lf->m_client_id]->addLoopframe(lf);
        }
        else
        {
            // 创建新的 framemanager
            FramemanagerPtr fmgr(new Framemanager(lf->m_client_id));
            m_fmgrs.insert(std::make_pair(fmgr->m_clientId, fmgr));
            fmgr->addLoopframe(lf);
        }
        // 将 ref 也放入到 map 中
        for (auto le = lf->m_edges.begin(); le != lf->m_edges.end(); le++)
        {
            m_les.insert(*le);
        }

        /**
         * PCM_MODE
         */
        if (m_opt_mode == OptimizationMode::PCM_OUTLIER)
        {
            LoopEdgeVector pcm_les;
            for (auto le : lf->m_edges)
            {
                pcm_les.push_back(le);
            }
            // pcm_les 包含了所有的约束，包括回环和里程计
            solver_->insertLoopEdgeAndUpdate(pcm_les,true);
        }

        m_viewer->showLoopframes(lf);
    }
    void Map::transformMap(const TransMatrixType &Ttc)
    {
        RWMutexType::WriteLock lk(m_mutex);
        for (auto it = m_fmgrs.begin(); it != m_fmgrs.end(); it++)
        {
            FramemanagerPtr fm = it->second;
            fm->transfromFrammanager(Ttc);
        }
    }
    void Map::mergeMap(MapPtr fuse, LoopEdgePtr le, const TransMatrixType &Tcf)
    {
        RWMutexType::WriteLock lk(m_mutex);
        fuse->transformMap(Tcf);

        // 将fuse 数据都加入到当前的 map 中
        m_les.insert(le);
        for (auto tmp_le : fuse->m_les)
        {
            m_les.insert(tmp_le);
        }

        /**
         * PCM_MODE
         */
        if (m_opt_mode == OptimizationMode::PCM_OUTLIER)
        {
            LoopEdgeVector pcm_les;
            pcm_les.push_back(le);
            for (auto tmp_le : fuse->m_les)
            {
                pcm_les.push_back(tmp_le);
            }
            solver_->insertLoopEdgeAndUpdate(pcm_les,true);
        }

        LoopframeVector update_view_lfs;
        for (auto tmp_fmgr : fuse->m_fmgrs)
        {
            m_fmgrs.insert(tmp_fmgr);
            for (auto lf : tmp_fmgr.second->m_lfs)
            {
                update_view_lfs.push_back(lf.second);
            }
        }
        m_viewer->showLoopframes(update_view_lfs);
    }
    bool Map::updateMapAfterOptimize()
    {
        RWMutexType::WriteLock lk(m_mutex);
        LoopframeVector updated_lfs;
        for (auto fmgr : m_fmgrs)
        {
            fmgr.second->updateFramesFromCeres();
            fmgr.second->updateInsertFrameWhileOptimize();
            for (auto lf : fmgr.second->m_lfs)
            {
                updated_lfs.push_back(lf.second);
            }
        }
        m_viewer->showLoopframes(updated_lfs);
    }
    void Map::updateMapAfterRPGO(const LoopframeValue& values){
        LoopframeVector view_update_loopframes;
        view_update_loopframes.reserve(values.size());
        for(const auto& key_pose : values){
            auto key = key_pose.first;
            auto pose = key_pose.second;
            auto client = GetKeyClientID(key);
            auto id = GetKeyLoopframeID(key);
            
            LoopframePtr lf = getLoopframe(client,id);
            if(!lf){
                SYLAR_ASSERT2(false,"solver frame is no exist in map.");
            }
            lf->m_twc = pose;
            view_update_loopframes.push_back(lf);
        }
        // 更新优化过程中插入的帧
        for(auto& [key,fmgr] : m_fmgrs){
            auto&& loopframes = fmgr->updateInsertFrameWhileOptimize();
            view_update_loopframes.reserve(view_update_loopframes.size() + loopframes.size());
            for(auto lf : loopframes){
                view_update_loopframes.push_back(lf);
            }
        }
        m_viewer->showLoopframes(view_update_loopframes);
    }
    std::string Map::dump()
    {
        RWMutexType::ReadLock lk(m_mutex);
        std::stringstream ss;
        ss << "Map INFO "
           << "[id:" << m_mapId
           << ",agents: { ";
        for (auto fm : m_fmgrs)
        {
            ss << fm.first << " ";
        }
        ss << "}";
        ss << "]";
        return ss.str();
    }
    void Map::saveMap()
    {
        // ToDO
    }
    void Map::setOptimizingMode()
    {
        RWMutexType::WriteLock lk(m_mutex);
        m_optimizing = true;
        for (auto fmgr : m_fmgrs)
        {
            fmgr.second->m_optimizing = true;
        }
    }
    void Map::setOptimizedMode()
    {
        RWMutexType::WriteLock lk(m_mutex);
        m_optimizing = false;
        for (auto fmgr : m_fmgrs)
        {
            fmgr.second->m_optimizing = false;
        }
    }
    LoopframeVector Map::getAllLoopframe()
    {
        RWMutexType::ReadLock lk(m_mutex);
        LoopframeVector lfs;
        for (auto fmgr : m_fmgrs)
        {
            for (auto lf : fmgr.second->m_lfs)
            {
                lfs.push_back(lf.second);
            }
        }
        return lfs;
    }
    /// @brief 在sim3 优化的时候使用的
    /// @return
    LoopEdgeVector Map::getAllLoopEdge()
    {
        RWMutexType::ReadLock lk(m_mutex);
        LoopEdgeVector les;
        les.reserve(m_les.size());
        for (auto le : m_les)
        {
            les.push_back(le);
        }
        return les;
    }
    void Map::updateLoopframeFromMsg(MsgLoopframePtr msg)
    {
        RWMutexType::WriteLock lk(m_mutex);
        LoopframePtr lf = nullptr;
        if (m_fmgrs.find(msg->m_client_id) != m_fmgrs.end())
        {
            lf = m_fmgrs[msg->m_client_id]->getLoopframeByKFId(msg->m_lf_id);
        }
        if (!lf)
        {
            SYLAR_ASSERT2(false, "找不到对应的 loopframe , msg 出错.");
        }
        else
        {
            lf->updateFromMsg(msg);
        }
    }
    Framemanager::Framemanager(int client_id)
        : m_clientId(client_id), m_optimizing(false)
    {
    }
    Framemanager::~Framemanager()
    {
    }
    void Framemanager::updateFramesFromCeres(){
        SYLAR_ASSERT2(m_optimizing, "framemanager 不是处于优化状态!");
        // 不需要ceres 更新
        for (auto lf : m_lfs)
        {
            lf.second->updateFromCeres();
        }
    }
    LoopframeVector Framemanager::updateInsertFrameWhileOptimize()
    {
        SYLAR_ASSERT2(m_optimizing, "framemanager 不是处于优化状态!");
        LoopframeVector result;
        result.reserve(m_optimizing_buf.size());
        // 更新未入档的 lf
        while (!m_optimizing_buf.empty())
        {
            auto lf = m_optimizing_buf.front();
            m_optimizing_buf.pop_front();

            size_t len = lf->m_ref_id.size();
            // 构建 reference
            for (int i = 0; i < len; i++)
            {
                auto tmp_id = lf->m_ref_id[i];
                auto tmp_T_tf = lf->m_ref_cf[i];
                if (m_lfs.find(tmp_id) == m_lfs.end())
                {
                    SYLAR_ASSERT2(false, "出现错误 ref 帧 id.");
                }
                auto ref_lf = m_lfs[tmp_id];
                lf->addReference(ref_lf, tmp_T_tf, 0, 0);
                lf->m_twc = ref_lf->m_twc * tmp_T_tf;
            }
            m_lfs.insert(std::make_pair(lf->m_lf_id, lf));
            result.push_back(lf);
        }

        m_optimizing = true; // 更新完之后就重新开锁，防止后面的 addLoopframe 还放进 buf 中
        return result;
    }
    /// @brief 添加新帧，生成点云信息
    /// @param lf
    void Framemanager::addLoopframe(LoopframePtr lf)
    {

        // 因为优化会导致生成的位置发生问题，所以必须要利用原来的位姿信息来生成点云
        genImiLidarScan(lf);

        if (m_optimizing)
        {
            m_optimizing_buf.push_back(lf);
            return;
        }
        size_t len = lf->m_ref_id.size();
        // 构建 reference
        for (int i = 0; i < len; i++)
        {
            auto tmp_id = lf->m_ref_id[i];
            auto tmp_T_tf = lf->m_ref_cf[i];
            if (m_lfs.find(tmp_id) == m_lfs.end())
            {
                SYLAR_ASSERT2(false, "出现错误 ref 帧 id.");
            }
            auto ref_lf = m_lfs[tmp_id];
            lf->addReference(ref_lf, tmp_T_tf, 0, 0);
            lf->m_twc = ref_lf->m_twc * tmp_T_tf.inverse();
        }
        m_lfs.insert(std::make_pair(lf->m_lf_id, lf));
    }
    void Framemanager::transfromFrammanager(const TransMatrixType &Ttc)
    {
        for (auto it = m_lfs.begin(); it != m_lfs.end(); it++)
        {
            LoopframePtr lf = it->second;
            lf->m_twc = lf->m_twc * Ttc.inverse();
        }
    }
    LoopframePtr Framemanager::getPrevLoopframe()
    {
        if (m_lfs.size() != 0)
        {
            return m_lfs.rbegin()->second;
        }
        return nullptr;
    }
    LoopframePtr Framemanager::getLoopframeByKFId(int_t kf_id)
    {
        if (m_lfs.find(kf_id) != m_lfs.end())
        {
            return m_lfs[kf_id];
        }
        return nullptr;
    }
    void Framemanager::genImiLidarScan(LoopframePtr lf)
    {
        Point3Vector pts_spherical;
        if (m_lidar_range > 0)
        {
            /* ====================== Extract points ================================ */
            float cx = lf->m_calib.getCx();
            float cy = lf->m_calib.getCy();
            float fx = lf->m_calib.getFx();
            float fy = lf->m_calib.getFy();
            TransMatrixType Twc = lf->m_twc;
            int cur_id = lf->m_lf_id;
            for (const auto &p : lf->m_points)
            {
                Eigen::Vector4d p_c((p.m_u - cx) / fx / p.m_idepth_scaled,
                                    (p.m_v - cy) / fy / p.m_idepth_scaled,
                                    1 / p.m_idepth_scaled, 1);
                // 世界坐标的点
                Eigen::Vector3d p_g = Twc.matrix3x4() * p_c;
                m_pts_nearby.emplace_back(std::pair<int, Eigen::Vector3d>(cur_id, p_g));
            }

            /* ============= Preprocess points to have sphereical shape ============= */
            m_id_pose_wc[cur_id] = Twc.log();
            // auto t0 = std::chrono::high_resolution_clock::now();
            generate_spherical_points(m_pts_nearby, m_id_pose_wc, Twc.inverse(),
                                      m_lidar_range, pts_spherical);
            // auto t1 = std::chrono::high_resolution_clock::now();
            // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0);
            // pts_generation_time_.emplace_back(std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0));
        }
        lf->m_pts_spherical = pts_spherical;
    }
    void Framemanager::dump()
    {
        // TODO
    }
    void Framemanager::showResult()
    {
        // TODO
    }
    void Framemanager::saveTrajectory(const std::string &save_path)
    {
        // TODO
    }

    Mapmanager::Mapmanager(PangolinViewerPtr viewer)
        : m_viewer(viewer)
    {
        m_thread.reset(new Thread(std::bind(&Mapmanager::Run, this), "map manager"));
    }
    Mapmanager::~Mapmanager()
    {
        m_runing = false;
        m_thread->join();
    }
    struct MapMergeLockTool{
        MapMergeLockTool(MapPtr from_map,MapPtr to_map){
            from = from_map;
            to = to_map;
            from_map->lockMerge();
            to_map->lockMerge();
        }
        ~MapMergeLockTool(){
            to->unlockMerge();
            from->unlockMerge();
        }
        MapPtr from;
        MapPtr to;
    };
    void Map::lockMerge(){
        solver_->lockOptimize();
        m_mutex.wrlock();
    }
    void Map::unlockMerge(){
        m_mutex.unlock();
        solver_->unlockOptimize();
    }
    Solver* Map::getSolver(){
        return solver_;    
    }
    void Mapmanager::MergeMap(MapPtr from_map,MapPtr to_map,LoopEdgePtr le){
        // 确保两个 map 都没有在优化中
        // 需要给两个 map 上写锁
        MapMergeLockTool lkt(from_map,to_map);
        // 需要将 pcm 原有的 pair 插入进去
        // 先将原有value 和 factor 并入，最后再将 le 并入
        PcmSolver* from_solver = from_map->getSolver();

        // 执行一次优化
        // TODO 实现合并处理
    }
    void Mapmanager::Run()
    {
        SYLAR_LOG_INFO(g_logger_sys) << "--> START map manager ";
        while (m_runing)
        {
            if (this->checkLoopclosureBuf())
            {
                this->processLoopClosures();
            }
            usleep(1000);
        }
        SYLAR_LOG_INFO(g_logger_sys) << "<-- END map manager ";
    }

    MapPtr Mapmanager::getMap(int_t clientId)
    {
        for (auto map : m_maps)
        {
            if (map->hasAgent(clientId))
            {
                return map;
            }
        }
        return nullptr;
    }

    /// @brief 负责创建新的 Map
    /// @param lf
    /// @return
    bool Mapmanager::addLoopframe(LoopframePtr lf)
    {
        // SYLAR_LOG_DEBUG(g_logger_sys) << "接收到新 Loopframe\n"<< lf->dump();
        MapPtr tmp_map = nullptr;
        for (auto it = m_maps.begin(); it != m_maps.end(); it++)
        {
            auto map = *it;
            if (map->hasAgent(lf->m_client_id))
            {
                tmp_map = map;
                break;
                ;
            }
        }
        if (!tmp_map)
        {
            tmp_map.reset(new Map(lf->m_client_id, m_viewer));
            m_maps.insert(tmp_map);
        }
        tmp_map->addLoopframe(lf);
        return true;
    }
    bool Mapmanager::checkLoopclosureBuf()
    {
        return !m_lc_buf.empty();
    }

    void Mapmanager::processLoopClosures()
    {
        LoopEdgePtr le;
        {
            std::unique_lock<std::mutex> lk(m_mtx_lc_buf);
            le = m_lc_buf.front();
            m_lc_buf.pop_front();
        }
        
        

        MapPtr map_from = getMap(le->m_from_lf->m_client_id);
        MapPtr map_to = getMap(le->m_to_lf->m_client_id);

        if (map_from->m_optimizing || map_to->m_optimizing)
        {
            SYLAR_LOG_DEBUG(g_logger_sys) << "maps is optimizing";
            // 正在优化中，先不执行这些处理
            std::unique_lock<std::mutex> lk(m_mtx_lc_buf);
            m_lc_buf.push_back(le);
            return;
        }

        SYLAR_ASSERT2(map_from && map_to, "perform Merge map is nullptr");

        le->m_to_lf->addConstrant(le);
        if (map_from == map_to)
        {

            SYLAR_LOG_DEBUG(g_logger_sys) << "\n" << le->m_t_tf.matrix();
            if(!map_to->checkIsNeedOptimize(le)){
                return;
            }
            map_to->addLoopEdge(le); // 添加factor，会识别是否需要优化
            SYLAR_LOG_DEBUG(g_logger_sys) << "Preformed Optimize.  "
                                          << map_from->dump();
        }
        else
        {
            SYLAR_LOG_INFO(g_logger_sys) << "--> MAP FUSION ["
                                         << le->m_to_lf->m_client_id << "->"
                                         << le->m_from_lf->m_client_id << "]";
            // TODO 如果其中一个 map 正在优化应该怎么处理？
            map_to->mergeMap(map_from, le, le->m_t_tf);
            SYLAR_LOG_DEBUG(g_logger_sys) << le->m_t_tf.matrix();
            m_maps.erase(map_from); // 移除旧 map
            SYLAR_LOG_INFO(g_logger_sys) << "<-- MAP FUSION\033[1;32m SUCCESS. \033[0m";
        }
    }
    void Mapmanager::createConstrant(LoopframePtr from, LoopframePtr to, TransMatrixType t_tf, precision_t icp_score, precision_t sc_score)
    {
        LoopEdgePtr le(new LoopEdge(from, to, t_tf, icp_score, sc_score, EdgeType::LOOPCLOSURE));
        std::unique_lock<std::mutex> lk(m_mtx_lc_buf);
        m_lc_buf.push_back(le);
    }

}