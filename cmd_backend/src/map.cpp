#include "map.hpp"
#include "loopframe.hpp"
#include "typedefs_backend.hpp"

#include "loop_closure/generate_spherical_points.h"

#include "optimization/optimization.hpp"

namespace cmd
{

    static LoggerPtr g_logger_sys = SYLAR_LOG_NAME("system");

    Map::Map(size_t id)
        : m_mapId(id), m_mainId(-1)
    {
    }
    bool Map::hasAgent(int client_id)
    {
        RWMutextType::ReadLock lk(m_mutex);
        return m_fmgrs.find(client_id) != m_fmgrs.end();
    }
    LoopframePtr Map::getLoopframe(size_t client_id, size_t kf_id, bool expect_null)
    {
        RWMutextType::ReadLock lk(m_mutex);
        if (m_fmgrs.find(client_id) != m_fmgrs.end())
        {
            return m_fmgrs[client_id]->getLoopframeByKFId(kf_id);
        }
        return nullptr;
    }
    /// @brief 在 merge 时候才放进去 Map 中
    /// @param le
    void Map::addLoopEdge(LoopEdgePtr le)
    {
        RWMutextType::WriteLock lk(m_mutex);
        m_les.insert(le);
    }

    /// @brief 添加帧和 edge
    /// @param lf
    void Map::addLoopframe(LoopframePtr lf)
    {
        RWMutextType::WriteLock lk(m_mutex);
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
    }
    void Map::transformMap(const TransMatrixType &Ttc)
    {
        RWMutextType::WriteLock lk(m_mutex);
        for (auto it = m_fmgrs.begin(); it != m_fmgrs.end(); it++)
        {
            FramemanagerPtr fm = it->second;
            fm->transfromFrammanager(Ttc);
        }
    }
    void Map::mergeMap(MapPtr fuse, LoopEdgePtr le, const TransMatrixType &Tcf)
    {
        RWMutextType::WriteLock lk(m_mutex);
        fuse->transformMap(Tcf);

        // 将fuse 数据都加入到当前的 map 中
        m_les.insert(le);
        for(auto tmp_le : fuse->m_les){
            m_les.emplace(tmp_le);
        }
        for(auto tmp_fmgr : fuse->m_fmgrs){
            m_fmgrs.emplace(tmp_fmgr);
        }
    }
    bool Map::updateMapAfterOptimize()
    {
        RWMutextType::WriteLock lk(m_mutex);
        for (auto fmgr : m_fmgrs)
        {
            fmgr.second->updateAfterOptimize();
        }
    }
    std::string Map::dump()
    {
        RWMutextType::ReadLock lk(m_mutex);
        std::stringstream ss;
        ss << "Map frame manager id : {";
        // TODO
    }
    void Map::saveMap()
    {
        // ToDO
    }
    void Map::setOptimizingMode()
    {
        RWMutextType::WriteLock lk(m_mutex);
        m_optimizing = true;
        for (auto fmgr : m_fmgrs)
        {
            fmgr.second->m_optimizing = true;
        }
    }
    void Map::setOptimizedMode()
    {
        RWMutextType::WriteLock lk(m_mutex);
        m_optimizing = false;
        for (auto fmgr : m_fmgrs)
        {
            fmgr.second->m_optimizing = false;
        }
    }
    std::vector<LoopframePtr> Map::getAllLoopframe()
    {
        RWMutextType::ReadLock lk(m_mutex);
        std::vector<LoopframePtr> lfs;
        for (auto fmgr : m_fmgrs)
        {
            for (auto lf : fmgr.second->m_lfs)
            {
                lfs.push_back(lf.second);
            }
        }
        return lfs;
    }
    std::vector<LoopEdgePtr> Map::getAllLoopEdge()
    {
        RWMutextType::ReadLock lk(m_mutex);
        std::vector<LoopEdgePtr> les;
        les.reserve(m_les.size());
        for (auto le : m_les)
        {
            les.push_back(le);
        }
        return les;
    }
    void Map::updateLoopframeFromMsg(MsgLoopframePtr msg)
    {
        RWMutextType::WriteLock lk(m_mutex);
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
    void Framemanager::updateAfterOptimize()
    {

        SYLAR_ASSERT2(m_optimizing, "framemanager 不是处于优化状态!");

        for (auto lf : m_lfs)
        {
            lf.second->updateFromCeres();
        }

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
        }

        m_optimizing = true; // 更新完之后就重新开锁，防止后面的 addLoopframe 还放进 buf 中
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
            lf->m_twc = ref_lf->m_twc * tmp_T_tf;
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
        std::vector<Eigen::Vector3d> pts_spherical;
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

    Mapmanager::Mapmanager(PangolinLoopViewerPtr viewer)
        :m_viewer(viewer)
    {
        m_thread.reset(new Thread(std::bind(&Mapmanager::Run, this), "map manager"));
    }
    Mapmanager::~Mapmanager()
    {
        m_runing = false;
        m_thread->join();
    }
    void Mapmanager::Run()
    {
        SYLAR_LOG_INFO(g_logger_sys) << "--> START map manager ";
        while (m_runing)
        {
            if (this->checkMerageBuf())
            {
                this->performMerge();
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
            tmp_map.reset(new Map(lf->m_client_id));
            m_maps.insert(tmp_map);
        }
        tmp_map->addLoopframe(lf);
        return true;
    }
    bool Mapmanager::checkMerageBuf()
    {
        return !m_merge_buf.empty();
    }
    void Mapmanager::performMerge()
    {
        SYLAR_LOG_INFO(g_logger_sys) << "----> Merge maps";

        auto le = m_merge_buf.front();
        m_merge_buf.pop_front();

        MapPtr map_from = getMap(le->m_from_lf->m_client_id);
        MapPtr map_to = getMap(le->m_to_lf->m_client_id);

        if (map_from->m_optimizing || map_to->m_optimizing)
        {
            SYLAR_LOG_WARN(g_logger_sys) << "maps is optimizing";
            // 正在优化中，先不执行这些处理
            m_merge_buf.push_back(le);
            return;
        }

        SYLAR_ASSERT2(map_from && map_to, "perform Merge map is nullptr");

        le->m_to_lf->addConstrant(le);
        if (map_from == map_to)
        {
            map_to->addLoopEdge(le);
            // TODO 设置优化间隔
            Optimization::Sim3PoseGraphOptimization(map_to);
            SYLAR_LOG_INFO(g_logger_sys) << "Preform Optimize.\n"
                                         << map_from->dump();
        }
        else
        {
            SYLAR_LOG_INFO(g_logger_sys) << "\033[1;32m+++ MAP FUSION +++\033[0m";
            // TODO 如果其中一个 map 正在优化应该怎么处理？
            map_to->mergeMap(map_from, le, le->m_t_tf);
            m_maps.erase(map_from); // 移除旧 map
            SYLAR_LOG_INFO(g_logger_sys) << "--> MAP FUSION " << le->m_to_lf->m_client_id << "->" << le->m_from_lf->m_client_id << "  SUCCESS. ";
        }
        SYLAR_LOG_INFO(g_logger_sys) << "----> Merge end.";
    }
    void Mapmanager::createConstrant(LoopframePtr from, LoopframePtr to, TransMatrixType t_tf, precision_t icp_score, precision_t sc_score)
    {
        LoopEdgePtr le(new LoopEdge(from, to, t_tf, icp_score, sc_score));
        m_merge_buf.push_back(le);
    }

}