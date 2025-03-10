#include "loop_handler.hpp"
#include "loopframe.hpp"

#include "loop_closure/scancontext.hpp"
#include "loop_closure/search_place.h"
#include "loop_closure/icp.h"

namespace cmd
{

    static LoggerPtr g_logger_sys = SYLAR_LOG_NAME("system");

    LoopHandler::LoopHandler(float lidar_range, float scan_context_thres, MapmanagerPtr mapMgr)
        : m_lidar_range(lidar_range),
          m_scan_context_thres(scan_context_thres),
          m_mapMgr(mapMgr)
    {
        // place recognition
        m_sc.reset(new ScanContext());
        // 创建存放sc的矩阵
        flann::Matrix<float> init_data(new float[m_sc->getHeight()], 1,
                                       m_sc->getHeight());
        m_ringkeys = new flann::Index<flann::L2<float>>(init_data,
                                                        flann::KDTreeIndexParams(4));
        m_ringkeys->buildIndex();

        m_running = true;
        m_main_thread.reset(new Thread(std::bind(&LoopHandler::Run, this), "loop handler thread."));
    }
    LoopHandler::~LoopHandler()
    {
        m_running = false;
        m_main_thread->join();
        SYLAR_LOG_INFO(g_logger_sys) << " --- join Loop Thread. --- ";
        delete m_ringkeys;
    }
    void LoopHandler::Run()
    {
        SYLAR_LOG_INFO(g_logger_sys) << "--> START Loop Thread.";
        while (m_running)
        {
            LoopframePtr query_frame = nullptr;
            {
                if (m_buf_lfs.empty())
                {
                    usleep(500);
                    continue;
                }
                MutextType::Lock lk(m_mtx_buf_lf);
                query_frame = m_buf_lfs.front();
                m_buf_lfs.pop_front();
            }
            SYLAR_ASSERT(query_frame);

            m_preocessed_lf.push_back(query_frame);
            // 没有对尺度优化信息的不进行优化
            if (m_lidar_range < 0 || query_frame->m_scale_error < 0)
            {
                SYLAR_LOG_INFO(g_logger_sys) << "skip loopframe " << query_frame->m_lf_id;
                usleep(500);
                continue;
            }

            /* == Get ringkey and signature from the aligned points by Scan Context = */
            // 生成当前帧的
            flann::Matrix<float> ringkey;
            SigType signature;           // sc的签名
            Eigen::Matrix4d tfm_pca_rig; // pca对齐矩阵

            m_sc->generate(query_frame->m_pts_spherical, ringkey, signature,
                           m_lidar_range, tfm_pca_rig);

            query_frame->m_t_pca_rig = TransMatrixType(tfm_pca_rig);
            query_frame->m_signature = signature;
            query_frame->m_ringkey = ringkey;

            bool has_new_connect = false;
            LoopframePtr connect_frame = nullptr;

            /* ======================== Loop Closure ================================ */
            // fast search by ringkey
            // 先用keyring进行初步筛选
            std::vector<int> ringkey_candidates;
            search_ringkey(query_frame->m_ringkey, m_ringkeys, ringkey_candidates);

            // 候选帧不为空，搜索到候选帧
            if (!ringkey_candidates.empty())
            {
                // search by ScanContext
                int matched_idx;
                float sc_diff;
                // 找出候选关键帧中距离最小的
                search_sc(query_frame->m_signature, m_preocessed_lf, ringkey_candidates,
                          m_sc->getWidth(), matched_idx, sc_diff);
                // 小于阈值表示检测到回环
                if (sc_diff < m_scan_context_thres)
                {
                    // 获取找到的回环帧
                    auto matched_frame = m_preocessed_lf[matched_idx];
                    SYLAR_LOG_INFO(g_logger_sys)
                        << "【agent id:" << query_frame->m_lf_id << ",iid:" << query_frame->m_incoming_id << "】-"
                        << "【agent id:" << matched_frame->m_lf_id << ",iid:" << matched_frame->m_incoming_id << "】"
                        << " SC: " << sc_diff << " ";
                    // 判断两帧之间的距离有没有太远
                    if (m_mapMgr->getMap(query_frame->m_client_id) == m_mapMgr->getMap(matched_frame->m_client_id))
                    {
                        precision_t dis = (query_frame->m_twc.translation() - matched_frame->m_twc.translation()).norm();

                        // 如果很远但信服度很高也会继续执行下去 &&  sc_diff > scan_context_thres_ - 0.08
                        if (dis > DISTANCE_THRES)
                        {
                            SYLAR_LOG_WARN(g_logger_sys) << "\033[1;31mDISTANCE TOO FAR.\033[0m " << dis;
                            continue;
                        }
                    }

                    // calculate the initial tfm_matched_cur from ScanContext
                    // 复原回他们之间由于pca所变化的位姿矩阵
                    TransMatrixType T_tf_icp =
                        query_frame->m_t_pca_rig.inverse() * matched_frame->m_t_pca_rig;

                    // first try direct alignment
                    float icp_error;
                    bool icp_succ = false;

                    Eigen::Matrix4d tfm_query_matched_icp = T_tf_icp.matrix();
                    icp_succ = icp(matched_frame->m_pts_spherical, query_frame->m_pts_spherical,
                                   tfm_query_matched_icp, icp_error);
                    T_tf_icp = TransMatrixType(tfm_query_matched_icp);
                    icp_error *= ICP_ERROR_SCALE;

                    // 第一帧对于icp的要求严格一点
                    if (!icp_succ)
                    {
                        std::cout << " No" << std::endl;
                        continue;
                    }
                    else
                    {
                        std::cout << "\033[1;32m Yes \033[0m" << std::endl;
                    }
                    // 自动合并 mapmanager 执行优化和合并
                    m_mapMgr->createConstrant(matched_frame, query_frame, T_tf_icp, icp_error, sc_diff);
                }
            }
        }
        SYLAR_LOG_INFO(g_logger_sys) << "<-- END Loop Thread.";
    }

    void LoopHandler::join()
    {
        m_running = false;
        m_main_thread->join();
        SYLAR_LOG_INFO(g_logger_sys) << "Main Thread Join.";
    }
    void LoopHandler::close()
    {
        join();
    }
    void LoopHandler::pushLoopframe2Buf(LoopframePtr lf)
    {
        m_buf_lfs.push_back(lf);
    }
}