#pragma once
#include "typedefs.hpp"

// SERIALIZATION
#include <cereal/cereal.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/types/concepts/pair_associative_container.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/access.hpp>

namespace cmd
{

    class Calibration
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        KType m_k;
        Size m_img_dims; // w  h
        Vec4Type m_dist_coeffs;
        Vec4Type m_intrinsics; // fx fy cx cy
        precision_t m_pyr_level;

    public:
        // const 类型只可以调用 const函数
        precision_t getFx() const { return m_intrinsics[0]; }
        precision_t getFy() const { return m_intrinsics[1]; }
        precision_t getCx() const { return m_intrinsics[2]; }
        precision_t getCy() const { return m_intrinsics[3]; }

    public:
        std::string dump();

        template <class Archive>
        void serialize(Archive &archive)
        {
            archive(img_dims, dist_coeffs, intrinsics, k);
        }
    };

    class MsgPoint
    {

        float u;
        float v;
        float idepth_scaled;
        float maxRelBaseline;
        float idepth_hessian;
        template <class Archive>

        void load(Archive &archive)
        {
            archive(u, v, idepth_scaled, maxRelBaseline, idepth_hessian);
        }
        template <class Archive>
        void save(Archive &archive) const
        {
            archive(u, v, idepth_scaled, maxRelBaseline, idepth_hessian);
        }
    };

    class MsgLoopframe
    {
    public:
        typedef std::shared_ptr<MsgLoopframe> Ptr;
        // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        MsgLoopframe();

        // Interfaces
        void setMsgType(int msg_size);
        void setMsgType(MsgType msgtype);
        std::string dump();

    public:
        // Infrastructure
        bool m_is_update_msg;

        // Identifier
        int_t m_kf_id;
        int_t m_client_id;
        int_t m_incoming_id; // increasing id, for ground truth
        uint64_t m_timestamp;
        MsgType m_msgtype; // 当前 msg 的类型

        Calibration m_calib; // DSO 参数

        // image 
        int_t m_img_x_min; // 的 xy范围，最大和最小值
        int_t m_img_y_min;
        int_t m_img_x_max;
        int_t m_img_y_max;

        TransMatrixType m_twc; // 本帧自身的位姿 quan+xyz double）

        // LoopEdge
        std::vector<idpair> m_id_le_refs;          // loopedge id 与 client_id？？？
        std::vector<TransQuatType> m_measurements; // 与 上一个关键帧之间的变换矩阵

        // pgo
        precision_t m_ab_exposure;   // 光度
        precision_t m_dso_error;     // dso 滑动窗口误差
        precision_t m_scale_error;   // 尺度误差（好像 stereo-DSO 没有作用）
        precision_t m_pose_r_weight; // 普通的权重参数

        std::vector<MsgPointPtr> m_msg_points; // 边缘化之后的 pointHessians，有深度信息了

    protected:
        friend class cereal::access; // Serialization
        /**
         * @brief 定义需要序列化的变量
         * @description:
         * @return {*}
         */
        template <class Archive>
        auto save(Archive &archive) const -> void
        {
            if (save_to_file)
            {
                archive(kf_id, client_id, incoming_id, timestamp,
                        calibration,
                        img_dim_x_min, img_dim_y_min, img_dim_x_max, img_dim_y_max, ab_exposure,
                        pts_spherical_, dso_pts, dIps_,
                        T_w_c, id_predecessor, id_reference,
                        id_le_refs, measurements, msg_type,
                        dso_error, scale_error, pose_r_weight,
                        pointHessians,
                        pointHessiansMarginalized,
                        pointHessiansOut);
            }
            else if (is_update_msg)
            {
                archive(kf_id, client_id, incoming_id, timestamp,
                        calibration,
                        img_dim_x_min, img_dim_y_min, img_dim_x_max, img_dim_y_max, ab_exposure,
                        pts_spherical_, dso_pts, dIps_,
                        T_w_c, id_predecessor, id_reference,
                        id_le_refs, measurements, msg_type,
                        dso_error, scale_error, pose_r_weight,
                        pointHessians,
                        pointHessiansMarginalized,
                        pointHessiansOut);
            }
            else
            {
                assert(msg_type.size() == 5);
                archive(kf_id, client_id, incoming_id, timestamp,
                        calibration,
                        img_dim_x_min, img_dim_y_min, img_dim_x_max, img_dim_y_max, ab_exposure,
                        pts_spherical_, dso_pts, dIps_,
                        T_w_c, id_predecessor, id_reference,
                        id_le_refs, measurements, msg_type,
                        dso_error, scale_error, pose_r_weight,
                        pointHessians,
                        pointHessiansMarginalized,
                        pointHessiansOut);
            }
        }

        /**
         * @brief 需要序列化的变量，应该和 save 是对应的
         * @description:
         * @return {*}
         */
        template <class Archive>
        auto load(Archive &archive) -> void
        {
            if (save_to_file)
            {
                archive(kf_id, client_id, incoming_id, timestamp,
                        calibration,
                        img_dim_x_min, img_dim_y_min, img_dim_x_max, img_dim_y_max, ab_exposure,
                        pts_spherical_, dso_pts, dIps_,
                        T_w_c, id_predecessor, id_reference,
                        id_le_refs, measurements, msg_type,
                        dso_error, scale_error, pose_r_weight,
                        pointHessians,
                        pointHessiansMarginalized,
                        pointHessiansOut);
            }
            else if (is_update_msg)
            {
                archive(kf_id, client_id, incoming_id, timestamp,
                        calibration,
                        img_dim_x_min, img_dim_y_min, img_dim_x_max, img_dim_y_max, ab_exposure,
                        pts_spherical_, dso_pts, dIps_,
                        T_w_c, id_predecessor, id_reference,
                        id_le_refs, measurements, msg_type,
                        dso_error, scale_error, pose_r_weight,
                        pointHessians,
                        pointHessiansMarginalized,
                        pointHessiansOut);
            }
            else
            {
                assert(msg_type.size() == 5);
                archive(kf_id, client_id, incoming_id, timestamp,
                        calibration,
                        img_dim_x_min, img_dim_y_min, img_dim_x_max, img_dim_y_max, ab_exposure,
                        pts_spherical_, dso_pts, dIps_,
                        T_w_c, id_predecessor, id_reference,
                        id_le_refs, measurements, msg_type,
                        dso_error, scale_error, pose_r_weight,
                        pointHessians,
                        pointHessiansMarginalized,
                        pointHessiansOut);
            }
        }
    };

} // end ns

namespace cereal
{
    // save and load function for Eigen::Matrix type
    //  对于保存Matrix的重写
    template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    inline
        typename std::enable_if<traits::is_output_serializable<BinaryData<_Scalar>, Archive>::value, void>::type
        save(Archive &ar, const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &matrix)
    {
        const std::int32_t rows = static_cast<std::int32_t>(matrix.rows());
        const std::int32_t cols = static_cast<std::int32_t>(matrix.cols());
        ar(rows);
        ar(cols);
        ar(binary_data(matrix.data(), rows * cols * sizeof(_Scalar)));
    }

    template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    inline
        typename std::enable_if<traits::is_input_serializable<BinaryData<_Scalar>, Archive>::value, void>::type
        load(Archive &ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &matrix)
    {
        std::int32_t rows;
        std::int32_t cols;
        ar(rows);
        ar(cols);

        matrix.resize(rows, cols);

        ar(binary_data(matrix.data(), static_cast<std::size_t>(rows * cols * sizeof(_Scalar))));
    }

} /* namespace cereal */
