#pragma once
#include "typedefs.hpp"

// SERIALIZATION
#include <cereal/access.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/concepts/pair_associative_container.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/vector.hpp>

namespace cmd {

class Calibration {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  KType m_k;
  Size m_img_dims;  // w  h
  Vec4Type m_dist_coeffs;
  Vec4Type m_intrinsics;  // fx fy cx cy

 public:
  Calibration();
  // const 类型只可以调用 const函数
  precision_t getFx() const { return m_intrinsics[0]; }
  precision_t getFy() const { return m_intrinsics[1]; }
  precision_t getCx() const { return m_intrinsics[2]; }
  precision_t getCy() const { return m_intrinsics[3]; }
  void setIntrinsics(precision_t fx, precision_t fy, precision_t cx,
                     precision_t cy) {
    m_k = KType::Identity();
    m_k(0, 0) = fx;
    m_k(1, 1) = fy;
    m_k(0, 2) = cx;
    m_k(1, 2) = cy;
    m_intrinsics = {fx, fy, cx, cy};
  }

 public:
  std::string dump();

  template <class Archive>
  void load(Archive &archive) {
    archive(m_img_dims, m_dist_coeffs, m_intrinsics, m_k);
  }

  template <class Archive>
  void save(Archive &archive) const {
    archive(m_img_dims, m_dist_coeffs, m_intrinsics, m_k);
  }
};

struct MsgPoint {
  float m_u;
  float m_v;
  float m_idepth_scaled;
  float m_maxRelBaseline;
  float m_idepth_hessian;

  template <class Archive>
  void load(Archive &archive) {
    archive(m_u, m_v, m_idepth_scaled, m_maxRelBaseline, m_idepth_hessian);
  }

  template <class Archive>
  void save(Archive &archive) const {
    archive(m_u, m_v, m_idepth_scaled, m_maxRelBaseline, m_idepth_hessian);
  }
};

class MsgLoopframe {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MsgLoopframe();
  MsgLoopframe(MsgType msgtype);
  ~MsgLoopframe();

  // Interfaces
  void setMsgType(int msg_size);
  void setMsgType(MsgType msgtype);
  std::string dump();

 public:
  // Infrastructure
  bool m_is_update_msg;

  // Identifier
  int_t m_lf_id;
  int_t m_client_id;
  int_t m_incoming_id;  // increasing id, for ground truth
  uint64_t m_timestamp;
  MsgType m_msgtype;  // 当前 msg 的类型 [msg 长度, SentOnce, id.first,
                      // id.second，类型（lf or point）]

  Calibration m_calib;  // DSO 参数

  EigenMatrix m_twc;
  // LoopEdge
  std::vector<int_t> m_ref_id;  // idpair：ref ,顺序是从近到远
  EigenMatrixVector m_ref_cf;  // 与 上一个关键帧之间的变换矩阵 m_tf

  // image
  int_t m_img_x_min;  // 的 xy范围，最大和最小值
  int_t m_img_y_min;
  int_t m_img_x_max;
  int_t m_img_y_max;

  // pgo
  precision_t m_ab_exposure;  // 光度
  precision_t m_dso_error;    // dso 滑动窗口误差
  precision_t m_scale_error;  // 尺度误差（好像 stereo-DSO 没有作用）
  precision_t m_pose_r_weight;  // 普通的权重参数

  std::vector<MsgPoint, Eigen::aligned_allocator<MsgPoint>>
      m_msg_points;  // 边缘化之后的 pointHessians，有深度信息了
 protected:
  friend class cereal::access;  // Serialization

  template <class Archive>
  auto save(Archive &archive) const -> void {
    if (m_is_update_msg) {
      archive(m_lf_id, m_client_id, m_incoming_id, m_timestamp, m_calib, m_twc,
              m_ref_id, m_ref_cf, m_img_x_min, m_img_y_min, m_img_x_max,
              m_img_y_max, m_ab_exposure, m_dso_error, m_scale_error,
              m_pose_r_weight, m_msg_points);
    } else {
      assert(m_msgtype.size() == 5);
      archive(m_lf_id, m_client_id, m_incoming_id, m_timestamp, m_calib, m_twc,
              m_ref_id, m_ref_cf, m_img_x_min, m_img_y_min, m_img_x_max,
              m_img_y_max, m_ab_exposure, m_dso_error, m_scale_error,
              m_pose_r_weight, m_msg_points);
    }
  }

  /**
   * @brief 需要序列化的变量，应该和 save 是对应的
   * @description:
   * @return {*}
   */
  template <class Archive>
  auto load(Archive &archive) -> void {
    if (m_is_update_msg) {
      archive(m_lf_id, m_client_id, m_incoming_id, m_timestamp, m_calib, m_twc,
              m_ref_id, m_ref_cf, m_img_x_min, m_img_y_min, m_img_x_max,
              m_img_y_max, m_ab_exposure, m_dso_error, m_scale_error,
              m_pose_r_weight, m_msg_points);
    } else {
      assert(m_msgtype.size() == 5);
      archive(m_lf_id, m_client_id, m_incoming_id, m_timestamp, m_calib, m_twc,
              m_ref_id, m_ref_cf, m_img_x_min, m_img_y_min, m_img_x_max,
              m_img_y_max, m_ab_exposure, m_dso_error, m_scale_error,
              m_pose_r_weight, m_msg_points);
    }
  }
};

}  // namespace cmd

namespace cereal {
// save and load function for Eigen::Matrix type
//  对于保存Matrix的重写
template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options,
          int _MaxRows, int _MaxCols>
inline typename std::enable_if<
    traits::is_output_serializable<BinaryData<_Scalar>, Archive>::value,
    void>::type
save(Archive &ar,
     const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>
         &matrix) {
  const std::int32_t rows = static_cast<std::int32_t>(matrix.rows());
  const std::int32_t cols = static_cast<std::int32_t>(matrix.cols());
  ar(rows);
  ar(cols);
  ar(binary_data(matrix.data(), rows * cols * sizeof(_Scalar)));
}

template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options,
          int _MaxRows, int _MaxCols>
inline typename std::enable_if<
    traits::is_input_serializable<BinaryData<_Scalar>, Archive>::value,
    void>::type
load(Archive &ar,
     Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>
         &matrix) {
  std::int32_t rows;
  std::int32_t cols;
  ar(rows);
  ar(cols);

  matrix.resize(rows, cols);

  ar(binary_data(matrix.data(),
                 static_cast<std::size_t>(rows * cols * sizeof(_Scalar))));
}
// 定义 TransMatrixType 的序列化方式
template <class Archive>
void serialize(Archive &ar, cmd::TransMatrixType &trans) {
  int32_t rows = 4;
  int32_t cols = 4;
  // ar(binary_data(trans.data(), static_cast<std::size_t>(rows * cols *
  // sizeof(double))));
  ar(binary_data(trans.data(),
                 static_cast<std::size_t>(rows * cols * sizeof(double))));
}
// template <class Archive>
// void serialize(Archive &ar, cmd::EigenMatrix &trans)
// {
//     // const std::int32_t rows = static_cast<std::int32_t>(matrix.rows());
//     // const std::int32_t cols = static_cast<std::int32_t>(matrix.cols());
//     // ar(binary_data(trans.data(), static_cast<std::size_t>(rows * cols *
//     sizeof(double))));
//     ar(binary_data(trans(0,0),trans(0,1),trans(0,2),trans(0,3)
//                     ,trans(1,0),trans(1,1),trans(1,2),trans(1,3)
//                     ,trans(2,0),trans(2,1),trans(2,2),trans(2,3)
//                     ,trans(3,0),trans(3,1),trans(3,2),trans(3,3));
// }

} /* namespace cereal */
