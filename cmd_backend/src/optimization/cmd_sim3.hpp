#include <iostream>
#include <stdio.h>

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <ceres/ceres.h>
#include <sophus/sim3.hpp>

namespace cmd
{
    /// @brief ceres::SizedCostFunction<7, 7, 7>  7个残差项，7个参数块，最后一个7没有看懂
    class PoseGraphError : public ceres::SizedCostFunction<7, 7, 7>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        PoseGraphError(const Sophus::Sim3d &Sji,
                       const Eigen::Matrix<double, 7, 7> &information);

        // Si means world frame in i frame, S_iw
        /// @brief 用于计算残差和可能的雅可比矩阵
        /// @param parameters_ptr 参数的信息，i和j的sim3位姿
        /// @param residuals_ptr i和j的残差数值
        /// @param jacobians_ptr i和j的雅可比矩阵
        /// @return 返回是否计算成功
        virtual bool Evaluate(double const *const *parameters_ptr,
                              double *residuals_ptr, double **jacobians_ptr) const;

        /// @brief 创建餐残差项的接口
        /// @param Sji
        /// @param sqrt_information
        /// @return
        static ceres::CostFunction *Create(const Sophus::Sim3d &Sji,
                                           const Eigen::Matrix<double, 7, 7> &sqrt_information);

    private:
        const Sophus::Sim3d Sji_;
        Eigen::Matrix<double, 7, 7> sqrt_information_;
    };

    // 定义Sim3这个参数类型 包括维度和加减方式，还有雅可比的求导方式
    class CERES_EXPORT Sim3Parameterization : public ceres::LocalParameterization
    {
    public:
        virtual ~Sim3Parameterization() {}
        virtual bool Plus(const double *x, const double *delta,
                          double *x_plus_delta) const;
        virtual bool ComputeJacobian(const double *x, double *jacobian) const;
        virtual int GlobalSize() const { return 7; }
        virtual int LocalSize() const { return 7; }

        static int Size() { return 7; }
    };
}