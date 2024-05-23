#include "cmd_sim3.hpp"

namespace cmd
{

    PoseGraphError::PoseGraphError(const Sophus::Sim3d &Sji,
                                   const Eigen::Matrix<double, 7, 7> &information)
        : Sji_(Sji)
    {
        // 初始化信息矩阵
        Eigen::LLT<Eigen::Matrix<double, 7, 7>> llt(information);
        sqrt_information_ = llt.matrixL();
    }

    // Si means world frame in i frame, S_iw
    /// @brief 用于计算残差和可能的雅可比矩阵
    /// @param parameters_ptr 参数的信息，i和j的sim3位姿
    /// @param residuals_ptr i和j的残差数值
    /// @param jacobians_ptr i和j的雅可比矩阵
    /// @return 返回是否计算成功
    bool PoseGraphError::Evaluate(double const *const *parameters_ptr,
                                  double *residuals_ptr, double **jacobians_ptr) const
    {
        // 两个帧 i和j的位姿
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> lie_j(*parameters_ptr);
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> lie_i(*(parameters_ptr + 1));

        Sophus::Sim3d Si = Sophus::Sim3d::exp(lie_i);
        Sophus::Sim3d Sj = Sophus::Sim3d::exp(lie_j);
        // 位姿图的误差
        Sophus::Sim3d error = Sji_ * Si * Sj.inverse();
        // Eigen::Map 貌似是可以改变 原有的输入参数的值的
        Eigen::Map<Eigen::Matrix<double, 7, 1>> residuals(residuals_ptr);
        // 得到7维的残差
        residuals = error.log();

        // 定义雅可比矩阵，将值初始化到一开始定义的Indentity中
        if (jacobians_ptr)
        {
            Eigen::Matrix<double, 7, 7> Jacobian_i;
            Eigen::Matrix<double, 7, 7> Jacobian_j;
            Eigen::Matrix<double, 7, 7> Jr = Eigen::Matrix<double, 7, 7>::Zero();

            Jr.block<3, 3>(0, 0) = Sophus::RxSO3d::hat(residuals.tail(4));
            Jr.block<3, 3>(0, 3) = Sophus::SO3d::hat(residuals.head(3));
            Jr.block<3, 1>(0, 6) = -residuals.head(3);
            Jr.block<3, 3>(3, 3) = Sophus::SO3d::hat(residuals.block<3, 1>(3, 0));
            Eigen::Matrix<double, 7, 7> I = Eigen::Matrix<double, 7, 7>::Identity();
            Jr = sqrt_information_ * (I + 0.5 * Jr + 1.0 / 12. * (Jr * Jr));

            Jacobian_i = Jr * Sj.Adj();
            Jacobian_j = -Jacobian_i;
            int k = 0;
            for (int i = 0; i < 7; i++)
            {
                for (int j = 0; j < 7; ++j)
                {
                    if (jacobians_ptr[0])
                        jacobians_ptr[0][k] = Jacobian_j(i, j);
                    if (jacobians_ptr[1])
                        jacobians_ptr[1][k] = Jacobian_i(i, j);

                    k++;
                }
            }
        }
        // 给残差值赋值上信息矩阵的权重信息
        residuals = sqrt_information_ * residuals;
        return true;
    }

    ceres::CostFunction *PoseGraphError::Create(const Sophus::Sim3d &Sji,
                                                const Eigen::Matrix<double, 7, 7> &sqrt_information)
    {
        return new PoseGraphError(Sji, sqrt_information);
    }

    /// @brief Sim3的更新方式，使用Sophus来实现
    /// @param x
    /// @param delta
    /// @param x_plus_delta
    /// @return
    bool Sim3Parameterization::Plus(const double *x, const double *delta,
                                    double *x_plus_delta) const
    {
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> lie(x);
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> delta_lie(delta);
        Sophus::Sim3d T = Sophus::Sim3d::exp(lie);
        Sophus::Sim3d delta_T = Sophus::Sim3d::exp(delta_lie);
        // 这里可以更新x_plus_delta么？？？？
        Eigen::Map<Eigen::Matrix<double, 7, 1>> x_plus_delta_lie(x_plus_delta);
        x_plus_delta_lie = (T * delta_T).log();
        return true;
    }
    /// @brief 这里没有定义雅可比的求法，而是临时给一个单位阵？为什么？？？
    /// @param x
    /// @param jacobian
    /// @return
    bool Sim3Parameterization::ComputeJacobian(const double *x,
                                               double *jacobian) const
    {
        ceres::MatrixRef(jacobian, 7, 7) = ceres::Matrix::Identity(7, 7);
        return true;
    }
}