/*
 * @Description: ceres residual block for sliding window marginalization
 * @Author: Ge Yao
 * @Date: 2021-01-05 21:57:49
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-23 11:26:35
 */
#ifndef LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_MARGINALIZATION_H_
#define LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_MARGINALIZATION_H_

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <sophus/so3.hpp>

#include "glog/logging.h"

namespace sliding_window {

constexpr double kEps = 1e-8;

class FactorPRVAGMarginalization : public ceres::SizedCostFunction<15, 15> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  static const int kIndexM{0};
  static const int kIndexR{15};

  FactorPRVAGMarginalization() {
    H_ = Eigen::MatrixXd::Zero(30, 30);
    b_ = Eigen::VectorXd::Zero(30);

    J_ = Eigen::MatrixXd::Zero(15, 15);
    e_ = Eigen::VectorXd::Zero(15);
  }

  void SetResMapMatchingPose(const ceres::CostFunction* residual,
                             const std::vector<double*>& parameter_blocks) {
    // init:
    ResidualBlockInfo res_map_matching_pose(residual, parameter_blocks);
    Eigen::VectorXd residuals;
    std::vector<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        jacobians;

    // compute: 提取残差 jacobian
    Evaluate(res_map_matching_pose, residuals, jacobians);
    // 由于地图约束只于最老帧相关 只用取一个jacobian
    const Eigen::MatrixXd& J_m = jacobians.at(0);

    //
    // TODO: Update H:
    //
    // a. H_mm:
    H_.block<15, 15>(0, 0) += J_m.transpose() * J_m;

    //
    // TODO: Update b:
    //
    // a. b_m:
    b_.block<15, 1>(0, 0) += J_m.transpose() * residuals;
  }

  void SetResRelativePose(const ceres::CostFunction* residual,
                          const std::vector<double*>& parameter_blocks) {
    // init:
    ResidualBlockInfo res_relative_pose(residual, parameter_blocks);
    Eigen::VectorXd residuals;
    std::vector<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        jacobians;

    // compute:
    Evaluate(res_relative_pose, residuals, jacobians);
    // 里程计约束与两帧相关 所以要取两个jacobian
    const Eigen::MatrixXd& J_m = jacobians.at(0);
    const Eigen::MatrixXd& J_r = jacobians.at(1);

    //
    // TODO: Update H:
    //
    // a. H_mm:
    H_.block<15, 15>(kIndexM, kIndexM) += J_m.transpose() * J_m;
    // b. H_mr:
    H_.block<15, 15>(kIndexM, kIndexR) += J_m.transpose() * J_r;
    // c. H_rm:
    H_.block<15, 15>(kIndexR, kIndexM) += J_r.transpose() * J_m;
    // d. H_rr:
    H_.block<15, 15>(kIndexR, kIndexR) += J_r.transpose() * J_r;

    //
    // TODO: Update b:
    //
    // a. b_m:
    b_.block<15, 1>(kIndexM, 0) += J_m.transpose() * residuals;
    // a. b_r:
    b_.block<15, 1>(kIndexR, 0) += J_r.transpose() * residuals;
  }

  void SetResIMUPreIntegration(const ceres::CostFunction* residual,
                               const std::vector<double*>& parameter_blocks) {
    // init:
    ResidualBlockInfo res_imu_pre_integration(residual, parameter_blocks);
    Eigen::VectorXd residuals;
    std::vector<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        jacobians;

    // compute:
    Evaluate(res_imu_pre_integration, residuals, jacobians);
    const Eigen::MatrixXd& J_m = jacobians.at(0);
    const Eigen::MatrixXd& J_r = jacobians.at(1);

    //
    // TODO: Update H:
    //
    // a. H_mm:
    H_.block<15, 15>(kIndexM, kIndexM) += J_m.transpose() * J_m;
    // b. H_mr:
    H_.block<15, 15>(kIndexM, kIndexR) += J_m.transpose() * J_r;
    // c. H_rm:
    H_.block<15, 15>(kIndexR, kIndexM) += J_r.transpose() * J_m;
    // d. H_rr:
    H_.block<15, 15>(kIndexR, kIndexR) += J_r.transpose() * J_r;

    //
    // TODO: Update b:
    //
    // a. b_m:
    b_.block<15, 1>(kIndexM, 0) += J_m.transpose() * residuals;
    // a. b_r:
    b_.block<15, 1>(kIndexR, 0) += J_r.transpose() * residuals;
  }

  void Marginalize(const double* raw_param_r_0) {
    // TODO: implement marginalization logic
    // 保存倒数第二帧的参数
    Eigen::Map<const Eigen::Matrix<double, 15, 1>> x_0(raw_param_r_0);
    x_0_ = x_0;

    // 舒尔补
    // 保证H_mm对称
    Eigen::Matrix<double, 15, 15> H_mm =
        0.5 * (H_.block<15, 15>(kIndexM, kIndexM) +
               H_.block<15, 15>(kIndexM, kIndexM).transpose());
    // Eigen::Matrix<double, 15, 15> H_mm = H_.block<15, 15>(kIndexM, kIndexM);
    Eigen::Matrix<double, 15, 15> H_mr = H_.block<15, 15>(kIndexM, kIndexR);
    Eigen::Matrix<double, 15, 15> H_rm = H_.block<15, 15>(kIndexR, kIndexM);
    Eigen::Matrix<double, 15, 15> H_rr = H_.block<15, 15>(kIndexR, kIndexR);

    Eigen::Matrix<double, 15, 1> b_m = b_.block<15, 1>(kIndexM, 0);
    Eigen::Matrix<double, 15, 1> b_r = b_.block<15, 1>(kIndexR, 0);

    // vins-mono的求逆
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(H_mm);
    Eigen::MatrixXd H_mm_inv =
        saes.eigenvectors() *
        Eigen::VectorXd((saes.eigenvalues().array() > kEps)
                            .select(saes.eigenvalues().array().inverse(), 0))
            .asDiagonal() *
        saes.eigenvectors().transpose();

    // // 普通求逆
    // Eigen::MatrixXd H_mm_inv = H_mm.inverse();

    Eigen::MatrixXd H_marg = H_rr - H_rm * H_mm_inv * H_mr;
    Eigen::MatrixXd b_marg = b_r - H_rm * H_mm_inv * b_m;

    // 通过H和b反算出jacoiban和residual 参考vins-mono
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(H_marg);
    Eigen::VectorXd S =
        Eigen::VectorXd((saes2.eigenvalues().array() > kEps)
                            .select(saes2.eigenvalues().array(), 0));
    Eigen::VectorXd S_inv =
        Eigen::VectorXd((saes2.eigenvalues().array() > kEps)
                            .select(saes2.eigenvalues().array().inverse(), 0));

    Eigen::VectorXd S_sqrt = S.cwiseSqrt();
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

    J_ = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
    e_ = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b_marg;
  }

  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const {
    //
    // parse parameters:
    //
    Eigen::Map<const Eigen::Matrix<double, 15, 1>> x(parameters[0]);
    Eigen::VectorXd dx = x - x_0_;

    //
    // TODO: compute residual:
    //
    Eigen::Map<Eigen::Matrix<double, 15, 1>> res(residuals);
    res = e_ + J_ * dx;

    //
    // TODO: compute jacobian:
    //
    if (jacobians) {
      if (jacobians[0]) {
        // implement computing:
        Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> jacobian(
            jacobians[0]);
        jacobian.setZero();

        jacobian = J_;
      }
    }

    return true;
  }

 private:
  struct ResidualBlockInfo {
    const ceres::CostFunction* residual = nullptr;
    std::vector<double*> parameter_blocks;

    ResidualBlockInfo(void) {}

    ResidualBlockInfo(const ceres::CostFunction* _residual,
                      const std::vector<double*>& _parameter_blocks)
        : residual(_residual)
        , parameter_blocks(_parameter_blocks) {}
  };

  static void Evaluate(
      ResidualBlockInfo& residual_info,
      Eigen::VectorXd& residuals,
      std::vector<
          Eigen::
              Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>&
          jacobians) {
    // init residual output: 计算有多少个残差块
    const int D = static_cast<int>(residual_info.residual->num_residuals());
    residuals.resize(D);  // 变成D*1的向量

    // init jacobians output:
    // block_sizes中的size表示有多少个参数块，int表示参数块中的维度
    const std::vector<int> block_sizes =
        residual_info.residual->parameter_block_sizes();
    const int N = static_cast<int>(block_sizes.size());

    // 在堆中开辟一个N维的数据，raw_jacobians指向这个N维数据，存放的数据是double*
    double** raw_jacobians = new double*[N];
    jacobians.resize(N);  // 变成N维的vector

    // create raw pointer adaptor:
    for (int i = 0; i < N; i++) {
      // 将vector中的元素变成D*size维的矩阵
      jacobians[i].resize(D, block_sizes[i]);
      // 将jacobians[i]的指针存放在raw_jacobians[i]里
      raw_jacobians[i] = jacobians[i].data();
    }

    // 提取残差 jacobian
    residual_info.residual->Evaluate(
        residual_info.parameter_blocks.data(), residuals.data(), raw_jacobians);
  }

  Eigen::MatrixXd H_;
  Eigen::VectorXd b_;

  Eigen::MatrixXd J_;
  Eigen::VectorXd e_;

  Eigen::VectorXd x_0_;
};

}  // namespace sliding_window

#endif  // LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_MARGINALIZATION_H_
