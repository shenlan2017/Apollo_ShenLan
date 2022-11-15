/*
 * @Description: ceres residual block for lidar frontend relative pose
 * measurement
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-23 14:19:26
 */
#ifndef LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_RELATIVE_POSE_H_
#define LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_RELATIVE_POSE_H_

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <sophus/so3.hpp>

#include "glog/logging.h"

namespace sliding_window {

class FactorPRVAGRelativePose : public ceres::SizedCostFunction<6, 15, 15> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  static const int kIndexP{0};
  static const int kIndexR{3};
  static const int kIndexV{6};
  static const int kIndexA{9};
  static const int kIndexG{12};

  FactorPRVAGRelativePose(){};

  void set_measurement(const Eigen::VectorXd& m) { measurement_ = m; }

  void set_information(const Eigen::MatrixXd& I) { information_ = I; }

  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const {
    //
    // parse parameters:
    //
    // a. pose i
    Eigen::Map<const Eigen::Vector3d> pos_i(&parameters[0][kIndexP]);
    Eigen::Map<const Eigen::Vector3d> log_ori_i(&parameters[0][kIndexR]);
    const Sophus::SO3d ori_i = Sophus::SO3d::exp(log_ori_i);

    // b. pose j
    Eigen::Map<const Eigen::Vector3d> pos_j(&parameters[1][kIndexP]);
    Eigen::Map<const Eigen::Vector3d> log_ori_j(&parameters[1][kIndexR]);
    const Sophus::SO3d ori_j = Sophus::SO3d::exp(log_ori_j);

    //
    // parse measurement:
    //
    const Eigen::Vector3d& pos_ij = measurement_.block<3, 1>(kIndexP, 0);
    const Eigen::Vector3d& log_ori_ij = measurement_.block<3, 1>(kIndexR, 0);
    const Sophus::SO3d ori_ij = Sophus::SO3d::exp(log_ori_ij);

    //
    // TODO: get square root of information matrix:
    //
    Eigen::Matrix<double, 6, 6> sqrt_information =
        Eigen::LLT<Eigen::Matrix<double, 6, 6>>(information_)
            .matrixL()
            .transpose();

    //
    // TODO: compute residual:
    //
    Eigen::Map<Eigen::Matrix<double, 6, 1>> res(residuals);

    res.block<3, 1>(kIndexP, 0) = ori_i.inverse() * (pos_j - pos_i) - pos_ij;
    res.block<3, 1>(kIndexR, 0) =
        (ori_i.inverse() * ori_j * ori_ij.inverse()).log();

    //
    // TODO: compute jacobians:
    //
    if (jacobians) {
      // compute shared intermediate results:
      const Eigen::Matrix3d J_r_inv = JacobianRInv(res.block<3, 1>(kIndexR, 0));
      const Eigen::Matrix3d R_i_inv = ori_i.inverse().matrix();

      if (jacobians[0]) {
        // implement computing:
        Eigen::Map<Eigen::Matrix<double, 6, 15, Eigen::RowMajor>> jacobian_i(
            jacobians[0]);
        jacobian_i.setZero();

        jacobian_i.block<3, 3>(kIndexP, kIndexP) = -R_i_inv;

        Eigen::Vector3d pos_ji = ori_i.inverse() * (pos_j - pos_i);
        jacobian_i.block<3, 3>(kIndexP, kIndexR) =
            Sophus::SO3d::hat(pos_ji).matrix();
        jacobian_i.block<3, 3>(kIndexR, kIndexR) =
            -J_r_inv * (ori_ij * ori_j.inverse() * ori_i).matrix();

        jacobian_i = sqrt_information * jacobian_i;
      }

      if (jacobians[1]) {
        // implement computing:
        Eigen::Map<Eigen::Matrix<double, 6, 15, Eigen::RowMajor>> jacobian_j(
            jacobians[1]);
        jacobian_j.setZero();

        jacobian_j.block<3, 3>(kIndexP, kIndexP) = R_i_inv;
        jacobian_j.block<3, 3>(kIndexR, kIndexR) = J_r_inv * ori_ij.matrix();

        jacobian_j = sqrt_information * jacobian_j;
      }
    }

    //
    // TODO: correct residual by square root of information matrix:
    //
    res = sqrt_information * res;

    return true;
  }

  void CheckJacobian(double** parameters) {
    double* residuals = new double[6];
    double** jacobians = new double*[2];
    jacobians[0] = new double[6 * 15];
    jacobians[1] = new double[6 * 15];
    Evaluate(parameters, residuals, jacobians);

    std::cout << "my jacobian: " << std::endl;
    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 15, Eigen::RowMajor>>(
                     jacobians[0])
              << std::endl;
    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 15, Eigen::RowMajor>>(
                     jacobians[1])
              << std::endl;
    std::cout << "my residual: " << std::endl;
    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 1>>(residuals).transpose()
              << std::endl;

    // a. pose i
    Eigen::Map<const Eigen::Vector3d> pos_i(&parameters[0][kIndexP]);
    Eigen::Map<const Eigen::Vector3d> log_ori_i(&parameters[0][kIndexR]);
    const Sophus::SO3d ori_i = Sophus::SO3d::exp(log_ori_i);
    Eigen::Map<const Eigen::Vector3d> vel_i(&parameters[0][kIndexV]);
    Eigen::Map<const Eigen::Vector3d> b_a_i(&parameters[0][kIndexA]);
    Eigen::Map<const Eigen::Vector3d> b_g_i(&parameters[0][kIndexG]);

    // b. pose j
    Eigen::Map<const Eigen::Vector3d> pos_j(&parameters[1][kIndexP]);
    Eigen::Map<const Eigen::Vector3d> log_ori_j(&parameters[1][kIndexR]);
    const Sophus::SO3d ori_j = Sophus::SO3d::exp(log_ori_j);
    Eigen::Map<const Eigen::Vector3d> vel_j(&parameters[1][kIndexV]);
    Eigen::Map<const Eigen::Vector3d> b_a_j(&parameters[1][kIndexA]);
    Eigen::Map<const Eigen::Vector3d> b_g_j(&parameters[1][kIndexG]);

    const Eigen::Vector3d& pos_ij = measurement_.block<3, 1>(kIndexP, 0);
    const Eigen::Vector3d& log_ori_ij = measurement_.block<3, 1>(kIndexR, 0);
    const Sophus::SO3d ori_ij = Sophus::SO3d::exp(log_ori_ij);

    Eigen::Matrix<double, 6, 6> sqrt_information =
        Eigen::LLT<Eigen::Matrix<double, 6, 6>>(information_)
            .matrixL()
            .transpose();

    Eigen::Matrix<double, 6, 1> residual;

    residual.block<3, 1>(kIndexP, 0) =
        ori_i.inverse() * (pos_j - pos_i) - pos_ij;
    residual.block<3, 1>(kIndexR, 0) =
        (ori_i.inverse() * ori_j * ori_ij.inverse()).log();
    residual = sqrt_information * residual;

    std::cout << "residual: " << std::endl;
    std::cout << residual.transpose() << std::endl;

    const double eps = 1e-6;
    Eigen::Matrix<double, 6, 30> num_jacobian;
    for (int k = 0; k < 30; ++k) {
      // pose i
      Eigen::Vector3d pos_i_num = pos_i;
      Sophus::SO3d ori_i_num = ori_i;
      Eigen::Vector3d vel_i_num = vel_i;
      Eigen::Vector3d b_a_i_num = b_a_i;
      Eigen::Vector3d b_g_i_num = b_g_i;
      // pose j
      Eigen::Vector3d pos_j_num = pos_j;
      Sophus::SO3d ori_j_num = ori_j;
      Eigen::Vector3d vel_j_num = vel_j;
      Eigen::Vector3d b_a_j_num = b_a_j;
      Eigen::Vector3d b_g_j_num = b_g_j;

      int a = k / 3, b = k % 3;
      Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

      if (a == 0)
        pos_i_num = pos_i_num + delta;
      else if (a == 1)
        ori_i_num = ori_i_num * Sophus::SO3d::exp(delta);
      else if (a == 2)
        vel_i_num = vel_i_num + delta;
      else if (a == 3)
        b_a_i_num = b_a_i_num + delta;
      else if (a == 4)
        b_g_i_num = b_g_i_num + delta;
      else if (a == 5)
        pos_j_num = pos_j_num + delta;
      else if (a == 6)
        ori_j_num = ori_j_num * Sophus::SO3d::exp(delta);
      else if (a == 7)
        vel_j_num = vel_j_num + delta;
      else if (a == 8)
        b_a_j_num = b_a_j_num + delta;
      else if (a == 9)
        b_g_j_num = b_g_j_num + delta;

      Eigen::Matrix<double, 6, 1> tmp_residual;
      tmp_residual.block<3, 1>(kIndexP, 0) =
          ori_i_num.inverse() * (pos_j_num - pos_i_num) - pos_ij;
      tmp_residual.block<3, 1>(kIndexR, 0) =
          (ori_i_num.inverse() * ori_j_num * ori_ij.inverse()).log();
      tmp_residual = sqrt_information * tmp_residual;

      num_jacobian.col(k) = (tmp_residual - residual) / eps;
    }
    std::cout << "num jacobian:" << std::endl;
    std::cout << num_jacobian.block<6, 15>(0, 0) << std::endl;
    std::cout << num_jacobian.block<6, 15>(0, 15) << std::endl << std::endl;
  }

 private:
  static Eigen::Matrix3d JacobianRInv(const Eigen::Vector3d& w) {
    Eigen::Matrix3d J_r_inv = Eigen::Matrix3d::Identity();

    const double theta = w.norm();

    if (theta > 1e-5) {
      Eigen::Vector3d a = w.normalized();
      Eigen::Matrix3d a_hat = Sophus::SO3d::hat(a);
      double theta_half = 0.5 * theta;
      double cot_theta = 1.0 / tan(theta_half);

      J_r_inv = theta_half * cot_theta * J_r_inv +
                (1.0 - theta_half * cot_theta) * a * a.transpose() +
                theta_half * a_hat;
    }

    return J_r_inv;
  }

  Eigen::VectorXd measurement_;
  Eigen::MatrixXd information_;
  bool output_flag_{true};
};

}  // namespace sliding_window

#endif  // LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_RELATIVE_POSE_H_
