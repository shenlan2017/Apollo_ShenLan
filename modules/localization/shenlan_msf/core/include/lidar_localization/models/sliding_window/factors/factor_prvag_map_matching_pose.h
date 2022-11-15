/*
 * @Description: ceres residual block for map matching pose measurement
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-23 11:27:56
 */
#ifndef LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_MAP_MATCHING_POSE_H_
#define LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_MAP_MATCHING_POSE_H_

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <sophus/so3.hpp>

#include "glog/logging.h"

namespace sliding_window {

class FactorPRVAGMapMatchingPose : public ceres::SizedCostFunction<6, 15> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  static const int kIndexP{0};
  static const int kIndexR{3};
  static const int kIndexV{6};
  static const int kIndexA{9};
  static const int kIndexG{12};

  FactorPRVAGMapMatchingPose(){};

  void set_measurement(const Eigen::VectorXd& m) { measurement_ = m; }

  void set_information(const Eigen::MatrixXd& I) { information_ = I; }

  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const {
    //
    // parse parameters:
    //
    // pose
    Eigen::Map<const Eigen::Vector3d> pos(&parameters[0][kIndexP]);
    Eigen::Map<const Eigen::Vector3d> log_ori(&parameters[0][kIndexR]);
    const Sophus::SO3d ori = Sophus::SO3d::exp(log_ori);

    //
    // parse measurement:
    //
    const Eigen::Vector3d& pos_prior = measurement_.block<3, 1>(kIndexP, 0);
    const Eigen::Vector3d& log_ori_prior = measurement_.block<3, 1>(kIndexR, 0);
    const Sophus::SO3d ori_prior = Sophus::SO3d::exp(log_ori_prior);

    //
    // TODO: get square root of information matrix:
    //
    // fix: 这里有问题
    Eigen::Matrix<double, 6, 6> sqrt_information =
        Eigen::LLT<Eigen::Matrix<double, 6, 6>>(information_)
            .matrixL()
            .transpose();

    //
    // TODO: compute residual:
    //
    Eigen::Map<Eigen::Matrix<double, 6, 1>> res(residuals);
    res.block<3, 1>(kIndexP, 0) = pos - pos_prior;
    res.block<3, 1>(kIndexR, 0) = (ori * ori_prior.inverse()).log();

    //
    // TODO: compute jacobians:
    //
    if (jacobians) {
      if (jacobians[0]) {
        // implement jacobian computing:
        Eigen::Map<Eigen::Matrix<double, 6, 15, Eigen::RowMajor>> jacobian(
            jacobians[0]);
        jacobian.setZero();

        jacobian.block<3, 3>(kIndexP, kIndexP) = Eigen::Matrix3d::Identity();
        jacobian.block<3, 3>(kIndexR, kIndexR) =
            JacobianRInv(res.block<3, 1>(kIndexR, 0)) * ori_prior.matrix();

        jacobian = sqrt_information * jacobian;
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
    double** jacobians = new double*[1];
    jacobians[0] = new double[6 * 15];
    Evaluate(parameters, residuals, jacobians);

    std::cout << "my jacobian: " << std::endl;
    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 15, Eigen::RowMajor>>(
                     jacobians[0])
              << std::endl;
    std::cout << "my residual: " << std::endl;
    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 1>>(residuals).transpose()
              << std::endl;

    // a. pose i
    Eigen::Map<const Eigen::Vector3d> pos(&parameters[0][kIndexP]);
    Eigen::Map<const Eigen::Vector3d> log_ori(&parameters[0][kIndexR]);
    const Sophus::SO3d ori = Sophus::SO3d::exp(log_ori);
    Eigen::Map<const Eigen::Vector3d> vel(&parameters[0][kIndexV]);
    Eigen::Map<const Eigen::Vector3d> b_a(&parameters[0][kIndexA]);
    Eigen::Map<const Eigen::Vector3d> b_g(&parameters[0][kIndexG]);

    const Eigen::Vector3d& pos_prior = measurement_.block<3, 1>(kIndexP, 0);
    const Eigen::Vector3d& log_ori_prior = measurement_.block<3, 1>(kIndexR, 0);
    const Sophus::SO3d ori_prior = Sophus::SO3d::exp(log_ori_prior);

    Eigen::Matrix<double, 6, 6> sqrt_information =
        Eigen::LLT<Eigen::Matrix<double, 6, 6>>(information_)
            .matrixL()
            .transpose();

    Eigen::Matrix<double, 6, 1> residual;

    residual.block<3, 1>(kIndexP, 0) = pos - pos_prior;
    residual.block<3, 1>(kIndexR, 0) = (ori * ori_prior.inverse()).log();
    residual = sqrt_information * residual;

    std::cout << "residual: " << std::endl;
    std::cout << residual.transpose() << std::endl;

    const double eps = 1e-6;
    Eigen::Matrix<double, 6, 15> num_jacobian;
    for (int k = 0; k < 15; ++k) {
      // pose i
      Eigen::Vector3d pos_num = pos;
      Sophus::SO3d ori_num = ori;
      Eigen::Vector3d vel_num = vel;
      Eigen::Vector3d b_a_num = b_a;
      Eigen::Vector3d b_g_num = b_g;

      int a = k / 3, b = k % 3;
      Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

      if (a == 0)
        pos_num = pos_num + delta;
      else if (a == 1)
        ori_num = ori_num * Sophus::SO3d::exp(delta);
      else if (a == 2)
        vel_num = vel_num + delta;
      else if (a == 3)
        b_a_num = b_a_num + delta;
      else if (a == 4)
        b_g_num = b_g_num + delta;

      Eigen::Matrix<double, 6, 1> tmp_residual;
      tmp_residual.block<3, 1>(kIndexP, 0) = pos_num - pos_prior;
      tmp_residual.block<3, 1>(kIndexR, 0) =
          (ori_num * ori_prior.inverse()).log();
      tmp_residual = sqrt_information * tmp_residual;

      num_jacobian.col(k) = (tmp_residual - residual) / eps;
    }
    std::cout << "num jacobian:" << std::endl;
    std::cout << num_jacobian << std::endl;
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
};

}  // namespace sliding_window

#endif  // LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_MAP_MATCHING_POSE_H_
