/*
 * @Description: ceres residual block for LIO IMU pre-integration measurement
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-23 11:10:25
 */
#ifndef LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_IMU_PRE_INTEGRATION_H_
#define LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_IMU_PRE_INTEGRATION_H_

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <sophus/so3.hpp>

#include "glog/logging.h"

namespace sliding_window {

class FactorPRVAGIMUPreIntegration
    : public ceres::SizedCostFunction<15, 15, 15> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  static const int kIndexP{0};
  static const int kIndexR{3};
  static const int kIndexV{6};
  static const int kIndexA{9};
  static const int kIndexG{12};

  FactorPRVAGIMUPreIntegration(){};

  void set_T(const double& T) { T_ = T; }

  void set_gravity(const Eigen::Vector3d& g) { gravity_ = g; }

  void set_measurement(const Eigen::VectorXd& m) { measurement_ = m; }

  void set_information(const Eigen::MatrixXd& I) { information_ = I; }

  void set_jacobian(const Eigen::MatrixXd& J) { jacobian_ = J; }

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

    //
    // parse measurement:
    //
    // const Eigen::Vector3d &alpha_ij = measurement_.block<3, 1>(kIndexP, 0);
    // const Eigen::Vector3d &theta_ij = measurement_.block<3, 1>(kIndexR, 0);
    // const Eigen::Vector3d  &beta_ij = measurement_.block<3, 1>(kIndexV, 0);

    // 原始预积分
    Eigen::Vector3d alpha_ij = measurement_.block<3, 1>(kIndexP, 0);
    Eigen::Vector3d theta_ij = measurement_.block<3, 1>(kIndexR, 0);
    Eigen::Vector3d beta_ij = measurement_.block<3, 1>(kIndexV, 0);
    // std::cout<<"befor: "<< alpha_ij.transpose()<<std::endl;

    // bias变化量
    Eigen::Vector3d dba = b_a_i - measurement_.block<3, 1>(kIndexA, 0);
    Eigen::Vector3d dbg = b_g_i - measurement_.block<3, 1>(kIndexG, 0);
    // std::cout<<"d_ba: " << dba.transpose()<<std::endl;
    // std::cout<<"ba_i: " << b_a_i.transpose()<<std::endl;
    // std::cout<<"ba: " << measurement_.block<3, 1>(kIndexA,
    // 0).transpose()<<std::endl<<std::endl;

    // 根据变化量使用jacobian重新计算预积分
    Eigen::Matrix3d J_q_bg = jacobian_.block<3, 3>(kIndexR, kIndexG);
    Eigen::Matrix3d J_v_ba = jacobian_.block<3, 3>(kIndexV, kIndexA);
    Eigen::Matrix3d J_v_bg = jacobian_.block<3, 3>(kIndexV, kIndexG);
    Eigen::Matrix3d J_p_ba = jacobian_.block<3, 3>(kIndexP, kIndexA);
    Eigen::Matrix3d J_p_bg = jacobian_.block<3, 3>(kIndexP, kIndexG);

    alpha_ij = alpha_ij + J_p_ba * dba + J_p_bg * dbg;
    // std::cout<<"after: "<< alpha_ij.transpose()<<std::endl<<std::endl;

    // theta_ij = theta_ij + J_q_bg*dbg;
    const Sophus::SO3d ori_ij =
        Sophus::SO3d::exp(theta_ij) * Sophus::SO3d::exp(J_q_bg * dbg);
    beta_ij = beta_ij + J_v_ba * dba + J_v_bg * dbg;

    //
    // TODO: get square root of information matrix:
    //
    Eigen::Matrix<double, 15, 15> sqrt_information =
        Eigen::LLT<Eigen::Matrix<double, 15, 15>>(information_)
            .matrixL()
            .transpose();

    //
    // TODO: compute residual:
    //
    Eigen::Map<Eigen::Matrix<double, 15, 1>> res(residuals);

    res.block<3, 1>(kIndexP, 0) =
        ori_i.inverse() *
            (pos_j - pos_i - vel_i * T_ + 0.5 * gravity_ * T_ * T_) -
        alpha_ij;
    res.block<3, 1>(kIndexR, 0) =
        (ori_ij.inverse() * ori_i.inverse() * ori_j).log();
    res.block<3, 1>(kIndexV, 0) =
        ori_i.inverse() * (vel_j - vel_i + gravity_ * T_) - beta_ij;
    res.block<3, 1>(kIndexA, 0) = b_a_j - b_a_i;
    res.block<3, 1>(kIndexG, 0) = b_g_j - b_g_i;

    //
    // TODO: compute jacobians:
    //
    if (jacobians) {
      // compute shared intermediate results:
      const Eigen::Matrix3d R_i_inv = ori_i.inverse().matrix();
      const Eigen::Matrix3d J_r_inv = JacobianRInv(res.block<3, 1>(kIndexR, 0));
      const Eigen::Matrix3d R_j_i = (ori_j.inverse() * ori_i).matrix();

      // Eigen::Matrix3d J_r =  JacobianRInv2(res.block<3,1>(kIndexR,0));
      // std::cout<<"my j: "<<std::endl<<J_r_inv<<std::endl;
      // std::cout<<"other j: "<<std::endl<<J_r<<std::endl<<std::endl;

      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> jacobian_i(
            jacobians[0]);
        jacobian_i.setZero();

        // a. residual, position:
        jacobian_i.block<3, 3>(kIndexP, kIndexP) = -R_i_inv;

        jacobian_i.block<3, 3>(kIndexP, kIndexR) =
            Sophus::SO3d::hat(ori_i.inverse() * (pos_j - pos_i - vel_i * T_ +
                                                 0.5 * gravity_ * T_ * T_))
                .matrix();

        jacobian_i.block<3, 3>(kIndexP, kIndexV) = -T_ * R_i_inv;

        jacobian_i.block<3, 3>(kIndexP, kIndexA) =
            -jacobian_.block<3, 3>(kIndexP, kIndexA);

        jacobian_i.block<3, 3>(kIndexP, kIndexG) =
            -jacobian_.block<3, 3>(kIndexP, kIndexG);

        // b. residual, orientation:
        jacobian_i.block<3, 3>(kIndexR, kIndexR) = -J_r_inv * R_j_i;
        Eigen::Matrix3d J_r =
            JacobianR(jacobian_.block<3, 3>(kIndexR, kIndexG) * dbg);
        jacobian_i.block<3, 3>(kIndexR, kIndexG) =
            -J_r_inv *
            (Sophus::SO3d::exp(res.block<3, 1>(kIndexR, 0)))
                .matrix()
                .inverse() *
            J_r * jacobian_.block<3, 3>(kIndexR, kIndexG);

        // c. residual, velocity:
        jacobian_i.block<3, 3>(kIndexV, kIndexR) = Sophus::SO3d::hat(
            ori_i.inverse() * (vel_j - vel_i + gravity_ * T_));

        jacobian_i.block<3, 3>(kIndexV, kIndexV) = -R_i_inv;

        jacobian_i.block<3, 3>(kIndexV, kIndexA) =
            -jacobian_.block<3, 3>(kIndexV, kIndexA);

        jacobian_i.block<3, 3>(kIndexV, kIndexG) =
            -jacobian_.block<3, 3>(kIndexV, kIndexG);

        // d. residual, bias accel:
        jacobian_i.block<3, 3>(kIndexA, kIndexA) = -Eigen::Matrix3d::Identity();

        // d. residual, bias accel:
        jacobian_i.block<3, 3>(kIndexG, kIndexG) = -Eigen::Matrix3d::Identity();

        jacobian_i = sqrt_information * jacobian_i;
      }

      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> jacobian_j(
            jacobians[1]);
        jacobian_j.setZero();

        // a. residual, position:
        jacobian_j.block<3, 3>(kIndexP, kIndexP) = R_i_inv;

        // b. residual, orientation:
        jacobian_j.block<3, 3>(kIndexR, kIndexR) = J_r_inv;

        // c. residual, velocity:
        jacobian_j.block<3, 3>(kIndexV, kIndexV) = R_i_inv;

        // d. residual, bias accel:
        jacobian_j.block<3, 3>(kIndexA, kIndexA) = Eigen::Matrix3d::Identity();

        // d. residual, bias accel:
        jacobian_j.block<3, 3>(kIndexG, kIndexG) = Eigen::Matrix3d::Identity();

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
    double* residuals = new double[15];
    double** jacobians = new double*[2];
    jacobians[0] = new double[15 * 15];
    jacobians[1] = new double[15 * 15];
    Evaluate(parameters, residuals, jacobians);

    std::cout << "my jacobian: " << std::endl;
    std::cout << Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>>(
                     jacobians[0])
              << std::endl;
    std::cout << Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>>(
                     jacobians[1])
              << std::endl;
    std::cout << "my residual: " << std::endl;
    std::cout << Eigen::Map<Eigen::Matrix<double, 15, 1>>(residuals).transpose()
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

    Eigen::Vector3d alpha_ij = measurement_.block<3, 1>(kIndexP, 0);
    Eigen::Vector3d theta_ij = measurement_.block<3, 1>(kIndexR, 0);
    Eigen::Vector3d beta_ij = measurement_.block<3, 1>(kIndexV, 0);

    Eigen::Matrix<double, 15, 15> sqrt_information =
        Eigen::LLT<Eigen::Matrix<double, 15, 15>>(information_)
            .matrixL()
            .transpose();

    Eigen::Matrix<double, 15, 1> residual;

    residual.block<3, 1>(kIndexP, 0) =
        ori_i.inverse() *
            (pos_j - pos_i - vel_i * T_ + 0.5 * gravity_ * T_ * T_) -
        alpha_ij;
    Sophus::SO3d ori_ij = Sophus::SO3d::exp(theta_ij);
    residual.block<3, 1>(kIndexR, 0) =
        (ori_ij.inverse() * ori_i.inverse() * ori_j).log();
    residual.block<3, 1>(kIndexV, 0) =
        ori_i.inverse() * (vel_j - vel_i + gravity_ * T_) - beta_ij;
    residual.block<3, 1>(kIndexA, 0) = b_a_j - b_a_i;
    residual.block<3, 1>(kIndexG, 0) = b_g_j - b_g_i;

    residual = sqrt_information * residual;

    std::cout << "residual: " << std::endl;
    std::cout << residual.transpose() << std::endl;

    const double eps = 1e-6;
    Eigen::Matrix<double, 15, 30> num_jacobian;
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

      // bias变化量
      Eigen::Vector3d dba = b_a_i_num - b_a_i;
      Eigen::Vector3d dbg = b_g_i_num - b_g_i;
      // std::cout<<"d_ba: " << dba.transpose()<<std::endl;
      // std::cout<<"ba_i: " << b_a_i.transpose()<<std::endl;
      // std::cout<<"ba: " << measurement_.block<3, 1>(kIndexA,
      // 0).transpose()<<std::endl<<std::endl;

      // 根据变化量使用jacobian重新计算预积分
      Eigen::Matrix3d J_q_bg = jacobian_.block<3, 3>(kIndexR, kIndexG);
      Eigen::Matrix3d J_v_ba = jacobian_.block<3, 3>(kIndexV, kIndexA);
      Eigen::Matrix3d J_v_bg = jacobian_.block<3, 3>(kIndexV, kIndexG);
      Eigen::Matrix3d J_p_ba = jacobian_.block<3, 3>(kIndexP, kIndexA);
      Eigen::Matrix3d J_p_bg = jacobian_.block<3, 3>(kIndexP, kIndexG);

      Eigen::Vector3d alpha_ij_turb = alpha_ij + J_p_ba * dba + J_p_bg * dbg;
      // std::cout<<"after: "<< alpha_ij.transpose()<<std::endl<<std::endl;

      // Eigen::Vector3d theta_ij_trub = theta_ij + J_q_bg*dbg;

      Eigen::Vector3d beta_ij_turb = beta_ij + J_v_ba * dba + J_v_bg * dbg;
      // Sophus::SO3d ori_ij_trub = Sophus::SO3d::exp(theta_ij_trub);
      Sophus::SO3d ori_ij_trub =
          Sophus::SO3d::exp(theta_ij) * Sophus::SO3d::exp(J_q_bg * dbg);

      Eigen::Matrix<double, 15, 1> tmp_residual;

      tmp_residual.block<3, 1>(kIndexP, 0) =
          ori_i_num.inverse() * (pos_j_num - pos_i_num - vel_i_num * T_ +
                                 0.5 * gravity_ * T_ * T_) -
          alpha_ij_turb;
      tmp_residual.block<3, 1>(kIndexR, 0) =
          (ori_ij_trub.inverse() * ori_i_num.inverse() * ori_j_num).log();
      tmp_residual.block<3, 1>(kIndexV, 0) =
          ori_i_num.inverse() * (vel_j_num - vel_i_num + gravity_ * T_) -
          beta_ij_turb;
      tmp_residual.block<3, 1>(kIndexA, 0) = b_a_j_num - b_a_i_num;
      tmp_residual.block<3, 1>(kIndexG, 0) = b_g_j_num - b_g_i_num;

      tmp_residual = sqrt_information * tmp_residual;

      num_jacobian.col(k) = (tmp_residual - residual) / eps;
    }
    std::cout << "num jacobian:" << std::endl;
    std::cout << num_jacobian.block<15, 15>(0, 0) << std::endl;
    std::cout << num_jacobian.block<15, 15>(0, 15) << std::endl << std::endl;
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

  static Eigen::Matrix3d JacobianR(const Eigen::Vector3d& w) {
    Eigen::Matrix3d J_r = Eigen::Matrix3d::Identity();

    const double theta = w.norm();

    if (theta > 1e-5) {
      Eigen::Vector3d a = w.normalized();
      Eigen::Matrix3d a_hat = Sophus::SO3d::hat(a);

      J_r = sin(theta) / theta * Eigen::Matrix3d::Identity() +
            (1.0 - sin(theta) / theta) * a * a.transpose() -
            (1.0 - cos(theta)) / theta * a_hat;
    }

    return J_r;
  }

  double T_{0.0};

  Eigen::Vector3d gravity_ = Eigen::Vector3d::Zero();

  Eigen::VectorXd measurement_;
  Eigen::MatrixXd information_;

  Eigen::MatrixXd jacobian_;
};

}  // namespace sliding_window

#endif  // LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_IMU_PRE_INTEGRATION_H_
