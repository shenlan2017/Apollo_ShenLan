/*
 * @Description: IMU pre-integrator for LIO mapping, implementation
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-11-02 09:21:27
 */
#include "lidar_localization/models/pre_integrator/imu_pre_integrator.h"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

IMUPreIntegrator::IMUPreIntegrator(const YAML::Node& node) {
  //
  // parse config:
  //
  // a. earth constants:
  earth_.gravity_manitude = node["earth"]["gravity_magnitude"].as<double>();
  // b. process noise:
  cov_.measurement.accel =
      node["covariance"]["measurement"]["accel"].as<double>();
  cov_.measurement.gyro =
      node["covariance"]["measurement"]["gyro"].as<double>();
  cov_.random_walk.accel =
      node["covariance"]["random_walk"]["accel"].as<double>();
  cov_.random_walk.gyro =
      node["covariance"]["random_walk"]["gyro"].as<double>();

  // prompt:
  std::cout << std::endl
            << "IMU Pre-Integration params:" << std::endl
            << "\tgravity magnitude: " << earth_.gravity_manitude << std::endl
            << std::endl
            << "\tprocess noise:" << std::endl
            << "\t\tmeasurement:" << std::endl
            << "\t\t\taccel.: " << cov_.measurement.accel << std::endl
            << "\t\t\tgyro.: " << cov_.measurement.gyro << std::endl
            << "\t\trandom_walk:" << std::endl
            << "\t\t\taccel.: " << cov_.random_walk.accel << std::endl
            << "\t\t\tgyro.: " << cov_.random_walk.gyro << std::endl
            << std::endl;

  // a. gravity constant:
  state_.g = Eigen::Vector3d(0.0, 0.0, earth_.gravity_manitude);

  // b. process noise:
  Q_.block<3, 3>(kIndexNoiseAccPrev, kIndexNoiseAccPrev) =
      Q_.block<3, 3>(kIndexNoiseAccCurr, kIndexNoiseAccCurr) =
          cov_.measurement.accel * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(kIndexNoiseGyrPrev, kIndexNoiseGyrPrev) =
      Q_.block<3, 3>(kIndexNoiseGyrCurr, kIndexNoiseGyrCurr) =
          cov_.measurement.gyro * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(kIndexNoiseBa, kIndexNoiseBa) =
      cov_.random_walk.accel * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(kIndexNoiseBg, kIndexNoiseBg) =
      cov_.random_walk.gyro * Eigen::Matrix3d::Identity();

  // c. process equation, state propagation:
  F_.block<3, 3>(kIndexAlpha, kIndexBeta) = Eigen::Matrix3d::Identity();
  F_.block<3, 3>(kIndexTheta, kIndexBg) = -Eigen::Matrix3d::Identity();

  // d. process equation, noise input:
  B_.block<3, 3>(kIndexTheta, kIndexNoiseGyrPrev) = B_.block<3, 3>(
      kIndexTheta, kIndexNoiseGyrCurr) = 0.50 * Eigen::Matrix3d::Identity();
  B_.block<3, 3>(kIndexBa, kIndexNoiseBa) =
      B_.block<3, 3>(kIndexBg, kIndexNoiseBg) = Eigen::Matrix3d::Identity();
}

/**
 * @brief  reset IMU pre-integrator
 * @param  init_imu_data, init IMU measurements
 * @return true if success false otherwise
 */
bool IMUPreIntegrator::Init(const IMUData& init_imu_data) {
  // reset pre-integrator state:
  ResetState(init_imu_data);

  // mark as inited:
  is_inited_ = true;

  return true;
}

/**
 * @brief  update IMU pre-integrator
 * @param  imu_data, current IMU measurements
 * @return true if success false otherwise
 */
bool IMUPreIntegrator::Update(const IMUData& imu_data) {
  if (imu_data_buff_.front().time_ < imu_data.time_) {
    // set buffer:
    imu_data_buff_.push_back(imu_data);

    // update state mean, covariance and Jacobian:
    UpdateState();

    // move forward:
    imu_data_buff_.pop_front();
  }

  return true;
}

void IMUPreIntegrator::GetIMUPreIntegration(
    IMUPreIntegration& imu_pre_integration, const double& time) {
  imu_pre_integration.T = time - time_;
  imu_pre_integration.g = state_.g;

  // set measurement:
  imu_pre_integration.alpha_ij = state_.alpha_ij;
  imu_pre_integration.theta_ij = state_.theta_ij;
  imu_pre_integration.beta_ij = state_.beta_ij;
  imu_pre_integration.b_a_i = state_.b_a_i;
  imu_pre_integration.b_g_i = state_.b_g_i;
  // set information:
  imu_pre_integration.P = P_;
  // set Jacobian:
  imu_pre_integration.J = J_;
};

/**
 * @brief  reset IMU pre-integrator using new init IMU measurement
 * @param  init_imu_data, new init IMU measurements
 * @param  output pre-integration result for constraint building as
 * IMUPreIntegration
 * @return true if success false otherwise
 */
bool IMUPreIntegrator::Reset(const IMUData& init_imu_data,
                             IMUPreIntegration& imu_pre_integration) {
  // one last update:
  Update(init_imu_data);

  // set output IMU pre-integration:
  imu_pre_integration.T = init_imu_data.time_ - time_;

  // set gravity constant:
  imu_pre_integration.g = state_.g;

  // set measurement:
  imu_pre_integration.alpha_ij = state_.alpha_ij;
  imu_pre_integration.theta_ij = state_.theta_ij;
  imu_pre_integration.beta_ij = state_.beta_ij;
  imu_pre_integration.b_a_i = state_.b_a_i;
  imu_pre_integration.b_g_i = state_.b_g_i;
  // set information:
  imu_pre_integration.P = P_;
  // set Jacobian:
  imu_pre_integration.J = J_;

  // reset:
  ResetState(init_imu_data);

  return true;
}

/**
 * @brief  reset pre-integrator state using IMU measurements
 * @param  void
 * @return void
 */
void IMUPreIntegrator::ResetState(const IMUData& init_imu_data) {
  // reset time:
  time_ = init_imu_data.time_;

  // a. reset relative translation:
  state_.alpha_ij = Eigen::Vector3d::Zero();
  // b. reset relative orientation:
  state_.theta_ij = Sophus::SO3d();
  // c. reset relative velocity:
  state_.beta_ij = Eigen::Vector3d::Zero();
  // d. set init bias, acceleometer:
  state_.b_a_i =
      Eigen::Vector3d(init_imu_data.accel_bias_.x, init_imu_data.accel_bias_.y,
                      init_imu_data.accel_bias_.z);
  // d. set init bias, gyroscope:
  state_.b_g_i =
      Eigen::Vector3d(init_imu_data.gyro_bias_.x, init_imu_data.gyro_bias_.y,
                      init_imu_data.gyro_bias_.z);

  // reset state covariance:
  P_ = MatrixP::Zero();

  // reset Jacobian:
  J_ = MatrixJ::Identity();

  // reset buffer:
  imu_data_buff_.clear();
  imu_data_buff_.push_back(init_imu_data);
}

/**
 * @brief  update pre-integrator state: mean, covariance and Jacobian
 * @param  void
 * @return void
 */
void IMUPreIntegrator::UpdateState() {
  static double T = 0.0;

  static Eigen::Vector3d w_mid = Eigen::Vector3d::Zero();
  static Eigen::Vector3d a_mid = Eigen::Vector3d::Zero();

  static Sophus::SO3d prev_theta_ij = Sophus::SO3d();
  static Sophus::SO3d curr_theta_ij = Sophus::SO3d();
  static Sophus::SO3d d_theta_ij = Sophus::SO3d();

  static Eigen::Matrix3d dR_inv = Eigen::Matrix3d::Zero();
  static Eigen::Matrix3d prev_R = Eigen::Matrix3d::Zero();
  static Eigen::Matrix3d curr_R = Eigen::Matrix3d::Zero();
  static Eigen::Matrix3d prev_R_a_hat = Eigen::Matrix3d::Zero();
  static Eigen::Matrix3d curr_R_a_hat = Eigen::Matrix3d::Zero();

  //
  // parse measurements:
  //
  // get measurement handlers:
  const IMUData& prev_imu_data = imu_data_buff_.at(0);
  const IMUData& curr_imu_data = imu_data_buff_.at(1);

  // get time delta:
  T = curr_imu_data.time_ - prev_imu_data.time_;

  // get measurements
  Eigen::Vector3d prev_w(prev_imu_data.angular_velocity_.x,
                         prev_imu_data.angular_velocity_.y,
                         prev_imu_data.angular_velocity_.z);
  Eigen::Vector3d curr_w(curr_imu_data.angular_velocity_.x,
                         curr_imu_data.angular_velocity_.y,
                         curr_imu_data.angular_velocity_.z);
  Eigen::Vector3d prev_a(prev_imu_data.linear_acceleration_.x,
                         prev_imu_data.linear_acceleration_.y,
                         prev_imu_data.linear_acceleration_.z);
  Eigen::Vector3d curr_a(curr_imu_data.linear_acceleration_.x,
                         curr_imu_data.linear_acceleration_.y,
                         curr_imu_data.linear_acceleration_.z);

  // // check
  // // Eigen::Vector3d turb(0.0001, -0.003, 0.003);
  // Eigen::Vector3d turb(0.000001, -0.00003, 0.00003);
  // Eigen::Vector3d curr_delta_p = Eigen::Vector3d::Zero();
  // Sophus::SO3d curr_delta_q = Sophus::SO3d();
  // Eigen::Vector3d curr_delta_v = Eigen::Vector3d::Zero();
  // Eigen::Vector3d curr_ba = Eigen::Vector3d::Zero();
  // Eigen::Vector3d curr_bg = Eigen::Vector3d::Zero();
  // MatrixF step_J = MatrixF::Zero();
  // MatrixB step_B = MatrixB::Zero();
  // // a. 位置加扰动
  // CheckJacobian(T, prev_a, prev_w, curr_a, curr_w,
  //     state_.alpha_ij_ + turb, state_.theta_ij_, state_.beta_ij_,
  //     state_.b_a_i_, state_.b_g_i_, curr_delta_p, curr_delta_q, curr_delta_v,
  //     curr_ba, curr_bg, step_J, step_B);
  // // b. 姿态加扰动
  // Sophus::SO3d q_turb = state_.theta_ij_ * Sophus::SO3d::exp(turb);
  // CheckJacobian(T, prev_a, prev_w, curr_a, curr_w,
  //     state_.alpha_ij_, q_turb, state_.beta_ij_, state_.b_a_i_,
  //     state_.b_g_i_, curr_delta_p, curr_delta_q, curr_delta_v, curr_ba,
  //     curr_bg, step_J, step_B);
  // // c. 速度加扰动
  // CheckJacobian(T, prev_a, prev_w, curr_a, curr_w,
  //     state_.alpha_ij_, state_.theta_ij_, state_.beta_ij_ + turb,
  //     state_.b_a_i_, state_.b_g_i_, curr_delta_p, curr_delta_q, curr_delta_v,
  //     curr_ba, curr_bg, step_J, step_B);
  // // d. ba加扰动
  // CheckJacobian(T, prev_a, prev_w, curr_a, curr_w,
  //     state_.alpha_ij_, state_.theta_ij_, state_.beta_ij_, state_.b_a_i_ +
  //     turb, state_.b_g_i_, curr_delta_p, curr_delta_q, curr_delta_v, curr_ba,
  //     curr_bg, step_J, step_B);
  // // e. bg加扰动
  // CheckJacobian(T, prev_a, prev_w, curr_a, curr_w,
  //     state_.alpha_ij_, state_.theta_ij_, state_.beta_ij_, state_.b_a_i_,
  //     state_.b_g_i_ + turb, curr_delta_p, curr_delta_q, curr_delta_v,
  //     curr_ba, curr_bg, step_J, step_B);
  // // f. acc_0 加扰动
  // CheckJacobian(T, prev_a + turb, prev_w, curr_a, curr_w,
  //     state_.alpha_ij_, state_.theta_ij_, state_.beta_ij_, state_.b_a_i_,
  //     state_.b_g_i_, curr_delta_p, curr_delta_q, curr_delta_v, curr_ba,
  //     curr_bg, step_J, step_B);
  // // g. gyr_0 加扰动
  // CheckJacobian(T, prev_a , prev_w + turb, curr_a, curr_w,
  //     state_.alpha_ij_, state_.theta_ij_, state_.beta_ij_, state_.b_a_i_,
  //     state_.b_g_i_, curr_delta_p, curr_delta_q, curr_delta_v, curr_ba,
  //     curr_bg, step_J, step_B);
  // // h. acc_1 加扰动
  // CheckJacobian(T, prev_a , prev_w , curr_a + turb, curr_w,
  //     state_.alpha_ij_, state_.theta_ij_, state_.beta_ij_, state_.b_a_i_,
  //     state_.b_g_i_, curr_delta_p, curr_delta_q, curr_delta_v, curr_ba,
  //     curr_bg, step_J, step_B);
  // // i. gyr_1 加扰动
  // CheckJacobian(T, prev_a , prev_w , curr_a , curr_w + turb,
  //     state_.alpha_ij_, state_.theta_ij_, state_.beta_ij_, state_.b_a_i_,
  //     state_.b_g_i_, curr_delta_p, curr_delta_q, curr_delta_v, curr_ba,
  //     curr_bg, step_J, step_B);
  // // check jcaobian
  // CheckJacobian(T, prev_a , prev_w , curr_a , curr_w,
  //     state_.alpha_ij_, state_.theta_ij_, state_.beta_ij_, state_.b_a_i_,
  //     state_.b_g_i_, curr_delta_p, curr_delta_q, curr_delta_v, curr_ba,
  //     curr_bg, step_J, step_B);
  // std::cout<<"J: "<<std::endl<<step_J<<std::endl;

  // unbiased
  prev_w = prev_w - state_.b_g_i;
  curr_w = curr_w - state_.b_g_i;
  prev_a = prev_a - state_.b_a_i;
  curr_a = curr_a - state_.b_a_i;

  //
  // a. update mean:
  //
  // 1. get w_mid:
  w_mid = 0.5 * (prev_w + curr_w);
  // 2. update relative orientation, so3:
  prev_theta_ij = state_.theta_ij;
  d_theta_ij = Sophus::SO3d::exp(w_mid * T);
  state_.theta_ij = state_.theta_ij * d_theta_ij;
  curr_theta_ij = state_.theta_ij;
  // 3. get a_mid:
  a_mid = 0.5 * (prev_theta_ij * prev_a + curr_theta_ij * curr_a);
  // 4. update relative translation:
  state_.alpha_ij += (state_.beta_ij + 0.5 * a_mid * T) * T;
  // 5. update relative velocity:
  state_.beta_ij += a_mid * T;

  // std::cout<<"check v: "<<curr_delta_v.transpose()<<std::endl;
  // std::cout<<"v: "<<state_.beta_ij.transpose()<<std::endl<<std::endl;

  //
  // b. update covariance:
  //
  // 1. intermediate results:
  dR_inv = d_theta_ij.inverse().matrix();
  prev_R = prev_theta_ij.matrix();
  curr_R = curr_theta_ij.matrix();
  prev_R_a_hat = prev_R * Sophus::SO3d::hat(prev_a);
  curr_R_a_hat = curr_R * Sophus::SO3d::hat(curr_a);

  //
  // 2. set up F:
  //
  // F12 & F32:
  F_.block<3, 3>(kIndexAlpha, kIndexTheta) = F_.block<3, 3>(
      kIndexBeta, kIndexTheta) = -0.50 * (prev_R_a_hat + curr_R_a_hat * dR_inv);
  F_.block<3, 3>(kIndexAlpha, kIndexTheta) =
      0.50 * T * F_.block<3, 3>(kIndexAlpha, kIndexTheta);
  // F14 & F34:
  F_.block<3, 3>(kIndexAlpha, kIndexBa) = F_.block<3, 3>(kIndexBeta, kIndexBa) =
      -0.50 * (prev_R + curr_R);
  F_.block<3, 3>(kIndexAlpha, kIndexBa) =
      0.50 * T * F_.block<3, 3>(kIndexAlpha, kIndexBa);
  // F15 & F35:
  F_.block<3, 3>(kIndexAlpha, kIndexBg) = F_.block<3, 3>(kIndexBeta, kIndexBg) =
      0.50 * T * curr_R_a_hat;
  F_.block<3, 3>(kIndexAlpha, kIndexBg) =
      0.50 * T * F_.block<3, 3>(kIndexAlpha, kIndexBg);
  // F22:
  F_.block<3, 3>(kIndexTheta, kIndexTheta) = -Sophus::SO3d::hat(w_mid);

  //
  // 3. set up G:
  //
  // G11 & G31:
  B_.block<3, 3>(kIndexAlpha, kIndexNoiseAccPrev) =
      B_.block<3, 3>(kIndexBeta, kIndexNoiseAccPrev) = +0.50 * prev_R;
  B_.block<3, 3>(kIndexAlpha, kIndexNoiseAccPrev) =
      0.50 * T * B_.block<3, 3>(kIndexAlpha, kIndexNoiseAccPrev);
  // G12 & G32:
  B_.block<3, 3>(kIndexAlpha, kIndexNoiseGyrPrev) =
      B_.block<3, 3>(kIndexBeta, kIndexNoiseGyrPrev) = -0.25 * T * curr_R_a_hat;
  B_.block<3, 3>(kIndexAlpha, kIndexNoiseGyrPrev) =
      0.50 * T * B_.block<3, 3>(kIndexAlpha, kIndexNoiseGyrPrev);
  // G13 & G33:
  B_.block<3, 3>(kIndexAlpha, kIndexNoiseAccCurr) =
      B_.block<3, 3>(kIndexBeta, kIndexNoiseAccCurr) = 0.5 * curr_R;
  B_.block<3, 3>(kIndexAlpha, kIndexNoiseAccCurr) =
      0.50 * T * B_.block<3, 3>(kIndexAlpha, kIndexNoiseAccCurr);
  // G14 & G34:
  B_.block<3, 3>(kIndexAlpha, kIndexNoiseGyrCurr) =
      B_.block<3, 3>(kIndexBeta, kIndexNoiseGyrCurr) = -0.25 * T * curr_R_a_hat;
  B_.block<3, 3>(kIndexAlpha, kIndexNoiseGyrCurr) =
      0.50 * T * B_.block<3, 3>(kIndexAlpha, kIndexNoiseGyrCurr);

  // 4. update P_:
  MatrixF F = MatrixF::Identity() + T * F_;
  MatrixB B = T * B_;

  // std::cout<<"my jacoiban:"<<std::endl<<F<<std::endl<<std::endl;

  P_ = F * P_ * F.transpose() + B * Q_ * B.transpose();

  //
  // c. update Jacobian:
  //
  J_ = F * J_;

  // check
  // // a. 位移扰动
  // std::cout<<"p diff: "<<(curr_delta_p -
  // state_.alpha_ij_).transpose()<<std::endl; std::cout<<"p jaco diff:
  // "<<(step_J.block<3,3>(0,0)*turb).transpose()<<std::endl; std::cout<<"q
  // diff: "<<((state_.theta_ij_.inverse() *
  // curr_delta_q).log()).transpose()<<std::endl; std::cout<<"q jaco diff:
  // "<<(step_J.block<3,3>(3,0)*turb).transpose()<<std::endl; std::cout<<"v
  // diff: "<<(curr_delta_v - state_.beta_ij_).transpose()<<std::endl;
  // std::cout<<"v jaco diff:
  // "<<(step_J.block<3,3>(6,0)*turb).transpose()<<std::endl; std::cout<<"ba
  // diff: "<<(curr_ba - state_.b_a_i_).transpose()<<std::endl; std::cout<<"ba
  // jaco diff: "<<(step_J.block<3,3>(9,0)*turb).transpose()<<std::endl;
  // std::cout<<"bg diff: "<<(curr_bg - state_.b_g_i_).transpose()<<std::endl;
  // std::cout<<"bg jaco diff:
  // "<<(step_J.block<3,3>(12,0)*turb).transpose()<<std::endl<<std::endl;

  // // b. 姿态扰动
  // std::cout<<"p diff: "<<(curr_delta_p -
  // state_.alpha_ij_).transpose()<<std::endl; std::cout<<"p jaco diff:
  // "<<(step_J.block<3,3>(0,3)*turb).transpose()<<std::endl; std::cout<<"q
  // diff: "<<((state_.theta_ij_.inverse() *
  // curr_delta_q).log()).transpose()<<std::endl; std::cout<<"q jaco diff:
  // "<<(step_J.block<3,3>(3,3)*turb).transpose()<<std::endl; std::cout<<"v
  // diff: "<<(curr_delta_v - state_.beta_ij_).transpose()<<std::endl;
  // std::cout<<"v jaco diff:
  // "<<(step_J.block<3,3>(6,3)*turb).transpose()<<std::endl; std::cout<<"ba
  // diff: "<<(curr_ba - state_.b_a_i_).transpose()<<std::endl; std::cout<<"ba
  // jaco diff: "<<(step_J.block<3,3>(9,3)*turb).transpose()<<std::endl;
  // std::cout<<"bg diff: "<<(curr_bg - state_.b_g_i_).transpose()<<std::endl;
  // std::cout<<"bg jaco diff:
  // "<<(step_J.block<3,3>(12,3)*turb).transpose()<<std::endl<<std::endl;

  // // c. 速度扰动
  // std::cout<<"p diff: "<<(curr_delta_p -
  // state_.alpha_ij_).transpose()<<std::endl; std::cout<<"p jaco diff:
  // "<<(step_J.block<3,3>(0,6)*turb).transpose()<<std::endl; std::cout<<"q
  // diff: "<<((state_.theta_ij_.inverse() *
  // curr_delta_q).log()).transpose()<<std::endl; std::cout<<"q jaco diff:
  // "<<(step_J.block<3,3>(3,6)*turb).transpose()<<std::endl; std::cout<<"v
  // diff: "<<(curr_delta_v - state_.beta_ij_).transpose()<<std::endl;
  // std::cout<<"v jaco diff:
  // "<<(step_J.block<3,3>(6,6)*turb).transpose()<<std::endl; std::cout<<"ba
  // diff: "<<(curr_ba - state_.b_a_i_).transpose()<<std::endl; std::cout<<"ba
  // jaco diff: "<<(step_J.block<3,3>(9,6)*turb).transpose()<<std::endl;
  // std::cout<<"bg diff: "<<(curr_bg - state_.b_g_i_).transpose()<<std::endl;
  // std::cout<<"bg jaco diff:
  // "<<(step_J.block<3,3>(12,6)*turb).transpose()<<std::endl<<std::endl;

  // // b. ba扰动
  // std::cout<<"p diff: "<<(curr_delta_p -
  // state_.alpha_ij_).transpose()<<std::endl; std::cout<<"p jaco diff:
  // "<<(step_J.block<3,3>(0,9)*turb).transpose()<<std::endl; std::cout<<"q
  // diff: "<<((state_.theta_ij_.inverse() *
  // curr_delta_q).log()).transpose()<<std::endl; std::cout<<"q jaco diff:
  // "<<(step_J.block<3,3>(3,9)*turb).transpose()<<std::endl; std::cout<<"v
  // diff: "<<(curr_delta_v - state_.beta_ij_).transpose()<<std::endl;
  // std::cout<<"v jaco diff:
  // "<<(step_J.block<3,3>(6,9)*turb).transpose()<<std::endl; std::cout<<"ba
  // diff: "<<(curr_ba - state_.b_a_i_).transpose()<<std::endl; std::cout<<"ba
  // jaco diff: "<<(step_J.block<3,3>(9,9)*turb).transpose()<<std::endl;
  // std::cout<<"bg diff: "<<(curr_bg - state_.b_g_i_).transpose()<<std::endl;
  // std::cout<<"bg jaco diff:
  // "<<(step_J.block<3,3>(12,9)*turb).transpose()<<std::endl<<std::endl;

  // // e. bg扰动
  // std::cout<<"p diff: "<<(curr_delta_p -
  // state_.alpha_ij_).transpose()<<std::endl; std::cout<<"p jaco diff:
  // "<<(step_J.block<3,3>(0,12)*turb).transpose()<<std::endl; std::cout<<"q
  // diff: "<<((state_.theta_ij_.inverse() *
  // curr_delta_q).log()).transpose()<<std::endl; std::cout<<"q jaco diff:
  // "<<(step_J.block<3,3>(3,12)*turb).transpose()<<std::endl; std::cout<<"v
  // diff: "<<(curr_delta_v - state_.beta_ij_).transpose()<<std::endl;
  // std::cout<<"v jaco diff:
  // "<<(step_J.block<3,3>(6,12)*turb).transpose()<<std::endl; std::cout<<"ba
  // diff: "<<(curr_ba - state_.b_a_i_).transpose()<<std::endl; std::cout<<"ba
  // jaco diff: "<<(step_J.block<3,3>(9,12)*turb).transpose()<<std::endl;
  // std::cout<<"bg diff: "<<(curr_bg - state_.b_g_i_).transpose()<<std::endl;
  // std::cout<<"bg jaco diff:
  // "<<(step_J.block<3,3>(12,12)*turb).transpose()<<std::endl<<std::endl;

  // // f. acc_0 扰动
  // std::cout<<"p diff: "<<(curr_delta_p -
  // state_.alpha_ij_).transpose()<<std::endl; std::cout<<"p jaco diff:
  // "<<(step_B.block<3,3>(0,0)*turb).transpose()<<std::endl; std::cout<<"q
  // diff: "<<((state_.theta_ij_.inverse() *
  // curr_delta_q).log()).transpose()<<std::endl; std::cout<<"q jaco diff:
  // "<<(step_B.block<3,3>(3,0)*turb).transpose()<<std::endl; std::cout<<"v
  // diff: "<<(curr_delta_v - state_.beta_ij_).transpose()<<std::endl;
  // std::cout<<"v jaco diff:
  // "<<(step_B.block<3,3>(6,0)*turb).transpose()<<std::endl; std::cout<<"ba
  // diff: "<<(curr_ba - state_.b_a_i_).transpose()<<std::endl; std::cout<<"ba
  // jaco diff: "<<(step_B.block<3,3>(9,0)*turb).transpose()<<std::endl;
  // std::cout<<"bg diff: "<<(curr_bg - state_.b_g_i_).transpose()<<std::endl;
  // std::cout<<"bg jaco diff:
  // "<<(step_B.block<3,3>(12,0)*turb).transpose()<<std::endl<<std::endl;

  // // g. gyr_0 扰动
  // std::cout<<"p diff: "<<(curr_delta_p -
  // state_.alpha_ij_).transpose()<<std::endl; std::cout<<"p jaco diff:
  // "<<(step_B.block<3,3>(0,3)*turb).transpose()<<std::endl; std::cout<<"q
  // diff: "<<((state_.theta_ij_.inverse() *
  // curr_delta_q).log()).transpose()<<std::endl; std::cout<<"q jaco diff:
  // "<<(step_B.block<3,3>(3,3)*turb).transpose()<<std::endl; std::cout<<"v
  // diff: "<<(curr_delta_v - state_.beta_ij_).transpose()<<std::endl;
  // std::cout<<"v jaco diff:
  // "<<(step_B.block<3,3>(6,3)*turb).transpose()<<std::endl; std::cout<<"ba
  // diff: "<<(curr_ba - state_.b_a_i_).transpose()<<std::endl; std::cout<<"ba
  // jaco diff: "<<(step_B.block<3,3>(9,3)*turb).transpose()<<std::endl;
  // std::cout<<"bg diff: "<<(curr_bg - state_.b_g_i_).transpose()<<std::endl;
  // std::cout<<"bg jaco diff:
  // "<<(step_B.block<3,3>(12,3)*turb).transpose()<<std::endl<<std::endl;

  // // h. acc_1 扰动
  // std::cout<<"p diff: "<<(curr_delta_p -
  // state_.alpha_ij_).transpose()<<std::endl; std::cout<<"p jaco diff:
  // "<<(step_B.block<3,3>(0,6)*turb).transpose()<<std::endl; std::cout<<"q
  // diff: "<<((state_.theta_ij_.inverse() *
  // curr_delta_q).log()).transpose()<<std::endl; std::cout<<"q jaco diff:
  // "<<(step_B.block<3,3>(3,6)*turb).transpose()<<std::endl; std::cout<<"v
  // diff: "<<(curr_delta_v - state_.beta_ij_).transpose()<<std::endl;
  // std::cout<<"v jaco diff:
  // "<<(step_B.block<3,3>(6,6)*turb).transpose()<<std::endl; std::cout<<"ba
  // diff: "<<(curr_ba - state_.b_a_i_).transpose()<<std::endl; std::cout<<"ba
  // jaco diff: "<<(step_B.block<3,3>(9,6)*turb).transpose()<<std::endl;
  // std::cout<<"bg diff: "<<(curr_bg - state_.b_g_i_).transpose()<<std::endl;
  // std::cout<<"bg jaco diff:
  // "<<(step_B.block<3,3>(12,6)*turb).transpose()<<std::endl<<std::endl;

  // // i. gyr_1 扰动
  // std::cout<<"p diff: "<<(curr_delta_p -
  // state_.alpha_ij_).transpose()<<std::endl; std::cout<<"p jaco diff:
  // "<<(step_B.block<3,3>(0,9)*turb).transpose()<<std::endl; std::cout<<"q
  // diff: "<<((state_.theta_ij_.inverse() *
  // curr_delta_q).log()).transpose()<<std::endl; std::cout<<"q jaco diff:
  // "<<(step_B.block<3,3>(3,9)*turb).transpose()<<std::endl; std::cout<<"v
  // diff: "<<(curr_delta_v - state_.beta_ij_).transpose()<<std::endl;
  // std::cout<<"v jaco diff:
  // "<<(step_B.block<3,3>(6,9)*turb).transpose()<<std::endl; std::cout<<"ba
  // diff: "<<(curr_ba - state_.b_a_i_).transpose()<<std::endl; std::cout<<"ba
  // jaco diff: "<<(step_B.block<3,3>(9,9)*turb).transpose()<<std::endl;
  // std::cout<<"bg diff: "<<(curr_bg - state_.b_g_i_).transpose()<<std::endl;
  // std::cout<<"bg jaco diff:
  // "<<(step_B.block<3,3>(12,9)*turb).transpose()<<std::endl<<std::endl;
}

void IMUPreIntegrator::CheckJacobian(
    double T, Eigen::Vector3d prev_a, Eigen::Vector3d prev_w,
    Eigen::Vector3d curr_a, Eigen::Vector3d curr_w,
    Eigen::Vector3d prev_delta_p, Sophus::SO3d prev_delta_q,
    Eigen::Vector3d prev_delta_v, Eigen::Vector3d prev_ba,
    Eigen::Vector3d prev_bg, Eigen::Vector3d& curr_delta_p,
    Sophus::SO3d& curr_delta_q, Eigen::Vector3d& curr_delta_v,
    Eigen::Vector3d& curr_ba, Eigen::Vector3d& curr_bg, MatrixF& step_J,
    MatrixB& step_B) {
  // unbiased
  prev_w = prev_w - prev_bg;
  curr_w = curr_w - prev_bg;
  prev_a = prev_a - prev_ba;
  curr_a = curr_a - prev_ba;

  curr_ba = prev_ba;
  curr_bg = prev_bg;

  Eigen::Matrix3d dR_inv = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d prev_R = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d curr_R = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d prev_R_a_hat = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d curr_R_a_hat = Eigen::Matrix3d::Zero();

  Eigen::Vector3d w_mid = 0.5 * (prev_w + curr_w);
  Sophus::SO3d d_theta_ij = Sophus::SO3d::exp(w_mid * T);
  curr_delta_q = prev_delta_q * d_theta_ij;

  Eigen::Vector3d a_mid = 0.5 * (prev_delta_q * prev_a + curr_delta_q * curr_a);
  curr_delta_p = prev_delta_p + (prev_delta_v + 0.5 * a_mid * T) * T;
  curr_delta_v = prev_delta_v + a_mid * T;

  // std::cout<<"check prev_delta_v: "<<prev_delta_v.transpose()<<std::endl;

  dR_inv = d_theta_ij.inverse().matrix();
  prev_R = prev_delta_q.matrix();
  curr_R = curr_delta_q.matrix();
  prev_R_a_hat = prev_R * Sophus::SO3d::hat(prev_a);
  curr_R_a_hat = curr_R * Sophus::SO3d::hat(curr_a);

  MatrixF F = MatrixF::Zero();
  MatrixB B = MatrixB::Zero();
  //
  // 2. set up F:
  //
  // // F12 & F32:
  // F.block<3, 3>(kIndexAlpha, kIndexTheta) = F.block<3, 3>(kIndexBeta,
  // kIndexTheta) = -0.50 * (prev_R_a_hat + curr_R_a_hat * dR_inv); F.block<3,
  // 3>(kIndexAlpha, kIndexTheta) = 0.50 * T * F.block<3, 3>(kIndexAlpha,
  // kIndexTheta);
  // // F14 & F34:
  // F.block<3, 3>(kIndexAlpha,   kIndexBa) = F.block<3, 3>(kIndexBeta,
  // kIndexBa) = -0.50 * (prev_R + curr_R); F.block<3, 3>(kIndexAlpha,
  // kIndexBa) = 0.50 * T * F.block<3, 3>(kIndexAlpha,   kIndexBa);
  // // F15 & F35:
  // F.block<3, 3>(kIndexAlpha,   kIndexBg) = F.block<3, 3>(kIndexBeta,
  // kIndexBg) = +0.50 * T * curr_R_a_hat; F.block<3, 3>(kIndexAlpha,
  // kIndexBg) = 0.50 * T * F.block<3, 3>(kIndexAlpha,   kIndexBg);
  // // F22:
  // F.block<3, 3>(kIndexTheta, kIndexTheta) = -Sophus::SO3d::hat(w_mid);

  // F12 & F32:
  // F32:
  F.block<3, 3>(kIndexBeta, kIndexTheta) =
      -0.5 * (prev_R_a_hat + curr_R_a_hat * dR_inv);
  // F12:
  F.block<3, 3>(kIndexAlpha, kIndexTheta) =
      0.5 * T * F.block<3, 3>(kIndexBeta, kIndexTheta);

  // F14 & F34:
  // F34:
  F.block<3, 3>(kIndexBeta, kIndexBa) = -0.5 * (prev_R + curr_R);
  // F14:
  F.block<3, 3>(kIndexAlpha, kIndexBa) =
      0.5 * T * F.block<3, 3>(kIndexBeta, kIndexBa);

  // F15 & F35:
  F.block<3, 3>(kIndexBeta, kIndexBg) = 0.5 * T * curr_R_a_hat;
  F.block<3, 3>(kIndexAlpha, kIndexBg) =
      0.5 * T * F.block<3, 3>(kIndexBeta, kIndexBg);

  // F22: 前面不用加单位阵 之后 F_=I+F_*T
  F.block<3, 3>(kIndexTheta, kIndexTheta) = -Sophus::SO3d::hat(w_mid);

  F.block<3, 3>(kIndexAlpha, kIndexBeta) = Eigen::Matrix3d::Identity();
  F.block<3, 3>(kIndexTheta, kIndexBg) = -Eigen::Matrix3d::Identity();

  //
  // 3. set up G:
  //
  // G11 & G31:
  // B.block<3, 3>(kIndexAlpha, kIndexNoiseAccPrev) = B.block<3, 3>(kIndexBeta,
  // kIndexNoiseAccPrev) = +0.50 * prev_R; B.block<3, 3>(kIndexAlpha,
  // kIndexNoiseAccPrev) = 0.50 * T * B.block<3, 3>(kIndexAlpha,
  // kIndexNoiseAccPrev);
  // // G12 & G32:
  // B.block<3, 3>(kIndexAlpha, kIndexNoiseGyrPrev) = B.block<3, 3>(kIndexBeta,
  // kIndexNoiseGyrPrev) = -0.25 * T * curr_R_a_hat; B.block<3, 3>(kIndexAlpha,
  // kIndexNoiseGyrPrev) = 0.50 * T * B.block<3, 3>(kIndexAlpha,
  // kIndexNoiseGyrPrev);
  // // G13 & G33:
  // B.block<3, 3>(kIndexAlpha, kIndexNoiseAccCurr) = B.block<3, 3>(kIndexBeta,
  // kIndexNoiseAccCurr) = 0.5 * curr_R; B.block<3, 3>(kIndexAlpha,
  // kIndexNoiseAccCurr) = 0.50 * T * B.block<3, 3>(kIndexAlpha,
  // kIndexNoiseAccCurr);
  // // G14 & G34:
  // B.block<3, 3>(kIndexAlpha, kIndexNoiseGyrCurr) = B.block<3, 3>(kIndexBeta,
  // kIndexNoiseGyrCurr) = -0.25 * T * curr_R_a_hat; B.block<3, 3>(kIndexAlpha,
  // kIndexNoiseGyrCurr) = 0.50 * T * B.block<3, 3>(kIndexAlpha,
  // kIndexNoiseGyrCurr);

  B.block<3, 3>(kIndexBeta, kIndexNoiseAccPrev) = 0.5 * prev_R;
  // G11
  B.block<3, 3>(kIndexAlpha, kIndexNoiseAccPrev) =
      0.5 * T * B.block<3, 3>(kIndexBeta, kIndexNoiseAccPrev);

  // G12 & G32:
  // G32
  B.block<3, 3>(kIndexBeta, kIndexNoiseGyrPrev) = -0.25 * T * curr_R_a_hat;
  // G12
  B.block<3, 3>(kIndexAlpha, kIndexNoiseGyrPrev) =
      0.5 * T * B.block<3, 3>(kIndexBeta, kIndexNoiseGyrPrev);

  // G13 & G33:
  // G33
  B.block<3, 3>(kIndexBeta, kIndexNoiseAccCurr) = 0.5 * curr_R;
  // G13
  B.block<3, 3>(kIndexAlpha, kIndexNoiseAccCurr) =
      0.5 * T * B.block<3, 3>(kIndexBeta, kIndexNoiseAccCurr);

  // G14 & G34:
  B.block<3, 3>(kIndexBeta, kIndexNoiseGyrCurr) = -0.25 * T * curr_R_a_hat;
  B.block<3, 3>(kIndexAlpha, kIndexNoiseGyrCurr) =
      0.5 * T * B.block<3, 3>(kIndexBeta, kIndexNoiseGyrCurr);

  B.block<3, 3>(kIndexTheta, kIndexNoiseGyrPrev) = B.block<3, 3>(
      kIndexTheta, kIndexNoiseGyrCurr) = 0.50 * Eigen::Matrix3d::Identity();
  B.block<3, 3>(kIndexBa, kIndexNoiseBa) =
      B.block<3, 3>(kIndexBg, kIndexNoiseBg) = Eigen::Matrix3d::Identity();

  F = MatrixF::Identity() + T * F;
  B = T * B;

  step_J = F;
  step_B = B;
}

}  // namespace lidar_localization
