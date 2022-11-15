/*
 * @Description: IMU pre-integrator for LIO mapping, interface
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-25 20:13:12
 */
#ifndef LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_IMU_PRE_INTEGRATOR_H_
#define LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_IMU_PRE_INTEGRATOR_H_

#include <sophus/so3.hpp>

#include "lidar_localization/models/graph_optimizer/g2o/edge/edge_prvag_imu_pre_integration.h"
#include "lidar_localization/models/pre_integrator/pre_integrator.h"
#include "lidar_localization/sensor_data/imu_data.h"

namespace lidar_localization {

class IMUPreIntegrator : public PreIntegrator {
 public:
  static const int kDimState{15};

  using MatrixP = Eigen::Matrix<double, kDimState, kDimState>;
  using MatrixJ = Eigen::Matrix<double, kDimState, kDimState>;

  struct IMUPreIntegration {
    Vector15d GetMeasurement() const {
      Vector15d measurement = Vector15d::Zero();

      measurement.block<3, 1>(g2o::EdgePRVAGIMUPreIntegration::kIndexP, 0) =
          alpha_ij;
      measurement.block<3, 1>(g2o::EdgePRVAGIMUPreIntegration::kIndexR, 0) =
          theta_ij.log();
      measurement.block<3, 1>(g2o::EdgePRVAGIMUPreIntegration::kIndexV, 0) =
          beta_ij;

      measurement.block<3, 1>(g2o::EdgePRVAGIMUPreIntegration::kIndexA, 0) =
          b_a_i;
      measurement.block<3, 1>(g2o::EdgePRVAGIMUPreIntegration::kIndexG, 0) =
          b_g_i;

      return measurement;
    }

    Eigen::MatrixXd GetInformation() const { return P.inverse(); }

    // time delta:
    double T{0.0};
    // gravity constant:
    Eigen::Vector3d g = Eigen::Vector3d::Zero();
    // a. measurement:
    // a.1. relative translation:
    Eigen::Vector3d alpha_ij = Eigen::Vector3d::Zero();
    // a.2. relative orientation:
    Sophus::SO3d theta_ij;
    // a.3. relative velocity:
    Eigen::Vector3d beta_ij = Eigen::Vector3d::Zero();
    // a.4. accel bias:
    Eigen::Vector3d b_a_i = Eigen::Vector3d::Zero();
    // a.5. gyro bias:
    Eigen::Vector3d b_g_i = Eigen::Vector3d::Zero();
    // b. information:
    MatrixP P = MatrixP::Zero();
    // c. Jacobian for update caused by bias:
    MatrixJ J = MatrixJ::Zero();
  };

  IMUPreIntegrator(const YAML::Node& node);

  /**
   * @brief  init IMU pre-integrator
   * @param  init_imu_data, init IMU measurements
   * @return true if success false otherwise
   */
  bool Init(const IMUData& init_imu_data);

  /**
   * @brief  update IMU pre-integrator
   * @param  imu_data, current IMU measurements
   * @return true if success false otherwise
   */
  bool Update(const IMUData& imu_data);

  /**
   * @brief  reset IMU pre-integrator using new init IMU measurement
   * @param  init_imu_data, new init IMU measurements
   * @param  output pre-integration result for constraint building as
   * IMUPreIntegration
   * @return true if success false otherwise
   */
  bool Reset(const IMUData& init_imu_data,
             IMUPreIntegration& imu_pre_integration);

  void GetIMUPreIntegration(IMUPreIntegration& imu_pre_integration, const double &time);

 private:
  static const int kDimNoise{18};

  static const int kIndexAlpha{0};
  static const int kIndexTheta{3};
  static const int kIndexBeta{6};
  static const int kIndexBa{9};
  static const int kIndexBg{12};

  static const int kIndexNoiseAccPrev{0};
  static const int kIndexNoiseGyrPrev{3};
  static const int kIndexNoiseAccCurr{6};
  static const int kIndexNoiseGyrCurr{9};
  static const int kIndexNoiseBa{12};
  static const int kIndexNoiseBg{15};

  using MatrixF = Eigen::Matrix<double, kDimState, kDimState>;
  using MatrixB = Eigen::Matrix<double, kDimState, kDimNoise>;
  using MatrixQ = Eigen::Matrix<double, kDimNoise, kDimNoise>;

  /**
   * @brief  reset pre-integrator state using IMU measurements
   * @param  void
   * @return void
   */
  void ResetState(const IMUData& init_imu_data);

  /**
   * @brief  update pre-integrator state: mean, covariance and Jacobian
   * @param  void
   * @return void
   */
  void UpdateState();

  void CheckJacobian(double T, Eigen::Vector3d prev_a, Eigen::Vector3d prev_w,
                     Eigen::Vector3d curr_a, Eigen::Vector3d curr_w,
                     Eigen::Vector3d prev_delta_p, Sophus::SO3d prev_delta_q,
                     Eigen::Vector3d prev_delta_v, Eigen::Vector3d prev_ba,
                     Eigen::Vector3d prev_bg, Eigen::Vector3d& curr_delta_p,
                     Sophus::SO3d& curr_delta_q, Eigen::Vector3d& curr_delta_v,
                     Eigen::Vector3d& curr_ba, Eigen::Vector3d& curr_bg,
                     MatrixF& step_J, MatrixB& step_B);

  // data buff:
  std::deque<IMUData> imu_data_buff_;

  // hyper-params:
  // a. earth constants:
  struct {
    double gravity_manitude{0.0};
  } earth_;
  // b. prior state covariance, process & measurement noise:
  struct {
    struct {
      double accel{0.0};
      double gyro{0.0};
    } random_walk;
    struct {
      double accel{0.0};
      double gyro{0.0};
    } measurement;
  } cov_;

  // pre-integration state:
  struct PreIntegrationState{
    // gravity constant:
    Eigen::Vector3d g = Eigen::Vector3d::Zero();

    // a. relative translation:
    Eigen::Vector3d alpha_ij = Eigen::Vector3d::Zero();
    // b. relative orientation:
    Sophus::SO3d theta_ij;
    // c. relative velocity:
    Eigen::Vector3d beta_ij = Eigen::Vector3d::Zero();
    // d. accel bias:
    Eigen::Vector3d b_a_i = Eigen::Vector3d::Zero();
    // e. gyro bias:
    Eigen::Vector3d b_g_i = Eigen::Vector3d::Zero();
  } state_;

  // state covariance:
  MatrixP P_ = MatrixP::Zero();

  // Jacobian:
  MatrixJ J_ = MatrixJ::Identity();

  // process noise:
  MatrixQ Q_ = MatrixQ::Zero();

  // process equation:
  MatrixF F_ = MatrixF::Zero();
  MatrixB B_ = MatrixB::Zero();
};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_IMU_PRE_INTEGRATOR_H_
