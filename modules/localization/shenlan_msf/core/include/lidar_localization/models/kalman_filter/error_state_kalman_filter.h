/*
 * @Description: Error-State Kalman Filter for IMU-Lidar-GNSS-Odo fusion
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-27 23:12:37
 */
#ifndef LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_ERROR_STATE_KALMAN_FILTER_H_
#define LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_ERROR_STATE_KALMAN_FILTER_H_

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <ostream>

#include "glog/logging.h"

// use sophus to handle so3 hat & SO3 log operations:
#include <sophus/so3.hpp>
// models
#include "lidar_localization/models/kalman_filter/kalman_filter.h"
// global
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

class ErrorStateKalmanFilter : public KalmanFilter {
 public:
  ErrorStateKalmanFilter(const YAML::Node& node);

  /**
   * @brief  init filter
   * @param  imu_data, input IMU measurements
   * @return true if success false otherwise
   */
  void Init(const Eigen::Matrix4d& init_pose,
            const Eigen::Vector3d& vel,
            const IMUData& imu_data);

  /**
   * @brief  Kalman update
   * @param  imu_data, input IMU measurements
   * @return true if success false otherwise
   */
  bool Update(const IMUData& imu_data);

  /**
   * @brief  Kalman correction, pose measurement
   * @param  measurement_type, input measurement type
   * @param  measurement, input measurement
   * @return void
   */
  bool Correct(const IMUData& imu_data,
               const MeasurementType& measurement_type,
               const Measurement& measurement);

  /**
   * @brief  Kalman correction, pose measurement and other measurement in body
   * frame
   * @param  T_nb, pose measurement
   * @param  v_b, velocity or magnetometer measurement
   * @return void
   */
  bool Correct(const IMUData& imu_data,
               const double& time,
               const MeasurementType& measurement_type,
               const Eigen::Matrix4f& T_nb,
               const Eigen::Vector3f& v_b);

  /**
   * @brief  get odometry estimation
   * @param  pose, init pose
   * @param  vel, init vel
   * @return void
   */
  void GetOdometry(Eigen::Matrix4f& pose, Eigen::Vector3f& vel);

  /**
   * @brief  get covariance estimation
   * @param  cov, covariance output
   * @return void
   */
  void GetCovariance(Cov& cov);

  /**
   * @brief  update observability analysis
   * @param  time, measurement time
   * @param  measurement_type, measurement type
   * @return void
   */
  void UpdateObservabilityAnalysis(const double& time,
                                   const MeasurementType& measurement_type);

  /**
   * @brief  save observability analysis to persistent storage
   * @param  measurement_type, measurement type
   * @return void
   */
  bool SaveObservabilityAnalysis(const MeasurementType& measurement_type);

 private:
  // indices:
  static constexpr int kDimState{15};

  static constexpr int kIndexErrorPos{0};
  static constexpr int kIndexErrorVel{3};
  static constexpr int kIndexErrorOri{6};
  static constexpr int kIndexErrorAccel{9};
  static constexpr int kIndexErrorGyro{12};

  static constexpr int kDimProcessNoise{12};

  static constexpr int kIndexNoiseAccel{0};
  static constexpr int kIndexNoiseGyro{3};
  static constexpr int kIndexNoiseBiasAccel{6};
  static constexpr int kIndexNoiseBiasGyro{9};

  // dimensions:
  static constexpr int kDimMeasurementPose{6};
  static constexpr int kDimMeasurementPoseNoise{6};

  static constexpr int kDimMeasurementPosePosi{9};
  static constexpr int kDimMeasurementPosePosiNoise{9};

  static constexpr int kDimMeasurementPoseVel{9};
  static constexpr int kDimMeasurementPoseVelNoise{9};

  static constexpr int kDimMeasurementPosiVel{6};
  static constexpr int kDimMeasurementPosiVelNoise{6};

  static constexpr int kDimMeasurementPosi{3};
  static constexpr int kDimMeasurementPosiNoise{3};

  // state:
  using VectorX = Eigen::Matrix<double, kDimState, 1>;
  using MatrixP = Eigen::Matrix<double, kDimState, kDimState>;

  // process equation:
  using MatrixF = Eigen::Matrix<double, kDimState, kDimState>;
  using MatrixB = Eigen::Matrix<double, kDimState, kDimProcessNoise>;
  using MatrixQ = Eigen::Matrix<double, kDimProcessNoise, kDimProcessNoise>;

  // measurement equation:
  using MatrixGPose = Eigen::Matrix<double, kDimMeasurementPose, kDimState>;
  using MatrixCPose =
      Eigen::Matrix<double, kDimMeasurementPose, kDimMeasurementPoseNoise>;
  using MatrixRPose =
      Eigen::Matrix<double, kDimMeasurementPoseNoise, kDimMeasurementPoseNoise>;

  using MatrixGPosePosi =
      Eigen::Matrix<double, kDimMeasurementPosePosi, kDimState>;
  using MatrixCPosePosi = Eigen::
      Matrix<double, kDimMeasurementPosePosi, kDimMeasurementPosePosiNoise>;
  using MatrixRPosePosi = Eigen::Matrix<double,
                                        kDimMeasurementPosePosiNoise,
                                        kDimMeasurementPosePosiNoise>;

  using MatrixGPoseVel =
      Eigen::Matrix<double, kDimMeasurementPoseVel, kDimState>;
  using MatrixCPoseVel = Eigen::
      Matrix<double, kDimMeasurementPoseVel, kDimMeasurementPoseVelNoise>;
  using MatrixRPoseVel = Eigen::
      Matrix<double, kDimMeasurementPoseVelNoise, kDimMeasurementPoseVelNoise>;

  using MatrixGPosiVel =
      Eigen::Matrix<double, kDimMeasurementPosiVel, kDimState>;
  using MatrixCPosiVel = Eigen::
      Matrix<double, kDimMeasurementPosiVel, kDimMeasurementPosiVelNoise>;
  using MatrixRPosiVel = Eigen::
      Matrix<double, kDimMeasurementPosiVelNoise, kDimMeasurementPosiVelNoise>;

  using MatrixGPoseVelCon =
      Eigen::Matrix<double, kDimMeasurementPoseVel - 1, kDimState>;
  using MatrixCPoseVelCon = Eigen::Matrix<double,
                                          kDimMeasurementPoseVel - 1,
                                          kDimMeasurementPoseVelNoise - 1>;
  using MatrixRPoseVelCon = Eigen::Matrix<double,
                                          kDimMeasurementPoseVelNoise - 1,
                                          kDimMeasurementPoseVelNoise - 1>;

  using MatrixGPosi = Eigen::Matrix<double, kDimMeasurementPosi, kDimState>;
  using MatrixCPosi =
      Eigen::Matrix<double, kDimMeasurementPosi, kDimMeasurementPosiNoise>;
  using MatrixRPosi =
      Eigen::Matrix<double, kDimMeasurementPosiNoise, kDimMeasurementPosiNoise>;

  // measurement:
  using VectorYPose = Eigen::Matrix<double, kDimMeasurementPose, 1>;
  using VectorYPosePosi = Eigen::Matrix<double, kDimMeasurementPosePosi, 1>;
  using VectorYPoseVel = Eigen::Matrix<double, kDimMeasurementPoseVel, 1>;
  using VectorYPosiVel = Eigen::Matrix<double, kDimMeasurementPosiVel, 1>;
  using VectorYPoseVelCon =
      Eigen::Matrix<double, kDimMeasurementPoseVel - 1, 1>;
  using VectorYPosi = Eigen::Matrix<double, kDimMeasurementPosi, 1>;

  // Kalman gain:
  using MatrixKPose = Eigen::Matrix<double, kDimState, kDimMeasurementPose>;
  using MatrixKPoseVel =
      Eigen::Matrix<double, kDimState, kDimMeasurementPoseVel>;
  using MatrixKPosiVel =
      Eigen::Matrix<double, kDimState, kDimMeasurementPosiVel>;

  // state observality matrix:
  using MatrixQPose =
      Eigen::Matrix<double, kDimState * kDimMeasurementPose, kDimState>;
  using MatrixQPoseVel =
      Eigen::Matrix<double, kDimState * kDimMeasurementPoseVel, kDimState>;
  using MatrixQPosiVel =
      Eigen::Matrix<double, kDimState * kDimMeasurementPosiVel, kDimState>;

  /**
   * @brief  get unbiased angular velocity in body frame
   * @param  angular_vel, angular velocity measurement
   * @param  R, corresponding orientation of measurement
   * @return unbiased angular velocity in body frame
   */
  Eigen::Vector3d GetUnbiasedAngularVel(const Eigen::Vector3d& angular_vel,
                                        const Eigen::Matrix3d& R);
  /**
   * @brief  get unbiased linear acceleration in navigation frame
   * @param  linear_acc, linear acceleration measurement
   * @param  R, corresponding orientation of measurement
   * @return unbiased linear acceleration in navigation frame
   */
  Eigen::Vector3d GetUnbiasedLinearAcc(const Eigen::Vector3d& linear_acc,
                                       const Eigen::Matrix3d& R);

  /**
   * @brief  get angular delta
   * @param  index_curr, current imu measurement buffer index
   * @param  index_prev, previous imu measurement buffer index
   * @param  angular_delta, angular delta output
   * @return true if success false otherwise
   */
  bool GetAngularDelta(const size_t index_curr,
                       const size_t index_prev,
                       Eigen::Vector3d& angular_delta,
                       Eigen::Vector3d& angular_vel_mid);
  /**
   * @brief  get velocity delta
   * @param  index_curr, current imu measurement buffer index
   * @param  index_prev, previous imu measurement buffer index
   * @param  R_curr, corresponding orientation of current imu measurement
   * @param  R_prev, corresponding orientation of previous imu measurement
   * @param  velocity_delta, velocity delta output
   * @param  linear_acc_mid, mid-value unbiased linear acc
   * @return true if success false otherwise
   */
  bool GetVelocityDelta(const size_t index_curr,
                        const size_t index_prev,
                        const Eigen::Matrix3d& R_curr,
                        const Eigen::Matrix3d& R_prev,
                        double& T,
                        Eigen::Vector3d& velocity_delta,
                        Eigen::Vector3d& linear_acc_mid);
  /**
   * @brief  update orientation with effective rotation angular_delta
   * @param  angular_delta, effective rotation
   * @param  R_curr, current orientation
   * @param  R_prev, previous orientation
   * @return void
   */
  void UpdateOrientation(const Eigen::Vector3d& angular_delta,
                         Eigen::Matrix3d& R_curr,
                         Eigen::Matrix3d& R_prev);
  /**
   * @brief  update orientation with effective velocity change velocity_delta
   * @param  velocity_delta, effective velocity change
   * @return void
   */
  void UpdatePosition(const double T, const Eigen::Vector3d& velocity_delta);
  /**
   * @brief  update IMU odometry estimation
   * @param  linear_acc_mid, output mid-value unbiased linear acc
   * @return void
   */
  void UpdateOdomEstimation(Eigen::Vector3d& linear_acc_mid,
                            Eigen::Vector3d& angular_vel_mid);

  /**
   * @brief  set process equation
   * @param  C_nb, rotation matrix, body frame -> navigation frame
   * @param  f_n, accel measurement in navigation frame
   * @return void
   */
  void SetProcessEquation(const Eigen::Matrix3d& C_nb,
                          const Eigen::Vector3d& f_b,
                          const Eigen::Vector3d& w_n);
  /**
   * @brief  update process equation
   * @param  linear_acc_mid, input mid-value unbiased linear acc
   * @return void
   */
  void UpdateProcessEquation(const Eigen::Vector3d& linear_acc_mid,
                             const Eigen::Vector3d& angular_vel_mid);

  /**
   * @brief  update error estimation
   * @param  linear_acc_mid, input mid-value unbiased linear acc
   * @return void
   */
  void UpdateErrorEstimation(const double T,
                             const Eigen::Vector3d& linear_acc_mid,
                             const Eigen::Vector3d& angular_vel_mid);

  /**
   * @brief  correct error estimation using pose measurement
   * @param  T_nb, input pose measurement
   * @return void
   */
  void CorrectErrorEstimationPose(const Eigen::Matrix4d& T_nb,
                                  Eigen::VectorXd& Y,
                                  Eigen::MatrixXd& G,
                                  Eigen::MatrixXd& K);

  void CorrectErrorEstimationPosePosi(const Eigen::Matrix4d& T_nb,
                                      const Eigen::Vector3d& p_nb,
                                      Eigen::VectorXd& Y,
                                      Eigen::MatrixXd& G,
                                      Eigen::MatrixXd& K);

  /**
   * @brief  correct error estimation using pose and body velocity measurement
   * @param  T_nb, input pose measurement
   * @param  v_b, input velocity measurement
   * @return void
   */
  void CorrectErrorEstimationPoseVel(const Eigen::Matrix4d& T_nb,
                                     const Eigen::Vector3d& v_b,
                                     const Eigen::Vector3d& w_b,
                                     Eigen::VectorXd& Y,
                                     Eigen::MatrixXd& G,
                                     Eigen::MatrixXd& K);

  void CorrectErrorEstimationPoseVelConstrain(const Eigen::Matrix4d& T_nb,
                                              Eigen::VectorXd& Y,
                                              Eigen::MatrixXd& G,
                                              Eigen::MatrixXd& K);

  /**
   * @brief  correct error estimation using navigation position and body
   * velocity measurement
   * @param  T_nb, input position measurement
   * @param  v_b, input velocity measurement
   * @return void
   */
  void CorrectErrorEstimationPosiVel(const Eigen::Matrix4d& T_nb,
                                     const Eigen::Vector3d& v_b,
                                     const Eigen::Vector3d& w_b,
                                     Eigen::VectorXd& Y,
                                     Eigen::MatrixXd& G,
                                     Eigen::MatrixXd& K);

  void CorrectErrorEstimationPosi(const Eigen::Matrix4d& T_nb,
                                  Eigen::VectorXd& Y,
                                  Eigen::MatrixXd& G,
                                  Eigen::MatrixXd& K);
  /**
   * @brief  correct error estimation
   * @param  measurement_type, measurement type
   * @param  measurement, input measurement
   * @return void
   */
  void CorrectErrorEstimation(const MeasurementType& measurement_type,
                              const Measurement& measurement);

  /**
   * @brief  eliminate error
   * @param  void
   * @return void
   */
  void EliminateError(void);

  /**
   * @brief  is covariance stable
   * @param  INDEX_OFSET, state index offset
   * @param  THRESH, covariance threshold, defaults to 1.0e-5
   * @return void
   */
  bool IsCovStable(const int INDEX_OFSET, const double THRESH = 1.0e-5);

  /**
   * @brief  reset filter state
   * @param  void
   * @return void
   */
  void ResetState(void);
  /**
   * @brief  reset filter covariance
   * @param  void
   * @return void
   */
  void ResetCovariance(void);

  /**
   * @brief  get Q analysis for pose measurement
   * @param  void
   * @return void
   */
  void GetQPose(Eigen::MatrixXd& Q, Eigen::VectorXd& Y);

  /**
   * @brief determine if car is turning
   * @param w_b angular velocity in body frame
   * @return true
   * @return false
   */
  bool IsTurning(const Eigen::Vector3d& w_b);

  // odometry estimation from IMU integration:
  Eigen::Matrix4d init_pose_ = Eigen::Matrix4d::Identity();

  Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();
  Eigen::Vector3d vel_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d accl_bias_ = Eigen::Vector3d::Zero();

  // state:
  VectorX X_ = VectorX::Zero();
  MatrixP P_ = MatrixP::Zero();
  // process & measurement equations:
  MatrixF F_ = MatrixF::Zero();
  MatrixB B_ = MatrixB::Zero();
  MatrixQ Q_ = MatrixQ::Zero();

  MatrixGPose GPose_ = MatrixGPose::Zero();
  MatrixCPose CPose_ = MatrixCPose::Zero();
  MatrixRPose RPose_ = MatrixRPose::Zero();
  MatrixQPose QPose_ = MatrixQPose::Zero();

  MatrixGPosePosi GPosePosi_ = MatrixGPosePosi::Zero();
  MatrixCPosePosi CPosePosi_ = MatrixCPosePosi::Zero();
  MatrixRPosePosi RPosePosi_ = MatrixRPosePosi::Zero();

  MatrixGPoseVel GPoseVel_ = MatrixGPoseVel::Zero();
  MatrixCPoseVel CPoseVel_ = MatrixCPoseVel::Zero();
  MatrixRPoseVel RPoseVel_ = MatrixRPoseVel::Zero();
  MatrixQPoseVel QPoseVel_ = MatrixQPoseVel::Zero();

  MatrixGPoseVelCon GPoseVelCon_ = MatrixGPoseVelCon::Zero();
  MatrixCPoseVelCon CPoseVelCon_ = MatrixCPoseVelCon::Zero();
  MatrixRPoseVelCon RPoseVelCon_ = MatrixRPoseVelCon::Zero();

  MatrixGPosiVel GPosiVel_ = MatrixGPosiVel::Zero();
  MatrixCPosiVel CPosiVel_ = MatrixCPosiVel::Zero();
  MatrixRPosiVel RPosiVel_ = MatrixRPosiVel::Zero();
  MatrixQPosiVel QPosiVel_ = MatrixQPosiVel::Zero();

  MatrixGPosi GPosi_ = MatrixGPosi::Zero();
  MatrixCPosi CPosi_ = MatrixCPosi::Zero();
  MatrixRPosi RPosi_ = MatrixRPosi::Zero();

  // measurement:
  VectorYPose YPose_ = VectorYPose::Zero();
  VectorYPosePosi YPosePosi_ = VectorYPosePosi::Zero();
  VectorYPoseVel YPoseVel_ = VectorYPoseVel::Zero();
  VectorYPosiVel YPosiVel_ = VectorYPosiVel::Zero();
  VectorYPoseVelCon YPoseVelCon_ = VectorYPoseVelCon::Zero();
  VectorYPosi YPosi_ = VectorYPosi::Zero();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_ERROR_STATE_KALMAN_FILTER_H_
