/*
 * @Description: Error-State Kalman Filter for IMU-Lidar-GNSS-Odo fusion
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-27 23:19:37
 */
#include "lidar_localization/models/kalman_filter/error_state_kalman_filter.h"

namespace lidar_localization {

ErrorStateKalmanFilter::ErrorStateKalmanFilter(const YAML::Node& node) {
  //
  // parse config:
  //
  // a. earth constants:
  earth_.gravity_magnitude = node["earth"]["gravity_magnitude"].as<double>();
  earth_.latitude = node["earth"]["latitude"].as<double>();
  earth_.latitude *= M_PI / 180.0;

  // b. prior state covariance:
  cov_.prior.posi = node["covariance"]["prior"]["pos"].as<double>();
  cov_.prior.vel = node["covariance"]["prior"]["vel"].as<double>();
  cov_.prior.ori = node["covariance"]["prior"]["ori"].as<double>();
  cov_.prior.epslion = node["covariance"]["prior"]["epsilon"].as<double>();
  cov_.prior.delta = node["covariance"]["prior"]["delta"].as<double>();

  // c. process noise:
  cov_.process.accel = node["covariance"]["process"]["accel"].as<double>();
  cov_.process.gyro = node["covariance"]["process"]["gyro"].as<double>();
  cov_.process.bias_accel =
      node["covariance"]["process"]["bias_accel"].as<double>();
  cov_.process.bias_gyro =
      node["covariance"]["process"]["bias_gyro"].as<double>();

  // d. measurement noise:
  cov_.measurement.pose.posi =
      node["covariance"]["measurement"]["pose"]["pos"].as<double>();
  cov_.measurement.pose.ori =
      node["covariance"]["measurement"]["pose"]["ori"].as<double>();
  cov_.measurement.posi = node["covariance"]["measurement"]["pos"].as<double>();
  cov_.measurement.vel = node["covariance"]["measurement"]["vel"].as<double>();
  // e. motion constraint:
  motion_constraint_.activated =
      node["motion_constraint"]["activated"].as<bool>();
  motion_constraint_.w_b_thresh =
      node["motion_constraint"]["w_b_thresh"].as<double>();

  // prompt:
  LOG(INFO) << std::endl
            << "Error-State Kalman Filter params:" << std::endl
            << "\tgravity magnitude: " << earth_.gravity_magnitude << std::endl
            << "\tlatitude: " << earth_.latitude << std::endl
            << std::endl
            << "\tprior cov. pos.: " << cov_.prior.posi << std::endl
            << "\tprior cov. vel.: " << cov_.prior.vel << std::endl
            << "\tprior cov. ori: " << cov_.prior.ori << std::endl
            << "\tprior cov. epsilon.: " << cov_.prior.epslion << std::endl
            << "\tprior cov. delta.: " << cov_.prior.delta << std::endl
            << std::endl
            << "\tprocess noise gyro.: " << cov_.process.gyro << std::endl
            << "\tprocess noise accel.: " << cov_.process.accel << std::endl
            << std::endl
            << "\tmeasurement noise pose.: " << std::endl
            << "\t\tpos: " << cov_.measurement.pose.posi
            << ", ori.: " << cov_.measurement.pose.ori << std::endl
            << "\tmeasurement noise pos.: " << cov_.measurement.posi
            << std::endl
            << "\tmeasurement noise vel.: " << cov_.measurement.vel << std::endl
            << std::endl
            << "\tmotion constraint: " << std::endl
            << "\t\tactivated: "
            << (motion_constraint_.activated ? "true" : "false") << std::endl
            << "\t\tw_b threshold: " << motion_constraint_.w_b_thresh
            << std::endl
            << std::endl;

  //
  // init filter:
  //
  // a. earth constants:
  g_ = Eigen::Vector3d(0.0, 0.0, earth_.gravity_magnitude);
  // b. prior state & covariance:
  ResetState();
  ResetCovariance();

  // c. process noise:
  Q_.block<3, 3>(kIndexNoiseAccel, kIndexNoiseAccel) =
      cov_.process.accel * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(kIndexNoiseGyro, kIndexNoiseGyro) =
      cov_.process.gyro * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(kIndexNoiseBiasAccel, kIndexNoiseBiasAccel) =
      cov_.process.bias_accel * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(kIndexNoiseBiasGyro, kIndexNoiseBiasGyro) =
      cov_.process.bias_gyro * Eigen::Matrix3d::Identity();

  // d. measurement noise:
  RPose_.block<3, 3>(0, 0) =
      cov_.measurement.pose.posi * Eigen::Matrix3d::Identity();
  RPose_.block<3, 3>(3, 3) =
      cov_.measurement.pose.ori * Eigen::Matrix3d::Identity();

  RPosePosi_.block<3, 3>(0, 0) =
      cov_.measurement.pose.posi * Eigen::Matrix3d::Identity();
  RPosePosi_.block<3, 3>(3, 3) =
      cov_.measurement.pose.ori * Eigen::Matrix3d::Identity();
  RPosePosi_.block<3, 3>(6, 6) =
      cov_.measurement.posi * Eigen::Matrix3d::Identity();

  RPoseVel_.block<3, 3>(0, 0) =
      cov_.measurement.pose.posi * Eigen::Matrix3d::Identity();
  RPoseVel_.block<3, 3>(6, 6) =
      cov_.measurement.pose.ori * Eigen::Matrix3d::Identity();
  RPoseVel_.block<3, 3>(3, 3) =
      cov_.measurement.vel * Eigen::Matrix3d::Identity();

  RPoseVelCon_.block<3, 3>(0, 0) =
      cov_.measurement.pose.posi * Eigen::Matrix3d::Identity();
  RPoseVelCon_.block<2, 2>(3, 3) =
      cov_.measurement.vel * Eigen::Matrix2d::Identity();
  RPoseVelCon_.block<3, 3>(5, 5) =
      cov_.measurement.pose.ori * Eigen::Matrix3d::Identity();

  RPosiVel_.block<3, 3>(0, 0) =
      cov_.measurement.posi * Eigen::Matrix3d::Identity();
  RPosiVel_.block<3, 3>(3, 3) =
      cov_.measurement.vel * Eigen::Matrix3d::Identity();

  RPosi_.block<3, 3>(0, 0) =
      cov_.measurement.posi * Eigen::Matrix3d::Identity();

  // e. process equation:
  F_.block<3, 3>(kIndexErrorPos, kIndexErrorVel) = Eigen::Matrix3d::Identity();
  F_.block<3, 3>(kIndexErrorOri, kIndexErrorGyro) =
      -Eigen::Matrix3d::Identity();

  B_.block<3, 3>(kIndexErrorOri, kIndexNoiseGyro) = Eigen::Matrix3d::Identity();
  B_.block<3, 3>(kIndexErrorAccel, kIndexNoiseBiasAccel) =
      Eigen::Matrix3d::Identity();
  B_.block<3, 3>(kIndexErrorGyro, kIndexNoiseBiasGyro) =
      Eigen::Matrix3d::Identity();

  // f. measurement equation:
  GPose_.block<3, 3>(0, kIndexErrorPos) = Eigen::Matrix3d::Identity();
  GPose_.block<3, 3>(3, kIndexErrorOri) = Eigen::Matrix3d::Identity();
  CPose_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  CPose_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

  GPosePosi_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  GPosePosi_.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();
  GPosePosi_.block<3, 3>(6, 0) = Eigen::Matrix3d::Identity();
  CPosePosi_ = MatrixCPosePosi::Identity();

  GPoseVel_.block<3, 3>(0, kIndexErrorPos) = Eigen::Matrix3d::Identity();
  GPoseVel_.block<3, 3>(6, kIndexErrorOri) = Eigen::Matrix3d::Identity();
  CPoseVel_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  CPoseVel_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
  CPoseVel_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();

  GPoseVelCon_.block<3, 3>(0, kIndexErrorPos) = Eigen::Matrix3d::Identity();
  GPoseVelCon_.block<3, 3>(5, kIndexErrorOri) = Eigen::Matrix3d::Identity();
  CPoseVelCon_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  CPoseVelCon_.block<2, 2>(3, 3) = Eigen::Matrix2d::Identity();
  CPoseVelCon_.block<3, 3>(5, 5) = Eigen::Matrix3d::Identity();

  GPosiVel_.block<3, 3>(0, kIndexErrorPos) = Eigen::Matrix3d::Identity();
  CPosiVel_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  CPosiVel_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

  GPosi_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  CPosi_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

  // init soms:
  QPose_.block<kDimMeasurementPose, kDimState>(0, 0) = GPose_;
  QPoseVel_.block<kDimMeasurementPoseVel, kDimState>(0, 0) = GPoseVel_;
  QPosiVel_.block<kDimMeasurementPosiVel, kDimState>(0, 0) = GPosiVel_;
}

/**
 * @brief  init filter
 * @param  pose, init pose
 * @param  vel, init vel
 * @param  imu_data, init IMU measurements
 * @return true if success false otherwise
 */
void ErrorStateKalmanFilter::Init(const Eigen::Matrix4d& init_pose,
                                  const Eigen::Vector3d& vel,
                                  const IMUData& imu_data) {
  // init odometry:
  // Eigen::Matrix3d C_nb = imu_data.GetOrientationMatrix().cast<double>();
  Eigen::Matrix3d C_nb = init_pose.block<3, 3>(0, 0);
  // a. init C_nb using IMU estimation:
  // pose_.block<3, 3>(0, 0) = C_nb;
  pose_ = init_pose;
  // b. convert flu velocity into navigation frame:
  vel_ = C_nb * vel;

  // save init pose:
  // init_pose_ = pose_;
  init_pose_ = init_pose;

  // init IMU data buffer:
  imu_data_buff_.clear();
  imu_data_buff_.push_back(imu_data);

  // init filter time:
  time_ = imu_data.time_;

  // set process equation in case of one step prediction & correction:
  Eigen::Vector3d linear_acc_init(imu_data.linear_acceleration_.x,
                                  imu_data.linear_acceleration_.y,
                                  imu_data.linear_acceleration_.z);
  Eigen::Vector3d angular_vel_init(imu_data.angular_velocity_.x,
                                   imu_data.angular_velocity_.y,
                                   imu_data.angular_velocity_.z);

  // using acc in body frame
  linear_acc_init = linear_acc_init - accl_bias_;
  // using gyr in navigation frame
  angular_vel_init = GetUnbiasedAngularVel(angular_vel_init, C_nb);

  // init process equation, in case of direct correct step:
  UpdateProcessEquation(linear_acc_init, angular_vel_init);

  LOG(INFO) << std::endl
            << "Kalman Filter Inited at " << static_cast<int>(time_)
            << std::endl
            << "Init Position: " << pose_(0, 3) << ", " << pose_(1, 3) << ", "
            << pose_(2, 3) << std::endl
            << "Init Velocity: " << vel_.x() << ", " << vel_.y() << ", "
            << vel_.z() << std::endl;
}

/**
 * @brief  Kalman update
 * @param  imu_data, input IMU measurements
 * @return true if success false otherwise
 */
bool ErrorStateKalmanFilter::Update(const IMUData& imu_data) {
  //
  // TODO: understand ESKF update workflow
  //
  // update IMU buff:
  if (time_ < imu_data.time_) {
    // update IMU odometry:
    Eigen::Vector3d linear_acc_mid;
    Eigen::Vector3d angular_vel_mid;
    imu_data_buff_.push_back(imu_data);
    UpdateOdomEstimation(linear_acc_mid, angular_vel_mid);
    imu_data_buff_.pop_front();

    // update error estimation:
    double T = imu_data.time_ - time_;
    UpdateErrorEstimation(T, linear_acc_mid, angular_vel_mid);

    // move forward:
    time_ = imu_data.time_;

    return true;
  }

  return false;
}

/**
 * @brief  Kalman correction, pose measurement and other measurement in body
 * frame
 * @param  measurement_type, input measurement type
 * @param  measurement, input measurement
 * @return void
 */
bool ErrorStateKalmanFilter::Correct(const IMUData& imu_data,
                                     const MeasurementType& measurement_type,
                                     const Measurement& measurement) {
  static Measurement measurement_;

  // get time delta:
  const double time_delta = measurement.time - time_;
  const double half_sampling_time = 0.05;  // LiDAR is 10hz
  if (time_delta > -half_sampling_time) {
    // perform Kalman prediction:
    if (time_ < measurement.time) {
      Update(imu_data);
    }

    // get observation in navigation frame:
    measurement_ = measurement;
    // measurement_.T_nb = init_pose_ * measurement_.T_nb;
    measurement_.T_nb = measurement_.T_nb;

    // correct error estimation:
    CorrectErrorEstimation(measurement_type, measurement_);

    // eliminate error:
    EliminateError();

    // reset error state:
    ResetState();

    return true;
  }

  LOG(INFO) << "ESKF Correct: Observation is not synced with filter. Skip, "
            << (int)measurement.time << " <-- " << (int)time_ << " @ "
            << time_delta << std::endl;

  return false;
}

/**
 * @brief  get odometry estimation
 * @param  pose, init pose
 * @param  vel, init vel
 * @return void
 */
void ErrorStateKalmanFilter::GetOdometry(Eigen::Matrix4f& pose,
                                         Eigen::Vector3f& vel) {
  // init:
  Eigen::Matrix4d pose_double = pose_;
  Eigen::Vector3d vel_double = vel_;

  // 这里的误差状态向量X都是0可以不用更新
  // // eliminate error:
  // // a. position:
  // pose_double.block<3, 1>(0, 3) =
  //     pose_double.block<3, 1>(0, 3) - X_.block<3, 1>(kIndexErrorPos, 0);
  // // b. velocity:
  // vel_double = vel_double - X_.block<3, 1>(kIndexErrorVel, 0);
  // // c. orientation:
  // Eigen::Matrix3d C_nn =
  //     Sophus::SO3d::exp(X_.block<3, 1>(kIndexErrorOri, 0)).matrix();
  // pose_double.block<3, 3>(0, 0) = C_nn * pose_double.block<3, 3>(0, 0);

  // // finally:
  // pose_double = init_pose_.inverse() * pose_double;
  // vel_double = init_pose_.block<3, 3>(0, 0).transpose() * vel_double;

  pose = pose_double.cast<float>();
  vel = vel_double.cast<float>();
}

/**
 * @brief  get covariance estimation
 * @param  cov, covariance output
 * @return void
 */
void ErrorStateKalmanFilter::GetCovariance(Cov& cov) {
  static int OFFSET_X = 0;
  static int OFFSET_Y = 1;
  static int OFFSET_Z = 2;

  // a. delta position:
  cov.pos.x = P_(kIndexErrorPos + OFFSET_X, kIndexErrorPos + OFFSET_X);
  cov.pos.y = P_(kIndexErrorPos + OFFSET_Y, kIndexErrorPos + OFFSET_Y);
  cov.pos.z = P_(kIndexErrorPos + OFFSET_Z, kIndexErrorPos + OFFSET_Z);

  // b. delta velocity:
  cov.vel.x = P_(kIndexErrorVel + OFFSET_X, kIndexErrorVel + OFFSET_X);
  cov.vel.y = P_(kIndexErrorVel + OFFSET_Y, kIndexErrorVel + OFFSET_Y);
  cov.vel.z = P_(kIndexErrorVel + OFFSET_Z, kIndexErrorVel + OFFSET_Z);

  // c. delta orientation:
  cov.ori.x = P_(kIndexErrorOri + OFFSET_X, kIndexErrorOri + OFFSET_X);
  cov.ori.y = P_(kIndexErrorOri + OFFSET_Y, kIndexErrorOri + OFFSET_Y);
  cov.ori.z = P_(kIndexErrorOri + OFFSET_Z, kIndexErrorOri + OFFSET_Z);

  // d. gyro. bias:
  cov.gyro_bias.x = P_(kIndexErrorGyro + OFFSET_X, kIndexErrorGyro + OFFSET_X);
  cov.gyro_bias.y = P_(kIndexErrorGyro + OFFSET_Y, kIndexErrorGyro + OFFSET_Y);
  cov.gyro_bias.z = P_(kIndexErrorGyro + OFFSET_Z, kIndexErrorGyro + OFFSET_Z);

  // e. accel bias:
  cov.accel_bias.x =
      P_(kIndexErrorAccel + OFFSET_X, kIndexErrorAccel + OFFSET_X);
  cov.accel_bias.y =
      P_(kIndexErrorAccel + OFFSET_Y, kIndexErrorAccel + OFFSET_Y);
  cov.accel_bias.z =
      P_(kIndexErrorAccel + OFFSET_Z, kIndexErrorAccel + OFFSET_Z);
}

/**
 * @brief  get unbiased angular velocity in body frame
 * @param  angular_vel, angular velocity measurement
 * @param  R, corresponding orientation of measurement
 * @return unbiased angular velocity in body frame
 */
inline Eigen::Vector3d ErrorStateKalmanFilter::GetUnbiasedAngularVel(
    const Eigen::Vector3d& angular_vel, const Eigen::Matrix3d& R) {
  return angular_vel - gyro_bias_;
}

/**
 * @brief  get unbiased linear acceleration in navigation frame
 * @param  linear_acc, linear acceleration measurement
 * @param  R, corresponding orientation of measurement
 * @return unbiased linear acceleration in navigation frame
 */
inline Eigen::Vector3d ErrorStateKalmanFilter::GetUnbiasedLinearAcc(
    const Eigen::Vector3d& linear_acc, const Eigen::Matrix3d& R) {
  return R * (linear_acc - accl_bias_) - g_;
}

/**
 * @brief  get angular delta
 * @param  index_curr, current imu measurement buffer index
 * @param  index_prev, previous imu measurement buffer index
 * @param  angular_delta, angular delta output
 * @return true if success false otherwise
 */
bool ErrorStateKalmanFilter::GetAngularDelta(const size_t index_curr,
                                             const size_t index_prev,
                                             Eigen::Vector3d& angular_delta,
                                             Eigen::Vector3d& angular_vel_mid) {
  if (index_curr <= index_prev || imu_data_buff_.size() <= index_curr) {
    return false;
  }

  const IMUData& imu_data_curr = imu_data_buff_.at(index_curr);
  const IMUData& imu_data_prev = imu_data_buff_.at(index_prev);

  double delta_t = imu_data_curr.time_ - imu_data_prev.time_;

  Eigen::Vector3d angular_vel_curr =
      Eigen::Vector3d(imu_data_curr.angular_velocity_.x,
                      imu_data_curr.angular_velocity_.y,
                      imu_data_curr.angular_velocity_.z);
  Eigen::Matrix3d R_curr = imu_data_curr.GetOrientationMatrix().cast<double>();
  angular_vel_curr = GetUnbiasedAngularVel(angular_vel_curr, R_curr);

  Eigen::Vector3d angular_vel_prev =
      Eigen::Vector3d(imu_data_prev.angular_velocity_.x,
                      imu_data_prev.angular_velocity_.y,
                      imu_data_prev.angular_velocity_.z);
  Eigen::Matrix3d R_prev = imu_data_prev.GetOrientationMatrix().cast<double>();
  angular_vel_prev = GetUnbiasedAngularVel(angular_vel_prev, R_prev);

  angular_delta = 0.5 * delta_t * (angular_vel_curr + angular_vel_prev);

  angular_vel_mid = 0.5 * (angular_vel_curr + angular_vel_prev);

  return true;
}

/**
 * @brief  update orientation with effective rotation angular_delta
 * @param  angular_delta, effective rotation
 * @param  R_curr, current orientation
 * @param  R_prev, previous orientation
 * @return void
 */
void ErrorStateKalmanFilter::UpdateOrientation(
    const Eigen::Vector3d& angular_delta,
    Eigen::Matrix3d& R_curr,
    Eigen::Matrix3d& R_prev) {
  // magnitude:
  double angular_delta_mag = angular_delta.norm();
  // direction:
  Eigen::Vector3d angular_delta_dir = angular_delta.normalized();

  // build delta q:
  double angular_delta_cos = cos(angular_delta_mag / 2.0);
  double angular_delta_sin = sin(angular_delta_mag / 2.0);
  Eigen::Quaterniond dq(angular_delta_cos,
                        angular_delta_sin * angular_delta_dir.x(),
                        angular_delta_sin * angular_delta_dir.y(),
                        angular_delta_sin * angular_delta_dir.z());
  Eigen::Quaterniond q(pose_.block<3, 3>(0, 0));

  // update:
  q = q * dq;

  // write back:
  R_prev = pose_.block<3, 3>(0, 0);
  pose_.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  R_curr = pose_.block<3, 3>(0, 0);
}

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
bool ErrorStateKalmanFilter::GetVelocityDelta(const size_t index_curr,
                                              const size_t index_prev,
                                              const Eigen::Matrix3d& R_curr,
                                              const Eigen::Matrix3d& R_prev,
                                              double& T,
                                              Eigen::Vector3d& velocity_delta,
                                              Eigen::Vector3d& linear_acc_mid) {
  if (index_curr <= index_prev || imu_data_buff_.size() <= index_curr) {
    return false;
  }

  const IMUData& imu_data_curr = imu_data_buff_.at(index_curr);
  const IMUData& imu_data_prev = imu_data_buff_.at(index_prev);

  T = imu_data_curr.time_ - imu_data_prev.time_;

  // a_m_curr
  Eigen::Vector3d linear_acc_curr =
      Eigen::Vector3d(imu_data_curr.linear_acceleration_.x,
                      imu_data_curr.linear_acceleration_.y,
                      imu_data_curr.linear_acceleration_.z);
  // a_curr = R(a_m_curr-ba) - g
  Eigen::Vector3d a_curr = GetUnbiasedLinearAcc(linear_acc_curr, R_curr);

  // a_m_prev
  Eigen::Vector3d linear_acc_prev =
      Eigen::Vector3d(imu_data_prev.linear_acceleration_.x,
                      imu_data_prev.linear_acceleration_.y,
                      imu_data_prev.linear_acceleration_.z);
  // a_prev = R(a_m_prev-ba) - g
  Eigen::Vector3d a_prev = GetUnbiasedLinearAcc(linear_acc_prev, R_prev);

  // mid-value acc can improve error state prediction accuracy:
  linear_acc_mid = 0.5 * (a_curr + a_prev);
  velocity_delta = T * linear_acc_mid;

  // return mid-value acc in body frame
  linear_acc_mid = 0.5 * (linear_acc_curr + linear_acc_prev) - accl_bias_;

  return true;
}

/**
 * @brief  update orientation with effective velocity change velocity_delta
 * @param  T, timestamp delta
 * @param  velocity_delta, effective velocity change
 * @return void
 */
void ErrorStateKalmanFilter::UpdatePosition(
    const double T, const Eigen::Vector3d& velocity_delta) {
  pose_.block<3, 1>(0, 3) += T * vel_ + 0.5 * T * velocity_delta;
  vel_ += velocity_delta;
}

/**
 * @brief  update IMU odometry estimation
 * @param  linear_acc_mid, output mid-value unbiased linear acc
 * @return void
 */
void ErrorStateKalmanFilter::UpdateOdomEstimation(
    Eigen::Vector3d& linear_acc_mid, Eigen::Vector3d& angular_vel_mid) {
  //
  // TODO: this is one possible solution to previous chapter, IMU Navigation,
  // assignment
  //
  // get deltas:
  Eigen::Vector3d angluar_delta;
  GetAngularDelta(1, 0, angluar_delta, angular_vel_mid);

  // update orientation:
  Eigen::Matrix3d R_curr, R_prev;
  UpdateOrientation(angluar_delta, R_curr, R_prev);

  // get velocity delta:
  double T = 0.0;
  Eigen::Vector3d velocity_delta;
  GetVelocityDelta(1, 0, R_curr, R_prev, T, velocity_delta, linear_acc_mid);

  // save mid-value unbiased linear acc for error-state update:

  // update position:
  UpdatePosition(T, velocity_delta);
}

/**
 * @brief  set process equation
 * @param  C_nb, rotation matrix, body frame -> navigation frame
 * @param  f_b, accel measurement in body frame
 * @return void
 */
void ErrorStateKalmanFilter::SetProcessEquation(const Eigen::Matrix3d& C_nb,
                                                const Eigen::Vector3d& f_b,
                                                const Eigen::Vector3d& w_b) {
  // TODO: set process / system equation:
  // a. set process equation for delta vel:
  F_.block<3, 3>(kIndexErrorVel, kIndexErrorOri) =
      -C_nb * Sophus::SO3d::hat(f_b).matrix();
  F_.block<3, 3>(kIndexErrorVel, kIndexErrorAccel) = -C_nb;

  // b. set process equation for delta ori:
  F_.block<3, 3>(kIndexErrorOri, kIndexErrorOri) =
      -Sophus::SO3d::hat(w_b).matrix();

  B_.block<3, 3>(kIndexErrorVel, kIndexNoiseAccel) = C_nb;
}

/**
 * @brief  update process equation
 * @param  imu_data, input IMU measurement
 * @param  T, output time delta
 * @return void
 */
void ErrorStateKalmanFilter::UpdateProcessEquation(
    const Eigen::Vector3d& linear_acc_mid,
    const Eigen::Vector3d& angular_vel_mid) {
  // set linearization point:
  Eigen::Matrix3d C_nb = pose_.block<3, 3>(0, 0);

  // set process equation:
  SetProcessEquation(C_nb, linear_acc_mid, angular_vel_mid);
}

/**
 * @brief  update error estimation
 * @param  linear_acc_mid, input mid-value unbiased linear acc
 * @return void
 */
void ErrorStateKalmanFilter::UpdateErrorEstimation(
    const double T,
    const Eigen::Vector3d& linear_acc_mid,
    const Eigen::Vector3d& angular_vel_mid) {
  static MatrixF F_1st;
  static MatrixF F_2nd;
  // TODO: update process equation:
  UpdateProcessEquation(linear_acc_mid, angular_vel_mid);

  // TODO: get discretized process equations:
  MatrixF F = MatrixF::Identity() + F_ * T;
  MatrixB B = B_ * T;
  B.block<6, 6>(kIndexErrorAccel, kIndexNoiseBiasAccel) =
      Eigen::Matrix<double, 6, 6>::Identity() * sqrt(T);

  // TODO: perform Kalman prediction
  P_ = F * P_ * F.transpose() + B * Q_ * B.transpose();
}

/**
 * @brief  correct error estimation using pose measurement
 * @param  T_nb, input pose measurement
 * @return void
 */
void ErrorStateKalmanFilter::CorrectErrorEstimationPose(
    const Eigen::Matrix4d& T_nb,
    Eigen::VectorXd& Y,
    Eigen::MatrixXd& G,
    Eigen::MatrixXd& K) {
  //
  // TODO: set measurement:
  //
  Eigen::Vector3d delta_p = pose_.block<3, 1>(0, 3) - T_nb.block<3, 1>(0, 3);
  Eigen::Matrix3d delta_R =
      T_nb.block<3, 3>(0, 0).transpose() * pose_.block<3, 3>(0, 0);
  Eigen::Vector3d delta_theta =
      Sophus::SO3d::vee(delta_R - Eigen::Matrix3d::Identity());

  // TODO: set measurement equation:
  YPose_.block<3, 1>(0, 0) = delta_p;
  YPose_.block<3, 1>(3, 0) = delta_theta;
  Y = YPose_;
  G = GPose_;

  // TODO: set Kalman gain:
  // 这里没必要C*R*C^T 因为C是一个单位阵
  K = P_ * G.transpose() * (G * P_ * G.transpose() + RPose_).inverse();
}

void ErrorStateKalmanFilter::CorrectErrorEstimationPosePosi(
    const Eigen::Matrix4d& T_nb,
    const Eigen::Vector3d& p_nb,
    Eigen::VectorXd& Y,
    Eigen::MatrixXd& G,
    Eigen::MatrixXd& K) {
  // lidar
  Eigen::Vector3d delta_p = pose_.block<3, 1>(0, 3) - T_nb.block<3, 1>(0, 3);
  Eigen::Matrix3d delta_R =
      T_nb.block<3, 3>(0, 0).transpose() * pose_.block<3, 3>(0, 0);
  Eigen::Vector3d delta_theta =
      Sophus::SO3d::vee(delta_R - Eigen::Matrix3d::Identity());
  // gnss
  Eigen::Vector3d gnss_p = pose_.block<3, 1>(0, 3) - p_nb;

  YPosePosi_.block<3, 1>(0, 0) = delta_p;
  YPosePosi_.block<3, 1>(3, 0) = delta_theta;
  YPosePosi_.block<3, 1>(6, 0) = gnss_p;
  Y = YPosePosi_;
  G = GPosePosi_;

  K = P_ * G.transpose() * (G * P_ * G.transpose() + RPosePosi_).inverse();
}

/**
 * @brief  correct error estimation using pose and body velocity measurement
 * @param  T_nb, input pose measurement
 * @param  v_b, input velocity measurement
 * @return void
 */
void ErrorStateKalmanFilter::CorrectErrorEstimationPoseVel(
    const Eigen::Matrix4d& T_nb,
    const Eigen::Vector3d& v_b,
    const Eigen::Vector3d& w_b,
    Eigen::VectorXd& Y,
    Eigen::MatrixXd& G,
    Eigen::MatrixXd& K) {
  //
  // TODO: set measurement:
  //
  Eigen::Vector3d delta_p = pose_.block<3, 1>(0, 3) - T_nb.block<3, 1>(0, 3);

  Eigen::Matrix3d delta_R =
      T_nb.block<3, 3>(0, 0).transpose() * pose_.block<3, 3>(0, 0);
  Eigen::Vector3d delta_theta =
      Sophus::SO3d::vee(delta_R - Eigen::Matrix3d::Identity());

  Eigen::Vector3d v_b_predict = pose_.block<3, 3>(0, 0).transpose() * vel_;
  Eigen::Vector3d delta_v = v_b_predict - v_b;

  // set measurement equation:
  YPoseVel_.block<3, 1>(0, 0) = delta_p;
  YPoseVel_.block<3, 1>(3, 0) = delta_v;
  YPoseVel_.block<3, 1>(6, 0) = delta_theta;
  Y = YPoseVel_;

  GPoseVel_.block<3, 3>(kIndexErrorVel, kIndexErrorVel) =
      pose_.block<3, 3>(0, 0).transpose();
  GPoseVel_.block<3, 3>(kIndexErrorVel, kIndexErrorOri) =
      Sophus::SO3d::hat(v_b_predict);
  G = GPoseVel_;

  // std::cout<<"G:"<<std::endl<<G<<std::endl;
  // std::cout<<"R:"<<std::endl<<RPoseVel_<<std::endl;

  //
  // TODO: set Kalman gain:
  //
  K = P_ * G.transpose() * (G * P_ * G.transpose() + RPoseVel_).inverse();
}

void ErrorStateKalmanFilter::CorrectErrorEstimationPoseVelConstrain(
    const Eigen::Matrix4d& T_nb,
    Eigen::VectorXd& Y,
    Eigen::MatrixXd& G,
    Eigen::MatrixXd& K) {
  Eigen::Vector3d delta_p = pose_.block<3, 1>(0, 3) - T_nb.block<3, 1>(0, 3);

  Eigen::Matrix3d delta_R =
      T_nb.block<3, 3>(0, 0).transpose() * pose_.block<3, 3>(0, 0);
  Eigen::Vector3d delta_theta =
      Sophus::SO3d::vee(delta_R - Eigen::Matrix3d::Identity());

  Eigen::Vector3d v_b_predict = pose_.block<3, 3>(0, 0).transpose() * vel_;
  Eigen::Vector2d delta_v =
      v_b_predict.block<2, 1>(1, 0) - Eigen::Vector2d::Zero();

  YPoseVelCon_.block<3, 1>(0, 0) = delta_p;
  YPoseVelCon_.block<2, 1>(3, 0) = delta_v;
  YPoseVelCon_.block<3, 1>(5, 0) = delta_theta;
  Y = YPoseVelCon_;

  GPoseVel_.block<3, 3>(kIndexErrorVel, kIndexErrorVel) =
      pose_.block<3, 3>(0, 0).transpose();
  GPoseVel_.block<3, 3>(kIndexErrorVel, kIndexErrorOri) =
      Sophus::SO3d::hat(v_b_predict);

  GPoseVelCon_.block<1, kDimState>(3, 0) = GPoseVel_.block<1, kDimState>(4, 0);
  GPoseVelCon_.block<1, kDimState>(4, 0) = GPoseVel_.block<1, kDimState>(5, 0);
  G = GPoseVelCon_;

  // std::cout<<"G:"<<std::endl<<G<<std::endl;
  // std::cout<<"R:"<<std::endl<<RPoseVelCon_<<std::endl;

  K = P_ * G.transpose() * (G * P_ * G.transpose() + RPoseVelCon_).inverse();
}

/**
 * @brief  correct error estimation using navigation position and body velocity
 * measurement
 * @param  T_nb, input position measurement
 * @param  v_b, input velocity measurement
 * @return void
 */
void ErrorStateKalmanFilter::CorrectErrorEstimationPosiVel(
    const Eigen::Matrix4d& T_nb,
    const Eigen::Vector3d& v_b,
    const Eigen::Vector3d& w_b,
    Eigen::VectorXd& Y,
    Eigen::MatrixXd& G,
    Eigen::MatrixXd& K) {
  // parse measurement:
  Eigen::Vector3d delta_p = pose_.block<3, 1>(0, 3) - T_nb.block<3, 1>(0, 3);

  Eigen::Vector3d v_b_predict = pose_.block<3, 3>(0, 0).transpose() * vel_;
  Eigen::Vector3d delta_v = v_b_predict - v_b;

  // set measurement equation:
  YPosiVel_.block<3, 1>(0, 0) = delta_p;
  YPosiVel_.block<3, 1>(3, 0) = delta_v;
  Y = YPosiVel_;

  GPosiVel_.block<3, 3>(kIndexErrorVel, kIndexErrorVel) =
      pose_.block<3, 3>(0, 0).transpose();
  GPosiVel_.block<3, 3>(kIndexErrorVel, kIndexErrorOri) =
      Sophus::SO3d::hat(v_b_predict);
  G = GPosiVel_;

  // set Kalman gain:
  K = P_ * G.transpose() * (G * P_ * G.transpose() + RPosiVel_).inverse();
}

void ErrorStateKalmanFilter::CorrectErrorEstimationPosi(
    const Eigen::Matrix4d& T_nb,
    Eigen::VectorXd& Y,
    Eigen::MatrixXd& G,
    Eigen::MatrixXd& K) {
  // parse measurement:
  Eigen::Vector3d delta_p = pose_.block<3, 1>(0, 3) - T_nb.block<3, 1>(0, 3);

  // set measurement equation:
  YPosi_.block<3, 1>(0, 0) = delta_p;
  Y = YPosi_;
  G = GPosi_;

  // set Kalman gain:
  K = P_ * G.transpose() * (G * P_ * G.transpose() + RPosi_).inverse();
}

/**
 * @brief  correct error estimation
 * @param  measurement_type, measurement type
 * @param  measurement, input measurement
 * @return void
 */
void ErrorStateKalmanFilter::CorrectErrorEstimation(
    const MeasurementType& measurement_type, const Measurement& measurement) {
  //
  // TODO: understand ESKF correct workflow
  //
  Eigen::VectorXd Y;
  Eigen::MatrixXd G, K;
  switch (measurement_type) {
  case MeasurementType::POSE:
    CorrectErrorEstimationPose(measurement.T_nb, Y, G, K);
    break;
  case MeasurementType::POSE_POSI:
    CorrectErrorEstimationPosePosi(measurement.T_nb, measurement.p_nb, Y, G, K);
    break;
  case MeasurementType::POSE_VEL:
    CorrectErrorEstimationPoseVel(
        measurement.T_nb, measurement.v_b, measurement.w_b, Y, G, K);
    break;
  case MeasurementType::POSE_VEL_CONS:
    // if is turning, using pose for correction
    if (IsTurning(measurement.w_b)) {
      CorrectErrorEstimationPose(measurement.T_nb, Y, G, K);
    }
    // using pose and motion constrain for correction
    else {
      CorrectErrorEstimationPoseVelConstrain(measurement.T_nb, Y, G, K);
    }
    break;
  case MeasurementType::POSI_VEL:
    CorrectErrorEstimationPosiVel(
        measurement.T_nb, measurement.v_b, measurement.w_b, Y, G, K);
    break;
  case MeasurementType::POSI:
    CorrectErrorEstimationPosi(measurement.T_nb, Y, G, K);
    break;
  default:
    break;
  }

  //
  // TODO: perform Kalman correct:
  //
  P_ = (MatrixP::Identity() - K * G) * P_;
  X_ = X_ + K * (Y - G * X_);
}

/**
 * @brief  eliminate error
 * @param  void
 * @return void
 */
void ErrorStateKalmanFilter::EliminateError() {
  //
  // TODO: correct state estimation using the state of ESKF
  //
  // a. position:
  // do it!
  pose_.block<3, 1>(0, 3) =
      pose_.block<3, 1>(0, 3) - X_.block<3, 1>(kIndexErrorPos, 0);
  // b. velocity:
  // do it!
  vel_ = vel_ - X_.block<3, 1>(kIndexErrorVel, 0);
  // c. orientation:
  // do it!
  Eigen::Matrix3d delta_R =
      Eigen::Matrix3d::Identity() -
      Sophus::SO3d::hat(X_.block<3, 1>(kIndexErrorOri, 0)).matrix();
  Eigen::Quaterniond dq = Eigen::Quaterniond(delta_R);
  dq = dq.normalized();
  // pose_.block<3,3>(0,0) *= dq.toRotationMatrix();
  pose_.block<3, 3>(0, 0) = pose_.block<3, 3>(0, 0) * dq.toRotationMatrix();

  // if (IsCovStable(kIndexErrorGyro)) {
  // gyro_bias_ += X_.block<3, 1>(kIndexErrorGyro, 0);
  gyro_bias_ -= X_.block<3, 1>(kIndexErrorGyro, 0);
  // std::cout<<"bg: "<< gyro_bias_.transpose()<<std::endl;
  // }

  // e. accel bias:
  // if (IsCovStable(kIndexErrorAccel)) {
  // accl_bias_ += X_.block<3, 1>(kIndexErrorAccel, 0);
  accl_bias_ -= X_.block<3, 1>(kIndexErrorAccel, 0);
  // std::cout<<"ba: "<< accl_bias_.transpose() <<std::endl;
  // }

  // std::cout<<"bg: "<< gyro_bias_.transpose()<<
  // std::endl<<"ba: "<< accl_bias_.transpose() <<std::endl;
}

/**
 * @brief  is covariance stable
 * @param  INDEX_OFSET, state index offset
 * @param  THRESH, covariance threshold, defaults to 1.0e-5
 * @return void
 */
bool ErrorStateKalmanFilter::IsCovStable(const int INDEX_OFSET,
                                         const double THRESH) {
  for (int i = 0; i < 3; ++i) {
    if (P_(INDEX_OFSET + i, INDEX_OFSET + i) > THRESH) {
      return false;
    }
  }

  return true;
}

/**
 * @brief  reset filter state
 * @param  void
 * @return void
 */
void ErrorStateKalmanFilter::ResetState() {
  // reset current state:
  X_ = VectorX::Zero();
}

/**
 * @brief determine if car is turning
 * @param w_b angular velocity in body frame
 * @return true
 * @return false
 */
bool ErrorStateKalmanFilter::IsTurning(const Eigen::Vector3d& w_b) {
  Eigen::Vector3d w_b_unbiased =
      GetUnbiasedAngularVel(w_b, pose_.block<3, 3>(0, 0));

  return (w_b_unbiased.norm() > motion_constraint_.w_b_thresh);
}

/**
 * @brief  reset filter covariance
 * @param  void
 * @return void
 */
void ErrorStateKalmanFilter::ResetCovariance() {
  P_ = MatrixP::Zero();

  P_.block<3, 3>(kIndexErrorPos, kIndexErrorPos) =
      cov_.prior.posi * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(kIndexErrorVel, kIndexErrorVel) =
      cov_.prior.vel * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(kIndexErrorOri, kIndexErrorOri) =
      cov_.prior.ori * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(kIndexErrorGyro, kIndexErrorGyro) =
      cov_.prior.epslion * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(kIndexErrorAccel, kIndexErrorAccel) =
      cov_.prior.delta * Eigen::Matrix3d::Identity();
}

/**
 * @brief  get Q analysis for pose measurement
 * @param  void
 * @return void
 */
void ErrorStateKalmanFilter::GetQPose(Eigen::MatrixXd& Q, Eigen::VectorXd& Y) {
  // build observability matrix for position measurement:
  Y = Eigen::VectorXd::Zero(kDimState * kDimMeasurementPose);
  Y.block<kDimMeasurementPose, 1>(0, 0) = YPose_;
  for (int i = 1; i < kDimState; ++i) {
    QPose_.block<kDimMeasurementPose, kDimState>(i * kDimMeasurementPose, 0) =
        (QPose_.block<kDimMeasurementPose, kDimState>(
             (i - 1) * kDimMeasurementPose, 0) *
         F_);

    Y.block<kDimMeasurementPose, 1>(i * kDimMeasurementPose, 0) = YPose_;
  }

  Q = QPose_;
}

/**
 * @brief  update observability analysis
 * @param  measurement_type, measurement type
 * @return void
 */
void ErrorStateKalmanFilter::UpdateObservabilityAnalysis(
    const double& time, const MeasurementType& measurement_type) {
  // get Q:
  Eigen::MatrixXd Q;
  Eigen::VectorXd Y;
  switch (measurement_type) {
  case MeasurementType::POSE:
    GetQPose(Q, Y);
    break;
  default:
    break;
  }

  observability_.time.push_back(time);
  observability_.Q.push_back(Q);
  observability_.Y.push_back(Y);
}

/**
 * @brief  save observability analysis to persistent storage
 * @param  measurement_type, measurement type
 * @return void
 */
bool ErrorStateKalmanFilter::SaveObservabilityAnalysis(
    const MeasurementType& measurement_type) {
  // get fusion strategy:
  std::string type;
  switch (measurement_type) {
  case MeasurementType::POSE:
    type = std::string("pose");
    break;
  case MeasurementType::POSI_VEL:
    type = std::string("position_velocity");
    break;
  default:
    return false;
    break;
  }

  // build Q_so:
  const int N = observability_.Q.at(0).rows();

  std::vector<std::vector<double>> q_data, q_so_data;

  Eigen::MatrixXd Qso(observability_.Q.size() * N, kDimState);
  Eigen::VectorXd Yso(observability_.Y.size() * N);

  for (size_t i = 0; i < observability_.Q.size(); ++i) {
    const double& time = observability_.time.at(i);

    const Eigen::MatrixXd& Q = observability_.Q.at(i);
    const Eigen::VectorXd& Y = observability_.Y.at(i);

    Qso.block(i * N, 0, N, kDimState) = Q;
    Yso.block(i * N, 0, N, 1) = Y;

    KalmanFilter::AnalyzeQ(kDimState, time, Q, Y, q_data);

    if (0 < i && (0 == i % 10)) {
      KalmanFilter::AnalyzeQ(kDimState,
                             observability_.time.at(i - 5),
                             Qso.block((i - 10), 0, 10 * N, kDimState),
                             Yso.block((i - 10), 0, 10 * N, 1),
                             q_so_data);
    }
  }

  std::string q_data_csv =
      WORK_SPACE_PATH + "/slam_data/observability/" + type + ".csv";
  std::string q_so_data_csv =
      WORK_SPACE_PATH + "/slam_data/observability/" + type + "_som.csv";

  KalmanFilter::WriteAsCSV(kDimState, q_data, q_data_csv);
  KalmanFilter::WriteAsCSV(kDimState, q_so_data, q_so_data_csv);

  return true;
}

}  // namespace lidar_localization