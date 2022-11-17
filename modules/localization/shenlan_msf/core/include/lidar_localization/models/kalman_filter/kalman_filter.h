/*
 * @Description: Kalman Filter interface.
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-27 23:13:16
 */
#ifndef LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_KALMAN_FILTER_H_
#define LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_KALMAN_FILTER_H_

#include <deque>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "glog/logging.h"

// SVD for observability analysis:
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
// snesor data
#include "lidar_localization/sensor_data/imu_data.h"
// tools
#include "lidar_localization/tools/CSVWriter.hpp"
// global
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

class KalmanFilter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  /**
   * @class MeasurementType
   * @brief enum for observation type
   */
  enum MeasurementType {
    POSE = 0,
    POSE_VEL,
    POSE_POSI,
    POSE_VEL_CONS,
    POSI,
    POSI_VEL,
    POSI_MAG,
    POSI_VEL_MAG,
    NUM_TYPES
  };

  /**
   * @class Measurement
   * @brief Kalman filter measurement data
   */
  struct Measurement {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // timestamp:
    double time{0.0};
    // a. pose observation, lidar/visual frontend:
    Eigen::Matrix4d T_nb = Eigen::Matrix4d::Zero();
    // b. body frame velocity observation, odometer:
    Eigen::Vector3d v_b = Eigen::Vector3d::Zero();
    // c. body frame angular velocity, needed by motion constraint:
    Eigen::Vector3d w_b = Eigen::Vector3d::Zero();
    // d. magnetometer:
    Eigen::Vector3d B_b = Eigen::Vector3d::Zero();
    // e. gnss:
    Eigen::Vector3d p_nb = Eigen::Vector3d::Zero();
  };

  /**
   * @class Cov
   * @brief Kalman filter process covariance data
   */
  struct Cov {
    struct {
      double x{0.0};
      double y{0.0};
      double z{0.0};
    } pos;
    struct {
      double x{0.0};
      double y{0.0};
      double z{0.0};
    } vel;
    // here quaternion is used for orientation representation:
    struct {
      double w{1.0};
      double x{0.0};
      double y{0.0};
      double z{0.0};
    } ori;
    struct {
      double x{0.0};
      double y{0.0};
      double z{0.0};
    } gyro_bias;
    struct {
      double x{0.0};
      double y{0.0};
      double z{0.0};
    } accel_bias;
  };

  /**
   * @brief  init filter
   * @param  imu_data, input IMU measurements
   * @return true if success false otherwise
   */
  virtual void Init(const Eigen::Matrix4d& init_pose,
                    const Eigen::Vector3d& vel,
                    const IMUData& imu_data) = 0;

  /**
   * @brief  update state & covariance estimation, Kalman prediction
   * @param  imu_data, input IMU measurements
   * @return true if success false otherwise
   */
  virtual bool Update(const IMUData& imu_data) = 0;

  /**
   * @brief  correct state & covariance estimation, Kalman correction
   * @param  measurement_type, input measurement type
   * @param  measurement, input measurement
   * @return void
   */
  virtual bool Correct(const IMUData& imu_data,
                       const MeasurementType& measurement_type,
                       const Measurement& measurement) = 0;

  /**
   * @brief  get filter time
   * @return filter time as double
   */
  double time() const { return time_; }

  /**
   * @brief  get odometry estimation
   * @param  pose, output pose
   * @param  vel, output vel
   * @return void
   */
  virtual void GetOdometry(Eigen::Matrix4f& pose, Eigen::Vector3f& vel) = 0;

  /**
   * @brief  get covariance estimation
   * @param  cov, output covariance
   * @return void
   */
  virtual void GetCovariance(Cov& cov) = 0;

  /**
   * @brief  update observability analysis
   * @param  time, measurement time
   * @param  measurement_type, measurement type
   * @return void
   */
  virtual void UpdateObservabilityAnalysis(
      const double& time, const MeasurementType& measurement_type) = 0;

  /**
   * @brief  save observability analysis to persistent storage
   * @param  measurement_type, measurement type
   * @return void
   */
  virtual bool SaveObservabilityAnalysis(
      const MeasurementType& measurement_type) = 0;

 protected:
  KalmanFilter() {}

  static void AnalyzeQ(const int DIM_STATE,
                       const double& time,
                       const Eigen::MatrixXd& Q,
                       const Eigen::VectorXd& Y,
                       std::vector<std::vector<double>>& data);

  static void WriteAsCSV(const int DIM_STATE,
                         const std::vector<std::vector<double>>& data,
                         const std::string filename);

  // time:
  double time_{0.0};

  // data buff:
  std::deque<IMUData, Eigen::aligned_allocator<IMUData>> imu_data_buff_;

  // earth constants:
  Eigen::Vector3d g_ = Eigen::Vector3d::Zero();
  // Eigen::Vector3d w_;
  // Eigen::Vector3d b_;

  // observability analysis:
  struct {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<double> time;
    std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> Q;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::MatrixXd>> Y;
  } observability_;

  // hyper-params:
  // a. earth constants:
  struct {
    double gravity_magnitude{0.0};
    double rotation_speed{0.0};
    double latitude{0.0};
    double longitude{0.0};
  } earth_;
  // b. prior state covariance, process & measurement noise:
  struct {
    struct {
      double posi{0.0};
      double vel{0.0};
      double ori{0.0};
      double epslion{0.0};  // acc bias prior noise
      double delta{0.0};    // gyro bias prior noise
      // double BIAS_GYRO;
    } prior;
    struct {
      double gyro{0.0};
      double accel{0.0};
      double bias_accel{0.0};
      double bias_gyro{0.0};
      double vel{0.0};
    } process;
    struct {
      struct {
        double posi{0.0};
        double ori{0.0};
      } pose;
      double posi{0.0};
      double vel{0.0};
      double ori{0.0};
    } measurement;
  } cov_;
  // c. motion constraint:
  struct {
    bool activated{false};
    // when w_b more than a thresh, disable motion constraint
    double w_b_thresh{0.0};
  } motion_constraint_;
};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_KALMAN_FILTER_H_
