/*
 * @Description: ceres sliding window optimizer, interface
 * @Author: Ge Yaokey_frame
 * @Date: 2021-01-03 14:53:21
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-23 14:36:53
 */
#ifndef LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_CERES_SLIDING_WINDOW_H_
#define LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_CERES_SLIDING_WINDOW_H_

#include <chrono>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "glog/logging.h"

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

#include "lidar_localization/models/pre_integrator/imu_pre_integrator.h"
#include "lidar_localization/models/sliding_window/factors/factor_prvag_imu_pre_integration.h"
#include "lidar_localization/models/sliding_window/factors/factor_prvag_map_matching_pose.h"
#include "lidar_localization/models/sliding_window/factors/factor_prvag_marginalization.h"
#include "lidar_localization/models/sliding_window/factors/factor_prvag_relative_pose.h"
#include "lidar_localization/models/sliding_window/params/param_prvag.h"
#include "lidar_localization/sensor_data/key_frame.h"

namespace lidar_localization {

class CeresSlidingWindow {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  static const int kIndexP{0};
  static const int kIndexR{3};
  static const int kIndexV{6};
  static const int kIndexA{9};
  static const int kIndexG{12};

  CeresSlidingWindow(const int N);
  ~CeresSlidingWindow();

  /**
   * @brief  add parameter block for LIO key frame
   * @param  lio_key_frame, LIO key frame with (pos, ori, vel, b_a and b_g)
   * @param  fixed, shall the param block be fixed to eliminate trajectory
   * estimation ambiguity
   * @return true if success false otherwise
   */
  void AddPRVAGParam(const KeyFrame& lio_key_frame, const bool fixed);

  /**
   * @brief  add residual block for relative pose constraint from lidar frontend
   * @param  param_index_i, param block ID of previous key frame
   * @param  param_index_j, param block ID of current key frame
   * @param  relative_pose, relative pose measurement
   * @param  noise, relative pose measurement noise
   * @return void
   */
  void AddPRVAGRelativePoseFactor(const int param_index_i,
                                  const int param_index_j,
                                  const Eigen::Matrix4d& relative_pose,
                                  const Eigen::VectorXd& noise);

  /**
   * @brief  add residual block for prior pose constraint from map matching
   * @param  param_index, param block ID of current key frame
   * @param  prior_pose, prior pose measurement
   * @param  noise, prior pose measurement noise
   * @return void
   */
  void AddPRVAGMapMatchingPoseFactor(const int param_index,
                                     const Eigen::Matrix4d& prior_pose,
                                     const Eigen::VectorXd& noise);

  /**
   * @brief  add residual block for IMU pre-integration constraint from IMU
   * measurement
   * @param  param_index_i, param block ID of previous key frame
   * @param  param_index_j, param block ID of current key frame
   * @param  imu_pre_integration, IMU pre-integration measurement
   * @return void
   */
  void AddPRVAGIMUPreIntegrationFactor(
      const int param_index_i,
      const int param_index_j,
      const IMUPreIntegrator::IMUPreIntegration& imu_pre_integration);

  // do optimization
  bool Optimize();

  // get num. of parameter blocks:
  int GetNumParamBlocks();

  /**
   * @brief  get optimized odometry estimation
   * @param  optimized_key_frame, output latest optimized key frame
   * @return true if success false otherwise
   */
  bool GetLatestOptimizedKeyFrame(KeyFrame& optimized_key_frame);

  /**
   * @brief  get optimized LIO key frame state estimation
   * @param  optimized_key_frames, output optimized LIO key frames
   * @return true if success false otherwise
   */
  bool GetOptimizedKeyFrames(
      std::deque<KeyFrame, Eigen::aligned_allocator<KeyFrame>>&
          optimized_key_frames);

  struct OptimizedKeyFrame {
    double time{0.0};
    double prvag[15];
    bool fixed{false};  // if fixed=true, do not optimize this frame
  };

  struct ResidualMapMatchingPose {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int param_index{0};

    Eigen::VectorXd m;
    Eigen::MatrixXd I;
  };

  struct ResidualRelativePose {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int param_index_i{0};
    int param_index_j{0};

    Eigen::VectorXd m;
    Eigen::MatrixXd I;
  };

  struct ResidualIMUPreIntegration {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int param_index_i{0};
    int param_index_j{0};

    double T{0.0};
    Eigen::Vector3d g = Eigen::Vector3d::Zero();
    Eigen::VectorXd m;
    Eigen::MatrixXd I;
    Eigen::MatrixXd J;
  };

 private:
  // sliding window config:
  const int kWindowSize{10};  // 滑窗大小

  /**
   * @brief  create information matrix from measurement noise specification
   * @param  noise, measurement noise covariances
   * @return information matrix as square Eigen::MatrixXd
   */
  static Eigen::MatrixXd GetInformationMatrix(const Eigen::VectorXd& noise);

  // b. optimizer config:
  struct {
    // 1. loss function:
    std::unique_ptr<ceres::LossFunction> loss_function_ptr{};
    // 2. solver:
    ceres::Solver::Options options;
  } config_;

  // c. data buffer:
  // c.1. param blocks:
  std::vector<OptimizedKeyFrame,
      Eigen::aligned_allocator<OptimizedKeyFrame>> optimized_key_frames_;

  // c.2. residual blocks:
  sliding_window::FactorPRVAGMapMatchingPose* GetResMapMatchingPose(
      const ResidualMapMatchingPose& res_map_matching_pose);
  sliding_window::FactorPRVAGRelativePose* GetResRelativePose(
      const ResidualRelativePose& res_relative_pose);
  sliding_window::FactorPRVAGIMUPreIntegration* GetResIMUPreIntegration(
      const ResidualIMUPreIntegration& res_imu_pre_integration);

  struct {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::deque<ResidualMapMatchingPose,
               Eigen::aligned_allocator<ResidualMapMatchingPose>>
        map_matching_pose;
    std::deque<ResidualRelativePose,
               Eigen::aligned_allocator<ResidualRelativePose>>
        relative_pose;
    std::deque<ResidualIMUPreIntegration,
               Eigen::aligned_allocator<ResidualIMUPreIntegration>>
        imu_pre_integration;
  } residual_blocks_;
};

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_CERES_SLIDING_WINDOW_H_
