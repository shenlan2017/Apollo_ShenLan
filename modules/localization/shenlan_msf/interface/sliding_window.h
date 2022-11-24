#pragma once
#include <deque>
#include <fstream>
#include <string>

#include <glog/logging.h>

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/models/pre_integrator/imu_pre_integrator.h"
#include "lidar_localization/models/sliding_window/ceres_sliding_window.h"
#include "lidar_localization/sensor_data/imu_data.h"
#include "lidar_localization/sensor_data/key_frame.h"
#include "lidar_localization/sensor_data/pose_data.h"
#include "lidar_localization/tools/file_manager.h"

using namespace ::lidar_localization;

namespace apollo{
namespace localization {

class SlidingWindow {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  SlidingWindow(const std::string& config_file_path);

  bool UpdateIMUPreIntegration(const IMUData& imu_data);

  bool Update(const PoseData& laser_odom,
              const PoseData& map_matching_odom,
              const IMUData& imu_data,
              const PoseData& gnss_pose);

  bool has_new_key_frame() const { return has_new_key_frame_; }

  bool has_new_optimized() const { return has_new_optimized_; }

  const KeyFrame& get_current_key_frame() const { return current_key_frame_; }

  const KeyFrame& get_current_key_gnss() const { return current_key_gnss_; }

  void GetLatestOptimizedOdometry(KeyFrame& key_frame);

  void GetOptimizedKeyFrames(
      std::deque<KeyFrame, Eigen::aligned_allocator<KeyFrame>>&
          key_frames_deque);

  bool SaveOptimizedTrajectory();

 private:
  bool InitWithConfig(const std::string& config_file_path);

  bool InitDataPath(const YAML::Node& config_node);

  bool InitKeyFrameSelection(const YAML::Node& config_node);

  bool InitSlidingWindow(const YAML::Node& config_node);

  bool InitIMUPreIntegrator(const YAML::Node& config_node);

  bool MaybeNewKeyFrame(const PoseData& laser_odom,
                        const PoseData& map_matching_odom,
                        const IMUData& imu_data,
                        const PoseData& gnss_pose);

  bool Update();

  bool MaybeOptimized();

  void ResetParam() {
    has_new_key_frame_ = false;
    has_new_optimized_ = false;
  }

  static bool SavePose(const Eigen::Matrix4f& pose, std::ofstream& ofs);

  std::string trajectory_path_{};

  bool has_new_key_frame_{false};
  bool has_new_optimized_{false};

  KeyFrame current_key_frame_;
  PoseData current_map_matching_pose_;
  KeyFrame current_key_gnss_;

  // key frame buffer:
  struct {
    std::deque<KeyFrame, Eigen::aligned_allocator<KeyFrame>> lidar;
    std::deque<KeyFrame, Eigen::aligned_allocator<KeyFrame>> optimized;
    std::deque<KeyFrame, Eigen::aligned_allocator<KeyFrame>> reference;
  } key_frames_;

  // pre-integrator:
  std::shared_ptr<IMUPreIntegrator> imu_pre_integrator_ptr_ = nullptr;
  std::shared_ptr<CeresSlidingWindow> sliding_window_ptr_ = nullptr;
  IMUPreIntegrator::IMUPreIntegration imu_pre_integration_;

  // key frame config:
  struct {
    float max_distance;
    float max_interval;
  } key_frame_config_;

  // optimizer:

  // measurement config:
  struct MeasurementConfig {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    struct {
      bool map_matching{false};
      bool imu_pre_integration{false};
    } source;

    struct {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      Eigen::VectorXd lidar_odometry;
      Eigen::VectorXd map_matching;
      Eigen::VectorXd gnss_position;
    } noise;
  };
  MeasurementConfig measurement_config_;
};

}
}
