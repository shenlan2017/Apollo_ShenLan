/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file msf_localization.h
 * @brief The class of MSFLocalization
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "yaml-cpp/yaml.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/drivers/gnss/proto/config.pb.h"
#include "modules/localization/proto/localization.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/math/euler_angles_zxy.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/common/util/time_util.h"
// #include "modules/localization/common/localization_gflags.h"

// #include "modules/localization/msf/local_integ/localization_params.h"
#include "lidar_localization/sensor_data/cloud_data.h"
#include "lidar_localization/sensor_data/gnss_data.h"
#include "lidar_localization/sensor_data/imu_data.h"
#include "lidar_localization/sensor_data/pose_data.h"
#include "lidar_localization/sensor_data/velocity_data.h"

#include "modules/localization/shenlan_msf/interface/back_end.h"
#include "modules/localization/shenlan_msf/interface/front_end.h"
#include "modules/localization/shenlan_msf/interface/post_processing.h"
#include "modules/localization/shenlan_msf/interface/loop_closing.h"

using namespace lidar_localization;

namespace apollo {
namespace localization {

class LocalizationMsgPublisher;

class SLMSFLocalization {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  SLMSFLocalization();
  ~SLMSFLocalization();

  struct LidarCombination {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PoseData lidar_pose;
    CloudData lidar_cloud;
  };

  apollo::common::Status Init();
  void InitParams();
  void OnOdometry(const std::shared_ptr<localization::Gps> &message);
  void OnInsStat(const std::shared_ptr<drivers::gnss::InsStat> &message);
  void OnPointCloud(const std::shared_ptr<drivers::PointCloud> &message);
  void OnChassis(const std::shared_ptr<canbus::Chassis> &message);
  void OnRawImu(const std::shared_ptr<drivers::gnss::Imu> &imu_msg);
  void OnRawImuCache(const std::shared_ptr<drivers::gnss::Imu> &imu_msg);
  void OnGnssBestPose(
      const std::shared_ptr<drivers::gnss::GnssBestPose> &message);

  void OnLoopClosing(const  std::shared_ptr<LoopPose> &message);

  void SetPublisher(const std::shared_ptr<LocalizationMsgPublisher> &publisher);
  void OnLocalizationTimer();

 private:
  std::string module_name_ = "shenlan_msf_localization";
  Eigen::Affine3d lidar_extrinsic;

  apollo::common::monitor::MonitorLogBuffer monitor_logger_;

  std::shared_ptr<drivers::gnss::Imu> raw_imu_msg_;

  std::thread back_end_loop;
  std::shared_ptr<FrontEnd> front_end_ptr_;
  std::shared_ptr<BackEnd> back_end_ptr_;
  std::shared_ptr<PostProcessing> post_processing_ptr_;
  std::shared_ptr<LoopClosing> loop_closing_ptr_;

  Eigen::Vector3f gnss_origin = Eigen::Vector3f::Zero();

  std::shared_ptr<LocalizationMsgPublisher> publisher_ = nullptr;

  bool odometry_inited = false;
  std::mutex mutex_finished;
  bool finished = false;

  LocalizationEstimate lidar_localization_result_{};
  LocalizationEstimate gnss_localization_result_{};
  LocalizationEstimate fusion_localization_result_{};
  uint32_t localization_seq_num_ = 0;
  Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();

  std::mutex mutex_imu_msg_;
  std::deque<IMUData, Eigen::aligned_allocator<IMUData>> imu_list_pre_integration_list;

  std::mutex fusion_pose_list_mutex_;
  std::deque<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> fusion_pose_list_;
  size_t fusion_pose_list_max_size_ = 500;

  std::mutex lidar_pose_list_mutex_;
  std::deque<LidarCombination, Eigen::aligned_allocator<LidarCombination>> lidar_pose_list_;
  size_t lidar_pose_list_max_size_ = 500;

  std::mutex imu_list_mutex_;
  std::deque<IMUData, Eigen::aligned_allocator<IMUData>> imu_list_;
  size_t imu_list_max_size_ = 500;

  std::mutex odom_list_mutex_;
  std::deque<PoseData, Eigen::aligned_allocator<PoseData>> odom_list_;
  size_t odom_list_max_size_ = 500;

  std::mutex chassis_list_mutex_;
  std::deque<VelocityData, Eigen::aligned_allocator<VelocityData>> chassis_list_;
  size_t chassis_list_max_size_ = 500;

  std::mutex odometry_status_list_mutex_;
  std::deque<drivers::gnss::InsStat> odometry_status_list_;
  size_t odometry_status_list_max_size_ = 500;

  std::mutex loop_list_mutex;
  std::deque<LoopPose, Eigen::aligned_allocator<LoopPose>> loop_pose_data_list_;

  // 定时器，100Hz
  std::unique_ptr<cyber::Timer> localization_timer_ = nullptr;

  double max_height_ = 100.0;

  static constexpr double DEG_TO_RAD = 0.017453292519943;
  static constexpr double wheel_speed_coff = 1.0;
  static constexpr double kEpsilon = 1e-7;
  static constexpr double wheel_base = 1.465;
  static constexpr double odometry_status_time_diff_threshold_ = 1.0;

  bool is_trans_gpstime_to_utctime_ = true;

  std::deque<KeyFrame, Eigen::aligned_allocator<KeyFrame>> key_frames_list_;
  std::deque<KeyFrame, Eigen::aligned_allocator<KeyFrame>> optimized_key_frames_list_;

 private:
  void LidarMsgTransfer(const std::shared_ptr<drivers::PointCloud> &msg,
                        CloudData *lidar_frame);
  void IMUMsgTransfer(const std::shared_ptr<drivers::gnss::Imu> &imu_msg,
                      IMUData &imu);
  void OdometryMsgTransfer(const std::shared_ptr<localization::Gps> &odom_msg,
                           PoseData &odom_frame);
  void ChassisMsgTransfer(const std::shared_ptr<canbus::Chassis> &chassis_msg,
                          VelocityData &chassis_frame);

  bool LoadLidarExtrinsic(const std::string &file_path,
                          Eigen::Affine3d *lidar_extrinsic);

  bool FindNearestOdometryStatus(const double odometry_timestamp,
                                 drivers::gnss::InsStat *status);

  bool UpdateLaserOdometry(CloudData &lidar_frame);

  void ComposeLocalizationResult(double time_stamp, const Eigen::Matrix4f &pose,
                                 LocalizationEstimate *localization);

  void FillLocalizationMsgHeader(LocalizationEstimate *localization);

  void LocalizationImplementation();
  bool InsertLoopClosurePose();
  
  void RequestFinish();
  bool isFinished();
  bool CheckFinish();
  void SetFinish();
  bool finish_requested_ = false;
  bool finished_ = false;
  std::mutex mutex_finish_;
};

}  // namespace localization
}  // namespace apollo
