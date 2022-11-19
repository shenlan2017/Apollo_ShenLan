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

#pragma once

#include <memory>
#include <string>
#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/message/raw_message.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/shenlan_config.pb.h"
#include "modules/transform/transform_broadcaster.h"

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

#include "lidar_localization/sensor_data/gnss_data.h"
#include "lidar_localization/sensor_data/cloud_data.h"
#include "lidar_localization/sensor_data/pos_vel_data.h"
#include "lidar_localization/sensor_data/velocity_data.h"
#include "lidar_localization/sensor_data/imu_data.h"
#include "lidar_localization/sensor_data/pose_data.h"
#include "lidar_localization/global_defination/global_defination.h"

using namespace lidar_localization;
namespace apollo {
namespace localization {

class PretreatComponent final
    : public cyber::Component<drivers::gnss::Imu> {
 public:
  PretreatComponent();
  ~PretreatComponent() = default;

  bool Init() override;
  bool InitConfig();
  bool InitIO();

  bool Proc(const std::shared_ptr<drivers::gnss::Imu>& message) override;
  void LidarCallback(const std::shared_ptr<drivers::PointCloud>& cloud_msg);
  void OdometryCallback(const std::shared_ptr<localization::Gps>& gnss_msg);
  void ChassisCallback(const std::shared_ptr<canbus::Chassis>& chassis_msg);

  bool SaveGnssOrigin();

 private:
  bool ReadData();
  bool InitCalibration();
  bool InitGNSS();
  bool HasData();
  bool ValidData();
  bool TransformData();
  bool PublishData() ;

  bool PretreatImplementation();

  void LidarMsgTransfer(const std::shared_ptr<drivers::PointCloud> &msg, CloudData *lidar_frame);
  void IMUMsgTransfer(const std::shared_ptr<drivers::gnss::Imu> &imu_msg, IMUData &imu);
  void OdometryMsgTransfer(const std::shared_ptr<localization::Gps> &odom_msg, PoseData &odom_frame);
  void ChassisMsgTransfer(const std::shared_ptr<canbus::Chassis> &chassis_msg, VelocityData &chassis_frame);
  bool LoadLidarExtrinsic(const std::string &file_path, Eigen::Affine3d *lidar_extrinsic);
  void OnInsStat(const std::shared_ptr<drivers::gnss::InsStat> &message);
  bool FindNearestOdometryStatus(const double odometry_timestamp, drivers::gnss::InsStat *status);

  void ComposeLocalizationMsg( const PoseData &pose_msg, LocalizationEstimate *localization);
  void FillLocalizationMsgHeader( LocalizationEstimate *localization);

  // subscriber
  std::shared_ptr<cyber::Reader<drivers::PointCloud>> cloud_sub_ptr_ = nullptr;
  std::shared_ptr<cyber::Reader<canbus::Chassis>> velocity_sub_ptr_ = nullptr;
  std::shared_ptr<cyber::Reader<drivers::gnss::Imu>> imu_sub_ptr_ = nullptr;
  std::shared_ptr<cyber::Reader<localization::Gps>> gnss_sub_ptr_ = nullptr;
  std::shared_ptr<cyber::Reader<drivers::gnss::InsStat>> ins_status_ptr_ = nullptr;
  // publisher
  std::shared_ptr<cyber::Writer<CloudData>> cloud_pub_ptr_ = nullptr;
  std::shared_ptr<cyber::Writer<IMUData>> imu_pub_ptr_ = nullptr;
  std::shared_ptr<cyber::Writer<PosVelData>> pos_vel_pub_ptr_ = nullptr;
  std::shared_ptr<cyber::Writer<LocalizationEstimate>> odometry_gnss_ptr_ = nullptr;

  std::unique_ptr<cyber::Timer> pretreat_timer_ = nullptr;

  Eigen::Affine3d lidar_extrinsic;
  // topic
  std::string lidar_topic_ = "";
  std::string odometry_topic_ = "";
  std::string chassis_topic_ = "";
  std::string imu_topic_ = "";
  std::string odometry_status_topic_ = "";
  std::string odometry_gnss_topic_ = "";

  std::string cloud_trans_topic_ = "";
  std::string synch_imu_topic_ = "";
  std::string synch_posvel_topic_ = "";
  std::string lidar_extrinsics_path_ = "";

  std::string module_name_= "pretreate";

  std::deque<CloudData, Eigen::aligned_allocator<CloudData>> cloud_data_buff_;
  std::deque<IMUData, Eigen::aligned_allocator<IMUData>> unsynced_imu_;
  std::deque<VelocityData, Eigen::aligned_allocator<VelocityData>> unsynced_velocity_;
  std::deque<PoseData, Eigen::aligned_allocator<PoseData>> unsynced_gnss_;

  std::deque<IMUData, Eigen::aligned_allocator<IMUData>> imu_data_buff_;
  std::deque<VelocityData, Eigen::aligned_allocator<VelocityData>> velocity_data_buff_;
  std::deque<PoseData, Eigen::aligned_allocator<PoseData>>  gnss_data_buff_;
  std::deque<drivers::gnss::InsStat> odometry_status_list_;

  size_t cloud_list_max_size_ = 10;
  std::mutex cloud_list_mutex_;

  size_t imu_list_max_size_=50;
  std::mutex imu_list_mutex_;

  size_t chassis_list_max_size_=50;
  std::mutex chassis_list_mutex_;

  size_t odom_list_max_size_=50;
  std::mutex odometry_list_mutex_;

  size_t odometry_status_list_max_size_ = 50;
  std::mutex odometry_status_list_mutex_;

  CloudData current_cloud_data_;
  IMUData current_imu_data_;
  VelocityData current_velocity_data_;
  PoseData current_gnss_data_;

  PosVelData pos_vel_;
  Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();

  bool is_mapping_{true};
  // GNSSData gnss_init_data_;
  Eigen::Vector3f gnss_origin_ = Eigen::Vector3f::Zero();

  Eigen::Matrix4f T_imu_lidar_ = Eigen::Matrix4f::Identity();

  double max_height_ = 100.0;
  static constexpr double DEG_TO_RAD = 0.017453292519943;
  static constexpr double wheel_speed_coff = 1.0;
  static constexpr double kEpsilon = 1e-7;
  static constexpr double wheel_base = 1.465;
  static constexpr double odometry_status_time_diff_threshold_ = 1.0;
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

CYBER_REGISTER_COMPONENT(PretreatComponent);

}  // namespace localization
}  // namespace apollo
