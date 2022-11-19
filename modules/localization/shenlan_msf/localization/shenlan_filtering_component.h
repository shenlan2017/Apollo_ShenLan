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

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/shenlan_config.pb.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/message/raw_message.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/math/euler_angles_zxy.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/common/util/time_util.h"
#include "modules/localization/shenlan_msf/interface/filtering.h"
#include "modules/localization/shenlan_msf/interface/msg_transfer.h"
#include "modules/transform/transform_broadcaster.h"

using namespace lidar_localization;
namespace apollo {
namespace localization {

class FilteringComponent final
    : public cyber::Component<drivers::gnss::Imu> {
 public:
  FilteringComponent();
  ~FilteringComponent() = default;

  bool Init() override;

  bool Proc(const std::shared_ptr<drivers::gnss::Imu>& message) override;

 private:
  bool InitConfig();
  bool InitIO();

  bool ReadData();
  bool HasInited()const;

  bool HasData();

  bool HasIMUData() const {
    if (!imu_raw_data_buff_.empty()) {
      double diff_filter_time =
          current_imu_raw_data_.time_ - filtering_ptr_->GetTime();
      if (diff_filter_time <= 0.01) {
        return true;
      }
    }

    return false;
}
  bool HasLidarData() const {
      return (!cloud_data_buff_.empty() && !imu_synced_data_buff_.empty() &&
          !pos_vel_data_buff_.empty());
  }
  bool HasIMUComesFirst() const {
    return imu_raw_data_buff_.front().time_ < cloud_data_buff_.front().time_;
  }

  bool ValidIMUData();
  bool ValidLidarData();

  bool InitCalibration();
  bool InitLocalization() const;

  bool UpdateLocalization();
  bool CorrectLocalization();

  bool UpdateOdometry(const double time);

  void FilteringImplementation();
  bool FindNearestOdometryStatus(const double odometry_timestamp, drivers::gnss::InsStat *status);

  /**
   * @brief  save pose in KITTI format for evo evaluation
   * @param  pose, input pose
   * @param  ofs, output file stream
   * @return true if success otherwise false
   */
  static bool SavePose(const Eigen::Matrix4f& pose, std::ofstream& ofs);

  bool ValidData();
  bool PublishData();

  void LidarCallback(const std::shared_ptr<CloudData>& cloud_msg);
  void OdometryCallback(const std::shared_ptr<localization::Gps>& gnss_msg);
  void OnInsStat(const std::shared_ptr<drivers::gnss::InsStat> &message);
  void ChassisCallback(const std::shared_ptr<canbus::Chassis>& chassis_msg);
  void PosVelCallback(const std::shared_ptr<PosVelData>& posvel_msg);
  void SynchImuCallback(const std::shared_ptr<IMUData>& synch_msg);

  bool PublishFusionOdom();
  bool PublishLidarOdom();
  void ComposeLocalizationMsg(const Eigen::Matrix4f &gps_msg, LocalizationEstimate *localization, const double time);
  void FillLocalizationMsgHeader(LocalizationEstimate *localization);
  std::shared_ptr<cyber::Reader<CloudData>> lidar_listener_ = nullptr;
  std::shared_ptr<cyber::Reader<localization::Gps>> odometry_listener_ = nullptr;
  std::shared_ptr<cyber::Reader<drivers::gnss::InsStat>> ins_status_listener_ = nullptr;
  std::shared_ptr<cyber::Reader<canbus::Chassis>> chassis_listener_ = nullptr;
  std::shared_ptr<cyber::Reader<PosVelData>> pos_vel_listener_ = nullptr;
  std::shared_ptr<cyber::Reader<IMUData>> imu_synced_listener_ = nullptr;

  std::shared_ptr<cyber::Writer<LocalizationEstimate>> localization_talker_ = nullptr;
  std::shared_ptr<cyber::Writer<LocalizationEstimate>> localization_lidar_talker_ = nullptr;

  std::string lidar_topic_ = "";
  std::string odometry_topic_ = "";
  std::string chassis_topic_ = "";
  std::string odometry_status_topic_ = "";
  std::string synch_imu_topic_ = "";
  std::string synch_posvel_topic_ = "";
  std::string lidar_extrinsics_path_ = "";

  std::string localization_topic_ = "";
  std::string localization_lidar_topic_ = "";
  std::string module_name_ = "filtering pose";


  std::unique_ptr<cyber::Timer> filtering_timer_ = nullptr;
  Eigen::Affine3d lidar_extrinsic;

  // filtering instance:
  std::shared_ptr<Filtering> filtering_ptr_;

  std::deque<CloudData,Eigen::aligned_allocator<CloudData>> cloud_data_buff_;
  size_t cloud_list_max_size_ = 10;
  std::mutex cloud_list_mutex_;

  std::deque<IMUData,Eigen::aligned_allocator<IMUData>> imu_raw_data_buff_;
  size_t imu_list_max_size_=50;
  std::mutex imu_list_mutex_;

  std::deque<VelocityData,Eigen::aligned_allocator<VelocityData>> chassis_data_buff_;
  size_t chassis_list_max_size_=50;
  std::mutex chassis_list_mutex_;

  std::deque<PoseData,Eigen::aligned_allocator<PoseData>> odometry_data_buff_;
  size_t odom_list_max_size_=50;
  std::mutex odometry_list_mutex_;

  std::deque<drivers::gnss::InsStat> odometry_status_list_;
  size_t odometry_status_list_max_size_ = 50;
  std::mutex odometry_status_list_mutex_;

  std::deque<PosVelData,Eigen::aligned_allocator<PosVelData>> pos_vel_data_buff_;
  size_t posvel_list_max_size_=10;
  std::mutex posvel_list_mutex_;

  std::deque<IMUData, Eigen::aligned_allocator<IMUData>> imu_synced_data_buff_;
  size_t imu_synced_list_max_size_=10;
  std::mutex imu_synced_list_mutex_;

  IMUData current_imu_raw_data_;
  CloudData current_cloud_data_;
  IMUData current_imu_synced_data_;
  PosVelData current_pos_vel_data_;
  PoseData current_gnss_data_;

  // lidar odometry frame in map frame:
  Eigen::Matrix4f fused_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Vector3f fused_vel_ = Eigen::Vector3f::Zero();
  Eigen::Matrix4f laser_pose_ = Eigen::Matrix4f::Identity();

  std::unique_ptr<apollo::transform::TransformBroadcaster> tf2_broadcaster_;
  double max_height_ = 100.0;

  static constexpr double DEG_TO_RAD = 0.017453292519943;
  static constexpr double wheel_speed_coff = 1.0;
  static constexpr double kEpsilon = 1e-7;
  static constexpr double wheel_base = 1.465;
  static constexpr double odometry_status_time_diff_threshold_ = 1.0;
  // trajectory for evo evaluation:
  struct {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    size_t N{0};
    std::deque<double> time;
    std::deque<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> fused;
    std::deque<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> lidar;
    std::deque<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> ref;
  } trajectory_;

  MsgTransfer tools;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

CYBER_REGISTER_COMPONENT(FilteringComponent);

}  // namespace localization
}  // namespace apollo
