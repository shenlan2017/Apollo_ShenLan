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

#include <iomanip>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "lidar_localization/sensor_data/cloud_data.h"
#include "lidar_localization/sensor_data/gnss_data.h"
#include "lidar_localization/sensor_data/imu_data.h"
#include "lidar_localization/sensor_data/pose_data.h"
#include "lidar_localization/sensor_data/velocity_data.h"
#include "yaml-cpp/yaml.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/drivers/gnss/proto/config.pb.h"
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
#include "cyber/component/timer_component.h"
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
#include "modules/localization/shenlan_msf/interface/front_end.h"
#include "modules/localization/shenlan_msf/interface/msg_transfer.h"
#include "modules/transform/transform_broadcaster.h"

using namespace lidar_localization;

namespace apollo {
namespace localization {

class LocalizationMsgPublisher {
 public:
  explicit LocalizationMsgPublisher(const std::shared_ptr<cyber::Node>& node);
  ~LocalizationMsgPublisher() = default;

  bool InitConfig();
  bool InitIO();
  void PublishLocalizationSLMSFGnss(const LocalizationEstimate& localization);
  void PublishLocalizationSLMSFLidar(const LocalizationEstimate& localization);

 private:
  std::shared_ptr<cyber::Node> node_;

  std::string lidar_local_topic_ = "";
  std::shared_ptr<cyber::Writer<LocalizationEstimate>> lidar_local_talker_ =
      nullptr;

  std::string gnss_local_topic_ = "";
  std::shared_ptr<cyber::Writer<LocalizationEstimate>> gnss_local_talker_ =
      nullptr;
};

class ShenLanFrontEndComponent final : public apollo::cyber::TimerComponent {
 public:
  ShenLanFrontEndComponent();
  ~ShenLanFrontEndComponent();

  bool Init() override;

  bool Proc() override;

 private:
  bool InitConfig();
  bool InitIO();

  void UpdateLaserOdometry();
  bool LoadLidarExtrinsic(const std::string& file_path,
                          Eigen::Affine3d* lidar_extrinsic);

  void OnOdometry(const std::shared_ptr<localization::Gps>& message);
  void OnInsStat(const std::shared_ptr<drivers::gnss::InsStat>& message);
  void OnPointCloud(const std::shared_ptr<drivers::PointCloud>& message);
  bool FindNearestOdometryStatus(const double odometry_timestamp,
                                 drivers::gnss::InsStat* status);
  //   void OnGnssBestPose(const std::shared_ptr<drivers::gnss::GnssBestPose>
  //   &message);

 private:
  std::shared_ptr<cyber::Reader<drivers::PointCloud>> lidar_listener_ = nullptr;
  std::string lidar_topic_ = "";

  //   std::shared_ptr<cyber::Reader<drivers::gnss::GnssBestPose>>
  //       bestgnsspos_listener_ = nullptr;
  //   std::string bestgnsspos_topic_ = "/apollo/sensor/gnss/best_pose";

  std::shared_ptr<cyber::Reader<localization::Gps>> odometry_listener_ =
      nullptr;
  std::string odometry_topic_ = "";

  std::shared_ptr<cyber::Reader<drivers::gnss::InsStat>>
      odometry_status_listener_ = nullptr;
  std::string odometry_status_topic_ = "";

  std::string lidar_extrinsics_file = "";

 private:
  std::thread front_end_loop;
  std::shared_ptr<LocalizationMsgPublisher> publisher_ = nullptr;
  std::shared_ptr<FrontEnd> front_end_ptr_ = nullptr;
  std::shared_ptr<MsgTransfer> tools = nullptr;

  Eigen::Affine3d lidar_extrinsic = Eigen::Affine3d::Identity();

  std::mutex lidar_frame_list_mutex_;
  std::deque<CloudData, Eigen::aligned_allocator<CloudData>>
      lidar_frame_list_{};
  size_t lidar_frame_list_max_size_ = 100;

  std::mutex lidar_pose_list_mutex_;
  std::deque<PoseData, Eigen::aligned_allocator<PoseData>> lidar_pose_list_{};
  size_t lidar_pose_list_max_size_ = 100;

  std::mutex odom_list_mutex_;
  std::deque<PoseData, Eigen::aligned_allocator<PoseData>> odom_list_{};
  size_t odom_list_max_size_ = 100;

  std::mutex odometry_status_list_mutex_;
  std::deque<drivers::gnss::InsStat> odometry_status_list_{};
  size_t odometry_status_list_max_size_ = 100;

  bool odometry_inited = false;
  double init_timestamp = std::numeric_limits<double>::max();
  Eigen::Vector3f gnss_origin = Eigen::Vector3f::Zero();

  LocalizationEstimate lidar_localization_result_{};
  LocalizationEstimate gnss_localization_result_{};

  void RequestFinish();
  bool isFinished();
  bool CheckFinish();
  void SetFinish();
  bool finish_requested_ = false;
  bool finished_ = false;
  std::mutex mutex_finish_;

  static constexpr double odometry_status_time_diff_threshold_ = 1.0;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

CYBER_REGISTER_COMPONENT(ShenLanFrontEndComponent);

}  // namespace localization
}  // namespace apollo
