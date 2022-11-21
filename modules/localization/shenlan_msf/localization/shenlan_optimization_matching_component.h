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
#include "modules/localization/shenlan_msf/interface/matching.h"
#include "modules/localization/shenlan_msf/interface/msg_transfer.h"
#include "modules/transform/transform_broadcaster.h"

using namespace lidar_localization;

namespace apollo {
namespace localization {

class MatchingComponent final : public apollo::cyber::TimerComponent {
 public:
  MatchingComponent();
  ~MatchingComponent() = default;

  bool Init() override;
  bool Proc() override;

 private:
  bool InitConfig();
  bool InitIO();
  void LidarCallback(const std::shared_ptr<CloudData>& cloud_msg);
  void OnGNSSLocalization(const std::shared_ptr<LocalizationEstimate>& message);

  bool HasData();
  bool ValidData();
  bool UpdateMatching();

  std::shared_ptr<cyber::Reader<CloudData>> lidar_listener_ = nullptr;
  std::shared_ptr<cyber::Reader<LocalizationEstimate>> gnss_pose_listener_ =
      nullptr;
  std::shared_ptr<cyber::Writer<LocalizationEstimate>>
      localization_lidar_talker_ = nullptr;
  std::shared_ptr<cyber::Writer<LocalizationEstimate>>
      localization_map_matcher_talker_ = nullptr;

  std::string lidar_topic_ = "";
  std::string odometry_gnss_topic_ = "";
  std::string lidar_extrinsics_path_ = "";
  std::string localization_lidar_topic_ = "";
  std::string map_matcher_topic_ = "";
  std::string module_name_ = "optimization pose";

  Eigen::Affine3d lidar_extrinsic;


  std::deque<CloudData,Eigen::aligned_allocator<CloudData>> cloud_data_buff_;
  size_t cloud_list_max_size_ = 10;
  std::mutex cloud_list_mutex_;

  std::mutex gnss_pose_list_mutex_;
  std::deque<PoseData, Eigen::aligned_allocator<PoseData>> gnss_data_buff_;
  size_t gnss_pose_list_max_size_ = 5000;

  // matching
  std::shared_ptr<Matching> matching_ptr_ = nullptr;

  CloudData current_cloud_data_;
  PoseData current_gnss_data_;

  Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f map_matching_odometry_ = Eigen::Matrix4f::Identity();

  MsgTransfer tools;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

CYBER_REGISTER_COMPONENT(MatchingComponent);

}  // namespace localization
}  // namespace apollo
