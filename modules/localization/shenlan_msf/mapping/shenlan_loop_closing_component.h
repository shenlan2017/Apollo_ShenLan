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

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/sensor_data/cloud_data.h"
#include "lidar_localization/sensor_data/gnss_data.h"
#include "lidar_localization/sensor_data/imu_data.h"
#include "lidar_localization/sensor_data/pose_data.h"
#include "lidar_localization/sensor_data/velocity_data.h"

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
#include "modules/localization/shenlan_msf/core/include/lidar_localization/sensor_data/cloud_data.h"
#include "modules/localization/shenlan_msf/core/include/lidar_localization/sensor_data/key_frame.h"
#include "modules/localization/shenlan_msf/interface/loop_closing.h"
#include "modules/transform/transform_broadcaster.h"

using namespace lidar_localization;
namespace apollo {
namespace localization {

class LoopClosingComponent final
    : public cyber::Component<drivers::PointCloud> {
 public:
  LoopClosingComponent();
  ~LoopClosingComponent() = default;

  bool Init() override;

  bool Proc(const std::shared_ptr<drivers::PointCloud>& message) override;

 private:
  bool InitConfig();
  bool InitIO();
  bool ReadData();
  bool HasData() ;
  bool ValidData();
  bool PublishData();

  void KeyGnssCallback(const std::shared_ptr<KeyFrame>& gnss_msg);
  void KeyFrameCallback(const std::shared_ptr<KeyFrame>& fram_msg);
  void PointCloudCallback(const std::shared_ptr<drivers::PointCloud>& cloud_msg);

  bool OnLoopClosingTimer();

  void LidarMsgTransfer(const std::shared_ptr<drivers::PointCloud> &msg, CloudData *lidar_frame);

  std::shared_ptr<cyber::Reader<KeyFrame>> key_gnss_listener_ = nullptr;
  std::string key_gnss_topic_ = "/apollo/localization/key_gnss";
  std::shared_ptr<cyber::Reader<KeyFrame>> key_frame_listener_ = nullptr;
  std::string key_frame_topic_ = "/apollo/localization/key_frame";
  std::shared_ptr<cyber::Writer<LoopPose>> loop_closing_publisher_ = nullptr;
  std::string loop_closing_topic_ = "/apollo/localization/loop_pose";

  std::shared_ptr<LoopClosing> loop_closing_;
  std::unique_ptr<cyber::Timer> loop_closing_timer_ = nullptr;

  std::deque<CloudData,Eigen::aligned_allocator<CloudData>> key_scan_buff_;
  size_t scan_list_max_size_ = 10;
  std::mutex scan_list_mutex_;

  std::deque<KeyFrame,Eigen::aligned_allocator<KeyFrame>> key_frame_buff_;
  size_t frame_list_max_size_ = 10;
  std::mutex frame_list_mutex_;

  std::deque<KeyFrame,Eigen::aligned_allocator<KeyFrame>> key_gnss_buff_;
  size_t gnss_list_max_size_ = 10;
  std::mutex gnss_list_mutex_;

  CloudData current_key_scan_;
  KeyFrame current_key_frame_;
  KeyFrame current_key_gnss_;

  double max_height_ = 100.0;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

CYBER_REGISTER_COMPONENT(LoopClosingComponent);

}  // namespace localization
}  // namespace apollo
