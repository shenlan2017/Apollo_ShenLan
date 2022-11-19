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
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/math/euler_angles_zxy.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/status/status.h"
#include "modules/common/util/time_util.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/localization/shenlan_msf/interface/back_end.h"
#include "modules/localization/shenlan_msf/interface/loop_closing.h"
#include "modules/localization/shenlan_msf/interface/msg_transfer.h"
#include "modules/localization/shenlan_msf/interface/post_processing.h"
#include "modules/transform/transform_broadcaster.h"

using namespace lidar_localization;

namespace apollo {
namespace localization {

class MsgPublisher {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  explicit MsgPublisher(const std::shared_ptr<cyber::Node>& node);
  ~MsgPublisher() = default;

  bool InitConfig();
  bool InitIO();

  void PublishPoseBroadcastTF(const LocalizationEstimate& localization);
  void PublishPoseBroadcastTopic(const LocalizationEstimate& localization);

  void PublishLocalizationSLMSFFusion(const LocalizationEstimate& localization);
  void PublishLocalizationStatus(const LocalizationStatus& localization_status);
  void PublishKeyFrame(const KeyFrame& key_frame);
  void PublishKeyGnssFrame(const KeyFrame& key_frame);

 private:
  std::shared_ptr<cyber::Node> node_;

  std::string broadcast_tf_frame_id_ = "";
  std::string broadcast_tf_child_frame_id_ = "";
  apollo::transform::TransformBroadcaster tf2_broadcaster_;

  std::string localization_topic_ = "";
  std::shared_ptr<cyber::Writer<LocalizationEstimate>> localization_talker_ =
      nullptr;

  std::string localization_status_topic_ = "";
  std::shared_ptr<cyber::Writer<LocalizationStatus>>
      localization_status_talker_ = nullptr;

  std::string localization_keyframe_topic_ = "/apollo/localization/key_frame";
  std::shared_ptr<cyber::Writer<KeyFrame>> localization_keyframe_talker =
      nullptr;

  std::string localization_keygnss_topic_ = "/apollo/localization/key_gnss";
  std::shared_ptr<cyber::Writer<KeyFrame>> localization_keygnss_talker =
      nullptr;

  double pre_system_time_ = 0.0;
};

class ShenLanBackEndComponent final : public apollo::cyber::TimerComponent {
 public:
  ShenLanBackEndComponent();
  ~ShenLanBackEndComponent();

  struct SynCombination {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PoseData lidar_pose;
    PoseData gnss_pose;
    IMUData imu_data;
    CloudData lidar_cloud;
  };

  bool Init() override;
  bool Proc() override;

  void OnPointCloud(const std::shared_ptr<drivers::PointCloud>& message);
  void OnLidarLocalization(
      const std::shared_ptr<LocalizationEstimate>& message);
  void OnGNSSLocalization(const std::shared_ptr<LocalizationEstimate>& message);
  void OnChassis(const std::shared_ptr<canbus::Chassis>& message);
  void OnRawImu(const std::shared_ptr<drivers::gnss::Imu>& imu_msg);
  void OnLoopClosing(const std::shared_ptr<LoopPose>& message);
  //   void OnGnssBestPose(
  //       const std::shared_ptr<drivers::gnss::GnssBestPose>& message);

 private:
  bool InitConfig();
  bool InitIO();
  bool InsertLoopClosurePose();
  void BackEndImplementation();
  bool UpdateIMUPreIntegration(double timestamp);

  bool ValidData();

 private:
  std::string lidar_extrinsics_file = "";

  std::shared_ptr<cyber::Reader<drivers::PointCloud>> lidar_listener_ = nullptr;
  std::string lidar_topic_ = "";

  std::shared_ptr<cyber::Reader<LocalizationEstimate>> lidar_pose_listener_ =
      nullptr;
  std::string lidar_pose_topic_ = "";

  std::shared_ptr<cyber::Reader<LocalizationEstimate>> gnss_pose_listener_ =
      nullptr;
  std::string gnss_pose_topic_ = "";

  std::shared_ptr<cyber::Reader<canbus::Chassis>> chassis_listener_ = nullptr;
  std::string chassis_topic_ = "";

  std::shared_ptr<cyber::Reader<drivers::gnss::Imu>> raw_imu_listener_ =
      nullptr;
  std::string raw_imu_topic_ = "";

  std::shared_ptr<cyber::Reader<LoopPose>> loop_closing_listener_ = nullptr;
  std::string loop_closing_topic_ = "";

  //   std::shared_ptr<cyber::Reader<drivers::gnss::GnssBestPose>>
  //       bestgnsspos_listener_ = nullptr;
  //   std::string bestgnsspos_topic_ = "/apollo/sensor/gnss/best_pose";

 private:
  std::shared_ptr<MsgPublisher> publisher_ = nullptr;
  std::shared_ptr<BackEnd> back_end_ptr_ = nullptr;
  std::shared_ptr<PostProcessing> post_processing_ptr_ = nullptr;
  std::shared_ptr<LoopClosing> loop_closing_ptr_ = nullptr;
  std::shared_ptr<MsgTransfer> tools = nullptr;

  LocalizationEstimate fusion_localization_result_{};
  SynCombination syn_data{};

  bool syn_start = false;

  Eigen::Affine3d lidar_extrinsic = Eigen::Affine3d::Identity();

  std::mutex fusion_pose_list_mutex_;
  std::deque<PoseData, Eigen::aligned_allocator<PoseData>> fusion_pose_list_;
  size_t fusion_pose_list_max_size_ = 5000;

  std::mutex imu_list_mutex_;
  std::deque<IMUData, Eigen::aligned_allocator<IMUData>> imu_list_;
  size_t imu_list_max_size_ = 5000;

  std::mutex imu_raw_list_mutex_;
  std::deque<IMUData, Eigen::aligned_allocator<IMUData>> imu_raw_list_;
  size_t imu_raw_list_max_size_ = 50000;

  std::mutex chassis_list_mutex_;
  std::deque<VelocityData, Eigen::aligned_allocator<VelocityData>>
      chassis_list_;
  size_t chassis_list_max_size_ = 5000;

  std::mutex lidar_pose_list_mutex_;
  std::deque<PoseData, Eigen::aligned_allocator<PoseData>> lidar_pose_list_;
  size_t lidar_pose_list_max_size_ = 5000;

  std::mutex lidar_cloud_list_mutex_;
  std::deque<CloudData, Eigen::aligned_allocator<CloudData>> lidar_cloud_list_;
  size_t lidar_cloud_list_max_size_ = 5000;

  std::mutex gnss_pose_list_mutex_;
  std::deque<PoseData, Eigen::aligned_allocator<PoseData>> gnss_pose_list_;
  size_t gnss_pose_list_max_size_ = 5000;

  std::mutex loop_list_mutex;
  std::deque<LoopPose, Eigen::aligned_allocator<LoopPose>> loop_pose_data_list_;

  bool is_trans_gpstime_to_utctime_ = true;

  std::deque<KeyFrame, Eigen::aligned_allocator<KeyFrame>> key_frames_list_;
  std::deque<KeyFrame, Eigen::aligned_allocator<KeyFrame>>
      optimized_key_frames_list_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

CYBER_REGISTER_COMPONENT(ShenLanBackEndComponent);

}  // namespace localization
}  // namespace apollo
