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
#include "modules/localization/shenlan_msf/shenlan_msf_localization.h"
#include "modules/transform/transform_broadcaster.h"

using namespace lidar_localization;

namespace apollo {
namespace localization {

class SLMSFLocalizationComponent final
    : public cyber::Component<drivers::gnss::Imu> {
 public:
  SLMSFLocalizationComponent();
  ~SLMSFLocalizationComponent();

  bool Init() override;

  bool Proc(const std::shared_ptr<drivers::gnss::Imu>& imu_msg) override;

 private:
  bool InitConfig();
  bool InitIO();

 private:
  std::shared_ptr<cyber::Reader<drivers::PointCloud>> lidar_listener_ = nullptr;
//   std::string lidar_topic_ = "/apollo/sensor/lidar16/compensator/PointCloud2";
  std::string lidar_topic_ = "/apollo/sensor/lidar/PointCloud2";

  std::shared_ptr<cyber::Reader<drivers::gnss::GnssBestPose>>
      bestgnsspos_listener_ = nullptr;
  std::string bestgnsspos_topic_ = "/apollo/sensor/gnss/best_pose";

  std::shared_ptr<cyber::Reader<localization::Gps>>
      odometry_listener_ = nullptr;
  std::string odometry_topic_ = "/apollo/sensor/gnss/odometry";

  std::shared_ptr<cyber::Reader<canbus::Chassis>> chassis_listener_ =
      nullptr;
  std::string chassis_topic_ = "/apollo/canbus/chassis";

  std::shared_ptr<cyber::Reader<drivers::gnss::InsStat>>
      odometry_status_listener_ = nullptr;
  std::string odometry_status_topic_ = "/apollo/sensor/gnss/ins_stat";

  std::shared_ptr<cyber::Reader<LoopPose>>
      loop_closing_listener_ = nullptr;
  std::string loop_closing_topic_ = "/apollo/localization/loop_pose";

 private:
  std::shared_ptr<LocalizationMsgPublisher> publisher_;
  SLMSFLocalization localization_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

CYBER_REGISTER_COMPONENT(SLMSFLocalizationComponent);

class LocalizationMsgPublisher {
 public:
  explicit LocalizationMsgPublisher(const std::shared_ptr<cyber::Node>& node);
  ~LocalizationMsgPublisher() = default;

  bool InitConfig();
  bool InitIO();

  void PublishPoseBroadcastTF(const LocalizationEstimate& localization);
  void PublishPoseBroadcastTopic(const LocalizationEstimate& localization);

  void PublishLocalizationSLMSFGnss(const LocalizationEstimate& localization);
  void PublishLocalizationSLMSFLidar(const LocalizationEstimate& localization);
  void PublishLocalizationSLMSFFusion(const LocalizationEstimate& localization);
  void PublishLocalizationStatus(const LocalizationStatus& localization_status);
  void PublishKeyFrame(const KeyFrame& key_frame);
  void PublishKeyGnssFrame(const KeyFrame& key_frame);


 private:
  std::shared_ptr<cyber::Node> node_;

  std::string broadcast_tf_frame_id_ = "";
  std::string broadcast_tf_child_frame_id_ = "";
  apollo::transform::TransformBroadcaster tf2_broadcaster_;

  std::string localization_topic_ = "/apollo/localization/pose";
  std::shared_ptr<cyber::Writer<LocalizationEstimate>> localization_talker_ =
      nullptr;

  std::string lidar_local_topic_ = "/apollo/localization/shenlan_msf_lidar";
  std::shared_ptr<cyber::Writer<LocalizationEstimate>> lidar_local_talker_ =
      nullptr;

  std::string gnss_local_topic_ = "/apollo/localization/shenlan_msf_gnss";
  std::shared_ptr<cyber::Writer<LocalizationEstimate>> gnss_local_talker_ =
      nullptr;

  std::string localization_status_topic_ = "/apollo/localization/shenlan_msf_status";
  std::shared_ptr<cyber::Writer<LocalizationStatus>>
      localization_status_talker_ = nullptr;


  std::string localization_keyframe_topic_ = "/apollo/localization/key_frame";
  std::shared_ptr<cyber::Writer<KeyFrame>>
      localization_keyframe_talker = nullptr;

  std::string localization_keygnss_topic_ = "/apollo/localization/key_gnss";
  std::shared_ptr<cyber::Writer<KeyFrame>>
      localization_keygnss_talker = nullptr;



  double pre_system_time_ = 0.0;

};

}  // namespace localization
}  // namespace apollo
