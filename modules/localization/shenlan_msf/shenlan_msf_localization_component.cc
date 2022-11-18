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

#include "modules/localization/shenlan_msf/shenlan_msf_localization_component.h"

#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/math/quaternion.h"
// #include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {

using apollo::cyber::Clock;

SLMSFLocalizationComponent::SLMSFLocalizationComponent() {}

SLMSFLocalizationComponent::~SLMSFLocalizationComponent()
{}

bool SLMSFLocalizationComponent::Init() {
    publisher_.reset(new LocalizationMsgPublisher(this->node_));

    // 初始化配置环境（定位器参数、发布器参数）
    if (!InitConfig()) {
        AERROR << "Init Config failed.";
        return false;
    }

    // 初始化发布器、订阅器
    if (!InitIO()) {
        AERROR << "Init IO failed.";
        return false;
    }
    AINFO << "SUCCESS INIT.";
    return true;
}

bool SLMSFLocalizationComponent::InitConfig() {
    // lidar_topic_ = FLAGS_lidar_topic;
    // bestgnsspos_topic_ = FLAGS_gnss_best_pose_topic;

    if (!publisher_->InitConfig()) {
        AERROR << "Init publisher config failed.";
        return false;
    }

      if (!localization_.Init().ok()) {
        AERROR << "Init class SLMSFLocalization failed.";
        return false;
      }

    return true;
}

bool SLMSFLocalizationComponent::InitIO() {
    cyber::ReaderConfig reader_config;
    reader_config.channel_name = lidar_topic_;
    reader_config.pending_queue_size = 1;

    // 订阅雷达信息
    std::function<void(const std::shared_ptr<drivers::PointCloud>&)>
        lidar_register_call = std::bind(&SLMSFLocalization::OnPointCloud,
                                        &localization_, std::placeholders::_1);
    lidar_listener_ = this->node_->CreateReader<drivers::PointCloud>(
        reader_config, lidar_register_call);

    // 订阅gnss信息
    std::function<void(const std::shared_ptr<drivers::gnss::GnssBestPose>&)>
        bestgnsspos_register_call =
            std::bind(&SLMSFLocalization::OnGnssBestPose, &localization_,
                    std::placeholders::_1);
    bestgnsspos_listener_ =
        this->node_->CreateReader<drivers::gnss::GnssBestPose>(
            bestgnsspos_topic_, bestgnsspos_register_call);

    // 订阅Odometry信息
    std::function<void(const std::shared_ptr<localization::Gps>&)>
        odometry_register_call =
            std::bind(&SLMSFLocalization::OnOdometry, &localization_,
                    std::placeholders::_1);
    odometry_listener_ =
        this->node_->CreateReader<localization::Gps>(
            odometry_topic_, odometry_register_call);

    // 订阅InsStat信息
    std::function<void(const std::shared_ptr<drivers::gnss::InsStat>&)>
        odometry_status_register_call =
            std::bind(&SLMSFLocalization::OnInsStat, &localization_,
                      std::placeholders::_1);
    odometry_status_listener_ =
        this->node_->CreateReader<drivers::gnss::InsStat>(
            odometry_status_topic_, odometry_status_register_call);

    // 订阅Chassis信息
    std::function<void(const std::shared_ptr<canbus::Chassis>&)>
        chassis_register_call =
            std::bind(&SLMSFLocalization::OnChassis, &localization_,
                      std::placeholders::_1);
    chassis_listener_ = this->node_->CreateReader<canbus::Chassis>(
        chassis_topic_, chassis_register_call);

    // 订阅LoopClosing信息
    std::function<void(const std::shared_ptr<LoopPose>&)>
        loop_closing_register_call =
            std::bind(&SLMSFLocalization::OnLoopClosing, &localization_,
                      std::placeholders::_1);
    loop_closing_listener_ = this->node_->CreateReader<LoopPose>(
        loop_closing_topic_, loop_closing_register_call);

    // 初始化发布器
    if (!publisher_->InitIO()) {
        AERROR << "Init publisher io failed.";
        return false;
    }

    localization_.SetPublisher(publisher_);

    return true;
}

bool SLMSFLocalizationComponent::Proc(
    const std::shared_ptr<drivers::gnss::Imu>& imu_msg) {
  localization_.OnRawImuCache(imu_msg);

  return true;
}

LocalizationMsgPublisher::LocalizationMsgPublisher(
    const std::shared_ptr<cyber::Node>& node)
    : node_(node), tf2_broadcaster_(node) {}

bool LocalizationMsgPublisher::InitConfig() {
  localization_topic_ = "/apollo/localization/pose";
  broadcast_tf_frame_id_ = "world";
  broadcast_tf_child_frame_id_ = "localization";
  // lidar_local_topic_ = FLAGS_localization_lidar_topic;
  // gnss_local_topic_ = FLAGS_localization_gnss_topic;
  localization_status_topic_ = "/apollo/localization/shenlan_msf_status";

  return true;
}

bool LocalizationMsgPublisher::InitIO() {
    localization_talker_ =
        node_->CreateWriter<LocalizationEstimate>(localization_topic_);

    localization_status_talker_ =
        node_->CreateWriter<LocalizationStatus>(localization_status_topic_);

    lidar_local_talker_ =
        node_->CreateWriter<LocalizationEstimate>(lidar_local_topic_);

    gnss_local_talker_ =
        node_->CreateWriter<LocalizationEstimate>(gnss_local_topic_);

    localization_keyframe_talker = 
        node_->CreateWriter<KeyFrame>(localization_keyframe_topic_);

    localization_keygnss_talker = 
        node_->CreateWriter<KeyFrame>(localization_keygnss_topic_);   

    return true;
}

void LocalizationMsgPublisher::PublishPoseBroadcastTF(
    const LocalizationEstimate& localization) {
    // broadcast tf message
    apollo::transform::TransformStamped tf2_msg;

    auto mutable_head = tf2_msg.mutable_header();
    mutable_head->set_timestamp_sec(localization.measurement_time());
    mutable_head->set_frame_id(broadcast_tf_frame_id_);
    tf2_msg.set_child_frame_id(broadcast_tf_child_frame_id_);

    auto mutable_translation = tf2_msg.mutable_transform()->mutable_translation();
    mutable_translation->set_x(localization.pose().position().x());
    mutable_translation->set_y(localization.pose().position().y());
    mutable_translation->set_z(localization.pose().position().z());

    auto mutable_rotation = tf2_msg.mutable_transform()->mutable_rotation();
    mutable_rotation->set_qx(localization.pose().orientation().qx());
    mutable_rotation->set_qy(localization.pose().orientation().qy());
    mutable_rotation->set_qz(localization.pose().orientation().qz());
    mutable_rotation->set_qw(localization.pose().orientation().qw());

    tf2_broadcaster_.SendTransform(tf2_msg);
}

void LocalizationMsgPublisher::PublishPoseBroadcastTopic(
    const LocalizationEstimate& localization) {
    double cur_system_time = localization.header().timestamp_sec();
    if (pre_system_time_ > 0.0 && cur_system_time - pre_system_time_ > 0.02) {
    AERROR << std::setprecision(16)
             << "the localization processing time enlonged more than 2 times "
                "according to system time, "
             << "the pre system time and current system time: "
             << pre_system_time_ << " " << cur_system_time;
    } else if (pre_system_time_ > 0.0 &&
             cur_system_time - pre_system_time_ < 0.0) {
    AERROR << std::setprecision(16)
             << "published localization message's time is eary than last imu "
                "message "
                "according to system time, "
             << "the pre system time and current system time: "
             << pre_system_time_ << " " << cur_system_time;
    }
    pre_system_time_ = cur_system_time;
    localization_talker_->Write(localization);
}


void LocalizationMsgPublisher::PublishLocalizationStatus(
    const LocalizationStatus& localization_status) {
    localization_status_talker_->Write(localization_status);
}
void LocalizationMsgPublisher::PublishKeyFrame(const KeyFrame& key_frame){
    localization_keyframe_talker->Write(key_frame);
}

void LocalizationMsgPublisher::PublishKeyGnssFrame(const KeyFrame& key_frame){
    localization_keygnss_talker->Write(key_frame);
}

void LocalizationMsgPublisher::PublishLocalizationSLMSFLidar(
    const LocalizationEstimate& localization) {
  lidar_local_talker_->Write(localization);
}

void LocalizationMsgPublisher::PublishLocalizationSLMSFGnss(
    const LocalizationEstimate& localization){
  gnss_local_talker_->Write(localization);
}

void LocalizationMsgPublisher::PublishLocalizationSLMSFFusion(
    const LocalizationEstimate& localization) {
  localization_talker_->Write(localization);
}

}  // namespace localization
}  // namespace apollo
