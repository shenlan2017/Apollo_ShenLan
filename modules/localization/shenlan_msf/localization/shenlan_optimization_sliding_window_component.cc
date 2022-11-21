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

#include "modules/localization/shenlan_msf/localization/shenlan_optimization_sliding_window_component.h"

#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/math/quaternion.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {

using apollo::cyber::Clock;

SlidingWindowComponent::SlidingWindowComponent() {}

SlidingWindowComponent::~SlidingWindowComponent() {
  sliding_window_ptr_->SaveOptimizedTrajectory();
}

bool SlidingWindowComponent::Init() {
  publisher_ = std::shared_ptr<MsgPublisher>(new MsgPublisher(node_));
  std::string config_file_path =
      WORK_SPACE_PATH + "/config/matching/shenlan_sliding_window.yaml";
  sliding_window_ptr_ =
      std::shared_ptr<SlidingWindow>(new SlidingWindow(config_file_path));
  post_processing_ptr_ = std::shared_ptr<PostProcessing>(new PostProcessing);

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

bool SlidingWindowComponent::InitConfig() {
  if (!publisher_->InitConfig()) {
    AERROR << "Init publisher config failed.";
    return false;
  }

  shenlan_config::Config mapping_config;
  if (!apollo::cyber::common::GetProtoFromFile(
          "/apollo/modules/localization/conf/shenlan_localization.pb.txt",
          &mapping_config)) {
    return false;
  }
  AINFO << "ShenLan Mapping Config: " << mapping_config.DebugString();

  lidar_extrinsics_file = mapping_config.lidar_extrinsics_path();
  lidar_topic_ = mapping_config.lidar_trans_topic();
  gnss_pose_topic_ = mapping_config.odometry_gnss_topic();
  lidar_pose_topic_ = mapping_config.lidar_pose_topic();
  raw_imu_topic_ = mapping_config.imu_topic();
  synch_imu_topic_ = mapping_config.synch_imu_topic();
  chassis_topic_ = mapping_config.chassis_topic();
  // loop_closing_topic_ = mapping_config.loop_closing_topic();

  return true;
}

bool SlidingWindowComponent::InitIO() {
  // 初始化发布器
  if (!publisher_->InitIO()) {
    AERROR << "Init publisher io failed.";
    return false;
  }

  // 订阅雷达信息
  cyber::ReaderConfig reader_config;
  reader_config.channel_name = lidar_topic_;
  reader_config.pending_queue_size = 10;
  lidar_listener_ = node_->CreateReader<CloudData>(
      reader_config, std::bind(&SlidingWindowComponent::OnPointCloud, this,
                               std::placeholders::_1));

  // Lidar localization
  reader_config.channel_name = lidar_pose_topic_;
  reader_config.pending_queue_size = 10;
  lidar_pose_listener_ = node_->CreateReader<LocalizationEstimate>(
      reader_config, std::bind(&SlidingWindowComponent::OnLidarLocalization,
                               this, std::placeholders::_1));

  // GNSS localization
  reader_config.channel_name = gnss_pose_topic_;
  reader_config.pending_queue_size = 10;
  gnss_pose_listener_ = node_->CreateReader<LocalizationEstimate>(
      reader_config, std::bind(&SlidingWindowComponent::OnGNSSLocalization,
                               this, std::placeholders::_1));

  // // 订阅Chassis信息
  // reader_config.channel_name = chassis_topic_;
  // reader_config.pending_queue_size = 10;
  // chassis_listener_ = node_->CreateReader<canbus::Chassis>(
  //     reader_config, std::bind(&SlidingWindowComponent::OnChassis, this,
  //                              std::placeholders::_1));

  // 订阅IMU信息
  reader_config.channel_name = raw_imu_topic_;
  reader_config.pending_queue_size = 10;
  raw_imu_listener_ = node_->CreateReader<drivers::gnss::Imu>(
      reader_config, std::bind(&SlidingWindowComponent::OnRawImu, this,
                               std::placeholders::_1));

  reader_config.channel_name = synch_imu_topic_;
  reader_config.pending_queue_size = 10;
  imu_synced_listener_ = node_->CreateReader<IMUData>(
      reader_config, std::bind(&SlidingWindowComponent::OnSynchImu, this,
                               std::placeholders::_1));

  // // 订阅LoopClosing信息
  // reader_config.channel_name = loop_closing_topic_;
  // reader_config.pending_queue_size = 1;
  // loop_closing_listener_ = node_->CreateReader<LoopPose>(
  //     reader_config, std::bind(&SlidingWindowComponent::OnLoopClosing, this,
  //                              std::placeholders::_1));

  return true;
}

bool SlidingWindowComponent::Proc() {
  while (HasData()) {
    if (!ValidData()) continue;

    UpdateBackEnd();
    PublishData();
  }

  return true;
}

bool SlidingWindowComponent::ValidData() {
  std::unique_lock<std::mutex> gnss_lock(gnss_pose_list_mutex_);
  std::unique_lock<std::mutex> lidar_lock(lidar_pose_list_mutex_);
  std::unique_lock<std::mutex> imu_lock(imu_synced_list_mutex_);
  std::unique_lock<std::mutex> cloud_lock(lidar_cloud_list_mutex_);
  current_laser_odom_data_ = lidar_pose_list_.front();
  // current_map_matching_odom_data_ = map_matching_odom_data_buff_.front();
  current_imu_data_ = imu_synced_list_.front();
  current_gnss_pose_data_ = gnss_pose_list_.front();
  current_cloud_data_ = lidar_cloud_list_.front();

  // const double diff_map_matching_odom_time =
  //     current_laser_odom_data_.time_ - current_map_matching_odom_data_.time_;
  const double diff_imu_time =
      current_laser_odom_data_.time_ - current_imu_data_.time_;
  const double diff_gnss_pose_time =
      current_laser_odom_data_.time_ - current_gnss_pose_data_.time_;

  const double half_sampling_time = 0.05;  // LiDAR is 10hz
  if (
    // diff_map_matching_odom_time < -half_sampling_time ||
      diff_imu_time < -half_sampling_time ||
      diff_gnss_pose_time < -half_sampling_time){
    lidar_pose_list_.pop_front();
    return false;
  }

  // if (diff_map_matching_odom_time > half_sampling_time) {
  //   map_matching_odom_data_buff_.pop_front();
  //   return false;
  // }

  if (diff_imu_time > half_sampling_time) {
    imu_synced_list_.pop_front();
    return false;
  }

  if (diff_gnss_pose_time > half_sampling_time) {
    gnss_pose_list_.pop_front();
    return false;
  }

  lidar_pose_list_.pop_front();
  // map_matching_odom_data_buff_.pop_front();
  imu_synced_list_.pop_front();
  gnss_pose_list_.pop_front();
  lidar_cloud_list_.pop_front();

  return true;
}

bool SlidingWindowComponent::HasData() {
  // map_matching_odom_data_buff_ drop
  if (lidar_pose_list_.empty() || imu_synced_list_.empty() ||
      gnss_pose_list_.empty() || lidar_cloud_list_.empty()) {
    return false;
  }
  return true;
}

bool SlidingWindowComponent::PublishData() {
  if (sliding_window_ptr_->has_new_key_frame()) {

    KeyFrame current_key_frame =
        sliding_window_ptr_->get_current_key_frame();
    key_frames_list_.emplace_back(current_key_frame);

    // KeyFrame current_key_gnss =
    //     sliding_window_ptr_->get_current_key_gnss();
  }

  if (sliding_window_ptr_->has_new_optimized()) {
    KeyFrame key_frame;
    sliding_window_ptr_->GetLatestOptimizedOdometry(key_frame);
    tools.ComposeLocalizationResult(key_frame.time_, key_frame.pose_,
                                    &fusion_localization_result_);
    publisher_->PublishLocalizationSLMSFFusion(fusion_localization_result_);
  }else{
    tools.ComposeLocalizationResult(current_gnss_pose_data_.time_,
                                    current_gnss_pose_data_.pose_,
                                    &fusion_localization_result_);
    publisher_->PublishLocalizationSLMSFFusion(fusion_localization_result_);
  }

  return true;
}

bool SlidingWindowComponent::UpdateBackEnd() {

  UpdateIMUPreIntegration();

  return sliding_window_ptr_->Update(
      current_laser_odom_data_, current_map_matching_odom_data_,
      current_imu_data_, current_gnss_pose_data_);
}

bool SlidingWindowComponent::UpdateIMUPreIntegration() {
  std::unique_lock<std::mutex> raw_imu_lock(imu_raw_list_mutex_);
  while (!imu_raw_list_.empty() &&
         imu_raw_list_.front().time_ < current_imu_data_.time_ &&
         sliding_window_ptr_->UpdateIMUPreIntegration(imu_raw_list_.front())) {
    imu_raw_list_.pop_front();
  }

  return true;
}

void SlidingWindowComponent::OnLoopClosing(
    const std::shared_ptr<LoopPose>& message) {
  std::unique_lock<std::mutex> lock(loop_list_mutex);
  loop_pose_data_list_.push_back(*message);
}

void SlidingWindowComponent::OnRawImu(
    const std::shared_ptr<drivers::gnss::Imu>& imu_msg) {
  IMUData imu;
  tools.IMUMsgTransfer(imu_msg, imu);

  std::unique_lock<std::mutex> raw_imu_lock(imu_raw_list_mutex_);
  if (imu_raw_list_.size() < imu_raw_list_max_size_) {
    imu_raw_list_.push_back(imu);
  } else {
    imu_raw_list_.pop_front();
    imu_raw_list_.push_back(imu);
  }
}

void SlidingWindowComponent::OnSynchImu(
    const std::shared_ptr<IMUData>& imu_synced_msg) {
  std::unique_lock<std::mutex> lock(imu_synced_list_mutex_);

  if (imu_synced_list_.size() < imu_synced_list_max_size_) {
    imu_synced_list_.push_back(*imu_synced_msg);
  } else {
    imu_synced_list_.pop_front();
    imu_synced_list_.push_back(*imu_synced_msg);
  }
}

void SlidingWindowComponent::OnLidarLocalization(
    const std::shared_ptr<LocalizationEstimate>& msg) {
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  Eigen::Quaternionf orientation;

  position[0] = msg->pose().position().x();
  position[1] = msg->pose().position().y();
  position[2] = msg->pose().position().z();
  orientation.x() = msg->pose().orientation().qx();
  orientation.y() = msg->pose().orientation().qy();
  orientation.z() = msg->pose().orientation().qz();
  orientation.w() = msg->pose().orientation().qw();

  PoseData lidar_pose_frame;
  lidar_pose_frame.time_ = msg->measurement_time();
  lidar_pose_frame.pose_.block<3, 3>(0, 0) = orientation.toRotationMatrix();
  lidar_pose_frame.pose_.block<3, 1>(0, 3) = position;
  if (msg->msf_status().local_lidar_quality() ==
      LocalLidarQuality::MSF_LOCAL_LIDAR_BAD) {
    std::vector<double> cov_tmp(36, 1);
    lidar_pose_frame.cov_ = cov_tmp;
  } else {
    std::vector<double> cov_tmp(36, 0);
    lidar_pose_frame.cov_ = cov_tmp;
  }

  std::unique_lock<std::mutex> lock(lidar_pose_list_mutex_);
  if (lidar_pose_list_.size() < lidar_pose_list_max_size_) {
    lidar_pose_list_.push_back(lidar_pose_frame);
  } else {
    lidar_pose_list_.pop_front();
    lidar_pose_list_.push_back(lidar_pose_frame);
  }
}

void SlidingWindowComponent::OnGNSSLocalization(
    const std::shared_ptr<LocalizationEstimate>& msg) {
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  Eigen::Quaternionf orientation;

  position[0] = msg->pose().position().x();
  position[1] = msg->pose().position().y();
  position[2] = msg->pose().position().z();
  orientation.x() = msg->pose().orientation().qx();
  orientation.y() = msg->pose().orientation().qy();
  orientation.z() = msg->pose().orientation().qz();
  orientation.w() = msg->pose().orientation().qw();

  PoseData gnss_pose_frame;
  gnss_pose_frame.time_ = msg->measurement_time();
  gnss_pose_frame.pose_.block<3, 3>(0, 0) = orientation.toRotationMatrix();
  gnss_pose_frame.pose_.block<3, 1>(0, 3) = position;
  if (msg->msf_status().local_lidar_quality() ==
      LocalLidarQuality::MSF_LOCAL_LIDAR_BAD) {
    std::vector<double> cov_tmp(36, 53);
    gnss_pose_frame.cov_ = cov_tmp;
  } else {
    std::vector<double> cov_tmp(36, 56);
    gnss_pose_frame.cov_ = cov_tmp;
  }

  std::unique_lock<std::mutex> lock(gnss_pose_list_mutex_);
  if (gnss_pose_list_.size() < gnss_pose_list_max_size_) {
    gnss_pose_list_.push_back(gnss_pose_frame);
  } else {
    gnss_pose_list_.pop_front();
    gnss_pose_list_.push_back(gnss_pose_frame);
  }
}

void SlidingWindowComponent::OnChassis(
    const std::shared_ptr<canbus::Chassis>& message) {
  VelocityData chassis;
  tools.ChassisMsgTransfer(message, chassis);
  std::unique_lock<std::mutex> lock(chassis_list_mutex_);
  if (chassis_list_.size() < chassis_list_max_size_) {
    chassis_list_.push_back(chassis);
  } else {
    chassis_list_.pop_front();
    chassis_list_.push_back(chassis);
  }
}

void SlidingWindowComponent::OnPointCloud(
    const std::shared_ptr<CloudData>& message) {
  std::unique_lock<std::mutex> cloud_lock(lidar_cloud_list_mutex_);
  if (lidar_cloud_list_.size() < lidar_cloud_list_max_size_) {
    lidar_cloud_list_.push_back(*message);
  } else {
    lidar_cloud_list_.pop_front();
    lidar_cloud_list_.push_back(*message);
  }
}

// --------------------------------------------------------------- //

MsgPublisher::MsgPublisher(const std::shared_ptr<cyber::Node>& node)
    : node_(node), tf2_broadcaster_(node) {}

bool MsgPublisher::InitConfig() {
  shenlan_config::Config mapping_config;
  if (!apollo::cyber::common::GetProtoFromFile(
          "/apollo/modules/localization/conf/shenlan_localization.pb.txt",
          &mapping_config)) {
    return false;
  }
  AINFO << "ShenLan Mapping Config: " << mapping_config.DebugString();
  localization_topic_ = mapping_config.localization_topic();
  localization_status_topic_ = mapping_config.localization_status_topic();
  broadcast_tf_frame_id_ = mapping_config.broadcast_tf_frame_id();
  broadcast_tf_child_frame_id_ = mapping_config.broadcast_tf_child_frame_id();
  return true;
}

bool MsgPublisher::InitIO() {
  localization_talker_ =
      node_->CreateWriter<LocalizationEstimate>(localization_topic_);

  localization_status_talker_ =
      node_->CreateWriter<LocalizationStatus>(localization_status_topic_);

  localization_keyframe_talker =
      node_->CreateWriter<KeyFrame>(localization_keyframe_topic_);

  localization_keygnss_talker =
      node_->CreateWriter<KeyFrame>(localization_keygnss_topic_);

  return true;
}

void MsgPublisher::PublishPoseBroadcastTF(
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

void MsgPublisher::PublishPoseBroadcastTopic(
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

void MsgPublisher::PublishLocalizationStatus(
    const LocalizationStatus& localization_status) {
  localization_status_talker_->Write(localization_status);
}

void MsgPublisher::PublishKeyFrame(const KeyFrame& key_frame) {
  localization_keyframe_talker->Write(key_frame);
}

void MsgPublisher::PublishKeyGnssFrame(const KeyFrame& key_frame) {
  localization_keygnss_talker->Write(key_frame);
}

void MsgPublisher::PublishLocalizationSLMSFFusion(
    const LocalizationEstimate& localization) {
  localization_talker_->Write(localization);
}

}  // namespace localization
}  // namespace apollo
