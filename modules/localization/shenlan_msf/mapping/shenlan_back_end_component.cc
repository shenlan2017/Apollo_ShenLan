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

#include "modules/localization/shenlan_msf/mapping/shenlan_back_end_component.h"

#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/math/quaternion.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {

using apollo::cyber::Clock;

ShenLanBackEndComponent::ShenLanBackEndComponent() {}

ShenLanBackEndComponent::~ShenLanBackEndComponent() {
  std::cout << "System Begin to Optimize and Please Wait A Few Seconds!\n";
  for (int i = 0; i < 3; ++i) {
    back_end_ptr_->ForceOptimize();
    back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames_list_);
  }
  back_end_ptr_->SaveOptimizedPose();

  std::cout << "Save Map Begin!\n";

  post_processing_ptr_->UpdateWithOptimizedKeyFrames(
      optimized_key_frames_list_);
  if (!post_processing_ptr_->SaveMap()) {
    std::cerr << "Fail to SaveMap!\n";
  }

  std::cout << "[ShenLanBackEndComponent] Finished All Activity and Will be "
               "Closed!\n";
}

bool ShenLanBackEndComponent::Init() {
  publisher_ = std::shared_ptr<MsgPublisher>(new MsgPublisher(node_));

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

bool ShenLanBackEndComponent::InitConfig() {
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
  lidar_topic_ = mapping_config.lidar_topic();
  gnss_pose_topic_ = mapping_config.odometry_gnss_topic();
  lidar_pose_topic_ = mapping_config.lidar_pose_topic();
  chassis_topic_ = mapping_config.chassis_topic();
  raw_imu_topic_ = mapping_config.imu_topic();
  loop_closing_topic_ = mapping_config.loop_closing_topic();


  const std::string config_file_path =
      WORK_SPACE_PATH + "/config/mapping/shenlan_lio_back_end.yaml";

  back_end_ptr_ = std::shared_ptr<BackEnd>(new BackEnd(config_file_path));
  loop_closing_ptr_ =
      std::shared_ptr<LoopClosing>(new LoopClosing((WORK_SPACE_PATH)));
  tools = std::shared_ptr<MsgTransfer>(new MsgTransfer());
  post_processing_ptr_ = std::shared_ptr<PostProcessing>(new PostProcessing);

  lidar_extrinsic = Eigen::Affine3d::Identity();
  if (!tools->LoadLidarExtrinsic(lidar_extrinsics_file, &lidar_extrinsic)) {
    AERROR << "Fail to Load Lidar Extrinsic!";
  }
  return true;
}

bool ShenLanBackEndComponent::InitIO() {
  // 初始化发布器
  if (!publisher_->InitIO()) {
    AERROR << "Init publisher io failed.";
    return false;
  }

  // 订阅雷达信息
  cyber::ReaderConfig reader_config;
  reader_config.channel_name = lidar_topic_;
  reader_config.pending_queue_size = 10;
  lidar_listener_ = node_->CreateReader<drivers::PointCloud>(
      reader_config, std::bind(&ShenLanBackEndComponent::OnPointCloud, this,
                               std::placeholders::_1));

  // Lidar localization
  reader_config.channel_name = lidar_pose_topic_;
  reader_config.pending_queue_size = 10;
  lidar_pose_listener_ = node_->CreateReader<LocalizationEstimate>(
      reader_config, std::bind(&ShenLanBackEndComponent::OnLidarLocalization,
                               this, std::placeholders::_1));

  // GNSS localization
  reader_config.channel_name = gnss_pose_topic_;
  reader_config.pending_queue_size = 10;
  gnss_pose_listener_ = node_->CreateReader<LocalizationEstimate>(
      reader_config, std::bind(&ShenLanBackEndComponent::OnGNSSLocalization,
                               this, std::placeholders::_1));

  // // 订阅gnss信息
  // bestgnsspos_listener_ = node_->CreateReader<drivers::gnss::GnssBestPose>(
  //     bestgnsspos_topic_, std::bind(&ShenLanBackEndComponent::OnGnssBestPose,
  //                                   this, std::placeholders::_1));

  // 订阅Chassis信息
  reader_config.channel_name = chassis_topic_;
  reader_config.pending_queue_size = 10;
  chassis_listener_ = node_->CreateReader<canbus::Chassis>(
      reader_config, std::bind(&ShenLanBackEndComponent::OnChassis, this,
                               std::placeholders::_1));

  // 订阅IMU信息
  reader_config.channel_name = raw_imu_topic_;
  reader_config.pending_queue_size = 10;
  raw_imu_listener_ = node_->CreateReader<drivers::gnss::Imu>(
      reader_config, std::bind(&ShenLanBackEndComponent::OnRawImu, this,
                               std::placeholders::_1));

  // 订阅LoopClosing信息
  reader_config.channel_name = loop_closing_topic_;
  reader_config.pending_queue_size = 1;
  loop_closing_listener_ = node_->CreateReader<LoopPose>(
      reader_config, std::bind(&ShenLanBackEndComponent::OnLoopClosing, this,
                               std::placeholders::_1));

  return true;
}

bool ShenLanBackEndComponent::UpdateIMUPreIntegration(double timestamp) {
  std::unique_lock<std::mutex> lock(imu_raw_list_mutex_);
  while (!imu_raw_list_.empty() && imu_raw_list_.front().time_ < timestamp &&
         back_end_ptr_->UpdateIMUPreIntegration(imu_raw_list_.front())) {
    imu_raw_list_.pop_front();
  }
  return true;
}

bool ShenLanBackEndComponent::Proc() {
  InsertLoopClosurePose();

  if (ValidData()) {
    // std::cout << "Running ... ... ... ...\r";

    CloudData lidar_frame = syn_data.lidar_cloud;
    PoseData lidar_pose_result = syn_data.lidar_pose;
    PoseData gnss_pose_result = syn_data.gnss_pose;
    IMUData imu_data_result = syn_data.imu_data;

    double timestamp = lidar_frame.time_;
    UpdateIMUPreIntegration(timestamp);

    back_end_ptr_->Update(lidar_frame, lidar_pose_result, gnss_pose_result,
                          imu_data_result);

    KeyFrame key_frame, gnss_frame;
    if (back_end_ptr_->has_new_key_frame()) {
      back_end_ptr_->GetLatestKeyFrame(key_frame);
      back_end_ptr_->GetLatestKeyGNSS(gnss_frame);
      key_frames_list_.emplace_back(key_frame);
    }

    if (back_end_ptr_->has_new_optimized()) {
      back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames_list_);
    }

    // ***************************************************** //

    post_processing_ptr_->UpdateWithNewKeyFrame(gnss_pose_result, lidar_frame,
                                                key_frames_list_);

    if (!optimized_key_frames_list_.empty()) {
      post_processing_ptr_->UpdateWithOptimizedKeyFrames(
          optimized_key_frames_list_);
    }

    Eigen::Matrix4f cur_pose = post_processing_ptr_->GetCurrentPose();
    tools->ComposeLocalizationResult(timestamp, cur_pose,
                                     &fusion_localization_result_);
    publisher_->PublishKeyFrame(key_frame);
    publisher_->PublishKeyGnssFrame(key_frame);
    publisher_->PublishLocalizationSLMSFFusion(fusion_localization_result_);
  }

  if (!optimized_key_frames_list_.empty()) {
    post_processing_ptr_->UpdateWithOptimizedKeyFrames(
        optimized_key_frames_list_);
  }

  return true;
}

bool ShenLanBackEndComponent::ValidData() {
  if (lidar_cloud_list_.empty() || lidar_pose_list_.empty() ||
      gnss_pose_list_.empty() || imu_list_.empty()) {
    return false;
  }

  CloudData lidar_frame;
  {
    std::unique_lock<std::mutex> cloud_lock(lidar_cloud_list_mutex_);
    lidar_frame = lidar_cloud_list_.front();
    lidar_cloud_list_.pop_front();
  }

  double timestamp = lidar_frame.time_;
  bool syn_lidar = false, syn_gnss = false, syn_imu = false;

  PoseData lidar_pose_result;
  {
    std::unique_lock<std::mutex> lidar_lock(lidar_pose_list_mutex_);
    int count_flag = 10;
    while (count_flag-- > 0) {
      if (PoseData::SyncData(timestamp, lidar_pose_list_, lidar_pose_result)) {
        syn_lidar = true;
        break;
      }
      lidar_lock.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      lidar_lock.lock();
    }
    if (!syn_lidar) {
      AINFO << std::setprecision(16) << "timestamp1: " << timestamp;
      AINFO << std::setprecision(16)
            << "timestamp2: " << lidar_pose_list_.back().time_;
      AERROR << "Fail to Sync Pose Data, Please Check!";
      return false;
    }
  }

  PoseData gnss_pose_result;
  {
    std::unique_lock<std::mutex> gnss_lock(gnss_pose_list_mutex_);
    // TODO:这块逻辑可以优化，这里处理有问题;
    int count_flag = 20;
    while (count_flag-- > 0) {
      if (PoseData::SyncData(timestamp, gnss_pose_list_, gnss_pose_result)) {
        syn_gnss = true;
        break;
      }
      gnss_lock.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      gnss_lock.lock();
    }
    if (!syn_gnss) {
      AINFO << std::setprecision(16) << "timestamp1: " << timestamp;
      AINFO << std::setprecision(16)
            << "timestamp2: " << gnss_pose_list_.front().time_;
      AINFO << std::setprecision(16)
            << "timestamp3: " << gnss_pose_list_.back().time_;
      AERROR << "Fail to Sync Pose Data, Please Check!";
      return false;
    }
  }

  IMUData imu_data_result;
  {
    std::unique_lock<std::mutex> imu_lock(imu_list_mutex_);
    int count_flag = 10;
    while (count_flag-- > 0) {
      if (IMUData::SyncData(timestamp, imu_list_, imu_data_result)) {
        syn_imu = true;
        break;
      }
      imu_lock.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      imu_lock.lock();
    }
    if (!syn_imu) {
      AINFO << std::setprecision(16) << "timestamp1: " << timestamp;
      AINFO << std::setprecision(16)
            << "timestamp2: " << imu_list_.back().time_;
      AERROR << "Fail to Sync Pose Data, Please Check!";
      return false;
    }
  }

  const double diff_lidar_time = timestamp - lidar_pose_result.time_;
  const double diff_gnss_time = timestamp - gnss_pose_result.time_;
  const double diff_imu_time = timestamp - imu_data_result.time_;

  const double half_sampling_time = 0.05;  // LiDAR is 10hz

  if (diff_lidar_time < -half_sampling_time ||
      diff_gnss_time < -half_sampling_time ||
      diff_imu_time < -half_sampling_time) {
    return false;
  }

  if (diff_lidar_time > half_sampling_time ||
      diff_gnss_time > half_sampling_time ||
      diff_imu_time > half_sampling_time) {
    return false;
  }

  syn_data.lidar_cloud = lidar_frame;
  syn_data.lidar_pose = lidar_pose_result;
  syn_data.gnss_pose = gnss_pose_result;
  syn_data.imu_data = imu_data_result;
  syn_start = true;
  return true;
}

void ShenLanBackEndComponent::OnLoopClosing(
    const std::shared_ptr<LoopPose>& message) {
  std::unique_lock<std::mutex> lock(loop_list_mutex);
  loop_pose_data_list_.push_back(*message);
}

void ShenLanBackEndComponent::OnRawImu(
    const std::shared_ptr<drivers::gnss::Imu>& imu_msg) {
  IMUData imu;
  tools->IMUMsgTransfer(imu_msg, imu);

  std::unique_lock<std::mutex> imu_lock(imu_list_mutex_);
  if (imu_list_.size() < imu_list_max_size_) {
    imu_list_.push_back(imu);
  } else {
    imu_list_.pop_front();
    imu_list_.push_back(imu);
  }
  imu_lock.unlock();

  std::unique_lock<std::mutex> raw_imu_lock(imu_raw_list_mutex_);
  if (imu_raw_list_.size() < imu_raw_list_max_size_) {
    imu_raw_list_.push_back(imu);
  } else {
    imu_raw_list_.pop_front();
    imu_raw_list_.push_back(imu);
  }
  raw_imu_lock.unlock();
}

void ShenLanBackEndComponent::OnLidarLocalization(
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

void ShenLanBackEndComponent::OnGNSSLocalization(
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

void ShenLanBackEndComponent::OnChassis(
    const std::shared_ptr<canbus::Chassis>& message) {
  VelocityData chassis;
  tools->ChassisMsgTransfer(message, chassis);
  std::unique_lock<std::mutex> lock(chassis_list_mutex_);
  if (chassis_list_.size() < chassis_list_max_size_) {
    chassis_list_.push_back(chassis);
  } else {
    chassis_list_.pop_front();
    chassis_list_.push_back(chassis);
  }
}

void ShenLanBackEndComponent::OnPointCloud(
    const std::shared_ptr<drivers::PointCloud>& message) {
  CloudData tmp_lidar_frame;
  tools->LidarMsgTransfer(message, &tmp_lidar_frame);

  CloudData::CloudTypePtr temp_cloud_ptr(new CloudData::CloudType());
  Eigen::Matrix4f T_imu_lidar_ = lidar_extrinsic.matrix().cast<float>();
  pcl::transformPointCloud(*tmp_lidar_frame.cloud_ptr_, *temp_cloud_ptr,
                           T_imu_lidar_);
  CloudData lidar_frame;
  lidar_frame.time_ = tmp_lidar_frame.time_;
  lidar_frame.cloud_ptr_ = temp_cloud_ptr;

  std::unique_lock<std::mutex> cloud_lock(lidar_cloud_list_mutex_);
  if (lidar_cloud_list_.size() < lidar_cloud_list_max_size_) {
    lidar_cloud_list_.push_back(lidar_frame);
  } else {
    lidar_cloud_list_.pop_front();
    lidar_cloud_list_.push_back(lidar_frame);
  }
}


bool ShenLanBackEndComponent::InsertLoopClosurePose() {
  std::unique_lock<std::mutex> lock(loop_list_mutex);
  while (loop_pose_data_list_.size() > 0) {
    back_end_ptr_->InsertLoopPose(loop_pose_data_list_.front());
    loop_pose_data_list_.pop_front();
  }
  return true;
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
