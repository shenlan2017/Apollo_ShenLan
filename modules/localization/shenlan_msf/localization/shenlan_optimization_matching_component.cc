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

#include "modules/localization/shenlan_msf/localization/shenlan_optimization_matching_component.h"

#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/math/quaternion.h"

namespace apollo {
namespace localization {

using apollo::cyber::Clock;
using common::util::TimeUtil;

MatchingComponent::MatchingComponent() {}

bool MatchingComponent::Init() {
  // 初始化配置环境
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

bool MatchingComponent::InitConfig() {
  shenlan_config::Config opt_config;
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                              &opt_config)) {
  return false;
  }
  AINFO << "ShenLan Optimization config: " << opt_config.DebugString();

  lidar_topic_ = opt_config.lidar_trans_topic();
  odometry_gnss_topic_ = opt_config.odometry_gnss_topic();
  localization_lidar_topic_ = opt_config.lidar_pose_topic();
  map_matcher_topic_ = opt_config.map_matcher_topic();

  return true;
}

bool MatchingComponent::InitIO() {
  matching_ptr_ = std::shared_ptr<Matching>(new Matching());

  // 订阅雷达信息
  cyber::ReaderConfig reader_config;
  reader_config.channel_name = lidar_topic_;
  reader_config.pending_queue_size = 10;
  lidar_listener_ = node_->CreateReader<CloudData>(
      reader_config, std::bind(&MatchingComponent::LidarCallback, this,
                               std::placeholders::_1));

  // 订阅Odometry信息
  reader_config.channel_name = odometry_gnss_topic_;
  reader_config.pending_queue_size = 100;
  gnss_pose_listener_ = node_->CreateReader<LocalizationEstimate>(
      reader_config, std::bind(&MatchingComponent::OnGNSSLocalization, this,
                               std::placeholders::_1));

  // publish
  localization_lidar_talker_ =
      node_->CreateWriter<LocalizationEstimate>(localization_lidar_topic_);

  // localization_map_matcher_talker_ =
  //     node_->CreateWriter<LocalizationEstimate>(map_matcher_topic_);

  return true;
}

bool MatchingComponent::Proc() {
  if (matching_ptr_->has_new_global_map()) {
  }

  // update local map if necessary:
  if (matching_ptr_->has_new_local_map()) {
  }

  while (HasData()) {
    if (!ValidData()) {
      AINFO << "Invalid data. Skip matching";
      continue;
    }

    if (UpdateMatching()) {
      const double timestamp_synced = current_cloud_data_.time_;
      LocalizationEstimate lidar_localization{};
      tools.ComposeLocalizationResult(timestamp_synced, laser_odometry_,
                                      &lidar_localization);
      localization_lidar_talker_->Write(lidar_localization);

      // // TODO: 源代码实际上没有这个数据，更确切的说，只有地图匹配数据而没有激光里程计数据
      // LocalizationEstimate map_matching_localization{};
      // tools.ComposeLocalizationResult(timestamp_synced, map_matching_odometry_,
      //                                 &map_matching_localization);
      // localization_map_matcher_talker_->Write(map_matching_localization);
    }
  }
  return true;
}

bool MatchingComponent::UpdateMatching() {
  if (!matching_ptr_->has_inited()) {
    matching_ptr_->Init(current_cloud_data_, current_gnss_data_.pose_);
    AINFO << "Initialization successed." << std::endl;
  }

  return matching_ptr_->Update(current_cloud_data_, laser_odometry_,
                               map_matching_odometry_);
}

bool MatchingComponent::HasData() {
  if (cloud_data_buff_.empty() || gnss_data_buff_.empty()) return false;

  if (matching_ptr_->has_inited()) return true;

  return true;
}

bool MatchingComponent::ValidData() {
  std::unique_lock<std::mutex> cloud_lock(cloud_list_mutex_);
  std::unique_lock<std::mutex> gnss_lock(gnss_pose_list_mutex_);
  current_cloud_data_ = cloud_data_buff_.front();

  if (matching_ptr_->has_inited()) {
    cloud_data_buff_.pop_front();
    gnss_data_buff_.clear();
    return true;
  }

  current_gnss_data_ = gnss_data_buff_.front();

  const double diff_time = current_cloud_data_.time_ - current_gnss_data_.time_;
  const double half_sampling_time = 0.05;  // LiDAR is 10hz
  if (diff_time < -half_sampling_time) {
    cloud_data_buff_.pop_front();
    return false;
  }

  if (diff_time > half_sampling_time) {
    gnss_data_buff_.pop_front();
    return false;
  }

  cloud_data_buff_.pop_front();
  gnss_data_buff_.pop_front();

  return true;
}

void MatchingComponent::LidarCallback(const std::shared_ptr<CloudData>& cloud_msg){
  std::unique_lock<std::mutex> lock(cloud_list_mutex_);
  if (cloud_data_buff_.size() < cloud_list_max_size_) {
    cloud_data_buff_.push_back(*cloud_msg);
  } else {
    cloud_data_buff_.pop_front();
    cloud_data_buff_.push_back(*cloud_msg);
  }
}

void MatchingComponent::OnGNSSLocalization(
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
  if (gnss_data_buff_.size() < gnss_pose_list_max_size_) {
    gnss_data_buff_.push_back(gnss_pose_frame);
  } else {
    gnss_data_buff_.pop_front();
    gnss_data_buff_.push_back(gnss_pose_frame);
  }
}


}  // namespace localization
}  // namespace apollo
