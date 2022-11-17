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

#include "modules/localization/shenlan_msf/shenlan_front_end_component.h"

#include "yaml-cpp/yaml.h"

#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/math/euler_angles_zxy.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/time_util.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {

using apollo::cyber::Clock;
using common::util::TimeUtil;

ShenLanFrontEndComponent::ShenLanFrontEndComponent() {}

ShenLanFrontEndComponent::~ShenLanFrontEndComponent() {
  std::cout << "Please Wait A Few Seconds!\n";
  RequestFinish();
  while (!isFinished()) usleep(5000);
  std::cout << "Finished All Activity and Will be Closed!\n";
}

bool ShenLanFrontEndComponent::Init() {
  publisher_ = std::shared_ptr<LocalizationMsgPublisher>(
      new LocalizationMsgPublisher(node_));

  if (!InitConfig()) {
    AERROR << "Init Config failed.";
    return false;
  }

  if (!InitIO()) {
    AERROR << "Init IO failed.";
    return false;
  }

  AINFO << "SUCCESS INIT.";
  return true;
}

bool ShenLanFrontEndComponent::InitConfig() {
  front_end_ptr_ = std::shared_ptr<FrontEnd>(new FrontEnd());
  tools = std::shared_ptr<MsgTransfer>(new MsgTransfer());
  front_end_loop =
      std::thread(&ShenLanFrontEndComponent::UpdateLaserOdometry, this);
  front_end_loop.detach();

  lidar_extrinsic = Eigen::Affine3d::Identity();
  if (!tools->LoadLidarExtrinsic(lidar_extrinsics_file, &lidar_extrinsic)) {
    AERROR << "Fail to Load Lidar Extrinsic!";
  }

  if (!publisher_->InitConfig()) {
    AERROR << "Init publisher config failed.";
    return false;
  }
  return true;
}

void ShenLanFrontEndComponent::RequestFinish() {
  std::unique_lock<std::mutex> lock(mutex_finish_);
  finish_requested_ = true;
}

bool ShenLanFrontEndComponent::CheckFinish() {
  std::unique_lock<std::mutex> lock(mutex_finish_);
  return finish_requested_;
}

void ShenLanFrontEndComponent::SetFinish() {
  std::unique_lock<std::mutex> lock(mutex_finish_);
  finished_ = true;
}

bool ShenLanFrontEndComponent::isFinished() {
  std::unique_lock<std::mutex> lock(mutex_finish_);
  return finished_;
}

bool ShenLanFrontEndComponent::InitIO() {
  cyber::ReaderConfig reader_config;
  reader_config.channel_name = lidar_topic_;
  reader_config.pending_queue_size = 10;

  // 订阅雷达信息
  lidar_listener_ = node_->CreateReader<drivers::PointCloud>(
      reader_config, std::bind(&ShenLanFrontEndComponent::OnPointCloud, this,
                               std::placeholders::_1));

  // 订阅Odometry信息
  odometry_listener_ = node_->CreateReader<localization::Gps>(
      odometry_topic_, std::bind(&ShenLanFrontEndComponent::OnOdometry, this,
                                 std::placeholders::_1));

  // 订阅InsStat信息
  odometry_status_listener_ = node_->CreateReader<drivers::gnss::InsStat>(
      odometry_status_topic_, std::bind(&ShenLanFrontEndComponent::OnInsStat,
                                        this, std::placeholders::_1));
  // 初始化发布器
  if (!publisher_->InitIO()) {
    return false;
  }

  return true;
}

bool ShenLanFrontEndComponent::Proc() {
  // 定时发布
  {
    std::unique_lock<std::mutex> lock(lidar_pose_list_mutex_);
    if (!lidar_pose_list_.empty()) {
      PoseData lidar_pose = lidar_pose_list_.front();
      double timestamp = lidar_pose.time_;
      Eigen::Matrix4f odometry_ = lidar_pose.pose_;
      lidar_pose_list_.pop_front();
      tools->ComposeLocalizationResult(timestamp, odometry_,
                                       &lidar_localization_result_,
                                       lidar_pose.cov_.at(0));
      publisher_->PublishLocalizationSLMSFLidar(lidar_localization_result_);
    }
  }

  {
    std::lock_guard<std::mutex> odom_lock(odom_list_mutex_);
    if (!odom_list_.empty() && odometry_inited) {
      PoseData odometry = odom_list_.front();
      odom_list_.pop_front();
      double timestamp = odometry.time_;
      if (timestamp >= init_timestamp) {
        odometry.pose_.block<3, 1>(0, 3) -= gnss_origin;
        tools->ComposeLocalizationResult(timestamp, odometry.pose_,
                                         &gnss_localization_result_,
                                         odometry.cov_.at(0));
        publisher_->PublishLocalizationSLMSFGnss(gnss_localization_result_);
      }
    }
  }

  return true;
}

void ShenLanFrontEndComponent::UpdateLaserOdometry() {
  while (1) {
    if (CheckFinish()) {
      std::cerr << "CheckFinish Detect Stop!\n";
      break;
    }

    std::unique_lock<std::mutex> lidar_lock(lidar_frame_list_mutex_);
    if (lidar_frame_list_.empty()) {
      lidar_lock.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    std::cout << "Running ... ... ... ...\r";
    CloudData lidar_frame = lidar_frame_list_.front();
    double timestamp = lidar_frame.time_;
    lidar_frame_list_.pop_front();
    lidar_lock.unlock();

    std::unique_lock<std::mutex> odom_lock(odom_list_mutex_);
    if (!odometry_inited) {
      if (odom_list_.size() > 0) {
        PoseData result;
        auto input = odom_list_;

        int count_flag = 10;
        bool syn_odom = false;
        while (count_flag-- > 0) {
          if (PoseData::SyncData(timestamp, input, result)) {
            syn_odom = true;
            break;
          }
          odom_lock.unlock();
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          odom_lock.lock();
        }
        if (!syn_odom) continue;
        Eigen::Matrix4f odom_pose;
        odom_pose = result.pose_;

        gnss_origin = odom_pose.block<3, 1>(0, 3);
        odom_pose.block<3, 1>(0, 3) = Eigen::Vector3f::Zero();

        AINFO << "Init Pose is : \n" << odom_pose;
        front_end_ptr_->SetInitPose(odom_pose);
        init_timestamp = result.time_;
        odometry_inited = true;
      } else {
        AWARN << "FAIL TO GET /apollo/gnss/odometry INFO.";
        continue;
      }
    }
    Eigen::Matrix4f lidar_pose_result;
    front_end_ptr_->Update(lidar_frame, lidar_pose_result);

    PoseData lidar_odometry_result;
    lidar_odometry_result.time_ = timestamp;
    lidar_odometry_result.pose_ = lidar_pose_result;
    std::vector<double> cov_tmp(36, front_end_ptr_->is_degeneracy());
    lidar_odometry_result.cov_ = cov_tmp;

    std::unique_lock<std::mutex> result_lock(lidar_pose_list_mutex_);
    lidar_pose_list_.emplace_back(lidar_odometry_result);
  }
  SetFinish();
}

void ShenLanFrontEndComponent::OnPointCloud(
    const std::shared_ptr<drivers::PointCloud>& message) {
  CloudData tmp_lidar_frame;
  tools->LidarMsgTransfer(message, &tmp_lidar_frame);

  CloudData::CloudTypePtr temp_cloud_ptr(new CloudData::CloudType());
  Eigen::Matrix4f T_imu_lidar_ = lidar_extrinsic.matrix().cast<float>();
  pcl::transformPointCloud(*tmp_lidar_frame.cloud_ptr_, *temp_cloud_ptr,
                           T_imu_lidar_);
  CloudData lidar_frame;
  double timestamp = tmp_lidar_frame.time_;
  lidar_frame.time_ = timestamp;
  lidar_frame.cloud_ptr_ = temp_cloud_ptr;

  std::unique_lock<std::mutex> lock(lidar_frame_list_mutex_);
  lidar_frame_list_.emplace_back(lidar_frame);
}

void ShenLanFrontEndComponent::OnOdometry(
    const std::shared_ptr<localization::Gps>& message) {
  PoseData odometry;
  tools->OdometryMsgTransfer(message, odometry);

  drivers::gnss::InsStat odometry_status;
  FindNearestOdometryStatus(odometry.time_, &odometry_status);
  for (size_t i = 0; i < 36; i++)
    odometry.cov_.push_back(odometry_status.pos_type());

  static int drop = 0;
  if (drop++ != 10) return;
  drop = 0;
  std::unique_lock<std::mutex> lock(odom_list_mutex_);
  if (odom_list_.size() < odom_list_max_size_) {
    odom_list_.push_back(odometry);
  } else {
    odom_list_.pop_front();
    odom_list_.push_back(odometry);
  }
}

bool ShenLanFrontEndComponent::FindNearestOdometryStatus(
    const double odometry_timestamp, drivers::gnss::InsStat* status) {
  CHECK_NOTNULL(status);

  std::unique_lock<std::mutex> lock(odometry_status_list_mutex_);
  auto odometry_status_list = odometry_status_list_;
  lock.unlock();

  double timestamp_diff_sec = 0.05;
  auto nearest_itr = odometry_status_list.end();
  for (auto itr = odometry_status_list.begin();
       itr != odometry_status_list.end(); ++itr) {
    double diff = std::abs(itr->header().timestamp_sec() - odometry_timestamp);
    if (diff < timestamp_diff_sec) {
      timestamp_diff_sec = diff;
      nearest_itr = itr;
    }
  }

  if (nearest_itr == odometry_status_list.end()) {
    return false;
  }

  if (timestamp_diff_sec > odometry_status_time_diff_threshold_) {
    return false;
  }

  *status = *nearest_itr;
  return true;
}

void ShenLanFrontEndComponent::OnInsStat(
    const std::shared_ptr<drivers::gnss::InsStat>& message) {
  std::lock_guard<std::mutex> lock(odometry_status_list_mutex_);
  if (odometry_status_list_.size() < odometry_status_list_max_size_) {
    odometry_status_list_.push_back(*message);
  } else {
    odometry_status_list_.pop_front();
    odometry_status_list_.push_back(*message);
  }
}
/* --------------------------------------------------------------------- */

LocalizationMsgPublisher::LocalizationMsgPublisher(
    const std::shared_ptr<cyber::Node>& node)
    : node_(node) {}

bool LocalizationMsgPublisher::InitConfig() {
  lidar_local_topic_ = "/apollo/localization/shenlan_msf_lidar";
  gnss_local_topic_ = "/apollo/localization/shenlan_msf_gnss";
  localization_status_topic_ = "/apollo/localization/shenlan_msf_status";
  return true;
}

bool LocalizationMsgPublisher::InitIO() {
  localization_status_talker_ =
      node_->CreateWriter<LocalizationStatus>(localization_status_topic_);

  lidar_local_talker_ =
      node_->CreateWriter<LocalizationEstimate>(lidar_local_topic_);

  gnss_local_talker_ =
      node_->CreateWriter<LocalizationEstimate>(gnss_local_topic_);

  return true;
}

void LocalizationMsgPublisher::PublishLocalizationStatus(
    const LocalizationStatus& localization_status) {
  localization_status_talker_->Write(localization_status);
}

void LocalizationMsgPublisher::PublishLocalizationSLMSFLidar(
    const LocalizationEstimate& localization) {
  lidar_local_talker_->Write(localization);
}

void LocalizationMsgPublisher::PublishLocalizationSLMSFGnss(
    const LocalizationEstimate& localization) {
  gnss_local_talker_->Write(localization);
}

}  // namespace localization
}  // namespace apollo
