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

#include "modules/localization/shenlan_msf/visualization/pangolin_visualizer_component.h"

#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/math/quaternion.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/localization/msf/common/io/pcl_point_types.h"
#include "modules/localization/msf/common/io/velodyne_utility.h"

namespace apollo {
namespace localization {

PangolinVisualizerComponent::PangolinVisualizerComponent() {}

bool PangolinVisualizerComponent::Init() {
  if (!InitConfig()) {
    AERROR << "InitParams failed.";
    return false;
  }

  if (!InitIO()) {
    AERROR << "InitIO failed.";
    return false;
  }
  AINFO << "SUCCESS VIEWER INIT.";
  viewer = std::make_shared<PangolinViewer>();
  AINFO << "SUCCESS VIEWER START.";

  return true;
}

bool PangolinVisualizerComponent::InitConfig() {
  shenlan_config::Config visualizer_config;
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               &visualizer_config)) {
    return false;
  }

  lidar_extrinsic_file_ = visualizer_config.lidar_extrinsics_path();
  fusion_local_topic_ = visualizer_config.localization_topic();
  gnss_local_topic_ = visualizer_config.odometry_gnss_topic();
  lidar_local_topic_ = visualizer_config.lidar_pose_topic();
  // map_folder_ = FLAGS_map_dir + "/" + FLAGS_local_map_name;
  bool success =
      msf::velodyne::LoadExtrinsic(lidar_extrinsic_file_, &velodyne_extrinsic);
  if (!success) {
    AERROR << "Load lidar extrinsic failed.";
    return false;
  }
  AINFO << "Load lidar extrinsic succeed.";
  return true;
}

bool PangolinVisualizerComponent::InitIO() {
  // Lidar localization
  cyber::ReaderConfig reader_config;
  reader_config.channel_name = lidar_local_topic_;
  reader_config.pending_queue_size = 10;
  lidar_local_listener_ = this->node_->CreateReader<LocalizationEstimate>(
      reader_config,
      std::bind(&PangolinVisualizerComponent::OnLidarLocalization, this,
                std::placeholders::_1));

  // GNSS localization
  reader_config.channel_name = gnss_local_topic_;
  reader_config.pending_queue_size = 10;
  gnss_local_listener_ = this->node_->CreateReader<LocalizationEstimate>(
      reader_config, std::bind(&PangolinVisualizerComponent::OnGNSSLocalization,
                               this, std::placeholders::_1));

  // Fusion localization
  reader_config.channel_name = fusion_local_topic_;
  reader_config.pending_queue_size = 10;
  fusion_local_listener_ = this->node_->CreateReader<LocalizationEstimate>(
      fusion_local_topic_,
      std::bind(&PangolinVisualizerComponent::OnFusionLocalization, this,
                std::placeholders::_1));

  return true;
}

bool PangolinVisualizerComponent::Proc(
    const std::shared_ptr<drivers::PointCloud> &msg) {
  CloudData lidar_frame;
  double timestamp = cyber::Time(msg->measurement_time()).ToSecond();
  LidarMsgTransfer(msg, &lidar_frame);

  CloudData::CloudTypePtr temp_cloud(new CloudData::CloudType());
  Eigen::Matrix4d T_imu_lidar_ = velodyne_extrinsic.matrix();
  pcl::transformPointCloud(*lidar_frame.cloud_ptr_, *temp_cloud, T_imu_lidar_);

  lidar_frame.cloud_ptr_.reset(new CloudData::CloudType());
  pcl::VoxelGrid<CloudData::PointType> sor;
  sor.setInputCloud(temp_cloud);
  sor.setLeafSize(0.5f, 0.5f, 0.5f);
  sor.filter(*lidar_frame.cloud_ptr_);

  if (!lidar_pose_list_.empty() && !gnss_pose_list_.empty() && !fusion_pose_list_.empty()) {
    PoseData lidar_pose_result, gnss_pose_result, fusion_pose_result;
    bool syn_lidar = false, syn_gnss = false, syn_fusion = false;
    {
      std::unique_lock<std::mutex> gnss_lock(gnss_pose_list_mutex_);
      int count_flag = 10;
      while (count_flag-- > 0) {
        if (PoseData::SyncData(timestamp, gnss_pose_list_, gnss_pose_result)) {
          syn_gnss = true;
          break;
        }
        gnss_lock.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        gnss_lock.lock();
      }
      if (!syn_gnss) {
        gnss_pose_result.time_ = 0.0f;
        AINFO << std::setprecision(16) << "timestamp1: " << timestamp;
        AINFO << std::setprecision(16)
              << "timestamp2: " << gnss_pose_list_.back().time_;
        AERROR << "Fail to Sync Pose Data, Please Check!";
      }
    }

    {
      std::unique_lock<std::mutex> lidar_lock(lidar_pose_list_mutex_);
      int count_flag = 10;
      while (count_flag-- > 0) {
        if (PoseData::SyncData(timestamp, lidar_pose_list_, lidar_pose_result)) {
          syn_lidar = true;
          break;
        }
        lidar_lock.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        lidar_lock.lock();
      }
      if (!syn_lidar) {
        lidar_pose_result.time_ = 0.0f;
        AINFO << std::setprecision(16) << "timestamp1: " << timestamp;
        AINFO << std::setprecision(16)
              << "timestamp2: " << lidar_pose_list_.back().time_;
        AERROR << "Fail to Sync Pose Data, Please Check!";
      }
    }

    {
      std::unique_lock<std::mutex> fusion_lock(fusion_pose_list_mutex_);
      int count_flag = 10;
      while (count_flag-- > 0) {
        if (PoseData::SyncData(timestamp, fusion_pose_list_,
                               fusion_pose_result)) {
          syn_fusion = true;
          break;
        }
        fusion_lock.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        fusion_lock.lock();
      }
      if (!syn_fusion) {
        fusion_pose_result.time_ = 0.0f;
        AINFO << std::setprecision(16) << "timestamp1: " << timestamp;
        AINFO << std::setprecision(16)
              << "timestamp2: " << fusion_pose_list_.back().time_;
        AERROR << "Fail to Sync Pose Data, Please Check!";
      }
    }

    if (!syn_gnss && !syn_lidar && !syn_fusion) return false;

    Combination res;
    res.lidar_frame = lidar_frame;
    if (syn_lidar) res.lidar_pose = lidar_pose_result;
    if (syn_gnss) res.gnss_pose = gnss_pose_result;
    if (syn_fusion) res.fusion_pose = fusion_pose_result;
    std::unique_lock<std::mutex> lock(result_list_mutex_);
    if (result_list_.size() < result_list_max_size_) {
      result_list_.push_back(res);
    } else {
      result_list_.pop_front();
      result_list_.push_back(res);
    }
    viewer->SendInfo(result_list_);
  }
  return true;
}

void PangolinVisualizerComponent::LidarMsgTransfer(
    const std::shared_ptr<drivers::PointCloud> &msg, CloudData *lidar_frame) {
  CHECK_NOTNULL(lidar_frame);

  if (msg->height() > 1 && msg->width() > 1) {
    for (unsigned int i = 0; i < msg->height(); ++i) {
      for (unsigned int j = 0; j < msg->width(); ++j) {
        CloudData::PointType pt3d;
        pt3d.x = msg->point(i * msg->width() + j).x();
        pt3d.y = msg->point(i * msg->width() + j).y();
        pt3d.z = msg->point(i * msg->width() + j).z();
        if (std::isnan(pt3d.x)) {
          continue;
        }
        lidar_frame->cloud_ptr_->push_back(pt3d);
      }
    }
  } else {
    AINFO << "Receiving un-organized-point-cloud, width " << msg->width()
          << " height " << msg->height() << "size " << msg->point_size();
    for (int i = 0; i < msg->point_size(); ++i) {
      CloudData::PointType pt3d;
      pt3d.x = msg->point(i).x();
      pt3d.y = msg->point(i).y();
      pt3d.z = msg->point(i).z();
      if (std::isnan(pt3d.x)) {
        continue;
      }
      lidar_frame->cloud_ptr_->push_back(pt3d);
    }
  }
  lidar_frame->time_ = cyber::Time(msg->measurement_time()).ToSecond();
}

void PangolinVisualizerComponent::OnLidarLocalization(
    const std::shared_ptr<LocalizationEstimate> &msg) {
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

  std::unique_lock<std::mutex> lock(lidar_pose_list_mutex_);
  if (lidar_pose_list_.size() < lidar_pose_list_max_size_) {
    lidar_pose_list_.push_back(lidar_pose_frame);
  } else {
    lidar_pose_list_.pop_front();
    lidar_pose_list_.push_back(lidar_pose_frame);
  }
}

void PangolinVisualizerComponent::OnGNSSLocalization(
    const std::shared_ptr<LocalizationEstimate> &msg) {
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

  std::unique_lock<std::mutex> lock(gnss_pose_list_mutex_);
  if (gnss_pose_list_.size() < gnss_pose_list_max_size_) {
    gnss_pose_list_.push_back(gnss_pose_frame);
  } else {
    gnss_pose_list_.pop_front();
    gnss_pose_list_.push_back(gnss_pose_frame);
  }
}

void PangolinVisualizerComponent::OnFusionLocalization(
    const std::shared_ptr<LocalizationEstimate> &msg) {
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  Eigen::Quaternionf orientation;

  position[0] = msg->pose().position().x();
  position[1] = msg->pose().position().y();
  position[2] = msg->pose().position().z();
  orientation.x() = msg->pose().orientation().qx();
  orientation.y() = msg->pose().orientation().qy();
  orientation.z() = msg->pose().orientation().qz();
  orientation.w() = msg->pose().orientation().qw();

  PoseData fusion_pose_frame;
  fusion_pose_frame.time_ = msg->measurement_time();
  fusion_pose_frame.pose_.block<3, 3>(0, 0) = orientation.toRotationMatrix();
  fusion_pose_frame.pose_.block<3, 1>(0, 3) = position;

  std::unique_lock<std::mutex> lock(fusion_pose_list_mutex_);
  if (fusion_pose_list_.size() < fusion_pose_list_max_size_) {
    fusion_pose_list_.push_back(fusion_pose_frame);
  } else {
    fusion_pose_list_.pop_front();
    fusion_pose_list_.push_back(fusion_pose_frame);
  }
}

}  // namespace localization
}  // namespace apollo
