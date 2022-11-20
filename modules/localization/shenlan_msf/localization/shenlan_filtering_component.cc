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

#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/math/quaternion.h"
#include "modules/localization/shenlan_msf/localization/shenlan_filtering_component.h"
// #include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {

using apollo::cyber::Clock;
using common::util::TimeUtil;

FilteringComponent::FilteringComponent() {}

FilteringComponent::~FilteringComponent() {
  AINFO<<"Saving trajectory";
  SaveOdometry();
}

bool FilteringComponent::Init() {
  tf2_broadcaster_.reset(new apollo::transform::TransformBroadcaster(node_));
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

bool FilteringComponent::InitConfig() {
  shenlan_config::Config filter_config;
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                              &filter_config)) {
  return false;
  }
  AINFO << "ShenLan Filter config: " << filter_config.DebugString();

  lidar_topic_ = filter_config.lidar_trans_topic();
  odometry_topic_ = filter_config.odometry_gnss_topic();
  lidar_extrinsics_path_ = filter_config.lidar_extrinsics_path();
  synch_imu_topic_ = filter_config.synch_imu_topic();
  synch_posvel_topic_ = filter_config.synch_posvel_topic();
  localization_topic_ = filter_config.localization_topic();
  localization_lidar_topic_ = filter_config.lidar_pose_topic();

  lidar_extrinsic = Eigen::Affine3d::Identity();
  if (!tools.LoadLidarExtrinsic(lidar_extrinsics_path_, &lidar_extrinsic)) {
    AERROR << "Fail to Load Lidar Extrinsic!";
  }

  return true;
}

bool FilteringComponent::InitIO() {
  filtering_ptr_ = std::shared_ptr<Filtering>(new Filtering);

  // transform poindcloud
  lidar_listener_ = node_->CreateReader<CloudData>(
      lidar_topic_, std::bind(&FilteringComponent::LidarCallback, this,
                            std::placeholders::_1));

  pos_vel_listener_ = node_->CreateReader<PosVelData>(
      synch_posvel_topic_, std::bind(&FilteringComponent::PosVelCallback, this,
                            std::placeholders::_1));

  imu_synced_listener_ = node_->CreateReader<IMUData>(
      synch_imu_topic_, std::bind(&FilteringComponent::SynchImuCallback, this,
                            std::placeholders::_1));

  odometry_listener_ = node_->CreateReader<LocalizationEstimate>(
      odometry_topic_, std::bind(&FilteringComponent::OdometryCallback, this,
                            std::placeholders::_1));

  // publish
  localization_talker_ =
      node_->CreateWriter<LocalizationEstimate>(localization_topic_);

  localization_lidar_talker_ =
    node_->CreateWriter<LocalizationEstimate>(localization_lidar_topic_);

  filtering_timer_.reset(new cyber::Timer(
      10, [this]() { this->FilteringImplementation(); }, false));
  filtering_timer_->Start();

  return true;
}

void FilteringComponent::FilteringImplementation(){
  while (HasData()) {
    if (!HasInited()) {
      if (ValidLidarData()) {
        InitLocalization();
      }
    }
    else {
      if (HasLidarData() && ValidLidarData()) {
        if (HasIMUData()) {
          while (HasIMUData() && ValidIMUData() &&
                 current_imu_raw_data_.time_ < current_cloud_data_.time_) {
            UpdateLocalization();
          }

          if (current_imu_raw_data_.time_ >= current_cloud_data_.time_) {
            imu_raw_data_buff_.push_back(current_imu_raw_data_);
          }
        }

        CorrectLocalization();

      }

      if (HasIMUData() && ValidIMUData()) {
        UpdateLocalization();
      }
    }
  }

}

bool FilteringComponent::Proc(
    const std::shared_ptr<drivers::gnss::Imu>& message) {
    IMUData imu_msg;
    tools.IMUMsgTransfer(message, imu_msg);

    if (imu_raw_data_buff_.size() < imu_list_max_size_) {
      imu_raw_data_buff_.push_back(imu_msg);
    } else {
      imu_raw_data_buff_.pop_front();
      imu_raw_data_buff_.push_back(imu_msg);
    }
    return true;
}

bool FilteringComponent::HasInited() const {
  return filtering_ptr_->has_inited();
  return false;
}

bool FilteringComponent::HasData() {
  if (!HasInited()) {
    if (!HasLidarData()) {
      return false;
    }
  } else {
    if (!HasIMUData() && !HasLidarData()) {
      return false;
    }
  }
  return true;
}

void FilteringComponent::LidarCallback(const std::shared_ptr<CloudData>& cloud_msg){
  std::unique_lock<std::mutex> lock(cloud_list_mutex_);

  if (cloud_data_buff_.size() < cloud_list_max_size_) {
    cloud_data_buff_.push_back(*cloud_msg);
  } else {
    cloud_data_buff_.pop_front();
    cloud_data_buff_.push_back(*cloud_msg);
  }
}

void FilteringComponent::PosVelCallback(const std::shared_ptr<PosVelData>& posvel_msg){
  std::unique_lock<std::mutex> lock(posvel_list_mutex_);
  if (pos_vel_data_buff_.size() < posvel_list_max_size_) {
    pos_vel_data_buff_.push_back(*posvel_msg);
  } else {
    pos_vel_data_buff_.pop_front();
    pos_vel_data_buff_.push_back(*posvel_msg);
  }
}

void FilteringComponent::SynchImuCallback(const std::shared_ptr<IMUData>& imu_synced_msg){
  std::unique_lock<std::mutex> lock(imu_synced_list_mutex_);

  if (imu_synced_data_buff_.size() < imu_synced_list_max_size_) {
    imu_synced_data_buff_.push_back(*imu_synced_msg);
  } else {
    imu_synced_data_buff_.pop_front();
    imu_synced_data_buff_.push_back(*imu_synced_msg);
  }
}

void FilteringComponent::OdometryCallback(const std::shared_ptr<LocalizationEstimate>& odometry_msg){
  PoseData odometry;
  tools.PoseMsgTransfer(odometry_msg, odometry);

  std::lock_guard<std::mutex> lock(odometry_list_mutex_);
  odometry_data_buff_.push_back(odometry);
}

bool FilteringComponent::ValidLidarData() {
  current_cloud_data_ = cloud_data_buff_.front();
  current_imu_synced_data_ = imu_synced_data_buff_.front();
  current_pos_vel_data_ = pos_vel_data_buff_.front();

  const double diff_imu_time =
      current_cloud_data_.time_ - current_imu_synced_data_.time_;
  const double diff_pos_vel_time =
      current_cloud_data_.time_ - current_pos_vel_data_.time_;

  const double half_sampling_time = 0.05;  // LiDAR is 10hz
  if (diff_imu_time < -half_sampling_time ||
      diff_pos_vel_time < -half_sampling_time) {
    cloud_data_buff_.pop_front();
    return false;
  }

  if (diff_imu_time > half_sampling_time) {
    imu_synced_data_buff_.pop_front();
    return false;
  }

  if (diff_pos_vel_time > half_sampling_time) {
    pos_vel_data_buff_.pop_front();
    return false;
  }

  cloud_data_buff_.pop_front();
  imu_synced_data_buff_.pop_front();
  pos_vel_data_buff_.pop_front();

  return true;
}

bool FilteringComponent::ValidIMUData() {
  current_imu_raw_data_ = imu_raw_data_buff_.front();

  imu_raw_data_buff_.pop_front();

  return true;
}

bool FilteringComponent::InitLocalization() const {
  AINFO << "InitLocalization";
  // ego vehicle velocity in body frame:
  Eigen::Vector3f init_vel = current_pos_vel_data_.vel_;
  Eigen::Matrix3f R = current_imu_synced_data_.GetOrientationMatrix();
  Eigen::Vector3f t = current_pos_vel_data_.pos_;
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;
  // using GNSS data for initialization
  // if (filtering_ptr_->Init(T, init_vel, current_imu_synced_data_)) {
  //   LOG(INFO) << "GNSS Localization Init Succeeded." << std::endl;
  // }

  if (filtering_ptr_->Init(
          current_cloud_data_, T, init_vel, current_imu_synced_data_)) {
    AINFO << "Relocalization Init Succeeded." << std::endl;
  }


  return true;
}

bool FilteringComponent::UpdateLocalization() {
  if (filtering_ptr_->Update(current_imu_raw_data_)) {
    PublishFusionOdom();
    return true;
  }
  return false;
}

bool FilteringComponent::CorrectLocalization() {
  const bool is_fusion_succeeded =
      filtering_ptr_->Correct(current_imu_synced_data_,
                              current_cloud_data_,
                              current_pos_vel_data_,
                              laser_pose_);


   PublishLidarOdom();

  if (is_fusion_succeeded) {
     PublishFusionOdom();
    // add to odometry output for evo evaluation:
     UpdateOdometry(current_cloud_data_.time_);

    return true;
  }

  return false;
}

bool FilteringComponent::UpdateOdometry(const double time) {
  trajectory_.time.push_back(time);

  trajectory_.fused.push_back(fused_pose_);
  trajectory_.lidar.push_back(laser_pose_);

  ++trajectory_.N;

  return true;
}

bool FilteringComponent::PublishFusionOdom()  {
  // get odometry from Kalman filter:
  filtering_ptr_->GetOdometry(fused_pose_, fused_vel_);
  LocalizationEstimate* localization = new LocalizationEstimate ;
  ComposeLocalizationMsg(fused_pose_,localization,current_imu_raw_data_.time_);
  localization_talker_->Write(*localization);
  return true;
}

bool FilteringComponent::PublishLidarOdom()  {
LocalizationEstimate* localization = new LocalizationEstimate ;
ComposeLocalizationMsg(laser_pose_,localization,current_cloud_data_.time_);
localization_lidar_talker_->Write(*localization);

return true;
}

void FilteringComponent::ComposeLocalizationMsg(
    const Eigen::Matrix4f &gps_msg, LocalizationEstimate *localization, const double time) {
  localization->Clear();
  FillLocalizationMsgHeader(localization);
  localization->set_measurement_time(time);
  // combine gps and imu
  auto mutable_pose = localization->mutable_pose();
  mutable_pose->mutable_position()->set_x(gps_msg(0,3));
  mutable_pose->mutable_position()->set_y(gps_msg(1,3));
  mutable_pose->mutable_position()->set_z(gps_msg(2,3));

  Eigen::Quaternionf oriention(gps_msg.block<3,3>(0,0));

  mutable_pose->mutable_orientation()->set_qw(oriention.w());
  mutable_pose->mutable_orientation()->set_qx(oriention.x());
  mutable_pose->mutable_orientation()->set_qy(oriention.y());
  mutable_pose->mutable_orientation()->set_qz(oriention.z());
}

void FilteringComponent::FillLocalizationMsgHeader( LocalizationEstimate *localization) {
  auto *header = localization->mutable_header();
  double timestamp = apollo::cyber::Clock::NowInSeconds();
  header->set_module_name(module_name_);
  header->set_timestamp_sec(timestamp);
}

bool FilteringComponent::SaveOdometry() {
  if (0 == trajectory_.N) {
    return false;
  }

  // init output files:
  std::ofstream fused_odom_ofs;
  std::ofstream laser_odom_ofs;
  std::ofstream ref_odom_ofs;
  if (!FileManager::CreateFile(
          WORK_SPACE_PATH + "/slam_data/trajectory/fused.txt",
          fused_odom_ofs) ||
      !FileManager::CreateFile(
          WORK_SPACE_PATH + "/slam_data/trajectory/laser.txt",
          laser_odom_ofs) ||
      !FileManager::CreateFile(
          WORK_SPACE_PATH + "/slam_data/trajectory/ground_truth.txt",
          ref_odom_ofs)) {
    return false;
  }

  // write outputs:
  const double half_sampling_time = 0.05;  // LiDAR is 10hz
  for (size_t i = 0; i < trajectory_.N; ++i) {
    // sync ref pose with gnss measurement:
    while (!odometry_data_buff_.empty() &&
           (odometry_data_buff_.front().time_ - trajectory_.time.at(i) <=
            -half_sampling_time)) {
      odometry_data_buff_.pop_front();
    }

    if (odometry_data_buff_.empty()) {
      break;
    }

    current_gnss_data_ = odometry_data_buff_.front();

    const Eigen::Vector3f& position_ref =
        current_gnss_data_.pose_.block<3, 1>(0, 3);
    const Eigen::Vector3f& position_lidar =
        trajectory_.lidar.at(i).block<3, 1>(0, 3);

    if ((position_ref - position_lidar).norm() > 3.0) {
      LOG(INFO) << "(position_ref - position_lidar).norm() > 3.0";
      continue;
    }

    SavePose(trajectory_.fused.at(i), fused_odom_ofs);
    SavePose(trajectory_.lidar.at(i), laser_odom_ofs);
    SavePose(current_gnss_data_.pose_, ref_odom_ofs);
  }

  return true;
}

bool FilteringComponent::SavePose(const Eigen::Matrix4f& pose,
                                    std::ofstream& ofs) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      ofs << pose(i, j);

      if (i == 2 && j == 3) {
        ofs << std::endl;
      } else {
        ofs << " ";
      }
    }
  }

  return true;
}
}  // namespace localization
}  // namespace apollo
