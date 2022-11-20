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

#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>

#include "lidar_localization/tools/file_manager.h"

#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/math/quaternion.h"
#include "modules/localization/shenlan_msf/localization/shenlan_pretreat_component.h"

namespace apollo {
namespace localization {

using apollo::cyber::Clock;
using common::util::TimeUtil;
PretreatComponent::PretreatComponent() {}

bool PretreatComponent::Init(){
    //tf2_broadcaster_.reset(new apollo::transform::TransformBroadcaster(node_));
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

bool PretreatComponent::InitConfig(){
  shenlan_config::Config preate_config;
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                              &preate_config)) {
  return false;
  }
  AINFO << "ShenLan Pretreat config: " << preate_config.DebugString();

  lidar_topic_ = preate_config.lidar_topic();
  odometry_topic_ = preate_config.gps_topic();
  chassis_topic_ = preate_config.chassis_topic();
  imu_topic_ = preate_config.imu_topic();
  odometry_status_topic_ = preate_config.gps_status_topic();
  odometry_gnss_topic_ =  preate_config.odometry_gnss_topic();

  cloud_trans_topic_ = preate_config.lidar_trans_topic();
  synch_imu_topic_ = preate_config.synch_imu_topic();
  synch_posvel_topic_ = preate_config.synch_posvel_topic();

  lidar_extrinsics_path_ = preate_config.lidar_extrinsics_path();

  lidar_extrinsic = Eigen::Affine3d::Identity();
  if (!LoadLidarExtrinsic(lidar_extrinsics_path_, &lidar_extrinsic)){
    AERROR << "Fail to Load Lidar Extrinsic!";
    return false;
  }

  T_imu_lidar_ = lidar_extrinsic.matrix().cast<float>();

  is_mapping_ = preate_config.is_mapping();

  AINFO << std::endl
            << "-----------------Init Data Pretreat-------------------"
            << std::endl
            << "is_mapping: " << is_mapping_ << std::endl
            << "T_imu_lidar: " << std::endl
            << T_imu_lidar_ << std::endl;
  return true;
}

bool PretreatComponent::InitIO(){
// transform poindcloud
  cloud_sub_ptr_ = node_->CreateReader<drivers::PointCloud>(
      lidar_topic_, std::bind(&PretreatComponent::LidarCallback, this,
                            std::placeholders::_1));
  ACHECK(cloud_sub_ptr_);

  gnss_sub_ptr_ = node_->CreateReader<localization::Gps>(
      odometry_topic_, std::bind(&PretreatComponent::OdometryCallback, this,
                            std::placeholders::_1));
  ACHECK(gnss_sub_ptr_);

  velocity_sub_ptr_ = node_->CreateReader<canbus::Chassis>(
      chassis_topic_,std::bind(&PretreatComponent::ChassisCallback, this,
                            std::placeholders::_1));
  ACHECK(velocity_sub_ptr_);

  ins_status_ptr_ = node_->CreateReader<drivers::gnss::InsStat>(
  odometry_status_topic_, std::bind(&PretreatComponent::OnInsStat, this,
                        std::placeholders::_1));
  ACHECK(ins_status_ptr_);

// publish
  cloud_pub_ptr_ =
      node_->CreateWriter<CloudData>(cloud_trans_topic_);
  ACHECK(cloud_pub_ptr_);

  imu_pub_ptr_ =
      node_->CreateWriter<IMUData>(synch_imu_topic_);
  ACHECK(imu_pub_ptr_);

  pos_vel_pub_ptr_ =
      node_->CreateWriter<PosVelData>(synch_posvel_topic_);
  ACHECK(pos_vel_pub_ptr_);

  odometry_gnss_ptr_ =
        node_->CreateWriter<LocalizationEstimate>(odometry_gnss_topic_);
  ACHECK(odometry_gnss_ptr_);

  pretreat_timer_.reset(new cyber::Timer(
      10, [this]() { this->PretreatImplementation(); }, false));
  pretreat_timer_->Start();

  return true;
}

bool PretreatComponent::Proc(const std::shared_ptr<drivers::gnss::Imu>& message){
  IMUData imu_msg;
  IMUMsgTransfer(message, imu_msg);

  std::unique_lock<std::mutex> lock(imu_list_mutex_);

  if (unsynced_imu_.size() < imu_list_max_size_) {
    unsynced_imu_.push_back(imu_msg);
  } else {
    unsynced_imu_.pop_front();
    unsynced_imu_.push_back(imu_msg);
  }
  return true;
}

void PretreatComponent::LidarCallback(const std::shared_ptr<drivers::PointCloud>& cloud_msg){
  CloudData lidar_frame;
  LidarMsgTransfer(cloud_msg, &lidar_frame);

  std::unique_lock<std::mutex> lock(cloud_list_mutex_);

  if (cloud_data_buff_.size() < cloud_list_max_size_) {
    cloud_data_buff_.push_back(lidar_frame);
  } else {
    cloud_data_buff_.pop_front();
    cloud_data_buff_.push_back(lidar_frame);
  }
}

void PretreatComponent::OdometryCallback(const std::shared_ptr<localization::Gps>& gnss_msg){
  PoseData odometry;
  OdometryMsgTransfer(gnss_msg, odometry);

  std::lock_guard<std::mutex> lock(odometry_list_mutex_);
  if (unsynced_gnss_.size() < odom_list_max_size_) {
    unsynced_gnss_.push_back(odometry);
  } else {
    unsynced_gnss_.pop_front();
    unsynced_gnss_.push_back(odometry);
  }
}

void PretreatComponent::ChassisCallback(const std::shared_ptr<canbus::Chassis>& chassis_msg){
  VelocityData chassis;
  ChassisMsgTransfer(chassis_msg,chassis);
  std::unique_lock<std::mutex> lock(chassis_list_mutex_);

  if (unsynced_velocity_.size() < chassis_list_max_size_) {
    unsynced_velocity_.push_back(chassis);
  } else {
    unsynced_velocity_.pop_front();
    unsynced_velocity_.push_back(chassis);
  }
}

void PretreatComponent::OnInsStat(const std::shared_ptr<drivers::gnss::InsStat> &message) {
  std::lock_guard<std::mutex> lock(odometry_status_list_mutex_);
  if (odometry_status_list_.size() < odometry_status_list_max_size_) {
    odometry_status_list_.push_back(*message);
  } else {
    odometry_status_list_.pop_front();
    odometry_status_list_.push_back(*message);
  }
}

void PretreatComponent::IMUMsgTransfer(
    const std::shared_ptr<drivers::gnss::Imu> &imu_msg, IMUData &imu) {
  double measurement_time = TimeUtil::Gps2Unix(imu_msg->measurement_time());
  imu.time_ = measurement_time;

  // 坐标系是右前上（RFU）还是前左上（FLU）
  if (1) {
    imu.linear_acceleration_.x = imu_msg->linear_acceleration().x();
    imu.linear_acceleration_.y = imu_msg->linear_acceleration().y();
    imu.linear_acceleration_.z = imu_msg->linear_acceleration().z();
    imu.angular_velocity_.x = imu_msg->angular_velocity().x();
    imu.angular_velocity_.y = imu_msg->angular_velocity().y();
    imu.angular_velocity_.z = imu_msg->angular_velocity().z();
  } else {
    imu.linear_acceleration_.x = -imu_msg->linear_acceleration().y();
    imu.linear_acceleration_.y = imu_msg->linear_acceleration().x();
    imu.linear_acceleration_.z = imu_msg->linear_acceleration().z();
    imu.angular_velocity_.x = -imu_msg->angular_velocity().y();
    imu.angular_velocity_.y = imu_msg->angular_velocity().x();
    imu.angular_velocity_.z = imu_msg->angular_velocity().z();
  }
}

void PretreatComponent::LidarMsgTransfer(
    const std::shared_ptr<drivers::PointCloud> &msg, CloudData *lidar_frame) {
  CHECK_NOTNULL(lidar_frame);

  if (msg->height() > 1 && msg->width() > 1) {
    for (unsigned int i = 0; i < msg->height(); ++i) {
      for (unsigned int j = 0; j < msg->width(); ++j) {
        CloudData::PointType pt3d;
        pt3d.x = msg->point(i * msg->width() + j).x();
        pt3d.y = msg->point(i * msg->width() + j).y();
        pt3d.z = msg->point(i * msg->width() + j).z();
        if (std::isnan(pt3d.x) || pt3d.z > max_height_) {
          continue;
        }
        //   unsigned char intensity = static_cast<unsigned char>(
        //       msg->point(i * msg->width() + j).intensity());
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
      if (std::isnan(pt3d.x) || pt3d.z > max_height_) {
        continue;
      }
      //   unsigned char intensity = static_cast<unsigned
      //   char>(msg->point(i).intensity());
      lidar_frame->cloud_ptr_->push_back(pt3d);
    }
  }
  lidar_frame->time_ = cyber::Time(msg->measurement_time()).ToSecond();
}

void PretreatComponent::ChassisMsgTransfer(
    const std::shared_ptr<canbus::Chassis> &chassis_msg,
    VelocityData &chassis_frame) {
  chassis_frame.time_ = chassis_msg->header().timestamp_sec();

  chassis_frame.linear_velocity_.x = chassis_msg->speed_mps();
  chassis_frame.linear_velocity_.y = 0.0f;
  chassis_frame.linear_velocity_.z = 0.0f;

  double wheel_spd_rl = chassis_msg->wheel_speed().wheel_spd_rl();
  double wheel_spd_rr = chassis_msg->wheel_speed().wheel_spd_rr();
  double angular_speed = 0.0f;

  if (std::fabs(wheel_spd_rr) > kEpsilon) {
    double kappa = wheel_spd_rl / wheel_spd_rr;
    angular_speed = -2.0 * (kappa - 1) / (kappa + 1) *
                    chassis_frame.linear_velocity_.x / wheel_base;
  }

  chassis_frame.angular_velocity_.x = 0.0f;
  chassis_frame.angular_velocity_.y = 0.0f;
  chassis_frame.angular_velocity_.z = wheel_speed_coff * angular_speed;
}

void PretreatComponent::OdometryMsgTransfer(
    const std::shared_ptr<localization::Gps> &odom_msg, PoseData &odom_frame) {
  odom_frame.time_ = odom_msg->header().timestamp_sec();
  // set the position:
  static double origin = 0.0;
  if(origin - 0.0 <= 1e-5){
    origin = odom_msg->localization().position().y();
  }

  odom_frame.pose_(0, 3) = odom_msg->localization().position().x();
  odom_frame.pose_(1, 3) = odom_msg->localization().position().y()-origin;
  odom_frame.pose_(2, 3) = odom_msg->localization().position().z();

  Eigen::Quaternionf q;
  q.x() = odom_msg->localization().orientation().qx();
  q.y() = odom_msg->localization().orientation().qy();
  q.z() = odom_msg->localization().orientation().qz();
  q.w() = odom_msg->localization().orientation().qw();
  odom_frame.pose_.block<3, 3>(0, 0) = q.matrix();

  // set the linear velocity:
  odom_frame.vel_.v.x() = odom_msg->localization().linear_velocity().x();
  odom_frame.vel_.v.y() = odom_msg->localization().linear_velocity().y();
  odom_frame.vel_.v.z() = odom_msg->localization().linear_velocity().z();

  drivers::gnss::InsStat odometry_status;
  FindNearestOdometryStatus(odom_frame.time_, &odometry_status);
  // if (odometry_status.ins_status() == 3 && odometry_status.pos_type() == 56){
  //   for (size_t i = 0; i < 36; i++) odom_frame.cov_.push_back(1e-5);
  // }else{
  //   for (size_t i = 0; i < 36; i++) odom_frame.cov_.push_back(1e+5);
  // }
  for (size_t i = 0; i < 36; i++)
    odom_frame.cov_.push_back(odometry_status.pos_type());

  // // set the angular velocity:
  // odom_frame.vel_.w.x() = odom_msg->localization().angular_velocity().x();
  // odom_frame.vel_.w.y() = odom_msg->localization().angular_velocity().y();
  // odom_frame.vel_.w.z() = odom_msg->localization().angular_velocity().z();
}

bool PretreatComponent::FindNearestOdometryStatus(
    const double odometry_timestamp, drivers::gnss::InsStat *status) {
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

bool PretreatComponent::LoadLidarExtrinsic(const std::string &file_path,
                                           Eigen::Affine3d *lidar_extrinsic) {
  CHECK_NOTNULL(lidar_extrinsic);

  YAML::Node config = YAML::LoadFile(file_path);
  if (config["transform"]) {
    if (config["transform"]["translation"]) {
      lidar_extrinsic->translation()(0) =
          config["transform"]["translation"]["x"].as<double>();
      lidar_extrinsic->translation()(1) =
          config["transform"]["translation"]["y"].as<double>();
      lidar_extrinsic->translation()(2) =
          config["transform"]["translation"]["z"].as<double>();
      if (config["transform"]["rotation"]) {
        double qx = config["transform"]["rotation"]["x"].as<double>();
        double qy = config["transform"]["rotation"]["y"].as<double>();
        double qz = config["transform"]["rotation"]["z"].as<double>();
        double qw = config["transform"]["rotation"]["w"].as<double>();
        lidar_extrinsic->linear() =
            Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
        return true;
      }
    }
  }
  return false;
}
bool PretreatComponent::PretreatImplementation() {
  if (!ReadData()) {
    return false;
  }

  if (!InitGNSS()) {
    return false;
  }

  while (HasData()) {
    if (!ValidData()) {
      continue;
    }

    TransformData();
    PublishData();
  }

  return true;
}

bool PretreatComponent::ReadData() {
  if (cloud_data_buff_.size() == 0) {
    return false;
  }
  // use timestamp of lidar measurement as reference:
  const double cloud_time = cloud_data_buff_.front().time_;
  // sync IMU, velocity and GNSS with lidar measurement:
  // find the two closest measurement around lidar measurement time
  // then use linear interpolation to generate synced measurement:
  const bool valid_imu =
      IMUData::SyncData(cloud_time, unsynced_imu_, imu_data_buff_);
  const bool valid_velocity = VelocityData::SyncData(
      cloud_time, unsynced_velocity_, velocity_data_buff_);
  const bool valid_gnss =
      PoseData::SyncData(cloud_time, unsynced_gnss_, gnss_data_buff_);

  if (!valid_imu || !valid_gnss || !valid_velocity) {
    if (unsynced_gnss_.front().time_ > cloud_time) {
      cloud_data_buff_.pop_front();
    } else if (unsynced_imu_.front().time_ > cloud_time) {
      cloud_data_buff_.pop_front();
    } else if (unsynced_velocity_.front().time_ > cloud_time) {
      cloud_data_buff_.pop_front();
    }

    return false;
  }

  return true;
}

bool PretreatComponent::InitGNSS() {
  static bool gnss_inited = false;
  if (gnss_data_buff_.empty()) {
    return false;
  }

  if (!gnss_inited) {
    // system is mapping mode
    if (is_mapping_) {
      gnss_origin_ = gnss_data_buff_.front().pose_.block<3, 1>(0, 3);
      gnss_inited = true;
    }
    // system is localization mode
    else {
      const std::string gnss_origin_path =
          WORK_SPACE_PATH + "/slam_data/map/gnss_origin.txt";
      // const std::string gnss_origin_path =
      //     WORK_SPACE_PATH + "/slam_data/map/gnss_origin.txt";
      if (!FileManager::IsValidDirectory(gnss_origin_path)) {
        LOG(ERROR)
            << "System is localization mode, can not find gnss origin point."
            << std::endl;
        return false;
      }

      std::ifstream ifs(gnss_origin_path);

      double x = 0.0;
      double y = 0.0;
      double z = 0.0;

      ifs >> x;
      ifs >> y;
      ifs >> z;

      gnss_origin_.x() = x;
      gnss_origin_.y() = y;
      gnss_origin_.z() = z;

      gnss_inited = true;
    }
  }

  return gnss_inited;
}

bool PretreatComponent::SaveGnssOrigin() {
  const std::string gnss_origin_path =
      WORK_SPACE_PATH + "/slam_data/map/gnss_origin.txt";

  std::ofstream ofs(gnss_origin_path);

  ofs << std::setprecision(16) << gnss_origin_.x() << " " << gnss_origin_.y()
      << " " << gnss_origin_.z();
  ofs.close();

  LOG(INFO) << "save gnss_origin in: " << gnss_origin_path << std::endl
            << " x: " << gnss_origin_.x() << " y: " << gnss_origin_.y()
            << " z: " << gnss_origin_.z() << std::endl;
  return true;
}

bool PretreatComponent::HasData() {
  if (cloud_data_buff_.size() == 0) {
    return false;
  }
  if (imu_data_buff_.size() == 0) {
    return false;
  }
  if (velocity_data_buff_.size() == 0) {
    return false;
  }
  if (gnss_data_buff_.size() == 0) {
    return false;
  }
  return true;
}

bool PretreatComponent::ValidData() {
  current_cloud_data_ = cloud_data_buff_.front();
  current_imu_data_ = imu_data_buff_.front();
  current_velocity_data_ = velocity_data_buff_.front();

  current_gnss_data_ = gnss_data_buff_.front();

  const double diff_imu_time =
      current_cloud_data_.time_ - current_imu_data_.time_;
  const double diff_velocity_time =
      current_cloud_data_.time_ - current_velocity_data_.time_;
  const double diff_gnss_time =
      current_cloud_data_.time_ - current_gnss_data_.time_;
  //
  // this check assumes the frequency of lidar is 10Hz:
  //
  const double half_sampling_time = 0.05;
  if (diff_imu_time < -half_sampling_time ||
      diff_gnss_time < -half_sampling_time) {
    cloud_data_buff_.pop_front();
    return false;
  }

  if (diff_imu_time > half_sampling_time) {
    imu_data_buff_.pop_front();
    return false;
  }

  if (diff_velocity_time > half_sampling_time) {
    velocity_data_buff_.pop_front();
    return false;
  }

  if (diff_gnss_time > half_sampling_time) {
    gnss_data_buff_.pop_front();
    return false;
  }

  cloud_data_buff_.pop_front();
  imu_data_buff_.pop_front();
  velocity_data_buff_.pop_front();
  gnss_data_buff_.pop_front();

  return true;
}

bool PretreatComponent::TransformData() {
  // static Eigen::Vector3f gnss_origin = Eigen::Vector3f::Zero();
  // static bool gnss_init = false;
  const double timestamp_synced = current_cloud_data_.time_;
  // a. init gnss origin
  // to make sure the gnss origin is [0,0,0]
  // if (!gnss_init) {
  //   gnss_origin = current_gnss_data_.pose_.block<3, 1>(0, 3);
  //   gnss_init = true;
  // }
  current_gnss_data_.pose_.block<3, 1>(0, 3) =
      current_gnss_data_.pose_.block<3, 1>(0, 3) - gnss_origin_;

  // b. motion compensation for lidar measurements:
  // convert the laser points to imu frame
  CloudData::CloudTypePtr temp_cloud(new CloudData::CloudType());
  pcl::transformPointCloud(
      *current_cloud_data_.cloud_ptr_, *temp_cloud, T_imu_lidar_);
  // *current_cloud_data_.cloud_ptr_ = *adjusted_cloud;
  *current_cloud_data_.cloud_ptr_ = *temp_cloud;

  // set synced pos vel
  pos_vel_.pos_.x() = current_gnss_data_.pose_(0, 3);
  pos_vel_.pos_.y() = current_gnss_data_.pose_(1, 3);
  pos_vel_.pos_.z() = current_gnss_data_.pose_(2, 3);
  // linear velocity in body frame
  pos_vel_.vel_.x() = current_velocity_data_.linear_velocity_.x;
  pos_vel_.vel_.y() = current_velocity_data_.linear_velocity_.y;
  pos_vel_.vel_.z() = current_velocity_data_.linear_velocity_.z;
  pos_vel_.time_ = timestamp_synced;
  Eigen::Quaterniond q = Eigen::Quaterniond(
      current_gnss_data_.pose_.block<3, 3>(0, 0).cast<double>());
  current_imu_data_.orientation_.w = q.w();
  current_imu_data_.orientation_.x = q.x();
  current_imu_data_.orientation_.y = q.y();
  current_imu_data_.orientation_.z = q.z();
  current_imu_data_.time_ = timestamp_synced;
  return true;
}

bool PretreatComponent::PublishData() {
  // take lidar measurement time as synced timestamp:
  cloud_pub_ptr_->Write(current_cloud_data_);
  imu_pub_ptr_->Write(current_imu_data_);
  pos_vel_pub_ptr_->Write(pos_vel_);
  LocalizationEstimate* localization = new LocalizationEstimate ;
  ComposeLocalizationMsg(current_gnss_data_,localization);
  odometry_gnss_ptr_->Write(*localization);
  return true;
}

void PretreatComponent::ComposeLocalizationMsg(
    const PoseData &pose_msg, LocalizationEstimate *localization) {
  localization->Clear();
  FillLocalizationMsgHeader(localization);
  localization->set_measurement_time(pose_msg.time_);
  // combine gps
  auto mutable_pose = localization->mutable_pose();
  mutable_pose->mutable_position()->set_x(pose_msg.pose_(0,3));
  mutable_pose->mutable_position()->set_y(pose_msg.pose_(1,3));
  mutable_pose->mutable_position()->set_z(pose_msg.pose_(2,3));

  mutable_pose->mutable_orientation()->set_qw(pose_msg.GetQuaternion().w());
  mutable_pose->mutable_orientation()->set_qx(pose_msg.GetQuaternion().x());
  mutable_pose->mutable_orientation()->set_qy(pose_msg.GetQuaternion().y());
  mutable_pose->mutable_orientation()->set_qz(pose_msg.GetQuaternion().z());
}

void PretreatComponent::FillLocalizationMsgHeader( LocalizationEstimate *localization) {
  auto *header = localization->mutable_header();
  double timestamp = apollo::cyber::Clock::NowInSeconds();
  header->set_module_name(module_name_);
  header->set_timestamp_sec(timestamp);
}


}  // namespace localization
}  // namespace apollo
