/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/localization/ndt/ndt_localization.h"

#include "Eigen/Geometry"
#include "yaml-cpp/yaml.h"

#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/string_util.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {
namespace ndt {

void NDTLocalization::Init() {
  tf_buffer_ = apollo::transform::Buffer::Instance();
  tf_buffer_->Init();

  resolution_id_ = 0;
  zone_id_ = FLAGS_local_utm_zone_id;
  online_resolution_ = FLAGS_online_resolution;
  ndt_debug_log_flag_ = FLAGS_ndt_debug_log_flag;
  tf_source_frame_id_ = FLAGS_broadcast_tf_frame_id;
  tf_target_frame_id_ = FLAGS_broadcast_tf_child_frame_id;
  std::string lidar_height_file = FLAGS_lidar_height_file;
  std::string lidar_extrinsics_file = FLAGS_lidar_extrinsics_file;
  bad_score_count_threshold_ = FLAGS_ndt_bad_score_count_threshold;
  warnning_ndt_score_ = FLAGS_ndt_warnning_ndt_score;
  error_ndt_score_ = FLAGS_ndt_error_ndt_score;


  std::string map_path_ =
      FLAGS_map_dir + "/" + FLAGS_ndt_map_dir + "/" + FLAGS_local_map_name;
  AINFO << "map folder: " << map_path_;
  velodyne_extrinsic_ = Eigen::Affine3d::Identity();
  bool success =
      LoadLidarExtrinsic(lidar_extrinsics_file, &velodyne_extrinsic_);
  if (!success) {
    AERROR << "LocalizationLidar: Fail to access the lidar"
              " extrinsic file: "
           << lidar_extrinsics_file;
    return;
  }
  Eigen::Quaterniond ext_quat(velodyne_extrinsic_.linear());
  AINFO << "lidar extrinsics: " << velodyne_extrinsic_.translation().x() << ", "
        << velodyne_extrinsic_.translation().y() << ", "
        << velodyne_extrinsic_.translation().z() << ", " << ext_quat.x() << ", "
        << ext_quat.y() << ", " << ext_quat.z() << ", " << ext_quat.w();

  success = LoadLidarHeight(lidar_height_file, &lidar_height_);
  if (!success) {
    AWARN << "LocalizationLidar: Fail to load the lidar"
             " height file: "
          << lidar_height_file << " Will use default value!";
    lidar_height_.height = FLAGS_lidar_height_default;
  }

  // try load zone id from local_map folder
  if (FLAGS_if_utm_zone_id_from_folder) {
    bool success = LoadZoneIdFromFolder(map_path_, &zone_id_);
    if (!success) {
      AWARN << "Can't load utm zone id from map folder, use default value.";
    }
  }
  AINFO << "utm zone id: " << zone_id_;

  lidar_locator_.SetMapFolderPath(map_path_);
  lidar_locator_.SetVelodyneExtrinsic(velodyne_extrinsic_);
  lidar_locator_.SetLidarHeight(lidar_height_.height);
  lidar_locator_.SetOnlineCloudResolution(
      static_cast<float>(online_resolution_));

  odometry_buffer_.clear();
  odometry_buffer_size_ = 0;

  is_service_started_ = false;
}
// receive odometry message
void NDTLocalization::OdometryCallback(
    const std::shared_ptr<localization::Gps>& odometry_msg) {
  double odometry_time = odometry_msg->header().timestamp_sec();
  static double pre_odometry_time = odometry_time;
  double time_delay = odometry_time - pre_odometry_time;
  if (time_delay > 0.1) {
    AINFO << "Odometry message loss more than 10ms, the pre time and cur time: "
          << pre_odometry_time << ", " << odometry_time;
  } else if (time_delay < 0.0) {
    AINFO << "Odometry message's time is earlier than last one, "
          << "the pre time and cur time: " << pre_odometry_time << ", "
          << odometry_time;
  }
  pre_odometry_time = odometry_time;
  Eigen::Affine3d odometry_pose = Eigen::Affine3d::Identity();
  if (odometry_msg->has_localization()) {
    odometry_pose.translation()[0] =
        odometry_msg->localization().position().x();
    odometry_pose.translation()[1] =
        odometry_msg->localization().position().y();
    odometry_pose.translation()[2] =
        odometry_msg->localization().position().z();
    Eigen::Quaterniond tmp_quat(
        odometry_msg->localization().orientation().qw(),
        odometry_msg->localization().orientation().qx(),
        odometry_msg->localization().orientation().qy(),
        odometry_msg->localization().orientation().qz());
    odometry_pose.linear() = tmp_quat.toRotationMatrix();
  }
  if (ZeroOdometry(odometry_pose)) {
    AINFO << "Detect Zero Odometry";
    return;
  }

  TimeStampPose timestamp_pose;
  timestamp_pose.timestamp = odometry_time;
  timestamp_pose.pose = odometry_pose;
  {
    std::lock_guard<std::mutex> lock(odometry_buffer_mutex_);
    if (odometry_buffer_size_ < max_odometry_buffer_size_) {
      odometry_buffer_.push_back(timestamp_pose);
      odometry_buffer_size_++;
    } else {
      odometry_buffer_.pop_front();
      odometry_buffer_.push_back(timestamp_pose);
    }
  }

  if (1) {
    AINFO << "NDTLocalization Debug Log: odometry msg: "
          << std::setprecision(15) << "time: " << odometry_time << ", "
          << "x: " << odometry_msg->localization().position().x() << ", "
          << "y: " << odometry_msg->localization().position().y() << ", "
          << "z: " << odometry_msg->localization().position().z() << ", "
          << "qx: " << odometry_msg->localization().orientation().qx() << ", "
          << "qy: " << odometry_msg->localization().orientation().qy() << ", "
          << "qz: " << odometry_msg->localization().orientation().qz() << ", "
          << "qw: " << odometry_msg->localization().orientation().qw();
  }

  Eigen::Affine3d localization_pose = Eigen::Affine3d::Identity();
  if (lidar_locator_.IsInitialized()) {
    localization_pose =
        pose_buffer_.UpdateOdometryPose(odometry_time, odometry_pose);
  }

  CorrectedImu imu_msg;
  FindMatchingIMU(odometry_time, &imu_msg);
  ComposeLocalizationEstimate(localization_pose, imu_msg, odometry_msg,
                              &localization_result_);

  drivers::gnss::InsStat odometry_status;
  FindNearestOdometryStatus(odometry_time, &odometry_status);
  ComposeLocalizationStatus(odometry_status, &localization_status_);
  is_service_started_ = true;
}

// receive lidar pointcloud message
void NDTLocalization::LidarCallback(
    const std::shared_ptr<drivers::PointCloud>& lidar_msg) {
  static unsigned int frame_idx = 0;
  LidarFrame lidar_frame;
  LidarMsgTransfer(lidar_msg, &lidar_frame);

  double time_stamp = lidar_frame.measurement_time;
  Eigen::Affine3d odometry_pose = Eigen::Affine3d::Identity();

// #define ONLY_USE_LIDAR
#ifndef ONLY_USE_LIDAR
  if (!QueryPoseFromBuffer(time_stamp, &odometry_pose)) {
    if (!QueryPoseFromTF(time_stamp, &odometry_pose)) {
      AERROR << "Can not query forecast pose";
      return;
    }
    AINFO << "Query pose from TF";
  } else {
    AINFO << "Query pose from buffer";
  }
#endif

  if (!lidar_locator_.IsInitialized()) {
#ifdef ONLY_USE_LIDAR
    if (!QueryPoseFromBuffer(time_stamp, &odometry_pose)) {
      if (!QueryPoseFromTF(time_stamp, &odometry_pose)) {
        AERROR << "Can not query forecast pose";
        return;
      }
      AINFO << "Query pose from TF";
    } else {
      AINFO << "Query pose from buffer";
    }
#endif
    lidar_locator_.Init(odometry_pose, resolution_id_, zone_id_);
    return;
  }
  lidar_locator_.Update(frame_idx++, odometry_pose, lidar_frame);
  lidar_pose_ = lidar_locator_.GetPose();
  pose_buffer_.UpdateLidarPose(time_stamp, lidar_pose_, odometry_pose);
  ComposeLidarResult(time_stamp, lidar_pose_, &lidar_localization_result_);
  ndt_score_ = lidar_locator_.GetFitnessScore();
  if (ndt_score_ > warnning_ndt_score_) {
    bad_score_count_++;
  } else {
    bad_score_count_ = 0;
  }
  if (ndt_debug_log_flag_) {
    Eigen::Quaterniond tmp_quat(lidar_pose_.linear());
    AINFO << "NDTLocalization Debug Log: lidar pose: " << std::setprecision(15)
          << "time: " << time_stamp << ", "
          << "x: " << lidar_pose_.translation()[0] << ", "
          << "y: " << lidar_pose_.translation()[1] << ", "
          << "z: " << lidar_pose_.translation()[2] << ", "
          << "qx: " << tmp_quat.x() << ", "
          << "qy: " << tmp_quat.y() << ", "
          << "qz: " << tmp_quat.z() << ", "
          << "qw: " << tmp_quat.w();

    Eigen::Quaterniond qbn(odometry_pose.linear());
    AINFO << "NDTLocalization Debug Log: odometry for lidar pose: "
          << std::setprecision(15) << "time: " << time_stamp << ", "
          << "x: " << odometry_pose.translation()[0] << ", "
          << "y: " << odometry_pose.translation()[1] << ", "
          << "z: " << odometry_pose.translation()[2] << ", "
          << "qx: " << qbn.x() << ", "
          << "qy: " << qbn.y() << ", "
          << "qz: " << qbn.z() << ", "
          << "qw: " << qbn.w();
  }
}

void NDTLocalization::OdometryStatusCallback(
    const std::shared_ptr<drivers::gnss::InsStat>& status_msg) {
  std::unique_lock<std::mutex> lock(odometry_status_list_mutex_);
  if (odometry_status_list_.size() < odometry_status_list_max_size_) {
    odometry_status_list_.push_back(*status_msg);
  } else {
    odometry_status_list_.pop_front();
    odometry_status_list_.push_back(*status_msg);
  }
}
// output localization result
void NDTLocalization::GetLocalization(
    LocalizationEstimate* localization) const {
  *localization = localization_result_;
}

void NDTLocalization::GetLidarLocalization(
    LocalizationEstimate* localization) const {
  *localization = lidar_localization_result_;
}

void NDTLocalization::GetLocalizationStatus(
    LocalizationStatus* localization_status) const {
  *localization_status = localization_status_;
}

bool NDTLocalization::IsServiceStarted() { return is_service_started_; }

void NDTLocalization::FillLocalizationMsgHeader(
    LocalizationEstimate* localization) {
  auto* header = localization->mutable_header();
  double timestamp = apollo::cyber::Clock::NowInSeconds();
  header->set_module_name(module_name_);
  header->set_timestamp_sec(timestamp);
  header->set_sequence_num(++localization_seq_num_);
}

void NDTLocalization::ImuCallback(
    const std::shared_ptr<localization::CorrectedImu>& imu_msg) {
  std::unique_lock<std::mutex> lock(imu_list_mutex_);
  if (imu_list_.size() < imu_list_max_size_) {
    imu_list_.push_back(*imu_msg);
  } else {
    imu_list_.pop_front();
    imu_list_.push_back(*imu_msg);
  }
}

bool NDTLocalization::FindMatchingIMU(const double gps_timestamp_sec,
                                      CorrectedImu* imu_msg) {
  if (imu_msg == nullptr) {
    AERROR << "imu_msg should NOT be nullptr.";
    return false;
  }

  std::unique_lock<std::mutex> lock(imu_list_mutex_);
  auto imu_list = imu_list_;
  lock.unlock();

  if (imu_list.empty()) {
    AERROR << "Cannot find Matching IMU. "
           << "IMU message Queue is empty! GPS timestamp[" << gps_timestamp_sec
           << "]";
    return false;
  }

  // scan imu buffer, find first imu message that is newer than the given
  // timestamp
  auto imu_it = imu_list.begin();
  for (; imu_it != imu_list.end(); ++imu_it) {
    if ((*imu_it).header().timestamp_sec() - gps_timestamp_sec >
        std::numeric_limits<double>::min()) {
      break;
    }
  }

  if (imu_it != imu_list.end()) {  // found one
    if (imu_it == imu_list.begin()) {
      AERROR << "IMU queue too short or request too old. "
             << "Oldest timestamp[" << imu_list.front().header().timestamp_sec()
             << "], Newest timestamp["
             << imu_list.back().header().timestamp_sec() << "], GPS timestamp["
             << gps_timestamp_sec << "]";
      *imu_msg = imu_list.front();  // the oldest imu
    } else {
      // here is the normal case
      auto imu_it_1 = imu_it;
      imu_it_1--;
      if (!(*imu_it).has_header() || !(*imu_it_1).has_header()) {
        AERROR << "imu1 and imu_it_1 must both have header.";
        return false;
      }
      if (!InterpolateIMU(*imu_it_1, *imu_it, gps_timestamp_sec, imu_msg)) {
        AERROR << "failed to interpolate IMU";
        return false;
      }
    }
  } else {
    // give the newest imu, without extrapolation
    *imu_msg = imu_list.back();
    if (imu_msg == nullptr) {
      AERROR << "Fail to get latest observed imu_msg.";
      return false;
    }

    if (!imu_msg->has_header()) {
      AERROR << "imu_msg must have header.";
      return false;
    }

    if (std::fabs(imu_msg->header().timestamp_sec() - gps_timestamp_sec) >
        0.02) {
      // 20ms threshold to report error
      AERROR << "Cannot find Matching IMU. IMU messages too old. "
             << "Newest timestamp[" << imu_list.back().header().timestamp_sec()
             << "], GPS timestamp[" << gps_timestamp_sec << "]";
    }
  }

  return true;
}

bool NDTLocalization::InterpolateIMU(const CorrectedImu& imu1,
                                     const CorrectedImu& imu2,
                                     const double timestamp_sec,
                                     CorrectedImu* imu_msg) {
  if (!(imu1.header().has_timestamp_sec() &&
        imu2.header().has_timestamp_sec())) {
    AERROR << "imu1 and imu2 has no header or no timestamp_sec in header";
    return false;
  }
  if (timestamp_sec < imu1.header().timestamp_sec()) {
    AERROR << "[InterpolateIMU1]: the given time stamp["
           << FORMAT_TIMESTAMP(timestamp_sec)
           << "] is older than the 1st message["
           << FORMAT_TIMESTAMP(imu1.header().timestamp_sec()) << "]";
    *imu_msg = imu1;
  } else if (timestamp_sec > imu2.header().timestamp_sec()) {
    AERROR << "[InterpolateIMU2]: the given time stamp[" << timestamp_sec
           << "] is newer than the 2nd message["
           << imu2.header().timestamp_sec() << "]";
    *imu_msg = imu2;
  } else {
    *imu_msg = imu1;
    imu_msg->mutable_header()->set_timestamp_sec(timestamp_sec);

    double time_diff =
        imu2.header().timestamp_sec() - imu1.header().timestamp_sec();
    if (fabs(time_diff) >= 0.001) {
      double frac1 =
          (timestamp_sec - imu1.header().timestamp_sec()) / time_diff;

      if (imu1.imu().has_angular_velocity() &&
          imu2.imu().has_angular_velocity()) {
        auto val = InterpolateXYZ(imu1.imu().angular_velocity(),
                                  imu2.imu().angular_velocity(), frac1);
        imu_msg->mutable_imu()->mutable_angular_velocity()->CopyFrom(val);
      }

      if (imu1.imu().has_linear_acceleration() &&
          imu2.imu().has_linear_acceleration()) {
        auto val = InterpolateXYZ(imu1.imu().linear_acceleration(),
                                  imu2.imu().linear_acceleration(), frac1);
        imu_msg->mutable_imu()->mutable_linear_acceleration()->CopyFrom(val);
      }

      if (imu1.imu().has_euler_angles() && imu2.imu().has_euler_angles()) {
        auto val = InterpolateXYZ(imu1.imu().euler_angles(),
                                  imu2.imu().euler_angles(), frac1);
        imu_msg->mutable_imu()->mutable_euler_angles()->CopyFrom(val);
      }
    }
  }
  return true;
}

template <class T>
T NDTLocalization::InterpolateXYZ(const T& p1, const T& p2,
                                  const double frac1) {
  T p;
  double frac2 = 1.0 - frac1;
  if (p1.has_x() && !std::isnan(p1.x()) && p2.has_x() && !std::isnan(p2.x())) {
    p.set_x(p1.x() * frac2 + p2.x() * frac1);
  }
  if (p1.has_y() && !std::isnan(p1.y()) && p2.has_y() && !std::isnan(p2.y())) {
    p.set_y(p1.y() * frac2 + p2.y() * frac1);
  }
  if (p1.has_z() && !std::isnan(p1.z()) && p2.has_z() && !std::isnan(p2.z())) {
    p.set_z(p1.z() * frac2 + p2.z() * frac1);
  }
  return p;
}

void NDTLocalization::ComposeLocalizationEstimate(
    const Eigen::Affine3d& pose, const localization::CorrectedImu& imu_msg,
    const std::shared_ptr<localization::Gps>& odometry_msg,
    LocalizationEstimate* localization) {
  localization->Clear();
  FillLocalizationMsgHeader(localization);

  localization->set_measurement_time(odometry_msg->header().timestamp_sec());
  auto mutable_pose = localization->mutable_pose();
  mutable_pose->mutable_position()->set_x(pose.translation().x());
  mutable_pose->mutable_position()->set_y(pose.translation().y());
  mutable_pose->mutable_position()->set_z(pose.translation().z());

  Eigen::Quaterniond quat(pose.linear());
  mutable_pose->mutable_orientation()->set_qw(quat.w());
  mutable_pose->mutable_orientation()->set_qx(quat.x());
  mutable_pose->mutable_orientation()->set_qy(quat.y());
  mutable_pose->mutable_orientation()->set_qz(quat.z());
  double heading =
      common::math::QuaternionToHeading(quat.w(), quat.x(), quat.y(), quat.z());
  mutable_pose->set_heading(heading);

  common::math::EulerAnglesZXYd euler(quat.w(), quat.x(), quat.y(), quat.z());
  mutable_pose->mutable_euler_angles()->set_x(euler.pitch());
  mutable_pose->mutable_euler_angles()->set_y(euler.roll());
  mutable_pose->mutable_euler_angles()->set_z(euler.yaw());

  const auto& odometry_pose = odometry_msg->localization();
  if (odometry_pose.has_linear_velocity()) {
    mutable_pose->mutable_linear_velocity()->CopyFrom(
        odometry_pose.linear_velocity());
  }
  if (odometry_pose.has_linear_acceleration()) {
    mutable_pose->mutable_linear_acceleration()->CopyFrom(
        odometry_pose.linear_acceleration());
  }
  if (odometry_pose.has_angular_velocity()) {
    mutable_pose->mutable_angular_velocity()->CopyFrom(
        odometry_pose.angular_velocity());
  }
  if (odometry_pose.has_linear_acceleration_vrf()) {
    mutable_pose->mutable_linear_acceleration_vrf()->CopyFrom(
        odometry_pose.linear_acceleration_vrf());
  }
  if (odometry_pose.has_angular_velocity_vrf()) {
    mutable_pose->mutable_angular_velocity_vrf()->CopyFrom(
        odometry_pose.angular_velocity_vrf());
  }

  if (imu_msg.has_imu()) {
    const auto& imu = imu_msg.imu();
    // linear acceleration
    if (imu.has_linear_acceleration()) {
      if (localization->pose().has_orientation()) {
        // linear_acceleration:
        // convert from vehicle reference to map reference
        Eigen::Vector3d orig(imu.linear_acceleration().x(),
                      imu.linear_acceleration().y(),
                      imu.linear_acceleration().z());
        Eigen::Vector3d vec = common::math::QuaternionRotate(
            localization->pose().orientation(), orig);
        mutable_pose->mutable_linear_acceleration()->set_x(vec[0]);
        mutable_pose->mutable_linear_acceleration()->set_y(vec[1]);
        mutable_pose->mutable_linear_acceleration()->set_z(vec[2]);

        // linear_acceleration_vfr
        mutable_pose->mutable_linear_acceleration_vrf()->CopyFrom(
            imu.linear_acceleration());
      } else {
        AERROR << "[PrepareLocalizationMsg]: "
               << "fail to convert linear_acceleration";
      }
    }

    // angular velocity
    if (imu.has_angular_velocity()) {
      if (localization->pose().has_orientation()) {
        // angular_velocity:
        // convert from vehicle reference to map reference
        Eigen::Vector3d orig(imu.angular_velocity().x(),
                             imu.angular_velocity().y(),
                             imu.angular_velocity().z());
        Eigen::Vector3d vec = common::math::QuaternionRotate(
            localization->pose().orientation(), orig);
        mutable_pose->mutable_angular_velocity()->set_x(vec[0]);
        mutable_pose->mutable_angular_velocity()->set_y(vec[1]);
        mutable_pose->mutable_angular_velocity()->set_z(vec[2]);

        // angular_velocity_vf
        mutable_pose->mutable_angular_velocity_vrf()->CopyFrom(
            imu.angular_velocity());
      } else {
        AERROR << "[PrepareLocalizationMsg]: fail to convert angular_velocity";
      }
    }

    // euler angle
    if (imu.has_euler_angles()) {
      mutable_pose->mutable_euler_angles()->CopyFrom(imu.euler_angles());
    }
  }
}

void NDTLocalization::ComposeLidarResult(double time_stamp,
                                         const Eigen::Affine3d& pose,
    LocalizationEstimate* localization) {
  localization->Clear();
  FillLocalizationMsgHeader(localization);

  localization->set_measurement_time(time_stamp);
  auto mutable_pose = localization->mutable_pose();
  mutable_pose->mutable_position()->set_x(pose.translation().x());
  mutable_pose->mutable_position()->set_y(pose.translation().y());
  mutable_pose->mutable_position()->set_z(pose.translation().z());

  Eigen::Quaterniond quat(pose.linear());
  mutable_pose->mutable_orientation()->set_qw(quat.w());
  mutable_pose->mutable_orientation()->set_qx(quat.x());
  mutable_pose->mutable_orientation()->set_qy(quat.y());
  mutable_pose->mutable_orientation()->set_qz(quat.z());
  double heading =
      common::math::QuaternionToHeading(quat.w(), quat.x(), quat.y(), quat.z());
  mutable_pose->set_heading(heading);

  common::math::EulerAnglesZXYd euler(quat.w(), quat.x(), quat.y(), quat.z());
  mutable_pose->mutable_euler_angles()->set_x(euler.pitch());
  mutable_pose->mutable_euler_angles()->set_y(euler.roll());
  mutable_pose->mutable_euler_angles()->set_z(euler.yaw());
}

bool NDTLocalization::QueryPoseFromTF(double time, Eigen::Affine3d* pose) {
  cyber::Time query_time(time);
  const float time_out = 0.01f;
  std::string err_msg = "";
  bool can_transform = tf_buffer_->canTransform(
      tf_target_frame_id_, tf_source_frame_id_, query_time, time_out, &err_msg);
  if (!can_transform) {
    AERROR << "Can not transform: " << err_msg;
    return false;
  }
  apollo::transform::TransformStamped query_transform;
  try {
    query_transform = tf_buffer_->lookupTransform(
        tf_target_frame_id_, tf_source_frame_id_, query_time);
  } catch (tf2::TransformException ex) {
    AERROR << ex.what();
    return false;
  }
  pose->translation()[0] = query_transform.transform().translation().x();
  pose->translation()[1] = query_transform.transform().translation().y();
  pose->translation()[2] = query_transform.transform().translation().z();
  Eigen::Quaterniond tmp_quat(query_transform.transform().rotation().qw(),
                              query_transform.transform().rotation().qx(),
                              query_transform.transform().rotation().qy(),
                              query_transform.transform().rotation().qz());
  pose->linear() = tmp_quat.toRotationMatrix();
  return true;
}

void NDTLocalization::ComposeLocalizationStatus(
    const drivers::gnss::InsStat& status,
    LocalizationStatus* localization_status) {
  apollo::common::Header* header = localization_status->mutable_header();
  double timestamp = apollo::cyber::Clock::NowInSeconds();
  header->set_timestamp_sec(timestamp);
  localization_status->set_measurement_time(status.header().timestamp_sec());

  if (!status.has_pos_type()) {
    localization_status->set_fusion_status(MeasureState::ERROR);
    localization_status->set_state_message(
        "Error: Current Localization Status Is Missing.");
    return;
  }

  auto pos_type = static_cast<drivers::gnss::SolutionType>(status.pos_type());
  if (ndt_score_ < warnning_ndt_score_ &&
      pos_type == drivers::gnss::SolutionType::INS_RTKFIXED) {
    localization_status->set_fusion_status(MeasureState::OK);
    localization_status->set_state_message("");
  } else if (bad_score_count_ > bad_score_count_threshold_ ||
             ndt_score_ > error_ndt_score_ ||
             (pos_type != drivers::gnss::SolutionType::INS_RTKFIXED &&
              pos_type != drivers::gnss::SolutionType::INS_RTKFLOAT)) {
    localization_status->set_fusion_status(MeasureState::ERROR);
    localization_status->set_state_message(
        "Error: Current Localization Is Very Unstable.");
  } else {
    localization_status->set_fusion_status(MeasureState::WARNNING);
    localization_status->set_state_message(
        "Warning: Current Localization Is Unstable.");
  }
}

bool NDTLocalization::isSupportInterpolationPose(double time,
                                                 TimeStampPose& pre_pose,
                                                 TimeStampPose& next_pose) {
  std::lock_guard<std::mutex> lock(odometry_buffer_mutex_);
  TimeStampPose timestamp_pose = odometry_buffer_.back();
  AINFO << std::setprecision(20) << time;
  AINFO << std::setprecision(20) << odometry_buffer_.front().timestamp;
  AINFO << std::setprecision(20) << odometry_buffer_.back().timestamp;
  // check abnormal timestamp
  if (time > timestamp_pose.timestamp) {
    AINFO << "-----------------------";
    AINFO << "query time is newer than latest odometry time, it doesn't "
             "make sense!";
    return false;
  }
  // search from reverse direction
  auto iter = odometry_buffer_.crbegin();
  for (; iter != odometry_buffer_.crend(); ++iter) {
    if (iter->timestamp < time) {
      break;
    }
  }
  if (iter == odometry_buffer_.crend()) {
    AINFO << "Cannot find matching pose from odometry buffer";
    return false;
  }
  pre_pose = *iter;
  next_pose = *(--iter);
  return true;
}

bool NDTLocalization::QueryPoseFromBuffer(double time, Eigen::Affine3d* pose) {
  CHECK_NOTNULL(pose);

  TimeStampPose pre_pose;
  TimeStampPose next_pose;

  bool interpolation_flag = false;
  int count_flag = 10;
  while (count_flag-- > 0) {
    if (isSupportInterpolationPose(time, pre_pose, next_pose)){
      interpolation_flag = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (!interpolation_flag){
    AINFO << "-----------------------";
    AERROR << "query time is newer than latest odometry time, it doesn't make sense!";
    return false;
  }
  // interpolation
  double v1 =
      (next_pose.timestamp - time) / (next_pose.timestamp - pre_pose.timestamp);
  double v2 =
      (time - pre_pose.timestamp) / (next_pose.timestamp - pre_pose.timestamp);
  pose->translation() =
      pre_pose.pose.translation() * v1 + next_pose.pose.translation() * v2;

  Eigen::Quaterniond pre_quat(pre_pose.pose.linear());
  Eigen::Quaterniond next_quat(next_pose.pose.linear());
  double tmp_quat[4] = {};
  tmp_quat[0] = pre_quat.w() * v1 + next_quat.w() * v2;
  tmp_quat[1] = pre_quat.x() * v1 + next_quat.x() * v2;
  tmp_quat[2] = pre_quat.y() * v1 + next_quat.y() * v2;
  tmp_quat[3] = pre_quat.z() * v1 + next_quat.z() * v2;

  Eigen::Quaterniond quat(tmp_quat[0], tmp_quat[1], tmp_quat[2], tmp_quat[3]);
  quat.normalize();
  pose->linear() = quat.toRotationMatrix();
  return true;
}

bool NDTLocalization::ZeroOdometry(const Eigen::Affine3d& pose) {
  double x = pose.translation().x();
  double y = pose.translation().y();
  double z = pose.translation().z();
  double norm = std::sqrt(x * x + y * y + z * z);
  if (norm <= 0.01) {
    return true;
  }
  return false;
}

void NDTLocalization::LidarMsgTransfer(
    const std::shared_ptr<drivers::PointCloud>& msg, LidarFrame* lidar_frame) {
  CHECK_NOTNULL(lidar_frame);

  if (msg->height() > 1 && msg->width() > 1) {
    for (unsigned int i = 0; i < msg->height(); ++i) {
      for (unsigned int j = 0; j < msg->width(); ++j) {
        Eigen::Vector3f pt3d;
        pt3d[0] = msg->point(i * msg->width() + j).x();
        pt3d[1] = msg->point(i * msg->width() + j).y();
        pt3d[2] = msg->point(i * msg->width() + j).z();
        if (!std::isnan(pt3d[0])) {
          Eigen::Vector3f pt3d_tem = pt3d;

          if (pt3d_tem[2] > max_height_) {
            continue;
          }
          unsigned char intensity = static_cast<unsigned char>(
              msg->point(i * msg->width() + j).intensity());
          lidar_frame->pt_xs.push_back(pt3d[0]);
          lidar_frame->pt_ys.push_back(pt3d[1]);
          lidar_frame->pt_zs.push_back(pt3d[2]);
          lidar_frame->intensities.push_back(intensity);
        }
      }
    }
  } else {
    AINFO << "Receiving un-organized-point-cloud, width " << msg->width()
          << " height " << msg->height() << "size " << msg->point_size();
    for (int i = 0; i < msg->point_size(); ++i) {
      Eigen::Vector3f pt3d;
      pt3d[0] = msg->point(i).x();
      pt3d[1] = msg->point(i).y();
      pt3d[2] = msg->point(i).z();
      if (!std::isnan(pt3d[0])) {
        Eigen::Vector3f pt3d_tem = pt3d;

        if (pt3d_tem[2] > max_height_) {
          continue;
        }
        unsigned char intensity =
            static_cast<unsigned char>(msg->point(i).intensity());
        lidar_frame->pt_xs.push_back(pt3d[0]);
        lidar_frame->pt_ys.push_back(pt3d[1]);
        lidar_frame->pt_zs.push_back(pt3d[2]);
        lidar_frame->intensities.push_back(intensity);
      }
    }
  }

  lidar_frame->measurement_time =
      cyber::Time(msg->measurement_time()).ToSecond();
  if (ndt_debug_log_flag_) {
    AINFO << std::setprecision(15)
          << "NDTLocalization Debug Log: velodyne msg. "
          << "[time:" << lidar_frame->measurement_time
          << "][height:" << msg->height() << "][width:" << msg->width()
          << "][point_cnt:" << msg->point_size() << "]";
  }
}

bool NDTLocalization::LoadLidarExtrinsic(const std::string& file_path,
                                         Eigen::Affine3d* lidar_extrinsic) {
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

bool NDTLocalization::LoadLidarHeight(const std::string& file_path,
                                      LidarHeight* height) {
  CHECK_NOTNULL(height);

  if (!cyber::common::PathExists(file_path)) {
    return false;
  }

  YAML::Node config = YAML::LoadFile(file_path);
  if (config["vehicle"]) {
    if (config["vehicle"]["parameters"]) {
      height->height = config["vehicle"]["parameters"]["height"].as<double>();
      height->height_var =
          config["vehicle"]["parameters"]["height_var"].as<double>();
      return true;
    }
  }
  return false;
}

bool NDTLocalization::LoadZoneIdFromFolder(const std::string& folder_path,
                                           int* zone_id) {
  std::string map_zone_id_folder;
  if (cyber::common::DirectoryExists(folder_path + "/map/000/north")) {
    map_zone_id_folder = folder_path + "/map/000/north";
  } else if (cyber::common::DirectoryExists(folder_path + "/map/000/south")) {
    map_zone_id_folder = folder_path + "/map/000/south";
  } else {
    return false;
  }

  auto folder_list = cyber::common::ListSubPaths(map_zone_id_folder);
  for (auto itr = folder_list.begin(); itr != folder_list.end(); ++itr) {
    *zone_id = std::stoi(*itr);
    return true;
  }
  return false;
}

bool NDTLocalization::FindNearestOdometryStatus(
    const double odometry_timestamp, drivers::gnss::InsStat* status) {
  CHECK_NOTNULL(status);

  std::unique_lock<std::mutex> lock(odometry_status_list_mutex_);
  auto odometry_status_list = odometry_status_list_;
  lock.unlock();

  double timestamp_diff_sec = 1e8;
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

}  // namespace ndt
}  // namespace localization
}  // namespace apollo
