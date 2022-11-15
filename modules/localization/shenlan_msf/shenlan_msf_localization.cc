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

#include "modules/localization/shenlan_msf/shenlan_msf_localization.h"

#include "yaml-cpp/yaml.h"

#include "modules/drivers/gnss/proto/config.pb.h"

#include "cyber/common/file.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/math/euler_angles_zxy.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/time_util.h"
// #include "modules/localization/common/localization_gflags.h"
#include "modules/localization/shenlan_msf/shenlan_msf_localization_component.h"

namespace apollo {
namespace localization {

using apollo::common::Status;
using common::util::TimeUtil;

SLMSFLocalization::SLMSFLocalization()
    : monitor_logger_(
          apollo::common::monitor::MonitorMessageItem::LOCALIZATION),
      raw_imu_msg_(nullptr) {
  const std::string config_file_path =
      WORK_SPACE_PATH + "/config/mapping/shenlan_lio_back_end.yaml";

  front_end_ptr_ = std::shared_ptr<FrontEnd>(new FrontEnd());
  back_end_ptr_ = std::shared_ptr<BackEnd>(new BackEnd(config_file_path));
  post_processing_ptr_ = std::shared_ptr<PostProcessing>(new PostProcessing);
  loop_closing_ptr_ = std::shared_ptr<LoopClosing> (new LoopClosing((WORK_SPACE_PATH)));
  
  back_end_loop =
    std::thread(&SLMSFLocalization::LocalizationImplementation,this);
  back_end_loop.detach();
  std::cout << "SLMSFLocalization Init Finished.\n";
}

SLMSFLocalization::~SLMSFLocalization() {
  std::cout << "System Begin to Optimize and Please Wait A Few Seconds!\n";
  RequestFinish();
  while (!isFinished()) usleep(5000);

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
  std::cout << "Finished All Activity and Will be Closed!\n";
}

void SLMSFLocalization::RequestFinish() {
  std::unique_lock<std::mutex> lock(mutex_finish_);
  finish_requested_ = true;
}

bool SLMSFLocalization::CheckFinish() {
  std::unique_lock<std::mutex> lock(mutex_finish_);
  return finish_requested_;
}

void SLMSFLocalization::SetFinish() {
  std::unique_lock<std::mutex> lock(mutex_finish_);
  finished_ = true;
}

bool SLMSFLocalization::isFinished() {
  std::unique_lock<std::mutex> lock(mutex_finish_);
  return finished_;
}

Status SLMSFLocalization::Init() {
  std::string lidar_extrinsics_file =
      "/apollo/modules/calibration/data/dev_kit_pix_hooke/lidar_params/"
      "lidar16_novatel_extrinsics.yaml";
  lidar_extrinsic = Eigen::Affine3d::Identity();
  if (!LoadLidarExtrinsic(lidar_extrinsics_file, &lidar_extrinsic)) {
    AERROR << "Fail to Load Lidar Extrinsic!";
  }

  localization_timer_.reset(new cyber::Timer(
      10, [this]() { this->OnLocalizationTimer(); }, false));
  localization_timer_->Start();
  return Status::OK();
}

void SLMSFLocalization::OnOdometry(
    const std::shared_ptr<localization::Gps> &message) {
  PoseData odometry;
  OdometryMsgTransfer(message, odometry);

  std::lock_guard<std::mutex> lock(odom_list_mutex_);
  if (odom_list_.size() < odom_list_max_size_) {
    odom_list_.push_back(odometry);
  } else {
    odom_list_.pop_front();
    odom_list_.push_back(odometry);
  }
  static size_t drop = 0;
  if (odometry_inited && ++drop % 5 == 0) {
    double timestamp = odometry.time_;
    odometry.pose_.block<3, 1>(0, 3) -= gnss_origin;
    ComposeLocalizationResult(timestamp, odometry.pose_,
                              &fusion_localization_result_);
    publisher_->PublishLocalizationSLMSFGnss(fusion_localization_result_);

    {
      // TODO: 平滑算法提高频率
      std::unique_lock<std::mutex> lock2(fusion_pose_list_mutex_);
      // case1:
      if (!fusion_pose_list_.empty()) {
        // Eigen::Matrix4f cur_pose = post_processing_ptr_->GetCurrentPose();
        Eigen::Matrix4f cur_pose = fusion_pose_list_.front();
        fusion_pose_list_.pop_front();
        ComposeLocalizationResult(timestamp, cur_pose,
                                  &fusion_localization_result_);
        publisher_->PublishLocalizationSLMSFFusion(fusion_localization_result_);
      }else{
        Eigen::Matrix4f cur_pose = odometry.pose_;
        ComposeLocalizationResult(timestamp, cur_pose,
                                  &fusion_localization_result_);
        publisher_->PublishLocalizationSLMSFFusion(fusion_localization_result_);
      }
    }

    // // case2
    // post_processing_ptr_->UpdateWithNewKeyFrame(lidar_pose_result,
    // lidar_frame,
    //                                             key_frames_list_);
    // if (!optimized_key_frames_list_.empty()) {
    //   post_processing_ptr_->UpdateWithOptimizedKeyFrames(optimized_key_frames_list_);
    // }
    // Eigen::Matrix4f cur_pose = post_processing_ptr_->GetCurrentPose();
    // ComposeLocalizationResult(timestamp, cur_pose,
    // &fusion_localization_result_);
    // publisher_->PublishLocalizationSLMSFFusion(fusion_localization_result_);
  }

}

void SLMSFLocalization::OnInsStat(
    const std::shared_ptr<drivers::gnss::InsStat> &message) {
  std::lock_guard<std::mutex> lock(odometry_status_list_mutex_);
  if (odometry_status_list_.size() < odometry_status_list_max_size_) {
    odometry_status_list_.push_back(*message);
  } else {
    odometry_status_list_.pop_front();
    odometry_status_list_.push_back(*message);
  }
}

void SLMSFLocalization::OnChassis(
    const std::shared_ptr<canbus::Chassis> &message) {
  VelocityData chassis;
  ChassisMsgTransfer(message, chassis);
  std::lock_guard<std::mutex> lock(chassis_list_mutex_);
  if (chassis_list_.size() < chassis_list_max_size_) {
    chassis_list_.push_back(chassis);
  } else {
    chassis_list_.pop_front();
    chassis_list_.push_back(chassis);
  }
}

void SLMSFLocalization::OnPointCloud(
    const std::shared_ptr<drivers::PointCloud> &message) {
  CloudData tmp_lidar_frame;
  LidarMsgTransfer(message, &tmp_lidar_frame);

  CloudData::CloudTypePtr temp_cloud(new CloudData::CloudType());
  Eigen::Matrix4f T_imu_lidar_ = lidar_extrinsic.matrix().cast<float>();
  pcl::transformPointCloud(*tmp_lidar_frame.cloud_ptr_, *temp_cloud,
                           T_imu_lidar_);
  CloudData lidar_frame;
  // TODO: fix timestamp
  double timestamp = tmp_lidar_frame.time_ - 0.1;
  lidar_frame.time_ = timestamp;
  *lidar_frame.cloud_ptr_ = *temp_cloud;

  // front_end modules
  UpdateLaserOdometry(lidar_frame);
  ComposeLocalizationResult(timestamp + 0.1, laser_odometry_,
                            &lidar_localization_result_);
  publisher_->PublishLocalizationSLMSFLidar(lidar_localization_result_);

  PoseData lidar_pose_result;
  lidar_pose_result.time_ = timestamp;
  lidar_pose_result.pose_ = laser_odometry_;
  std::vector<double> cov_tmp(36, front_end_ptr_->is_degeneracy());
  lidar_pose_result.cov_ = cov_tmp;

  LidarCombination lidar_comb;
  lidar_comb.lidar_pose = lidar_pose_result;
  lidar_comb.lidar_cloud = lidar_frame;

  {
    std::unique_lock<std::mutex> lock1(lidar_pose_list_mutex_);
    lidar_pose_list_.emplace_back(lidar_comb);
  }
std::cout << " test1 ";
  {
    std::unique_lock<std::mutex> lock2(fusion_pose_list_mutex_);
    // case1:
    if (!fusion_pose_list_.empty()) {
      // Eigen::Matrix4f cur_pose = post_processing_ptr_->GetCurrentPose();
      Eigen::Matrix4f cur_pose = fusion_pose_list_.front();
      fusion_pose_list_.pop_front();
      ComposeLocalizationResult(timestamp, cur_pose,
                                &fusion_localization_result_);
      publisher_->PublishLocalizationSLMSFFusion(fusion_localization_result_);
    }
  }
std::cout << "test2 \n";
  // // case2
  // post_processing_ptr_->UpdateWithNewKeyFrame(lidar_pose_result, lidar_frame,
  //                                             key_frames_list_);
  // if (!optimized_key_frames_list_.empty()) {
  //   post_processing_ptr_->UpdateWithOptimizedKeyFrames(optimized_key_frames_list_);
  // }
  // Eigen::Matrix4f cur_pose = post_processing_ptr_->GetCurrentPose();
  // ComposeLocalizationResult(timestamp, cur_pose,
  // &fusion_localization_result_);
  // publisher_->PublishLocalizationSLMSFFusion(fusion_localization_result_);
}

// 另一个线程，使数据和算法分离
void SLMSFLocalization::LocalizationImplementation() {
  std::cout << "LocalizationImplementation Start.\n";
  while (1) {
    if (CheckFinish()) {
      std::cerr << "CheckFinish Detect Stop!\n";
      break;
    }
    //std::cout << "Running ... ... ... ...\r";

    PoseData lidar_pose_result;
    CloudData lidar_frame;
    {
      std::unique_lock<std::mutex> lock(lidar_pose_list_mutex_);
      size_t nums = lidar_pose_list_.size();
      if (nums == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      lidar_pose_result = lidar_pose_list_.front().lidar_pose;
      lidar_frame = lidar_pose_list_.front().lidar_cloud;
      lidar_pose_list_.pop_front();
    }
    double timestamp = lidar_pose_result.time_;

    PoseData gnss_pose_result;
    bool syn_gnss = false;
    {
      std::unique_lock<std::mutex> lock(odom_list_mutex_);
      int count_flag = 20;
      while (count_flag-- > 0) {
        if (PoseData::SyncData(timestamp, odom_list_, gnss_pose_result) &&
            odometry_inited) {
          syn_gnss = true;
          gnss_pose_result.pose_.block<3, 1>(0, 3) -= gnss_origin;
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      if (!syn_gnss) {
        AINFO << std::setprecision(16) << "timestamp1: " << timestamp;
        AINFO << std::setprecision(16)
              << "timestamp2: " << odom_list_.back().time_;
        AERROR << "Fail to Sync Pose Data, Please Check!";
      }
    }

    IMUData imu_data_result;
    bool syn_imu = false;
    {
      std::unique_lock<std::mutex> lock(imu_list_mutex_);
      int count_flag = 20;
      while (count_flag-- > 0) {
        if (IMUData::SyncData(timestamp, imu_list_, imu_data_result)) {
          syn_imu = true;
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      if (!syn_imu) {
        AINFO << std::setprecision(16) << "timestamp1: " << timestamp;
        AINFO << std::setprecision(16)
              << "timestamp2: " << imu_list_.back().time_;
        AERROR << "Fail to Sync Pose Data, Please Check!";
      }
      // imu_list_mutex_.unlock();
    }

    if (syn_gnss && syn_imu) {
      std::unique_lock<std::mutex> lock(fusion_pose_list_mutex_);
      // add loop poses for graph optimization:
      InsertLoopClosurePose();

      back_end_ptr_->Update(lidar_frame, lidar_pose_result, gnss_pose_result,
                            imu_data_result);
      
      // publish gnss_pose_result

      if (back_end_ptr_->has_new_key_frame()) {
        KeyFrame key_frame;
        back_end_ptr_->GetLatestKeyFrame(key_frame);
        key_frames_list_.emplace_back(key_frame);
        // publish keyframe
        publisher_->PublishKeyFrame(key_frame);

        back_end_ptr_->GetLatestKeyGNSS(key_frame);
        publisher_->PublishKeyGnssFrame(key_frame);
      }

      if (back_end_ptr_->has_new_optimized()) {
        back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames_list_);
      }

      post_processing_ptr_->UpdateWithNewKeyFrame(gnss_pose_result, lidar_frame,
                                                  key_frames_list_);

      if (!optimized_key_frames_list_.empty()) {
        post_processing_ptr_->UpdateWithOptimizedKeyFrames(
            optimized_key_frames_list_);
      }

      fusion_pose_list_.emplace_back(post_processing_ptr_->GetCurrentPose());
    }
  }

  SetFinish();
};

/**
 * @brief  add loop closure for backend optimization
 * @param  void
 * @return true if success false otherwise
 */
bool SLMSFLocalization::InsertLoopClosurePose() {
  std::unique_lock<std::mutex> lock(loop_list_mutex);
  while (loop_pose_data_list_.size() > 0) {
    back_end_ptr_->InsertLoopPose(loop_pose_data_list_.front());
    loop_pose_data_list_.pop_front();
  }
  loop_list_mutex.unlock();
  return true;
}

void SLMSFLocalization::ComposeLocalizationResult(
    double time_stamp, const Eigen::Matrix4f &pose,
    LocalizationEstimate *localization) {
  localization->Clear();
  FillLocalizationMsgHeader(localization);

  localization->set_measurement_time(time_stamp);
  auto *mutable_pose = localization->mutable_pose();
  Eigen::Vector3f t = pose.block<3, 1>(0, 3);
  Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
  // mutable_pose->mutable_position()->set_x(t.x() + gnss_origin.x());
  // mutable_pose->mutable_position()->set_y(t.y() + gnss_origin.y());
  // mutable_pose->mutable_position()->set_z(t.z() + gnss_origin.z());
  mutable_pose->mutable_position()->set_x(t.x());
  mutable_pose->mutable_position()->set_y(t.y());
  mutable_pose->mutable_position()->set_z(t.z());
  mutable_pose->mutable_orientation()->set_qw(quat.w());
  mutable_pose->mutable_orientation()->set_qx(quat.x());
  mutable_pose->mutable_orientation()->set_qy(quat.y());
  mutable_pose->mutable_orientation()->set_qz(quat.z());
  // double heading =
  //     common::math::QuaternionToHeading(quat.w(), quat.x(), quat.y(),
  //     quat.z());
  // mutable_pose->set_heading(heading);

  // common::math::EulerAnglesZXYd euler(quat.w(), quat.x(), quat.y(),
  // quat.z()); mutable_pose->mutable_euler_angles()->set_x(euler.pitch());
  // mutable_pose->mutable_euler_angles()->set_y(euler.roll());
  // mutable_pose->mutable_euler_angles()->set_z(euler.yaw());
}

void SLMSFLocalization::FillLocalizationMsgHeader(
    LocalizationEstimate *localization) {
  auto *header = localization->mutable_header();
  double timestamp = apollo::cyber::Clock::NowInSeconds();
  header->set_module_name(module_name_);
  header->set_timestamp_sec(timestamp);
  // header->set_sequence_num(++localization_seq_num_);
}

bool SLMSFLocalization::UpdateLaserOdometry(CloudData &lidar_frame) {
  if (!odometry_inited) {
    odometry_inited = true;
    if (odom_list_.size() > 0) {
      std::lock_guard<std::mutex> lock(odom_list_mutex_);
      double timestamp = lidar_frame.time_;
      PoseData result;
      std::deque<PoseData, Eigen::aligned_allocator<PoseData>> input =
          odom_list_;

      Eigen::Matrix4f odom_pose;
      // std::deque<PoseData, Eigen::aligned_allocator<PoseData>> result;
      if (PoseData::SyncData(timestamp, input, result)) {
        odom_pose = result.pose_;
      } else {
        odom_pose = odom_list_.back().pose_;
      }
      gnss_origin = odom_pose.block<3, 1>(0, 3);
      odom_pose.block<3, 1>(0, 3) = Eigen::Vector3f::Zero();

      std::cout << "Init Pose is : \n" << odom_pose << std::endl;
      front_end_ptr_->SetInitPose(odom_pose);
      std::cout << "Init Finished. " << std::endl;

    } else {
      front_end_ptr_->SetInitPose(Eigen::Matrix4f::Identity());
      AWARN << "FAIL TO GET /apollo/gnss/odometry INFO.";
    }
    return true;
  }
  return front_end_ptr_->Update(lidar_frame, laser_odometry_);
}

void SLMSFLocalization::OnRawImu(
    const std::shared_ptr<drivers::gnss::Imu> &imu_msg) {
  IMUData imu;
  IMUMsgTransfer(imu_msg, imu);

  std::unique_lock<std::mutex> lock(imu_list_mutex_);
  imu_list_.push_back(imu);
  imu_list_mutex_.unlock();
  // imu_list_pre_integration_list.push_back(imu);

  // std::unique_lock<std::mutex> lock2(fusion_pose_list_mutex_);
  back_end_ptr_->UpdateIMUPreIntegration(imu);
  // PoseData last_pose = back_end_ptr_->GetLastPose();
  // IMUPreIntegrator::IMUPreIntegration imu_pre_integration =
  //     back_end_ptr_->GetPreIntegration(imu.time_);
  // lock2.unlock();

  // // std::cout << (imu_pre_integration.alpha_ij).cast<float>() << std::endl;
  // Eigen::Matrix4f cur_pose = Eigen::Matrix4f::Identity();
  // float T = imu_pre_integration.T;
  // Eigen::Vector3f g = imu_pre_integration.g.cast<float>();
  // cur_pose.block<3, 1>(0, 3) =
  //   last_pose.pose_.block<3, 1>(0, 3) +
  //   (last_pose.vel_.v - 0.5 * g * T) * T +
  //   last_pose.pose_.block<3, 3>(0, 0) *
  //   (imu_pre_integration.alpha_ij.cast<float>());

  // const Sophus::SO3d prev_theta(
  //     Eigen::Quaterniond(last_pose.pose_.block<3, 3>(0, 0).cast<double>()));
  // const Sophus::SO3d curr_theta = prev_theta * imu_pre_integration.theta_ij;
  // cur_pose.block<3, 3>(0, 0) = curr_theta.matrix().cast<float>();

  // ComposeLocalizationResult(imu.time_, cur_pose,
  // &fusion_localization_result_);
  // publisher_->PublishLocalizationSLMSFFusion(fusion_localization_result_);
}

void SLMSFLocalization::OnRawImuCache(
    const std::shared_ptr<drivers::gnss::Imu> &imu_msg) {
  if (imu_msg) {
    std::lock_guard<std::mutex> lock(mutex_imu_msg_);
    raw_imu_msg_ = const_cast<std::shared_ptr<drivers::gnss::Imu> &>(imu_msg);
  }
}

void SLMSFLocalization::OnGnssBestPose(
    const std::shared_ptr<drivers::gnss::GnssBestPose> &message) {}

void SLMSFLocalization::OnLoopClosing(const  std::shared_ptr<LoopPose> &message){
  std::unique_lock<std::mutex> lock(loop_list_mutex);
  loop_pose_data_list_.push_back(*message);
  loop_list_mutex.unlock();
  std::cout << "received loop closing pose"<< std::endl;
}
void SLMSFLocalization::OnLocalizationTimer() {
  if (!raw_imu_msg_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_imu_msg_);
  OnRawImu(raw_imu_msg_);
  raw_imu_msg_= nullptr;
}

void SLMSFLocalization::SetPublisher(
    const std::shared_ptr<LocalizationMsgPublisher> &publisher) {
  publisher_ = publisher;
}

void SLMSFLocalization::LidarMsgTransfer(
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

void SLMSFLocalization::IMUMsgTransfer(
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

void SLMSFLocalization::OdometryMsgTransfer(
    const std::shared_ptr<localization::Gps> &odom_msg, PoseData &odom_frame) {
  odom_frame.time_ = odom_msg->header().timestamp_sec();
  // set the position:
  odom_frame.pose_(0, 3) = odom_msg->localization().position().x();
  odom_frame.pose_(1, 3) = odom_msg->localization().position().y();
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

void SLMSFLocalization::ChassisMsgTransfer(
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

bool SLMSFLocalization::LoadLidarExtrinsic(const std::string &file_path,
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

bool SLMSFLocalization::FindNearestOdometryStatus(
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

}  // namespace localization
}  // namespace apollo
