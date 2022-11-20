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

#include "modules/localization/shenlan_msf/mapping/shenlan_loop_closing_component.h"

#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/math/quaternion.h"
// #include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {

using apollo::cyber::Clock;

LoopClosingComponent::LoopClosingComponent() {}
bool LoopClosingComponent::Init() {
  // 初始化配置环境
  loop_closing_ = std::shared_ptr<LoopClosing>(new LoopClosing(WORK_SPACE_PATH));
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
bool LoopClosingComponent::InitConfig() { return true; }
bool LoopClosingComponent::InitIO() {

  key_gnss_listener_ = node_->CreateReader<KeyFrame>(
      key_gnss_topic_, std::bind(&LoopClosingComponent::KeyGnssCallback,
                                 this, std::placeholders::_1));
  ACHECK(key_gnss_listener_);

  key_frame_listener_ = node_->CreateReader<KeyFrame>(
      key_frame_topic_, std::bind(&LoopClosingComponent::KeyFrameCallback,
                                  this, std::placeholders::_1));
  ACHECK(key_frame_listener_);

  loop_closing_publisher_ = node_->CreateWriter<LoopPose>(loop_closing_topic_);
  ACHECK(loop_closing_publisher_);

  loop_closing_timer_.reset(new cyber::Timer(
      100, [this]() { this->OnLoopClosingTimer(); }, false));
  loop_closing_timer_->Start();

  return true;
}

bool LoopClosingComponent::Proc(
    const std::shared_ptr<drivers::PointCloud>& message) {
    PointCloudCallback(message);

    return true;
}

void LoopClosingComponent::KeyGnssCallback(const std::shared_ptr<KeyFrame>& gnss_msg) {
  std::unique_lock<std::mutex> lock(gnss_list_mutex_);
  if (key_gnss_buff_.size() < gnss_list_max_size_) {
    key_gnss_buff_.push_back(*gnss_msg);
  } else {
    key_gnss_buff_.pop_front();
    key_gnss_buff_.push_back(*gnss_msg);
  }
  lock.unlock();
}
void LoopClosingComponent::KeyFrameCallback(const std::shared_ptr<KeyFrame>& fram_msg) {
  std::unique_lock<std::mutex> lock(frame_list_mutex_);
  if (key_frame_buff_.size() < frame_list_max_size_) {
    key_frame_buff_.push_back(*fram_msg);
  } else {
    key_frame_buff_.pop_front();
    key_frame_buff_.push_back(*fram_msg);
  }
  lock.unlock();
}
void LoopClosingComponent::PointCloudCallback(const std::shared_ptr<drivers::PointCloud>& cloud_msg) {
  // transform PointCLoud to Cloudata
  CloudData lidar_frame;
  LidarMsgTransfer(cloud_msg, &lidar_frame);
  std::unique_lock<std::mutex> lock(scan_list_mutex_);
  if (key_scan_buff_.size() < scan_list_max_size_) {
    key_scan_buff_.push_back(lidar_frame);
  } else {
    key_scan_buff_.pop_front();
    key_scan_buff_.push_back(lidar_frame);
  }
  lock.unlock();
}
bool LoopClosingComponent::OnLoopClosingTimer(){
  while (HasData()) {
    if (!ValidData()) {
      continue;
    }
    loop_closing_->Update(
        current_key_scan_, current_key_frame_, current_key_gnss_);

    PublishData();
  }

  return true;
}
bool LoopClosingComponent::HasData() {
  std::unique_lock<std::mutex> lock(scan_list_mutex_);
  std::unique_lock<std::mutex> lock_(frame_list_mutex_);
  std::unique_lock<std::mutex> lock_1(gnss_list_mutex_);
  if (key_scan_buff_.size() == 0 || key_frame_buff_.size() == 0 ||
      key_gnss_buff_.size() == 0) {
    return false;
  }
  return true;
}

bool LoopClosingComponent::ValidData() {
  std::unique_lock<std::mutex> lock(scan_list_mutex_);
  std::unique_lock<std::mutex> lock_(frame_list_mutex_);
  std::unique_lock<std::mutex> lock_1(gnss_list_mutex_);


  current_key_scan_ = key_scan_buff_.front();
  current_key_frame_= key_frame_buff_.front();
  current_key_gnss_ = key_gnss_buff_.front();

  const double diff_frame_time =
      current_key_frame_.time_ - current_key_scan_.time_;

  const double half_sampling_time = 0.05;  // LiDAR is 10hz
  if (diff_frame_time < -half_sampling_time) {
    key_frame_buff_.pop_front();
    return false;
  }

  if (diff_frame_time > half_sampling_time) {
    key_scan_buff_.pop_front();
    return false;
  }

  key_scan_buff_.pop_front();
  key_frame_buff_.pop_front();
  key_gnss_buff_.pop_front();

  return true;
}

void LoopClosingComponent::LidarMsgTransfer(
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

bool LoopClosingComponent::PublishData() {
  if (loop_closing_->has_new_loop_pose())
  {
    loop_closing_publisher_->Write(loop_closing_->current_loop_pose());
  }
  return true;
}
}  // namespace localization
}  // namespace apollo
