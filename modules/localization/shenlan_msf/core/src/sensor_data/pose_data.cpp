/*
 * @Description:
 * @Author: Ren Qian
 * @Date: 2020-02-28 18:50:16
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-11-02 09:59:10
 */
#include "lidar_localization/sensor_data/pose_data.h"
#include <iostream>
#include <iomanip>

namespace lidar_localization {
bool PoseData::SyncData(
    double sync_time,
    std::deque<PoseData, Eigen::aligned_allocator<PoseData>>& UnsyncedData,
    std::deque<PoseData, Eigen::aligned_allocator<PoseData>>& SyncedData) {
  while (UnsyncedData.size() >= 2) {
    // UnsyncedData.front().time_ should be <= sync_time:
    if (UnsyncedData.front().time_ > sync_time) {
      return false;
    }
    // sync_time should be <= UnsyncedData.at(1).time_:
    if (UnsyncedData.at(1).time_ < sync_time) {
      UnsyncedData.pop_front();
      continue;
    }

    const double two_sampling_time = 2;  // GNSS is 1hz
    if (sync_time - UnsyncedData.front().time_ > two_sampling_time) {
      UnsyncedData.pop_front();
      return false;
    }
    if (UnsyncedData.at(1).time_ - sync_time > two_sampling_time) {
      UnsyncedData.pop_front();
      return false;
    }
    break;
  }
  if (UnsyncedData.size() < 2) {
    return false;
  }

  PoseData front_data = UnsyncedData.at(0);
  PoseData back_data = UnsyncedData.at(1);
  PoseData synced_data;

  const double front_scale =
      (back_data.time_ - sync_time) / (back_data.time_ - front_data.time_);
  const double back_scale =
      (sync_time - front_data.time_) / (back_data.time_ - front_data.time_);
  synced_data.time_ = sync_time;
  // posi
  const Eigen::Vector3f front_t = front_data.pose_.block<3, 1>(0, 3);
  const Eigen::Vector3f back_t = back_data.pose_.block<3, 1>(0, 3);
  Eigen::Vector3f synced_t = front_scale * front_t + back_scale * back_t;
  // vel
  Eigen::Vector3f synced_v =
      front_scale * front_data.vel_.v + back_scale * back_data.vel_.v;
  // ori
  const double s =
      (sync_time - front_data.time_) / (back_data.time_ - front_data.time_);
  Eigen::Quaternionf front_q =
      Eigen::Quaternionf(front_data.pose_.block<3, 3>(0, 0));
  Eigen::Quaternionf back_q =
      Eigen::Quaternionf(back_data.pose_.block<3, 3>(0, 0));
  Eigen::Quaternionf synced_q = front_q.slerp(s, back_q);

  synced_data.pose_.block<3, 1>(0, 3) = synced_t;
  synced_data.pose_.block<3, 3>(0, 0) = synced_q.toRotationMatrix();
  synced_data.vel_.v = synced_v;
  synced_data.cov_ = front_data.cov_;

  SyncedData.push_back(synced_data);

  return true;
}

bool PoseData::SyncData(
    double sync_time,
    std::deque<PoseData, Eigen::aligned_allocator<PoseData>>& UnsyncedData,
    PoseData& SyncedData) {
  while (UnsyncedData.size() >= 2) {
    // UnsyncedData.front().time_ should be <= sync_time:
    if (UnsyncedData.front().time_ > sync_time) {
      return false;
    }
    // sync_time should be <= UnsyncedData.at(1).time_:
    if (UnsyncedData.at(1).time_ < sync_time) {
      UnsyncedData.pop_front();
      continue;
    }

    const double two_sampling_time = 2;  // GNSS is 1hz
    if (sync_time - UnsyncedData.front().time_ > two_sampling_time) {
      UnsyncedData.pop_front();
      return false;
    }
    if (UnsyncedData.at(1).time_ - sync_time > two_sampling_time) {
      UnsyncedData.pop_front();
      return false;
    }
    break;
  }
  if (UnsyncedData.size() < 2) {
    return false;
  }

  PoseData front_data = UnsyncedData.at(0);
  PoseData back_data = UnsyncedData.at(1);
  PoseData synced_data;

  const double front_scale =
      (back_data.time_ - sync_time) / (back_data.time_ - front_data.time_);
  const double back_scale =
      (sync_time - front_data.time_) / (back_data.time_ - front_data.time_);
  synced_data.time_ = sync_time;
  // posi
  const Eigen::Vector3f front_t = front_data.pose_.block<3, 1>(0, 3);
  const Eigen::Vector3f back_t = back_data.pose_.block<3, 1>(0, 3);
  Eigen::Vector3f synced_t = front_scale * front_t + back_scale * back_t;
  // vel
  Eigen::Vector3f synced_v =
      front_scale * front_data.vel_.v + back_scale * back_data.vel_.v;
  // ori
  const double s =
      (sync_time - front_data.time_) / (back_data.time_ - front_data.time_);
  Eigen::Quaternionf front_q =
      Eigen::Quaternionf(front_data.pose_.block<3, 3>(0, 0));
  Eigen::Quaternionf back_q =
      Eigen::Quaternionf(back_data.pose_.block<3, 3>(0, 0));
  Eigen::Quaternionf synced_q = front_q.slerp(s, back_q).normalized();

  synced_data.pose_.block<3, 1>(0, 3) = synced_t;
  synced_data.pose_.block<3, 3>(0, 0) = synced_q.toRotationMatrix();
  synced_data.vel_.v = synced_v;
  synced_data.cov_ = front_data.cov_;

  SyncedData = synced_data;
  return true;
}

}  // namespace lidar_localization
