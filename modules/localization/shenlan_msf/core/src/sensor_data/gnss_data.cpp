/*
 * @Description:
 * @Author: Ren Qian
 * @Date: 2020-02-06 20:42:23
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-27 09:39:16
 */
#include "lidar_localization/sensor_data/gnss_data.h"
#include <iostream>
//静态成员变量必须在类外初始化
double lidar_localization::GNSSData::origin_latitude_ = 0.0;
double lidar_localization::GNSSData::origin_longitude_ = 0.0;
double lidar_localization::GNSSData::origin_altitude_ = 0.0;
bool lidar_localization::GNSSData::origin_position_inited_ = false;
GeographicLib::LocalCartesian lidar_localization::GNSSData::geo_converter_;

namespace lidar_localization {
void GNSSData::InitOriginPosition() {
  geo_converter_.Reset(latitude_, longitude_, altitude_);

  origin_longitude_ = latitude_;
  origin_longitude_ = longitude_;
  origin_longitude_ = altitude_;

  origin_position_inited_ = true;
}

void GNSSData::UpdateXYZ() {
  if (!origin_position_inited_) {
    std::cout << "WARNING: GeoConverter is NOT initialized.\n";
  }

  geo_converter_.Forward(latitude_, longitude_, altitude_, local_E_, local_N_,
                         local_U_);
}

void GNSSData::Reverse(const double local_E, const double local_N,
                       const double local_U, double& lat, double& lon,
                       double& alt) {
  if (!origin_position_inited_) {
    std::cout << "WARNING: GeoConverter is NOT initialized.\n";
  }

  geo_converter_.Reverse(local_E_, local_N_, local_U_, lat, lon, alt);
}

bool GNSSData::SyncData(
    const double sync_time,
    std::deque<GNSSData, Eigen::aligned_allocator<GNSSData>>& UnsyncedData,
    std::deque<GNSSData, Eigen::aligned_allocator<GNSSData>>& SyncedData) {
  // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
  // 即找到与同步时间相邻的左右两个数据
  // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
  while (UnsyncedData.size() >= 2) {
    if (UnsyncedData.front().time_ > sync_time) return false;
    if (UnsyncedData.at(1).time_ < sync_time) {
      UnsyncedData.pop_front();
      continue;
    }
    if (sync_time - UnsyncedData.front().time_ > 0.2) {
      UnsyncedData.pop_front();
      return false;
    }
    if (UnsyncedData.at(1).time_ - sync_time > 0.2) {
      UnsyncedData.pop_front();
      return false;
    }
    break;
  }
  if (UnsyncedData.size() < 2) return false;

  GNSSData front_data = UnsyncedData.at(0);
  GNSSData back_data = UnsyncedData.at(1);
  GNSSData synced_data;

  const double front_scale =
      (back_data.time_ - sync_time) / (back_data.time_ - front_data.time_);
  const double back_scale =
      (sync_time - front_data.time_) / (back_data.time_ - front_data.time_);
  synced_data.time_ = sync_time;
  synced_data.status_ = back_data.status_;
  synced_data.longitude_ =
      front_data.longitude_ * front_scale + back_data.longitude_ * back_scale;
  synced_data.latitude_ =
      front_data.latitude_ * front_scale + back_data.latitude_ * back_scale;
  synced_data.altitude_ =
      front_data.altitude_ * front_scale + back_data.altitude_ * back_scale;
  synced_data.local_E_ =
      front_data.local_E_ * front_scale + back_data.local_E_ * back_scale;
  synced_data.local_N_ =
      front_data.local_N_ * front_scale + back_data.local_N_ * back_scale;
  synced_data.local_U_ =
      front_data.local_U_ * front_scale + back_data.local_U_ * back_scale;

  SyncedData.push_back(synced_data);

  return true;
}

}  // namespace lidar_localization
