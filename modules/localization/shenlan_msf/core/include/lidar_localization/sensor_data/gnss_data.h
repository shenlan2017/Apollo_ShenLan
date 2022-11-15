/*
 * @Description:
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:25:13
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 23:20:04
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_H_
#define LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_H_

#include <deque>
#include <ostream>
#include <Eigen/Core>

#include <GeographicLib/LocalCartesian.hpp>

namespace lidar_localization {
class GNSSData {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  void InitOriginPosition();

  void UpdateXYZ();

  void Reverse(const double local_E,
               const double local_N,
               const double local_U,
               double& lat,
               double& lon,
               double& alt);

  static bool SyncData(
      const double sync_time,
      std::deque<GNSSData, Eigen::aligned_allocator<GNSSData>>& UnsyncedData,
      std::deque<GNSSData, Eigen::aligned_allocator<GNSSData>>& SyncedData);

  double time_{0.0};
  double latitude_{0.0};
  double longitude_{0.0};
  double altitude_{0.0};
  double local_E_{0.0};
  double local_N_{0.0};
  double local_U_{0.0};
  int status_{0};
  int service_{0};

  static double origin_longitude_;
  static double origin_latitude_;
  static double origin_altitude_;

 private:
  static GeographicLib::LocalCartesian geo_converter_;
  static bool origin_position_inited_;
};
}  // namespace lidar_localization
#endif
