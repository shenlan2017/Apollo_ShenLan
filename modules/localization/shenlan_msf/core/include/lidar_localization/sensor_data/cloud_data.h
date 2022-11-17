/*
 * @Description:
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:17:49
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-23 22:00:57
 */

#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_H_
#define LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_H_

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_localization {
class CloudData {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  using PointType = pcl::PointXYZ;
  using CloudType = pcl::PointCloud<PointType>;
  using CloudTypePtr = CloudType::Ptr;

  CloudData()
      : cloud_ptr_(new CloudType()) {}

  double time_{0.0};
  CloudTypePtr cloud_ptr_{};
};
}  // namespace lidar_localization

#endif
