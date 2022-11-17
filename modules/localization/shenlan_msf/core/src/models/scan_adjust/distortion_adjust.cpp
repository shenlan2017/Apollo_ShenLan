/*
 * @Description: 点云畸变补偿
 * @Author: Ren Qian
 * @Date: 2020-02-25 14:39:00
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 21:36:00
 */
#include "lidar_localization/models/scan_adjust/distortion_adjust.h"

namespace lidar_localization {
void DistortionAdjust::SetMotionInfo(const VelocityData& velocity_data,
                                     const float scan_period) {
  scan_period_ = scan_period;
  velocity_ << velocity_data.linear_velocity_.x,
      velocity_data.linear_velocity_.y, velocity_data.linear_velocity_.z;
  angular_rate_ << velocity_data.angular_velocity_.x,
      velocity_data.angular_velocity_.y, velocity_data.angular_velocity_.z;
}

// old version
// bool DistortionAdjust::AdjustCloud(
//     const CloudData::CloudTypePtr& input_cloud_ptr,
//     CloudData::CloudTypePtr& output_cloud_ptr) {
//   CloudData::CloudTypePtr origin_cloud_ptr(
//       new CloudData::CloudType(*input_cloud_ptr));
//   output_cloud_ptr.reset(new CloudData::CloudType());

//   float orientation_space = 2.0 * M_PI;
//   float delete_space = 5.0 * M_PI / 180.0;
//   float start_orientation =
//       atan2(origin_cloud_ptr->points[0].y, origin_cloud_ptr->points[0].x);

//   Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3f::UnitZ());
//   Eigen::Matrix3f rotate_matrix = t_V.matrix();
//   Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
//   transform_matrix.block<3, 3>(0, 0) = rotate_matrix.inverse();
//   pcl::transformPointCloud(
//       *origin_cloud_ptr, *origin_cloud_ptr, transform_matrix);

//   velocity_ = rotate_matrix * velocity_;
//   angular_rate_ = rotate_matrix * angular_rate_;

//   for (size_t point_index = 1; point_index < origin_cloud_ptr->points.size();
//        ++point_index) {
//     float orientation = atan2(origin_cloud_ptr->points[point_index].y,
//                               origin_cloud_ptr->points[point_index].x);
//     if (orientation < 0.0)
//       orientation += 2.0 * M_PI;

//     if (orientation < delete_space || 2.0 * M_PI - orientation <
//     delete_space)
//       continue;

//     float real_time = fabs(orientation) / orientation_space * scan_period_ -
//                       scan_period_ / 2.0;

//     Eigen::Vector3f origin_point(origin_cloud_ptr->points[point_index].x,
//                                  origin_cloud_ptr->points[point_index].y,
//                                  origin_cloud_ptr->points[point_index].z);

//     Eigen::Matrix3f current_matrix = UpdateMatrix(real_time);
//     Eigen::Vector3f rotated_point = current_matrix * origin_point;
//     Eigen::Vector3f adjusted_point = rotated_point + velocity_ * real_time;
//     CloudData::PointType point;
//     point.x = adjusted_point(0);
//     point.y = adjusted_point(1);
//     point.z = adjusted_point(2);
//     output_cloud_ptr->points.push_back(point);
//   }

//   pcl::transformPointCloud(
//       *output_cloud_ptr, *output_cloud_ptr, transform_matrix.inverse());
//   return true;
// }

// New version modified by ZiJieChen
// Old version has something wrong in velocity_, angular_rate_ and orientation
bool DistortionAdjust::AdjustCloud(
    const CloudData::CloudTypePtr& input_cloud_ptr,
    CloudData::CloudTypePtr& output_cloud_ptr) {
  CloudData::CloudTypePtr origin_cloud_ptr(
      new CloudData::CloudType(*input_cloud_ptr));
  output_cloud_ptr.reset(new CloudData::CloudType());

  const float orientation_space = 2.0 * M_PI;
  const float delete_space = 5.0 * M_PI / 180.0;
  const float start_orientation =
      atan2(origin_cloud_ptr->points[0].y, origin_cloud_ptr->points[0].x);

  const Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3f::UnitZ());
  const Eigen::Matrix3f rotate_matrix = t_V.matrix();
  Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
  transform_matrix.block<3, 3>(0, 0) = rotate_matrix.inverse();
  pcl::transformPointCloud(
      *origin_cloud_ptr, *origin_cloud_ptr, transform_matrix);

  velocity_ = rotate_matrix.transpose() * velocity_;
  angular_rate_ = rotate_matrix.transpose() * angular_rate_;

  for (size_t point_index = 1; point_index < origin_cloud_ptr->points.size();
       ++point_index) {
    float orientation = -atan2(origin_cloud_ptr->points[point_index].y,
                               origin_cloud_ptr->points[point_index].x);
    if (orientation < 0.0)
      orientation += 2.0 * M_PI;

    if (orientation < delete_space || 2.0 * M_PI - orientation < delete_space)
      continue;

    const float real_time =
        fabs(orientation) / orientation_space * scan_period_ -
        scan_period_ / 2.0;

    const Eigen::Vector3f origin_point(origin_cloud_ptr->points[point_index].x,
                                       origin_cloud_ptr->points[point_index].y,
                                       origin_cloud_ptr->points[point_index].z);

    const Eigen::Matrix3f current_matrix = UpdateMatrix(real_time);
    const Eigen::Vector3f rotated_point = current_matrix * origin_point;
    const Eigen::Vector3f adjusted_point =
        rotated_point + velocity_ * real_time;
    CloudData::PointType point;
    point.x = adjusted_point(0);
    point.y = adjusted_point(1);
    point.z = adjusted_point(2);
    output_cloud_ptr->points.push_back(point);
  }

  pcl::transformPointCloud(
      *output_cloud_ptr, *output_cloud_ptr, transform_matrix.inverse());
  return true;
}
}  // namespace lidar_localization