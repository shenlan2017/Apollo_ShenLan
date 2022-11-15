/*
 * @Description: 从点云中截取一个立方体部分
 * @Author: Ren Qian
 * @Date: 2019-03-12 23:38:31
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 21:04:38
 */
#include "lidar_localization/models/cloud_filter/box_filter.h"

namespace lidar_localization {
BoxFilter::BoxFilter(const YAML::Node& node) {
  size_.resize(6);
  edge_.resize(6);
  origin_.resize(3);

  for (size_t i = 0; i < size_.size(); i++) {
    size_.at(i) = node["box_filter_size"][i].as<float>();
  }
  set_size(size_);
}

bool BoxFilter::Filter(const CloudData::CloudTypePtr& input_cloud_ptr,
                       CloudData::CloudTypePtr& output_cloud_ptr) {
  output_cloud_ptr->clear();
  pcl_box_filter_.setMin(
      Eigen::Vector4f(edge_.at(0), edge_.at(2), edge_.at(4), 1.0e-6));
  pcl_box_filter_.setMax(
      Eigen::Vector4f(edge_.at(1), edge_.at(3), edge_.at(5), 1.0e6));
  pcl_box_filter_.setInputCloud(input_cloud_ptr);
  pcl_box_filter_.filter(*output_cloud_ptr);

  return true;
}

void BoxFilter::set_size(const std::vector<float>& size) {
  size_ = size;
  std::cout << "Box Filter params:" << std::endl
            << "min_x: " << size.at(0) << ", "
            << "max_x: " << size.at(1) << ", "
            << "min_y: " << size.at(2) << ", "
            << "max_y: " << size.at(3) << ", "
            << "min_z: " << size.at(4) << ", "
            << "max_z: " << size.at(5) << std::endl
            << std::endl;

  CalculateEdge();
}

void BoxFilter::set_origin(const std::vector<float>& origin) {
  origin_ = origin;
  CalculateEdge();
}

void BoxFilter::CalculateEdge() {
  for (size_t i = 0; i < origin_.size(); ++i) {
    edge_.at(2 * i) = size_.at(2 * i) + origin_.at(i);
    edge_.at(2 * i + 1) = size_.at(2 * i + 1) + origin_.at(i);
  }
}

}  // namespace lidar_localization