/*
 * @Description: 从点云中截取一个立方体部分
 * @Author: Ren Qian
 * @Date: 2019-03-12 23:38:31
 * @LastEditors: ZiJieChen
 * @LastEditTime: 2022-09-22 21:04:38
 */
#include "lidar_localization/models/cloud_filter/box_filter.h"
#include "pcl/filters/crop_hull.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/io/pcd_io.h"
#include "pcl/kdtree/impl/kdtree_flann.hpp"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/surface/concave_hull.h"

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
  
  CloudData::CloudTypePtr local_edge_ptr(new CloudData::CloudType);
  for(size_t i=0; i<2; i++){
    for(size_t j=0; j<2; j++){
      CloudData::PointType p;
      p.x = edge_.at(i);
      p.y = edge_.at(2+j);
      p.z = 0;
      local_edge_ptr->emplace_back(p);
    }
  }
  pcl::ConvexHull<CloudData::PointType> hull;
  hull.setInputCloud(local_edge_ptr);
  hull.setDimension(2);
  std::vector<int> filter_indices;
  std::vector<pcl::Vertices> polygons;
  CloudData::CloudTypePtr surface_hull(new CloudData::CloudType);
  hull.reconstruct(*surface_hull, polygons);
  pcl::CropHull<CloudData::PointType> bb_filter;
  bb_filter.setDim(2);
  bb_filter.setInputCloud(input_cloud_ptr);
  bb_filter.setHullIndices(polygons);
  bb_filter.setHullCloud(surface_hull);
  bb_filter.filter(filter_indices);

  std::shared_ptr<std::vector<int>> index_ptr =
      std::make_shared<std::vector<int>>(filter_indices);

  pcl::ExtractIndices<CloudData::PointType> extract;
  extract.setInputCloud(input_cloud_ptr);
  extract.setIndices(index_ptr);
  extract.setNegative(false);
  extract.filter(*output_cloud_ptr);
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