#pragma once

#include <deque>
#include <fstream>

#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
// #include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include "lidar_localization/models/cloud_filter/cloud_filter_interface.h"
#include "lidar_localization/models/cloud_filter/no_filter.h"
#include "lidar_localization/models/cloud_filter/voxel_filter.h"
#include "lidar_localization/models/registration/ndt_registration.h"
#include "lidar_localization/models/registration/point2plane_icp.h"
#include "lidar_localization/models/registration/registration_interface.h"
#include "lidar_localization/sensor_data/cloud_data.h"

namespace apollo {
namespace localization {

using namespace ::lidar_localization;

class FrontEnd {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Frame {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    CloudData cloud_data;
  };

  FrontEnd();

  bool Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose);

  bool SetInitPose(const Eigen::Matrix4f& init_pose);

  bool is_degeneracy() const { return is_degeneracy_; }

 private:
  bool InitWithConfig();
  bool InitParam(const YAML::Node& config_node);
  bool InitRegistration(
      std::shared_ptr<RegistrationInterface>& registration_ptr,
      const YAML::Node& config_node);
  bool InitFilter(std::string filter_user,
                  std::shared_ptr<CloudFilterInterface>& filter_ptr,
                  const YAML::Node& config_node);
  bool UpdateWithNewFrame(const Frame& new_key_frame);

  std::string data_path_{};

  // scan filter:
  std::shared_ptr<CloudFilterInterface> frame_filter_ptr_{};
  // local map filter:
  std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_{};
  // point cloud registrator:
  std::shared_ptr<RegistrationInterface> registration_ptr_{};

  std::deque<Frame, Eigen::aligned_allocator<Frame>> local_map_frames_;

  CloudData::CloudTypePtr local_map_ptr_;
  Frame current_frame_;

  Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
  bool is_degeneracy_{false};

  float key_frame_distance_{2.0};  // m
  int local_frame_num_{20};
};
}  // namespace localization
}  // namespace apollo
