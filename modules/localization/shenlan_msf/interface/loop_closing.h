#pragma once

#include <deque>
#include <fstream>
#include <string>

#include <Eigen/Dense>
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include "lidar_localization/sensor_data/cloud_data.h"
#include "lidar_localization/sensor_data/key_frame.h"
#include "lidar_localization/sensor_data/loop_pose.h"
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.h"
#include "lidar_localization/models/cloud_filter/no_filter.h"
#include "lidar_localization/models/cloud_filter/voxel_filter.h"
#include "lidar_localization/models/registration/registration_interface.h"
#include "lidar_localization/models/registration/ndt_registration.h"
#include "lidar_localization/models/registration/point2plane_icp.h"

using namespace ::lidar_localization;

namespace apollo {
namespace localization {
class LoopClosing {
 public:
  LoopClosing(const std::string& work_space_path);


  bool Update(const CloudData& key_scan,
              const KeyFrame& key_frame,
              const KeyFrame& key_gnss);

  // bool HasNewLoopPose();
  // LoopPose& GetCurrentLoopPose();
  bool has_new_loop_pose() const { return has_new_loop_pose_; }

  const LoopPose& current_loop_pose() const { return current_loop_pose_; }

 private:
  bool InitWithConfig(const std::string& work_space_path);

  bool InitParam(const YAML::Node& config_node);

  bool InitDataPath(const YAML::Node& config_node,const std::string& work_space_path);

  bool InitFilter(const YAML::Node& config_node,
                  const std::string filter_user,
                  std::shared_ptr<CloudFilterInterface>& filter_ptr);

  bool InitLoopClosure(const YAML::Node& config_node);

  bool InitRegistration(
      const YAML::Node& config_node,
      std::shared_ptr<RegistrationInterface>& registration_ptr);

  bool DetectNearestKeyFrame(int& key_frame_index);

  bool CloudRegistration(const int key_frame_index);

  bool DetectNearestKeyFrameBaseOnRadius(int& key_frame_index);

  bool CloudRegistrationBaseOnRadius(const int key_frame_index);

  bool JointMap(const int key_frame_index,
                CloudData::CloudTypePtr& map_cloud_ptr,
                Eigen::Matrix4f& map_pose);

  bool JointScan(CloudData::CloudTypePtr& scan_cloud_ptr,
                 Eigen::Matrix4f& scan_pose);

  bool Registration(CloudData::CloudTypePtr& map_cloud_ptr,
                    CloudData::CloudTypePtr& scan_cloud_ptr,
                    Eigen::Matrix4f& scan_pose,
                    Eigen::Matrix4f& result_pose);

  std::string key_frames_path_{};

  int extend_frame_num_{3};
  int loop_step_{2};
  int diff_num_{10};
  float detect_area_{10.0};  // loop closure detect area
  float fitness_score_limit_{0.2};
  float key_frame_distance_{2.0};

  std::shared_ptr<CloudFilterInterface> scan_filter_ptr_;
  std::shared_ptr<CloudFilterInterface> map_filter_ptr_;
  std::shared_ptr<RegistrationInterface> registration_ptr_;

  std::deque<KeyFrame,Eigen::aligned_allocator<KeyFrame>> all_key_frames_;
  std::deque<KeyFrame,Eigen::aligned_allocator<KeyFrame>> all_key_gnss_;

  LoopPose current_loop_pose_;
  bool has_new_loop_pose_{false};
  bool use_gnss_{true};  // if true, detect loop clousure base on gnss pose
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace localization
}  // namespace apollo
