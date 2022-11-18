#pragma once
#include <string>

#include <Eigen/Dense>
#include "glog/logging.h"
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>
#include "lidar_localization/models/cloud_filter/voxel_filter.h"
#include "lidar_localization/sensor_data/cloud_data.h"
#include "lidar_localization/sensor_data/key_frame.h"
#include "lidar_localization/sensor_data/pose_data.h"
#include "lidar_localization/tools/file_manager.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace apollo {
namespace localization {

using namespace ::lidar_localization;

class PostProcessing {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  PostProcessing();

  bool UpdateWithOptimizedKeyFrames(std::deque<KeyFrame, Eigen::aligned_allocator<KeyFrame>>& optimized_key_frames);

  bool UpdateWithNewKeyFrame(const PoseData& transformed_data,
                             const CloudData& cloud_data,
                             std::deque<KeyFrame, Eigen::aligned_allocator<KeyFrame>>& new_key_frames);

  bool SaveMap();

  const Eigen::Matrix4f& GetCurrentPose() const;

  const CloudData::CloudTypePtr& GetCurrentScan();

  bool GetLocalMap(CloudData::CloudTypePtr& local_map_ptr);

  bool GetGlobalMap(CloudData::CloudTypePtr& local_map_ptr);

  bool has_new_local_map() const { return has_new_local_map_; }
  bool has_new_global_map() const { return has_new_global_map_; }

 private:
  bool InitWithConfig();
  bool InitParam(const YAML::Node& config_node);
  bool InitDataPath(const YAML::Node& config_node);
  bool InitFilter(std::string filter_user,
                  std::shared_ptr<CloudFilterInterface>& filter_ptr,
                  const YAML::Node& config_node);

  bool OptimizeKeyFrames();
  bool JointGlobalMap(CloudData::CloudTypePtr& global_map_ptr);
  bool JointLocalMap(CloudData::CloudTypePtr& local_map_ptr);
  bool JointCloudMap(const std::deque<KeyFrame, Eigen::aligned_allocator<KeyFrame>>& key_frames, CloudData::CloudTypePtr& map_cloud_ptr);

  std::string data_path_{};
  int local_frame_num_{20};

  std::string key_frames_path_{};
  std::string map_path_{};

  std::shared_ptr<CloudFilterInterface> frame_filter_ptr_{};
  std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_{};
  std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_{};

  Eigen::Matrix4f pose_to_optimize_ = Eigen::Matrix4f::Identity();
  PoseData optimized_odom_;
  CloudData optimized_cloud_;
  std::deque<KeyFrame, Eigen::aligned_allocator<KeyFrame>>
      optimized_key_frames_;
  std::deque<KeyFrame, Eigen::aligned_allocator<KeyFrame>> all_key_frames_;

  bool has_new_global_map_{false};
  bool has_new_local_map_{false};
};
}  // namespace localization
}
