#pragma once

#include <memory>
#include <thread>

#include <Eigen/Dense>
#include <pangolin/pangolin.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "lidar_localization/models/cloud_filter/box_filter.h"
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.h"
#include "lidar_localization/models/cloud_filter/no_filter.h"
#include "lidar_localization/models/cloud_filter/voxel_filter.h"
#include "lidar_localization/models/kalman_filter/error_state_kalman_filter.h"
#include "lidar_localization/models/kalman_filter/kalman_filter.h"
#include "lidar_localization/sensor_data/cloud_data.h"
#include "lidar_localization/sensor_data/pose_data.h"
#include "yaml-cpp/yaml.h"

#include "cyber/cyber.h"

using namespace lidar_localization;

namespace apollo {
namespace localization {
class Combination {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PoseData lidar_pose;
  PoseData gnss_pose;
  PoseData fusion_pose;
  CloudData lidar_frame;
};

class PangolinViewer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
 PangolinViewer(const YAML::Node& config_node);
 PangolinViewer();

 ~PangolinViewer();

 void Run();

 void SendInfo(
     std::deque<Combination, Eigen::aligned_allocator<Combination>>& input);

 bool LoadGlobalMap(const YAML::Node& config_node);

private:
 std::deque<Combination, Eigen::aligned_allocator<Combination>> cur_draw;
 std::deque<Combination, Eigen::aligned_allocator<Combination>> input_list;
 std::mutex info_list_mutex_;
 std::thread render_loop;
 std::string window_name = "trajectory_display";

 CloudData::CloudTypePtr global_map_ptr_ = nullptr;
};

}  // namespace localization
}  // namespace apollo
