#pragma once

#include <iomanip>
#include <memory>
#include <string>
#include <vector>

#include <pangolin/pangolin.h>
#include <pcl/filters/voxel_grid.h>

#include "glog/logging.h"

#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/shenlan_config.pb.h"
#include "modules/localization/shenlan_msf/visualization/pangolin_visualizer.h"

using namespace lidar_localization;

namespace apollo {
namespace localization {

class PangolinVisualizerComponent final
    : public cyber::Component<drivers::PointCloud> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  PangolinVisualizerComponent();

  ~PangolinVisualizerComponent(){
    viewer.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::cerr << "~PangolinVisualizerComponent Finished!" << std::endl;
  };

  bool Init() override;
  bool Proc(const std::shared_ptr<drivers::PointCloud> &msg) override;

 private:
  bool InitConfig();
  bool InitIO();

  void OnLidarLocalization(const std::shared_ptr<LocalizationEstimate> &message);
  void OnGNSSLocalization(const std::shared_ptr<LocalizationEstimate> &message);
  void OnFusionLocalization(const std::shared_ptr<LocalizationEstimate> &message);

  void LidarMsgTransfer(
    const std::shared_ptr<drivers::PointCloud> &msg, CloudData *lidar_frame);

 private:

  std::string lidar_extrinsic_file_;
  Eigen::Affine3d velodyne_extrinsic;

  std::string map_folder_;
  std::shared_ptr<PangolinViewer> viewer = nullptr;

  // std::mutex lidar_list_mutex_;
  // std::deque<CloudData> lidar_list_;
  // size_t lidar_list_max_size_ = 100000;

  std::mutex result_list_mutex_;
  std::deque<Combination, Eigen::aligned_allocator<Combination>> result_list_;
  size_t result_list_max_size_ = 100000;

  std::mutex lidar_pose_list_mutex_;
  std::deque<PoseData, Eigen::aligned_allocator<PoseData>> lidar_pose_list_;
  size_t lidar_pose_list_max_size_ = 10000000;

  std::mutex gnss_pose_list_mutex_;
  std::deque<PoseData, Eigen::aligned_allocator<PoseData>> gnss_pose_list_;
  size_t gnss_pose_list_max_size_ = 10000000;

  std::mutex fusion_pose_list_mutex_;
  std::deque<PoseData, Eigen::aligned_allocator<PoseData>> fusion_pose_list_;
  size_t fusion_pose_list_max_size_ = 10000000;

  std::shared_ptr<cyber::Reader<LocalizationEstimate>> lidar_local_listener_ =
      nullptr;
  std::string lidar_local_topic_ = "";

  std::shared_ptr<cyber::Reader<LocalizationEstimate>> gnss_local_listener_ =
      nullptr;
  std::string gnss_local_topic_ = "";

  std::shared_ptr<cyber::Reader<LocalizationEstimate>> fusion_local_listener_ =
      nullptr;
  std::string fusion_local_topic_ = "";
};

CYBER_REGISTER_COMPONENT(PangolinVisualizerComponent);
}  // namespace localization
}  // namespace apollo
