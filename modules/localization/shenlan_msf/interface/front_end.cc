#include "modules/localization/shenlan_msf/interface/front_end.h"

namespace apollo {
namespace localization {
FrontEnd::FrontEnd() : local_map_ptr_(new CloudData::CloudType()) {
  InitWithConfig();
}

bool FrontEnd::InitWithConfig() {
  const std::string config_file_path =
      "/apollo/modules/localization/shenlan_msf/config/mapping/front_end.yaml";
  YAML::Node config_node = YAML::LoadFile(config_file_path);

  std::cout << "-----------------Init Lidar Frontend-------------------\n";
  InitParam(config_node);
  InitRegistration(registration_ptr_, config_node);
  InitFilter("local_map", local_map_filter_ptr_, config_node);
  InitFilter("frame", frame_filter_ptr_, config_node);

  return true;
}

bool FrontEnd::InitParam(const YAML::Node& config_node) {
  key_frame_distance_ = config_node["key_frame_distance"].as<float>();
  local_frame_num_ = config_node["local_frame_num"].as<int>();

  return true;
}

bool FrontEnd::InitRegistration(
    std::shared_ptr<RegistrationInterface>& registration_ptr,
    const YAML::Node& config_node)
{
  const std::string registration_method =
      config_node["registration_method"].as<std::string>();
  std::cout << "\tPoint Cloud Registration Method: " << registration_method << std::endl;

  if (registration_method == "NDT") {
    registration_ptr = std::shared_ptr<NDTRegistration>(new NDTRegistration(config_node[registration_method]));
  } else if (registration_method == "Point2Plane") {
    registration_ptr = std::shared_ptr<Point2PlaneICP>(new Point2PlaneICP(config_node[registration_method]));
  } else {
    std::cerr << "Registration method " << registration_method << std::endl;
    return false;
  }

  return true;
}

bool FrontEnd::InitFilter(std::string filter_user,
                          std::shared_ptr<CloudFilterInterface>& filter_ptr,
                          const YAML::Node& config_node) {
  const std::string filter_mothod =
      config_node[filter_user + "_filter"].as<std::string>();
  std::cout << "\tFilter Method for " << filter_user << ": " << filter_mothod
            << std::endl;

  if (filter_mothod == "voxel_filter") {
    filter_ptr = std::shared_ptr<VoxelFilter>(new VoxelFilter(config_node[filter_mothod][filter_user]));
  } else if (filter_mothod == "no_filter") {
    filter_ptr = std::shared_ptr<NoFilter>(new NoFilter());
  } else {
    std::cerr << "Filter method " << filter_mothod << " for " << filter_user
               << " NOT FOUND!\n";
    return false;
  }

  return true;
}

bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose) {
  init_pose_ = init_pose;
  return true;
}

bool FrontEnd::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose) {
  static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
  static Eigen::Matrix4f last_pose = init_pose_;
  static Eigen::Matrix4f predict_pose = init_pose_;
  static Eigen::Matrix4f last_key_frame_pose = init_pose_;
  //
  // set up current scan:
  //
  current_frame_.cloud_data.time_ = cloud_data.time_;
  // a. remove invalid measurements:
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr_,
                               *current_frame_.cloud_data.cloud_ptr_, indices);
  // b. apply filter to current scan:
  CloudData::CloudTypePtr filtered_cloud_ptr(new CloudData::CloudType());
  frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr_,
                            filtered_cloud_ptr);
  // set up local map:
  if (local_map_frames_.size() == 0) {
    current_frame_.pose = init_pose_;
    UpdateWithNewFrame(current_frame_);
    cloud_pose = current_frame_.pose;
    return true;
  }
  // update lidar odometry using scan match result:
  CloudData::CloudTypePtr result_cloud_ptr(new CloudData::CloudType());
  registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose,
                               result_cloud_ptr, current_frame_.pose);
  cloud_pose = current_frame_.pose;
  is_degeneracy_ = registration_ptr_->IsDegeneracy();
  if (is_degeneracy_) {
    std::cerr << "lidar odometry is degeneracy.\n";
  }
  // update init pose for next scan match:
  step_pose = last_pose.inverse() * current_frame_.pose;
  predict_pose = current_frame_.pose * step_pose;
  last_pose = current_frame_.pose;

  // shall the key frame set be updated:
  if (fabs(last_key_frame_pose(0, 3) - current_frame_.pose(0, 3)) +
          fabs(last_key_frame_pose(1, 3) - current_frame_.pose(1, 3)) +
          fabs(last_key_frame_pose(2, 3) - current_frame_.pose(2, 3)) >
      key_frame_distance_) {
    UpdateWithNewFrame(current_frame_);
    last_key_frame_pose = current_frame_.pose;
  }
  return true;
}

bool FrontEnd::UpdateWithNewFrame(const Frame& new_key_frame) {
  Frame key_frame;
  key_frame.pose = new_key_frame.pose;
  pcl::transformPointCloud(*new_key_frame.cloud_data.cloud_ptr_,
                           *key_frame.cloud_data.cloud_ptr_,
                           new_key_frame.pose);
  local_map_frames_.push_back(key_frame);

  while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_)) {
    local_map_frames_.pop_front();
  }

  // transform all local frame measurements to map frame
  // to create local map:
  local_map_ptr_.reset(new CloudData::CloudType());
  for (size_t i = 0; i < local_map_frames_.size(); ++i) {
    *local_map_ptr_ += *local_map_frames_.at(i).cloud_data.cloud_ptr_;
  }

  // scan-to-map matching:
  // set target as local map:
  if (local_map_frames_.size() < 10) {
    registration_ptr_->SetInputTarget(local_map_ptr_);
  } else {
    CloudData::CloudTypePtr filtered_local_map_ptr(new CloudData::CloudType());
    local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);
    registration_ptr_->SetInputTarget(filtered_local_map_ptr);
  }

  return true;
}
}  // namespace localization
}  // namespace apollo
