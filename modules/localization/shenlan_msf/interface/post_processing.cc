#include "modules/localization/shenlan_msf/interface/post_processing.h"
namespace apollo {
namespace localization {

PostProcessing::PostProcessing() { InitWithConfig(); }

bool PostProcessing::InitWithConfig() {
  const std::string config_file_path =
      WORK_SPACE_PATH + "/config/mapping/viewer.yaml";
  YAML::Node config_node = YAML::LoadFile(config_file_path);

  std::cout << "-----------------Post-Processing Init-------------------"
            << std::endl;
  InitParam(config_node);
  InitDataPath(config_node);
  InitFilter("frame", frame_filter_ptr_, config_node);
  InitFilter("local_map", local_map_filter_ptr_, config_node);
  InitFilter("global_map", global_map_filter_ptr_, config_node);

  return true;
}

bool PostProcessing::InitParam(const YAML::Node& config_node) {
  local_frame_num_ = config_node["local_frame_num"].as<int>();
  return true;
}

bool PostProcessing::InitDataPath(const YAML::Node& config_node) {
  std::string data_path = config_node["data_path"].as<std::string>();
  if (data_path == "./") {
    data_path = WORK_SPACE_PATH;
  }

  key_frames_path_ = data_path + "/slam_data/key_frames";
  map_path_ = data_path + "/slam_data/map";

  if (!FileManager::CreateDirectory(map_path_, "点云地图文件")){
    std::cerr << "Fail to CreateDirectory \n";
    return false;
  }

  return true;
}

bool PostProcessing::InitFilter(
    std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr,
    const YAML::Node& config_node) {
  const std::string filter_mothod =
      config_node[filter_user + "_filter"].as<std::string>();
  std::cout << "显示模块" << filter_user << "选择的滤波方法为："
            << filter_mothod << std::endl;

  if (filter_mothod == "voxel_filter") {
    filter_ptr = std::shared_ptr<VoxelFilter>(
        new VoxelFilter(config_node[filter_mothod][filter_user]));
  } else {
    LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_mothod
               << " 相对应的滤波方法!";
    return false;
  }

  return true;
}

bool PostProcessing::UpdateWithOptimizedKeyFrames(
    std::deque<KeyFrame, Eigen::aligned_allocator<KeyFrame>>& optimized_key_frames) {
  has_new_global_map_ = false;

  if (optimized_key_frames.size() > 0) {
    optimized_key_frames_ = optimized_key_frames;
    optimized_key_frames.clear();
    OptimizeKeyFrames();
    has_new_global_map_ = true;
  }

  return has_new_global_map_;
}

bool PostProcessing::UpdateWithNewKeyFrame(
    const PoseData& transformed_data, const CloudData& cloud_data,
    std::deque<KeyFrame, Eigen::aligned_allocator<KeyFrame>>& new_key_frames) {
  has_new_local_map_ = false;

  if (new_key_frames.size() > 0) {
    KeyFrame key_frame;
    for (size_t i = 0; i < new_key_frames.size(); ++i) {
      key_frame = new_key_frames.at(i);
      key_frame.pose_ = pose_to_optimize_ * key_frame.pose_;
      all_key_frames_.push_back(key_frame);
    }
    new_key_frames.clear();
    has_new_local_map_ = true;
  }

  optimized_odom_ = transformed_data;
  optimized_odom_.pose_ = pose_to_optimize_ * optimized_odom_.pose_;
  // optimized_odom_.pose_ = optimized_odom_.pose_ * pose_to_optimize_;

  optimized_cloud_ = cloud_data;
  pcl::transformPointCloud(*cloud_data.cloud_ptr_, *optimized_cloud_.cloud_ptr_,
                           optimized_odom_.pose_);

  return true;
}

bool PostProcessing::OptimizeKeyFrames() {
  size_t optimized_index = 0;
  size_t all_index = 0;
  while (optimized_index < optimized_key_frames_.size() &&
         all_index < all_key_frames_.size()) {
    if (optimized_key_frames_.at(optimized_index).index_ <
        all_key_frames_.at(all_index).index_) {
      optimized_index++;
    } else if (optimized_key_frames_.at(optimized_index).index_ >
               all_key_frames_.at(all_index).index_) {
      all_index++;
    } else {
      pose_to_optimize_ = optimized_key_frames_.at(optimized_index).pose_ *
                          all_key_frames_.at(all_index).pose_.inverse();
      // pose_to_optimize_ = all_key_frames_.at(all_index).pose_.inverse() *
      //                     optimized_key_frames_.at(optimized_index).pose_;
      all_key_frames_.at(all_index) = optimized_key_frames_.at(optimized_index);
      optimized_index++;
      all_index++;
    }
  }

  while (all_index < all_key_frames_.size()) {
    all_key_frames_.at(all_index).pose_ =
        pose_to_optimize_ * all_key_frames_.at(all_index).pose_;
    all_index++;
  }

  return true;
}

bool PostProcessing::JointGlobalMap(CloudData::CloudTypePtr& global_map_ptr) {
  JointCloudMap(optimized_key_frames_, global_map_ptr);
  return true;
}

bool PostProcessing::JointLocalMap(CloudData::CloudTypePtr& local_map_ptr) {
  size_t begin_index = 0;
  if (all_key_frames_.size() > (size_t)local_frame_num_)
    begin_index = all_key_frames_.size() - (size_t)local_frame_num_;

  std::deque<KeyFrame, Eigen::aligned_allocator<KeyFrame>> local_key_frames;
  for (size_t i = begin_index; i < all_key_frames_.size(); ++i) {
    local_key_frames.push_back(all_key_frames_.at(i));
  }

  JointCloudMap(local_key_frames, local_map_ptr);
  return true;
}

bool PostProcessing::JointCloudMap(
    const std::deque<KeyFrame, Eigen::aligned_allocator<KeyFrame>>&
        key_frames, CloudData::CloudTypePtr& map_cloud_ptr)
{
  map_cloud_ptr.reset(new CloudData::CloudType());

  CloudData::CloudTypePtr cloud_ptr(new CloudData::CloudType());
  std::string file_path = "";

  for (size_t i = 0; i < key_frames.size(); ++i) {
    file_path = key_frames_path_ + "/key_frame_" +
                std::to_string(key_frames.at(i).index_) + ".pcd";
    pcl::io::loadPCDFile(file_path, *cloud_ptr);
    pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, key_frames.at(i).pose_);
    *map_cloud_ptr += *cloud_ptr;
  }
  return true;
}

bool PostProcessing::SaveMap() {
  if (optimized_key_frames_.size() == 0) {
    return false;
  }
  // 生成地图
  CloudData::CloudTypePtr global_map_ptr(new CloudData::CloudType());
  JointCloudMap(optimized_key_frames_, global_map_ptr);
  // 保存原地图
  std::string map_file_path = map_path_ + "/map.pcd";
  pcl::io::savePCDFileBinary(map_file_path, *global_map_ptr);
  // 保存滤波后地图
  if (global_map_ptr->points.size() > 1000000) {
    std::shared_ptr<VoxelFilter> map_filter_ptr =
        std::make_shared<VoxelFilter>(0.5, 0.5, 0.5);
    map_filter_ptr->Filter(global_map_ptr, global_map_ptr);
  }
  std::string filtered_map_file_path = map_path_ + "/filtered_map.pcd";
  pcl::io::savePCDFileBinary(filtered_map_file_path, *global_map_ptr);

  std::cout << "地图保存完成，地址是：" << std::endl
            << map_path_ << std::endl
            << std::endl;

  return true;
}

const Eigen::Matrix4f& PostProcessing::GetCurrentPose() const {
  return optimized_odom_.pose_;
}

const CloudData::CloudTypePtr& PostProcessing::GetCurrentScan() {
  frame_filter_ptr_->Filter(optimized_cloud_.cloud_ptr_,
                            optimized_cloud_.cloud_ptr_);
  return optimized_cloud_.cloud_ptr_;
}

bool PostProcessing::GetLocalMap(CloudData::CloudTypePtr& local_map_ptr) {
  JointLocalMap(local_map_ptr);
  local_map_filter_ptr_->Filter(local_map_ptr, local_map_ptr);
  return true;
}

bool PostProcessing::GetGlobalMap(CloudData::CloudTypePtr& global_map_ptr) {
  JointGlobalMap(global_map_ptr);
  global_map_filter_ptr_->Filter(global_map_ptr, global_map_ptr);
  return true;
}

}  // namespace localization
}  // namespace apollo
