#include "modules/localization/shenlan_msf/interface/matching.h"

namespace apollo {
namespace localization {

Matching::Matching()
    : global_map_ptr_(new CloudData::CloudType()),
      local_map_ptr_(new CloudData::CloudType()),
      current_scan_ptr_(new CloudData::CloudType()) {
  InitWithConfig();

  ResetLocalMap(0.0, 0.0, 0.0);
}

bool Matching::InitWithConfig() {
  //
  // load lio localization frontend config file:
  //
  std::string config_file_path =
      WORK_SPACE_PATH + "/config/matching/shenlan_matching.yaml";
  YAML::Node config_node = YAML::LoadFile(config_file_path);

  // prompt:
  LOG(INFO)
      << std::endl
      << "----------------- Init LIO Localization, Frontend -------------------"
      << std::endl;

  // a. init point cloud map & measurement processors:
  InitPointCloudProcessors(config_node);
  // b. load global map:
  InitGlobalMap(config_node);
  // c. init lidar frontend for relative pose estimation:
  InitRegistration(config_node, registration_ptr_);

  return true;
}

bool Matching::InitFilter(
    const YAML::Node& config_node, std::string filter_user,
    std::shared_ptr<CloudFilterInterface>& filter_ptr) {
  std::string filter_mothod =
      config_node[filter_user + "_filter"].as<std::string>();

  // prompt:
  LOG(INFO) << "\t\tFilter Method for " << filter_user << ": " << filter_mothod
            << std::endl;

  if (filter_mothod == "voxel_filter") {
    filter_ptr = std::shared_ptr<VoxelFilter>(
        new VoxelFilter(config_node[filter_mothod][filter_user]));
  } else if (filter_mothod == "no_filter") {
    filter_ptr = std::shared_ptr<NoFilter>(new NoFilter());
  } else {
    LOG(ERROR) << "Filter method " << filter_mothod << " for " << filter_user
               << " NOT FOUND!";
    return false;
  }

  return true;
}

bool Matching::InitLocalMapSegmenter(const YAML::Node& config_node) {
  local_map_segmenter_ptr_ =
      std::shared_ptr<BoxFilter>(new BoxFilter(config_node));
  return true;
}

bool Matching::InitPointCloudProcessors(const YAML::Node& config_node) {
  // prompt:
  LOG(INFO) << "\tInit Point Cloud Processors:" << std::endl;

  // a. global map filter:
  InitFilter(config_node, "global_map", global_map_filter_ptr_);

  // b.1. local map segmenter:
  InitLocalMapSegmenter(config_node);
  // b.2. local map filter:
  InitFilter(config_node, "local_map", local_map_filter_ptr_);

  // c. scan filter --
  InitFilter(config_node, "frame", frame_filter_ptr_);

  InitRelocalization(config_node);

  return true;
}

bool Matching::InitGlobalMap(const YAML::Node& config_node) {
  std::string map_path = config_node["map_path"].as<std::string>();

  // load map:
  pcl::io::loadPCDFile(map_path, *global_map_ptr_);
  // apply local map filter to global map for later scan-map matching:
  local_map_filter_ptr_->Filter(global_map_ptr_, global_map_ptr_);

  // prompt:
  LOG(INFO) << "\tLoad Global Map, size:" << global_map_ptr_->points.size();

  has_new_global_map_ = true;

  return true;
}

bool Matching::InitRegistration(
    const YAML::Node& config_node,
    std::shared_ptr<RegistrationInterface>& registration_ptr) {
  std::string registration_method =
      config_node["registration_method"].as<std::string>();

  // prompt:
  LOG(INFO) << "\tLidar Frontend Estimation Method: " << registration_method
            << std::endl;

  if (registration_method == "NDT") {
    registration_ptr = std::shared_ptr<NDTRegistration>(
        new NDTRegistration(config_node[registration_method]));
  } else if (registration_method == "Point2Plane") {
    registration_ptr = std::shared_ptr<Point2PlaneICP>(
        new Point2PlaneICP(config_node[registration_method]));
  } else {
    LOG(ERROR) << "Estimation method " << registration_method << " NOT FOUND!";
    return false;
  }

  return true;
}

bool Matching::InitRelocalization(const YAML::Node& config_node) {
  relocalization_registration_ptr_ = std::shared_ptr<NDTRegistration>(
      new NDTRegistration(config_node["relocalization"]));

  enable_relocalization_ =
      config_node["relocalization"]["enable_relocalization"].as<bool>();

  relocalization_guess_pos_.x() =
      config_node["relocalization"]["guess_pos"][0].as<float>();
  relocalization_guess_pos_.y() =
      config_node["relocalization"]["guess_pos"][1].as<float>();
  relocalization_guess_pos_.z() =
      config_node["relocalization"]["guess_pos"][2].as<float>();

  fitness_score_threshold_ =
      config_node["relocalization"]["fitness_socre_threshold"].as<double>();

  LOG(INFO) << "\tenable_relocalization: " << enable_relocalization_
            << std::endl
            << "\tfitness_score_threshold: " << fitness_score_threshold_
            << std::endl
            << "\tguess_pos: " << relocalization_guess_pos_.transpose()
            << std::endl;

  return true;
}

bool Matching::ResetLocalMap(float x, float y, float z) {
  std::vector<float> origin = {x, y, z};

  // use ROI filtering for local map segmentation:
  local_map_segmenter_ptr_->set_origin(origin);
  local_map_segmenter_ptr_->Filter(global_map_ptr_, local_map_ptr_);

  registration_ptr_->SetInputTarget(local_map_ptr_);

  has_new_local_map_ = true;

  const std::vector<float>& edge = local_map_segmenter_ptr_->edge();
  LOG(INFO) << "New local map:" << edge.at(0) << "," << edge.at(1) << ","
            << edge.at(2) << "," << edge.at(3) << "," << edge.at(4) << ","
            << edge.at(5) << std::endl
            << std::endl;

  return true;
}

bool Matching::SetInitPose(const Eigen::Matrix4f& init_pose) {
  init_pose_ = init_pose;

  ResetLocalMap(init_pose(0, 3), init_pose(1, 3), init_pose(2, 3));

  return true;
}

bool Matching::Init(const CloudData& init_scan,
                           const Eigen::Matrix4f& init_pose) {
  if (enable_relocalization_) {
    return SetPoseWithConfig(init_scan);
  } else {
    return SetGNSSPose(init_pose);
  }

  return false;
}

bool Matching::SetGNSSPose(const Eigen::Matrix4f& gnss_pose) {
  // static int gnss_cnt = 0;

  current_gnss_pose_ = gnss_pose;

  // if (gnss_cnt == 0) {
  //   SetInitPose(gnss_pose);
  // } else if (gnss_cnt > 3) {
  //   has_inited_ = true;
  // }
  // gnss_cnt++;

  SetInitPose(gnss_pose);
  has_inited_ = true;

  return true;
}

bool Matching::SetPoseWithConfig(const CloudData& init_scan) {
  ResetLocalMap(relocalization_guess_pos_.x(), relocalization_guess_pos_.y(),
                relocalization_guess_pos_.z());

  relocalization_registration_ptr_->SetInputTarget(local_map_ptr_);

  // downsample:
  CloudData::CloudTypePtr filtered_cloud_ptr(new CloudData::CloudType());
  frame_filter_ptr_->Filter(init_scan.cloud_ptr_, filtered_cloud_ptr);

  Eigen::Matrix4f relocalization_pose = Eigen::Matrix4f::Identity();
  if (Relocalization(filtered_cloud_ptr, relocalization_pose)) {
    SetInitPose(relocalization_pose);
    has_inited_ = true;

    return true;
  } else {
    LOG(INFO) << "relocalization fail, please set right guess_pos.";
  }

  return false;
}

bool Matching::Relocalization(
    const CloudData::CloudTypePtr& current_scan_ptr,
    Eigen::Matrix4f& relocalization_pose) {
  const size_t N = 36;

  std::multimap<
      double, Eigen::Matrix4f, std::greater<double>,
      Eigen::aligned_allocator<std::pair<const double, Eigen::Matrix4f>>>
      result;

  for (size_t i = 0; i < N; i++) {
    double guess_yaw = ((double)(i) * (2.0 * M_PI)) / (double)(N);
    if (guess_yaw > M_PI) {
      guess_yaw -= 2.0 * M_PI;
    }

    Eigen::AngleAxisf guess_orientation(guess_yaw, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f guess_pose = Eigen::Matrix4f::Identity();
    guess_pose.block<3, 3>(0, 0) = guess_orientation.toRotationMatrix();
    guess_pose.block<3, 1>(0, 3) = relocalization_guess_pos_;

    Eigen::Matrix4f result_pose = Eigen::Matrix4f::Identity();
    CloudData::CloudTypePtr result_cloud_ptr(new CloudData::CloudType());
    relocalization_registration_ptr_->ScanMatch(current_scan_ptr, guess_pose,
                                                result_cloud_ptr, result_pose);
    double fitness_socre = relocalization_registration_ptr_->GetFitnessScore();
    LOG(INFO) << "i: " << i << ", yaw: " << guess_yaw
              << ", socre: " << fitness_socre;
    result.insert(std::make_pair(fitness_socre, result_pose));
  }

  if (result.begin()->first < fitness_score_threshold_) {
    relocalization_pose = result.begin()->second;

    LOG(INFO) << "Relocalization successed, fitness_socre: "
              << result.begin()->first << " < " << fitness_score_threshold_
              << std::endl
              << "relocalization_pose: " << std::endl
              << relocalization_pose;

    return true;
  }

  // 下次重定位根据本次的最优位置进行
  relocalization_guess_pos_ = result.begin()->second.block<3, 1>(0, 3);

  return false;
}

bool Matching::Update(const CloudData& cloud_data,
                             Eigen::Matrix4f& laser_pose,
                             Eigen::Matrix4f& map_matching_pose) {
  static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
  static Eigen::Matrix4f last_pose = init_pose_;
  static Eigen::Matrix4f predict_pose = init_pose_;

  // remove invalid measurements:
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr_, *cloud_data.cloud_ptr_,
                               indices);

  // downsample current scan:
  CloudData::CloudTypePtr filtered_cloud_ptr(new CloudData::CloudType());
  frame_filter_ptr_->Filter(cloud_data.cloud_ptr_, filtered_cloud_ptr);

  // if (!has_inited_) {
  //   predict_pose = current_gnss_pose_;
  // }

  // matching:
  CloudData::CloudTypePtr result_cloud_ptr(new CloudData::CloudType());
  registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose,
                               result_cloud_ptr, laser_pose);
  pcl::transformPointCloud(*cloud_data.cloud_ptr_, *current_scan_ptr_,
                           laser_pose);

  // update predicted pose:
  step_pose = last_pose.inverse() * laser_pose;
  last_pose = laser_pose;
  predict_pose = laser_pose * step_pose;

  // shall the local map be updated:
  const std::vector<float>& edge = local_map_segmenter_ptr_->edge();
  for (int i = 0; i < 3; i++) {
    if (fabs(laser_pose(i, 3) - edge.at(2 * i)) > 50.0 &&
        fabs(laser_pose(i, 3) - edge.at(2 * i + 1)) > 50.0) {
      continue;
    }

    ResetLocalMap(laser_pose(0, 3), laser_pose(1, 3), laser_pose(2, 3));
    break;
  }

  return true;
}

}
}
