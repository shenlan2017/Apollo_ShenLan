/******************************************************************************
 * Copyright 2022 The Shenlan Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "Eigen/Dense"
#include "absl/strings/str_cat.h"
#include "gflags/gflags.h"
#include "pcl/filters/crop_hull.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/io/pcd_io.h"
#include "pcl/kdtree/impl/kdtree_flann.hpp"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/surface/concave_hull.h"

#include "cyber/common/file.h"
#include "modules/localization/msf/common/io/pcl_point_types.h"
#include "modules/localization/msf/common/io/velodyne_utility.h"
#include "modules/perception/base/object.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/base/point_cloud.h"
#include "modules/perception/common/io/io_util.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/common/point_cloud_processing/common.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/app/lidar_obstacle_segmentation.h"
#include "modules/perception/lidar/app/lidar_obstacle_tracking.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/lidar/common/lidar_frame_pool.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/common/pcl_util.h"

DEFINE_string(pcd_folders, "/apollo/data/bag/calibration/parsed_data/00000/pcd", "provide the pcd folders");
DEFINE_string(pose_files, "/apollo/data/bag/calibration/parsed_data/00000/pcd/corrected_poses.txt", "provide pose files");
DEFINE_bool(enable_tracking, false, "option to enable tracking");
DEFINE_double(min_life_time, -1.0, "minimum track time for output");
DEFINE_bool(use_hdmap, false, "option to enable using hdmap");
DEFINE_bool(use_tracking_info, false, "option to use tracking info");
DEFINE_string(sensor_name, "velodyne64", "sensor name");

namespace apollo {
namespace perception {
namespace lidar {

using ::apollo::common::EigenAffine3dVec;
using ::apollo::common::EigenVector3dVec;
using apollo::localization::msf::velodyne::PointXYZIT;
using cyber::common::GetFileName;

void Split(const std::string& str, char ch, std::vector<std::string>* result) {
  result->clear();
  std::stringstream ss(str);
  std::string segment;
  while (std::getline(ss, segment, ch)) {
    result->push_back(segment);
  }
}

class OfflineLidarObstaclePerception {
 public:
  OfflineLidarObstaclePerception() = default;

  ~OfflineLidarObstaclePerception() = default;

  bool setup() {
    // FLAGS_config_manager_path = "./conf";
    if (!lib::ConfigManager::Instance()->Init()) {
      AERROR << "Failed to init ConfigManage.";
      return false;
    }
    lidar_segmentation_.reset(new LidarObstacleSegmentation);
    if (lidar_segmentation_ == nullptr) {
      AERROR << "Failed to get LidarObstacleDetection instance.";
      return false;
    }
    segment_init_options_.enable_hdmap_input = FLAGS_use_hdmap;
    segment_init_options_.sensor_name = FLAGS_sensor_name;
    if (!lidar_segmentation_->Init(segment_init_options_)) {
      AINFO << "Failed to init LidarObstacleDetection.";
      return false;
    }
    lidar_tracking_.reset(new LidarObstacleTracking);
    if (lidar_tracking_ == nullptr) {
      AERROR << "Failed to get LidarObstacleTracking instance.";
      return false;
    }
    tracking_init_options_.sensor_name = FLAGS_sensor_name;
    if (!lidar_tracking_->Init(tracking_init_options_)) {
      AINFO << "Failed to init LidarObstacleDetection.";
      return false;
    }

    if (!common::SensorManager::Instance()->GetSensorInfo(FLAGS_sensor_name,
                                                          &sensor_info_)) {
      AERROR << "Failed to get sensor info, sensor name: " << FLAGS_sensor_name;
      return false;
    }

    ADEBUG << "Sensor_name: " << sensor_info_.name;
    return true;
  }

  bool run() {
    // double timestamp = 0.f;
    std::vector<std::string> pcd_folder_paths;
    std::vector<std::string> pose_files;
    Split(FLAGS_pcd_folders, ',', &pcd_folder_paths);
    Split(FLAGS_pose_files, ',', &pose_files);


    if (pcd_folder_paths.size() != pose_files.size()) {
      AERROR << "The count of pcd folders is not equal pose files";
      return -1;
    }

    const size_t num_trials = pcd_folder_paths.size();

    // load all poses
    AINFO << "Pcd folders are as follows:";
    for (size_t i = 0; i < num_trials; ++i) {
      AINFO << pcd_folder_paths[i];
    }
    std::vector<EigenAffine3dVec> ieout_poses(num_trials);
    std::vector<std::vector<double>> time_stamps(num_trials);
    std::vector<std::vector<unsigned int>> pcd_indices(num_trials);
    for (size_t i = 0; i < pose_files.size(); ++i) {
      apollo::localization::msf::velodyne::LoadPcdPoses(
          pose_files[i], &ieout_poses[i], &time_stamps[i], &pcd_indices[i]);
    }

    for (size_t trial = 0; trial < num_trials; ++trial) {
      for (size_t i = 0; i < pcd_indices[trial].size(); ++i) {
        const auto& id = pcd_indices[trial][i];

        std::string pcd_file_name =
            pcd_folder_paths[trial] + "/" + std::to_string(id) + ".pcd";
        AINFO << "Processing: " << pcd_file_name;

        frame_ = LidarFramePool::Instance().Get();
        frame_->sensor_info = sensor_info_;
        frame_->reserve = pcd_file_name;
        if (frame_->cloud == nullptr) {
          frame_->cloud = base::PointFCloudPool::Instance().Get();
        }
        LoadPCLPCD(pcd_file_name, frame_->cloud.get());
        frame_->timestamp = time_stamps[trial][i];
        frame_->lidar2world_pose = ieout_poses[trial][i];

        LidarProcessResult segment_result =
            lidar_segmentation_->Process(segment_options_, frame_.get());
        if (segment_result.error_code != LidarErrorCode::Succeed) {
          AINFO << segment_result.log;
          continue;
        }
        if (FLAGS_enable_tracking) {
          AINFO << "Enable tracking.";
          LidarProcessResult tracking_result =
              lidar_tracking_->Process(tracking_options_, frame_.get());
          if (tracking_result.error_code != LidarErrorCode::Succeed) {
            AINFO << tracking_result.log;
            continue;
          }
          if (FLAGS_use_tracking_info) {
            auto& objects = frame_->segmented_objects;
            auto& result_objects = frame_->tracked_objects;
            std::sort(
                objects.begin(), objects.end(),
                [](const base::ObjectPtr& lhs, const base::ObjectPtr& rhs) {
                  return lhs->id < rhs->id;
                });
            std::sort(
                result_objects.begin(), result_objects.end(),
                [](const base::ObjectPtr& lhs, const base::ObjectPtr& rhs) {
                  return lhs->id < rhs->id;
                });
            for (std::size_t j = 0; j < objects.size(); ++j) {
              ACHECK(objects[j]->id == result_objects[j]->id);
              objects[j]->track_id = result_objects[j]->track_id;
              objects[j]->tracking_time = result_objects[j]->tracking_time;
              objects[j]->center = frame_->lidar2world_pose.inverse() *
                                   result_objects[j]->center;
              Eigen::Vector3d direction =
                  result_objects[j]->direction.cast<double>();
              direction =
                  frame_->lidar2world_pose.linear().transpose() * direction;
              objects[j]->direction = direction.cast<float>();
              objects[j]->size = result_objects[j]->size;
              auto velocity = frame_->lidar2world_pose.linear().transpose() *
                              result_objects[j]->velocity.cast<double>();
              objects[j]->velocity = velocity.cast<float>();

              objects[j]->type = result_objects[j]->type;
              objects[j]->type_probs = result_objects[j]->type_probs;
              objects[j]->polygon = result_objects[j]->polygon;
              common::TransformPointCloud(result_objects[j]->polygon,
                                          frame_->lidar2world_pose.inverse(),
                                          &objects[j]->polygon);
            }
          }
        }

        std::vector<base::ObjectPtr>& result_objects =
            frame_->segmented_objects;
        std::vector<base::ObjectPtr> filtered_objects;
        for (auto& object : result_objects) {
          if (object->tracking_time >= FLAGS_min_life_time) {
            filtered_objects.push_back(object);
          }
        }
        std::cout <<"************************" <<result_objects.size()
        <<"    "<<pcd_file_name<< std::endl;
        int n = 0;
        while(n++ < 10)
        {
          sleep(0.1);
          std::cout << "************************"<<std::endl;
        }



        if (!MapObstacleFilter(filtered_objects, pcd_file_name)) {
          continue;
        }
      }
    }

    return true;
  }

  bool MapObstacleFilter(const std::vector<base::ObjectPtr>& objects,
                         const std::string& pcd_file_name) {
    if (objects.empty()) {
      return false;
    }
    pcl::PointCloud<PointXYZIT>::Ptr cloud_read(
        new pcl::PointCloud<PointXYZIT>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>());
    if (pcl::io::loadPCDFile(pcd_file_name, *cloud_read) == -1) {
      AWARN << "couldn't read file" << pcd_file_name;
      return false;
    }
    for (const auto& p : cloud_read->points) {
      pcl::PointXYZI pt;
      pt.x = p.x;
      pt.y = p.y;
      pt.z = p.z;
      pt.intensity = p.intensity;
      cloud->push_back(pt);
    }
    int ii=0;
    std::vector<int> filter_indices;
    for (auto& object : objects) {
      auto& object_cloud = object->lidar_supplement.cloud;
      pcl::PointCloud<pcl::PointXYZI>::Ptr boundingbox_ptr(
          new pcl::PointCloud<pcl::PointXYZI>);
          std::cout <<"object_cloud.size()--" <<object_cloud.size()<< std::endl;
        
      if(object_cloud.size() > 3)
      {
        ii++;
        std::cout << "center: "<< object->center(0) << " " << object->center(1)
           << " " << object->center(2) << std::endl;
      Eigen::Vector3f center(object->center(0),object->center(1),object->center(2));
        
      std::cout << "size: "<< object->size.transpose()<< std::endl;
      std::cout << "yaw: "<< object->theta<< std::endl;
      float theta = object->theta;
      float  length = object->size(0)*2;
      float width = object->size(1)*2;
      std::cout << "length "<< length << "width " << width<< "theta: "<<theta<<std::endl;
      pcl::PointXYZI p;
        p.x =center(0)+(length / 2 )* cos(theta)  - (width / 2 )* sin(theta);
        p.y = center(1)-(length / 2 )* sin(theta)  - (width / 2 )* cos(theta);
        p.z = 0;
        std::cout << "a: "<< p.x<< " "<< p.y << std::endl;
        boundingbox_ptr->emplace_back(p);
        p.x =center(0)- (length / 2 )* cos(theta)  - (width / 2 )* sin(theta) ;
        p.y = center(1)+ (length / 2 )* sin(theta)  - width / 2 * cos(theta);
        p.z = 0;
        std::cout << "b: "<< p.x<< " "<< p.y << std::endl;
        boundingbox_ptr->emplace_back(p);
        p.x =center(0)-length / 2 * cos(theta) + width / 2 * sin(theta) ;
        p.y = center(1)+length / 2 * sin(theta)+ width / 2 * cos(theta);
        p.z = 0;
        std::cout << "c: "<< p.x<< " "<< p.y << std::endl;
        boundingbox_ptr->emplace_back(p);
        p.x =center(0)+(length / 2 )* cos(theta) + (width / 2 )* sin(theta) ;
        p.y = center(1)-(length / 2 )* sin(theta) + (width / 2 )* cos(theta);
        p.z = 0;
        std::cout << "d: "<< p.x<< " "<< p.y << std::endl;
        boundingbox_ptr->emplace_back(p);
  
      pcl::ConvexHull<pcl::PointXYZI> hull;
      hull.setInputCloud(boundingbox_ptr);
      hull.setDimension(2);

      std::vector<pcl::Vertices> polygons;
      pcl::PointCloud<pcl::PointXYZI>::Ptr surface_hull(
          new pcl::PointCloud<pcl::PointXYZI>);
      hull.reconstruct(*surface_hull, polygons);

      pcl::CropHull<pcl::PointXYZI> bb_filter;
      bb_filter.setDim(2);
      bb_filter.setInputCloud(cloud);
      bb_filter.setHullIndices(polygons);
      bb_filter.setHullCloud(surface_hull);
      bb_filter.filter(filter_indices);
      }
    }
    std::shared_ptr<std::vector<int>> index_ptr =
        std::make_shared<std::vector<int>>(filter_indices);
    if(ii !=0 && index_ptr->size() >0){
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    std::cout << "point size "<< index_ptr->size()<< std::endl;
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(index_ptr);
    extract.setNegative(true);
    extract.filter(*filtered_cloud);

    pcl::PointCloud<PointXYZIT>::Ptr cloud_save(
        new pcl::PointCloud<PointXYZIT>);
    for (const auto& p : filtered_cloud->points) {
      PointXYZIT pt;
      pt.x = p.x;
      pt.y = p.y;
      pt.z = p.z;
      pt.intensity = p.intensity;
      cloud_save->push_back(pt);
    }
    // pcl::io::savePCDFileBinary(pcd_file_name, *cloud_save);
    pcl::io::savePCDFileBinaryCompressed(pcd_file_name, *cloud_save);
    }
   
    return true;
  }

 protected:
  std::shared_ptr<LidarFrame> frame_;
  LidarObstacleSegmentationInitOptions segment_init_options_;
  LidarObstacleSegmentationOptions segment_options_;
  LidarObstacleTrackingInitOptions tracking_init_options_;
  LidarObstacleTrackingOptions tracking_options_;
  std::unique_ptr<LidarObstacleSegmentation> lidar_segmentation_;
  std::unique_ptr<LidarObstacleTracking> lidar_tracking_;
  base::SensorInfo sensor_info_;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = 1;
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  apollo::perception::lidar::OfflineLidarObstaclePerception test;
  if (!test.setup()) {
    AINFO << "Failed to setup OfflineLidarObstaclePerception";
    return -1;
  }
  return test.run() ? 0 : -1;
}

