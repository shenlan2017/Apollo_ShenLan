/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#pragma once

#include <memory>
#include <string>

#include "Eigen/Dense"

#include "modules/perception/lidar/common/lidar_error_code.h"
#include "modules/perception/lidar/lib/detection/lidar_point_pillars/point_pillars_detection.h"
#include "modules/perception/lidar/lib/pointcloud_preprocessor/pointcloud_preprocessor.h"

namespace apollo {
namespace perception {
namespace lidar {

struct LidarObstacleDetectionInitOptions {
  std::string sensor_name = "velodyne64";
};

struct LidarObstacleDetectionOptions {
  std::string sensor_name;
  Eigen::Affine3d sensor2novatel_extrinsics;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

class LidarObstacleDetection {
 public:
  LidarObstacleDetection() = default;
  ~LidarObstacleDetection() = default;

  bool Init(const LidarObstacleDetectionInitOptions& options =
                LidarObstacleDetectionInitOptions());

  LidarProcessResult Process(
      const LidarObstacleDetectionOptions& options,
      const std::shared_ptr<apollo::drivers::PointCloud const>& message,
      LidarFrame* frame);

  LidarProcessResult Process(const LidarObstacleDetectionOptions& options,
                             LidarFrame* frame);

  std::string Name() const { return "LidarObstacleDetection"; }

 private:
  LidarProcessResult ProcessCommon(const LidarObstacleDetectionOptions& options,
                                   LidarFrame* frame);

 private:
  PointCloudPreprocessor cloud_preprocessor_;
  std::unique_ptr<PointPillarsDetection> detector_;
  // params
  std::string detector_name_;
};  // class LidarObstacleDetection

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
