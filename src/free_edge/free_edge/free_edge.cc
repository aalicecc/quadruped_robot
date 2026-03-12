#include "free_edge/lidar_accumulate.h"
#include "free_edge/free_edge.h"
#include <pcl/common/transforms.h>
#include <algorithm>
#include <chrono>

namespace postprocess {

bool FreeEdge::Init() {
  static DataInterface& data_interface = DataInterface::GetSingleton();
  static LidarAccumulate& lidar_accumulate = LidarAccumulate::GetSingleton();

  // Parameters
  kparameter_filter_ = data_interface.GetParameterFilter();
  kparameter_flag_ = data_interface.GetParameterFlag();
  kparameter_obstacle_ = data_interface.GetParameterObstacle();
  kparameter_sensor_ = data_interface.GetParameterSensor();

  // Variables
  acc_lidar_ = StampedCloud();
  obstacle_cloud_ = StampedCloud();
  obstacle_scan_ = sensor_msgs::LaserScan();
  obstacle_scan_.distances = std::vector<float>(kparameter_filter_.fov_num,
                                                kparameter_filter_.distance[1]);
  obstacle_scan_.intensities =
      std::vector<float>(kparameter_filter_.fov_num, 10.0);
  obstacle_index_ = std::vector<int32_t>(kparameter_filter_.fov_num,
                                         kparameter_filter_.distance_num);
  height_map_.resize(kparameter_filter_.fov_num);
  point_map_.resize(kparameter_filter_.fov_num);
  for (int32_t i = 0; i < kparameter_filter_.fov_num; i++) {
    height_map_[i].resize(kparameter_filter_.distance_num);
    point_map_[i].resize(kparameter_filter_.distance_num);
    for (int32_t j = 0; j < kparameter_filter_.distance_num; j++) {
      height_map_[i][j] = std::vector<double>();
      point_map_[i][j] = pcl::PointCloud<pcl::PointXYZ>::Ptr(
          new pcl::PointCloud<pcl::PointXYZ>);
    }
  }

  // Other Components
  lidar_accumulate.Init();

  return true;
}

bool FreeEdge::Work() {
  static DataInterface& data_interface = DataInterface::GetSingleton();
  static LidarAccumulate& lidar_accumulate = LidarAccumulate::GetSingleton();

  if (data_interface.GetOdomFlag()) {
    std::vector<nav_msgs::Odometry> new_odom =
        std::vector<nav_msgs::Odometry>(data_interface.GetOdomBuffer());
    data_interface.ResetOdomFlag();
    lidar_accumulate.AddOdom(new_odom);
  }

  if (data_interface.GetLidarFlag()) {
    std::vector<StampedCloud> lidar_buffer =
        std::vector<StampedCloud>(data_interface.GetLidarBuffer());
    data_interface.ResetLidarFlag();
    acc_lidar_ = lidar_accumulate.AccumulateCloud(lidar_buffer);

    // trans: P_out = T * P_in
    pcl::transformPointCloud(*acc_lidar_.points, *acc_lidar_.points,
                             kparameter_sensor_.sensor_in_base);

    MapPoints();
    MarkObstacle();

    CreateObstacleScan();
    data_interface.PublishObstacleScan(obstacle_scan_);

    if (kparameter_flag_.pub_cloud) {
      CreateObstacleCloud();
      // trans: P_out = T * P_in
      pcl::transformPointCloud(*obstacle_cloud_.points, *obstacle_cloud_.points,
                               kparameter_sensor_.sensor_in_base.inverse());
      data_interface.PublishObstacleCloud(obstacle_cloud_);
    }

    ResetVariables();

    // data_interface.Log(LogLevel::kInfo, "Work Time : %f ms",
    //                    std::chrono::duration<double, std::milli>(
    //                        std::chrono::steady_clock::now() - begin_time)
    //                        .count());
  }

  return true;
}

void FreeEdge::MapPoints() {
  for (const auto& point : acc_lidar_.points->points) {
    if (!PointValid(point)) {
      continue;
    }

    if (point.z < kparameter_filter_.height[0] ||
        point.z > kparameter_filter_.height[1]) {
      continue;
    }

    Eigen::Vector2i index = PointToIndex(point);
    if (index[0] < 0 || index[0] >= kparameter_filter_.fov_num ||
        index[1] < 0 || index[1] >= kparameter_filter_.distance_num) {
      continue;
    }

    height_map_[index[0]][index[1]].emplace_back(point.z);
    point_map_[index[0]][index[1]]->points.emplace_back(point);
  }
}

void FreeEdge::MarkObstacle() {
  for (int32_t i = 0; i < kparameter_filter_.fov_num; i++) {
    for (int32_t j = 0; j < kparameter_filter_.distance_num; j++) {
      Eigen::Vector2i index({i, j});

      // preprocess the height_map
      if (height_map_[index[0]][index[1]].empty()) {
        continue;
      }
      std::sort(height_map_[index[0]][index[1]].begin(),
                height_map_[index[0]][index[1]].end());

      // mark the obstacle
      double distance = kparameter_filter_.distance[0] +
                        (j + 0.5) * kparameter_filter_.distance_step;
      if (distance < kparameter_obstacle_.change_distance) {
        // mark near obstacle
        if (height_map_[index[0]][index[1]].back() >
            kparameter_obstacle_.near_factor * distance +
                kparameter_obstacle_.near_constant) {
          obstacle_index_[i] = j;
          break;
        }
      } else {
        // mark far obstacle
        Eigen::Vector2d metric = EvaluateGrid(index);
        if (metric[0] > kparameter_obstacle_.far_range ||
            metric[1] > kparameter_obstacle_.far_gap) {
          obstacle_index_[i] = j;
          break;
        }
      }
    }
  }
}

void FreeEdge::CreateObstacleScan() {
  obstacle_scan_.time = acc_lidar_.time;
  for (int32_t i = 0; i < kparameter_filter_.fov_num; i++) {
    obstacle_scan_.distances[i] = kparameter_filter_.distance[1] - 1e-5;
    if (obstacle_index_[i] < kparameter_filter_.distance_num) {
      obstacle_scan_.distances[i] =
          kparameter_filter_.distance[0] +
          (obstacle_index_[i] + 0.5) * kparameter_filter_.distance_step;
    }
  }
}

void FreeEdge::CreateObstacleCloud() {
  obstacle_cloud_.time = acc_lidar_.time;
  for (int32_t i = 0; i < kparameter_filter_.fov_num; i++) {
    for (int32_t j = obstacle_index_[i]; j < kparameter_filter_.distance_num;
         j++) {
      *obstacle_cloud_.points += *point_map_[i][j];
    }
  }
}

void FreeEdge::ResetVariables() {
  acc_lidar_.points->clear();
  obstacle_cloud_.points->clear();
  obstacle_scan_.distances = std::vector<float>(kparameter_filter_.fov_num,
                                                kparameter_filter_.distance[1]);
  obstacle_index_ = std::vector<int32_t>(kparameter_filter_.fov_num,
                                         kparameter_filter_.distance_num);
  for (int32_t i = 0; i < kparameter_filter_.fov_num; i++) {
    for (int32_t j = 0; j < kparameter_filter_.distance_num; j++) {
      height_map_[i][j].clear();
      point_map_[i][j]->clear();
    }
  }
}

bool FreeEdge::PointValid(const pcl::PointXYZ& point) {
  return (std::isfinite(point.x) && std::isfinite(point.y) &&
          std::isfinite(point.z) && !std::isnan(point.x) &&
          !std::isnan(point.y) && !std::isnan(point.z));
}

Eigen::Vector2i FreeEdge::PointToIndex(const pcl::PointXYZ& point) {
  static double fov_step_reciprocal = 1.0 / kparameter_filter_.fov_step;
  static double distance_step_reciprocal =
      1.0 / kparameter_filter_.distance_step;
  double fov = atan2(point.y, point.x);
  double distance = sqrt(point.x * point.x + point.y * point.y);

  Eigen::Vector2i index({0, 0});
  index[0] = static_cast<int32_t>((fov - kparameter_filter_.fov[0]) *
                                  fov_step_reciprocal);
  index[1] = static_cast<int32_t>((distance - kparameter_filter_.distance[0]) *
                                  distance_step_reciprocal);
  return index;
}

Eigen::Vector2d FreeEdge::EvaluateGrid(const Eigen::Vector2i& index) {
  // metric: (range, max_gap)
  Eigen::Vector2d metric({0.0, 0.0});

  metric[0] = height_map_[index[0]][index[1]].back() -
              height_map_[index[0]][index[1]].front();

  double last_height = height_map_[index[0]][index[1]].front();
  for (const auto& height : height_map_[index[0]][index[1]]) {
    if (height - last_height > metric[1]) {
      metric[1] = height - last_height;
    }
    last_height = height;
  }

  return metric;
}

}  // namespace postprocess
