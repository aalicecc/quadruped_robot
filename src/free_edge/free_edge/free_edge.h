#ifndef FREE_EDGE_FREE_EDGE_H_
#define FREE_EDGE_FREE_EDGE_H_

#include "free_edge/data_interface/data_interface.h"

namespace postprocess {

class FreeEdge {
 public:
  static FreeEdge& GetSingleton() {
    static FreeEdge singleton;
    return singleton;
  }

  // Work Handle
  bool Init();
  bool Work();

 private:
  FreeEdge() = default;
  virtual ~FreeEdge() = default;

  // Work Handle
  void MapPoints();
  void MarkObstacle();
  void CreateObstacleScan();
  void CreateObstacleCloud();
  void ResetVariables();

  // Help Handle
  bool PointValid(const pcl::PointXYZ&);
  // index: (fov, distance)
  Eigen::Vector2i PointToIndex(const pcl::PointXYZ&);
  // metric: (range, max_gap), index: (fov, distance)
  Eigen::Vector2d EvaluateGrid(const Eigen::Vector2i&);

  // Parameters
  ParameterFilter kparameter_filter_;
  ParameterFlag kparameter_flag_;
  ParameterObstacle kparameter_obstacle_;
  ParameterSensor kparameter_sensor_;

  // Variables
  StampedCloud acc_lidar_;
  StampedCloud obstacle_cloud_;
  sensor_msgs::LaserScan obstacle_scan_;
  std::vector<int32_t> obstacle_index_;
  std::vector<std::vector<std::vector<double>>> height_map_;
  std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> point_map_;
};

}  // namespace postprocess

#endif  // FREE_EDGE_FREE_EDGE_H_
