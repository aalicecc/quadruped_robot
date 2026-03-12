#ifndef FREE_EDGE_LIDAR_ACCUMULATE_H_
#define FREE_EDGE_LIDAR_ACCUMULATE_H_

# include <free_edge/data_interface/base_interface.h>

namespace postprocess {

class LidarAccumulate {
 public:
  static LidarAccumulate& GetSingleton() {
    static LidarAccumulate singleton;
    return singleton;
  }

  // Work Handle
  bool Init();
  void AddOdom(const std::vector<nav_msgs::Odometry>&);
  const StampedCloud& AccumulateCloud(const std::vector<StampedCloud>&);

 private:
  LidarAccumulate() = default;
  virtual ~LidarAccumulate() = default;

  // Work Handle
  bool RemoveOldOdom(const ros::Time&);

  // Help Handle
  Eigen::Affine3d CurrentTrans(const ros::Time&);
  Eigen::Affine3d InterpolateTrans(const Eigen::Affine3d&,
                                   const Eigen::Affine3d&, double);
  Eigen::Affine3d PredictTrans(const Eigen::Affine3d&, const Eigen::Affine3d&,
                               double);

  // Parameters
  ParameterObstacle kparameter_obstacle_;
  ParameterSensor kparameter_sensor_;

  // Variables
  std::deque<StampedCloud> lidar_queue_;
  std::deque<nav_msgs::Odometry> odom_queue_;
  StampedCloud acc_lidar_;
};

}  // namespace postprocess

#endif  // FREE_EDGE_LIDAR_ACCUMULATE_H_
