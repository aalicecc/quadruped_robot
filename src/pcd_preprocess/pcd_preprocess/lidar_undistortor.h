#ifndef PCD_PREPROCESS_LIDAR_UNDISTORTOR_H_
#define PCD_PREPROCESS_LIDAR_UNDISTORTOR_H_

#include "pcd_preprocess/data_interface/data_interface.h"

namespace preprocess {

class LidarUndistortor {
 public:
  static LidarUndistortor& GetSingleton() {
    static LidarUndistortor singleton;
    return singleton;
  }

  // Work Handle
  bool Init();
  const StampedCloud& UndistortCloud(const StampedCloud&,
                                     std::deque<sensor_msgs::Imu>&);
  const StampedCloud& UndistortCloud(const StampedCloud&,
                                     std::deque<nav_msgs::Odometry>&);

 private:
  LidarUndistortor() = default;
  virtual ~LidarUndistortor() = default;

  // Work Handle
  void CalculatePoses(const ros::Time&, std::deque<sensor_msgs::Imu>&);
  void CalculatePoses(const ros::Time&, std::deque<nav_msgs::Odometry>&);
  void Undistort(const StampedCloud&);

  // Help Handle
  Eigen::Affine3d CurrentTrans(const ros::Time&);
  Eigen::Affine3d InterpolateTrans(const Eigen::Affine3d&,
                                   const Eigen::Affine3d&, double);
  Eigen::Affine3d PredictTrans(const Eigen::Affine3d&, const Eigen::Affine3d&,
                               double);

  // Parameters
  ParameterFlag kparameter_flag_;
  ParameterSensor kparameter_sensor_;

  // Variables
  Eigen::Affine3d sensor_in_pose_;
  std::vector<StampedPose> pose_list_;
  StampedCloud processed_lidar_;
};

}  // namespace preprocess

#endif  // PCD_PREPROCESS_LIDAR_UNDISTORTOR_H_
