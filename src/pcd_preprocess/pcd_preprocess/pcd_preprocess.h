#ifndef PCD_PREPROCESS_PCD_PREPROCESS_H_
#define PCD_PREPROCESS_PCD_PREPROCESS_H_

#include "pcd_preprocess/data_interface/data_interface.h"

namespace preprocess {

class PcdPreprocess {
 public:
  static PcdPreprocess& GetSingleton() {
    static PcdPreprocess singleton;
    return singleton;
  }

  // Work Handle
  bool Init();
  bool Work();

 private:
  PcdPreprocess() = default;
  virtual ~PcdPreprocess() = default;

  // Work Handle
  void RawModeLoop();
  void ImuModeLoop();
  void OdomModeLoop();

  // Help Handle
  bool ImuUndistortCloud();
  bool OdomUndistortCloud();
  bool FilterCloud();

  // Parameters
  ParameterFilter kparameter_filter_;
  ParameterFlag kparameter_flag_;
  ParameterSensor kparameter_sensor_;

  // Variables
  StampedCloud raw_lidar_;
  StampedCloud processed_lidar_;
  std::deque<sensor_msgs::Imu> imu_queue_;
  std::deque<nav_msgs::Odometry> odom_queue_;
};

}  // namespace preprocess

#endif  // PCD_PREPROCESS_PCD_PREPROCESS_H_
