#ifndef PCD_PREPROCESS_LIDAR_FILTER_H_
#define PCD_PREPROCESS_LIDAR_FILTER_H_

#include "pcd_preprocess/data_interface/data_interface.h"

namespace preprocess {

class LidarFilter {
 public:
  static LidarFilter& GetSingleton() {
    static LidarFilter singleton;
    return singleton;
  }

  // Work Handle
  bool Init();
  const StampedCloud& FilterCloud(const StampedCloud&);

 private:
  LidarFilter() = default;
  virtual ~LidarFilter() = default;

  // Parameters
  ParameterFilter kparameter_filter_;
  ParameterSensor kparameter_sensor_;
  double kdistance_min_square_;
  double kdistance_max_square_;

  // Variables
  Eigen::Affine3d sensor_in_base_;
  StampedCloud processed_lidar_;
};

}  // namespace preprocess

#endif  // PCD_PREPROCESS_LIDAR_FILTER_H_
