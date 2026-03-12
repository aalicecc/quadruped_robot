#ifndef PCD_LOCALIZATION_PCD_LOCALIZATION_H_
#define PCD_LOCALIZATION_PCD_LOCALIZATION_H_

#include "pcd_localization/data_interface/data_interface.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

namespace localization {

enum class SystemState { kMove = 0, kStart, kFinish, kCharge };

class PcdLocalization {
 public:
  static PcdLocalization& GetSingleton() {
    static PcdLocalization singleton;
    return singleton;
  }

  // Work Handle
  bool Init();
  bool Work();

 private:
  PcdLocalization() = default;
  virtual ~PcdLocalization() = default;

  // Work Handle
  void UpdateTarget();
  const StampedTrans& PointsAlignment(const Eigen::Affine3f&,
                                         const StampedCloud&);
  void ResultProcess(const StampedTrans&, const Eigen::Affine3f&);

  // Help Handle
  const StampedTrans& NdtAlignment(
      const Eigen::Affine3f&, const pcl::PointCloud<pcl::PointXYZ>::Ptr&,
      ros::Time);
  // const StampedTrans& DoubleIcpAlignment(
  //     const Eigen::Affine3f&, const pcl::PointCloud<pcl::PointXYZ>::Ptr&,
  //     const pcl::PointCloud<pcl::PointXYZ>::Ptr&, double);
  pcl::PointCloud<pcl::PointXYZ>::Ptr DownSample(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr&, double);
  pcl::PointCloud<pcl::PointXYZ>::Ptr CropPointsBox(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr&, double, double, double,
      const Eigen::Vector3f&);
  pcl::PointCloud<pcl::PointXYZ>::Ptr CropPointsCylinder(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr&, double, double, double,
      double);

  // Parameters
  ParametersFlag kparameters_flag_;
  ParametersFrame kparameters_frame_;
  ParametersNdt kparameters_ndt_;
  ParametersSource kparameters_source_;
  ParametersTarget kparameters_target_;

  // Variables
  bool start_flag_;
  Eigen::Vector3f map_center_;
  Eigen::Affine3f last_trans_;
  Eigen::Affine3f delta_trans_;
  Eigen::Affine3f sensor_in_map_;
  Eigen::Affine3f odom_in_map_;
  Eigen::Affine3f rough_sensor_in_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_points_;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_register_;
};

}  // namespace localization

#endif  // PCD_LOCALIZATION_PCD_LOCALIZATION_H_
