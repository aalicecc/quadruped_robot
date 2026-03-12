#include "free_edge/lidar_accumulate.h"
#include <pcl/common/transforms.h>
#include <algorithm>

namespace postprocess {

bool LidarAccumulate::Init() {
  static DataInterface& data_interface = DataInterface::GetSingleton();

  // Parameters
  kparameter_obstacle_ = data_interface.GetParameterObstacle();
  kparameter_sensor_ = data_interface.GetParameterSensor();

  // Variables
  lidar_queue_.clear();
  odom_queue_.clear();

  return true;
}

void LidarAccumulate::AddOdom(
    const std::vector<nav_msgs::Odometry>& new_odom_vector) {
  size_t old_size = odom_queue_.size();
  odom_queue_.insert(odom_queue_.end(), new_odom_vector.begin(),
                     new_odom_vector.end());
  std::inplace_merge(odom_queue_.begin(), odom_queue_.begin() + old_size,
                     odom_queue_.end(),
                     [](const nav_msgs::Odometry& a, const nav_msgs::Odometry& b) {
                       return a.time < b.time;
                     });
}

const StampedCloud& LidarAccumulate::AccumulateCloud(
    const std::vector<StampedCloud>& new_lidar_vector) {
  static DataInterface& data_interface = DataInterface::GetSingleton();
  acc_lidar_ = StampedCloud();

  size_t old_size = lidar_queue_.size();
  lidar_queue_.insert(lidar_queue_.end(), new_lidar_vector.begin(),
                      new_lidar_vector.end());
  std::inplace_merge(lidar_queue_.begin(), lidar_queue_.begin() + old_size,
                     lidar_queue_.end(),
                     [](const StampedCloud& a, const StampedCloud& b) {
                       return a.time < b.time;
                     });

  if (lidar_queue_.empty()) {
    data_interface.Log(LogLevel::kError, "lidar list empty");
    return acc_lidar_;
  }

  while (lidar_queue_.size() >
         static_cast<size_t>(kparameter_obstacle_.cloud_num)) {
    lidar_queue_.pop_front();
  }

  if (!RemoveOldOdom(lidar_queue_.front().time)) {
    return acc_lidar_;
  }

  //---------------------------------------------------------------------------
  // transform poses from odom to lidar
  //---------------------------------------------------------------------------
  // Frame: b => base_link, l => lidar
  // Time : k => current time, e => end time
  // sensor_in_pose_ => l_n_in_b_n
  Eigen::Affine3d o_in_b_e = CurrentTrans(lidar_queue_.back().time).inverse();
  Eigen::Affine3d l_n_in_b_n = kparameter_sensor_.sensor_in_base;
  Eigen::Affine3d b_n_in_l_n = kparameter_sensor_.sensor_in_base.inverse();
  acc_lidar_.time = lidar_queue_.back().time;
  acc_lidar_.points->clear();
  for (auto& lidar : lidar_queue_) {
    Eigen::Affine3d b_k_in_o = CurrentTrans(lidar.time);
    Eigen::Affine3d l_k_in_l_e = b_n_in_l_n * o_in_b_e * b_k_in_o * l_n_in_b_n;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transed_points =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    transed_points->clear();
    // trans: P_out = T * P_in
    pcl::transformPointCloud(*lidar.points, *transed_points, l_k_in_l_e);
    *acc_lidar_.points += *transed_points;
  }

  return acc_lidar_;
}

bool LidarAccumulate::RemoveOldOdom(const ros::Time& timestamp) {
  static DataInterface& data_interface = DataInterface::GetSingleton();
  ros::Time time_bound = timestamp - 4.0 * kparameter_sensor_.odom_period;

  if (odom_queue_.empty()) {
    data_interface.Log(LogLevel::kError, "pose list empty");
    return false;
  }

  if (lidar_queue_.front().time > timestamp) {
    data_interface.Log(LogLevel::kWarn, "Time of lidar is ahead of imu");
    return false;
  }

  while (odom_queue_.front().time < time_bound) {
    odom_queue_.pop_front();
    if (odom_queue_.empty()) {
      return false;
    }
  }

  return true;
}

Eigen::Affine3d LidarAccumulate::CurrentTrans(const ros::Time& timestamp) {
  static DataInterface& data_interface = DataInterface::GetSingleton();

  if (odom_queue_.empty()) {
    data_interface.Log(LogLevel::kError, "pose list empty");
    return Eigen::Affine3d::Identity();
  }

  if (timestamp < odom_queue_.front().time) {
    data_interface.Log(LogLevel::kError,
                       "free_edge: timestamp is too early: %fs earlier",
                       timestamp - odom_queue_.front().time);
    return Eigen::Affine3d::Identity();
  }

  if (timestamp > odom_queue_.back().time) {
    size_t back_index = odom_queue_.size() - 1;
    double alpha =
        (timestamp - odom_queue_[back_index].time) /
        (odom_queue_[back_index].time - odom_queue_[back_index - 1].time);
    return PredictTrans(odom_queue_[back_index].base_in_odom,
                        odom_queue_[back_index - 1].base_in_odom, alpha);
  }

  for (size_t i = 0; i < odom_queue_.size() - 1; ++i) {
    if (timestamp >= odom_queue_[i].time &&
        timestamp < odom_queue_[i + 1].time) {
      double alpha = (timestamp - odom_queue_[i].time) /
                     (odom_queue_[i + 1].time - odom_queue_[i].time);
      return InterpolateTrans(odom_queue_[i].base_in_odom,
                              odom_queue_[i + 1].base_in_odom, alpha);
    }
  }

  return Eigen::Affine3d::Identity();
}

Eigen::Affine3d LidarAccumulate::InterpolateTrans(
    const Eigen::Affine3d& trans_1, const Eigen::Affine3d& trans_2,
    double alpha) {
  //---------------------------------------------------------------------------
  // interpolation formula:
  // trans_interp = (1 - alpha) * trans_1 + alpha * trans_2
  //---------------------------------------------------------------------------
  // divide transformation into translation and quaternion
  Eigen::Vector3d translation_1 = trans_1.translation();
  Eigen::Vector3d translation_2 = trans_2.translation();
  Eigen::Quaterniond quaternion_1(trans_1.rotation());
  Eigen::Quaterniond quaternion_2(trans_2.rotation());
  // calculate weights
  double weight_1 = 1 - alpha;
  double weight_2 = alpha;

  // translation interpolation
  Eigen::Vector3d translation_interp =
      weight_1 * translation_1 + weight_2 * translation_2;

  // quaternion interpolation (SLERP)
  Eigen::Quaterniond quaternion_interp =
      quaternion_1.slerp(alpha, quaternion_2);

  // combine translation and quaternion
  Eigen::Affine3d trans_interp = Eigen::Affine3d::Identity();
  trans_interp.matrix().block<3, 3>(0, 0) =
      quaternion_interp.toRotationMatrix();
  trans_interp.matrix().block<3, 1>(0, 3) = translation_interp;

  return trans_interp;
}

Eigen::Affine3d LidarAccumulate::PredictTrans(const Eigen::Affine3d& trans_1,
                                              const Eigen::Affine3d& trans_2,
                                              double alpha) {
  //---------------------------------------------------------------------------
  // predict formula:
  // trans_interp = (1 + alpha) * trans_1 - alpha * trans_2
  //---------------------------------------------------------------------------
  // divide transformation into translation and quaternion
  Eigen::Vector3d translation_1 = trans_1.translation();
  Eigen::Vector3d translation_2 = trans_2.translation();
  Eigen::Quaterniond quaternion_1(trans_1.rotation());
  Eigen::Quaterniond quaternion_2(trans_2.rotation());
  // calculate weights
  double weight_1 = 1 + alpha;
  double weight_2 = -alpha;

  // translation interpolation
  Eigen::Vector3d translation_interp =
      weight_1 * translation_1 + weight_2 * translation_2;

  // quaternion interpolation (SLERP)
  Eigen::Quaterniond quaternion_final =
      quaternion_1 * (quaternion_2.conjugate() * quaternion_1);
  Eigen::Quaterniond quaternion_interp =
      quaternion_1.slerp(alpha, quaternion_final);

  // combine translation and quaternion
  Eigen::Affine3d trans_interp = Eigen::Affine3d::Identity();
  trans_interp.matrix().block<3, 3>(0, 0) =
      quaternion_interp.toRotationMatrix();
  trans_interp.matrix().block<3, 1>(0, 3) = translation_interp;

  return trans_interp;
}

}  // namespace postprocess
