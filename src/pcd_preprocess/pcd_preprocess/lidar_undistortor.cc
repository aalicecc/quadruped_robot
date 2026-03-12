#include "pcd_preprocess/lidar_undistortor.h"
#include <omp.h>

#define THREAD_NUM 4
namespace preprocess {

bool LidarUndistortor::Init() {
  static DataInterface& data_interface = DataInterface::GetSingleton();

  // Parameter
  kparameter_flag_ = data_interface.GetParameterFlag();
  kparameter_sensor_ = data_interface.GetParameterSensor();

  // Variable
  switch (kparameter_flag_.mode) {
    case ModeUndistortion::kImuMode: {
      sensor_in_pose_ = kparameter_sensor_.imu_in_sensor.inverse();
      break;
    }
    case ModeUndistortion::kOdomMode: {
      sensor_in_pose_ = kparameter_sensor_.sensor_in_base;
      break;
    }
    default: {
      data_interface.Log(LogLevel::kError, "Unknown Mode");
      sensor_in_pose_ = Eigen::Affine3d::Identity();
      break;
    }
  }

  pose_list_.clear();
  processed_lidar_ = StampedCloud();

  return true;
}

const StampedCloud& LidarUndistortor::UndistortCloud(
    const StampedCloud& raw_lidar, std::deque<sensor_msgs::Imu>& imu_queue) {
  static DataInterface& data_interface = DataInterface::GetSingleton();

  if (imu_queue.empty()) {
    data_interface.Log(LogLevel::kWarn, "Imu queue is empty");
    processed_lidar_.time = ros::Time();
    return processed_lidar_;
  }

  if (imu_queue.front().header.stamp > raw_lidar.time) {
    data_interface.Log(LogLevel::kWarn, "Time of lidar is ahead of imu");
    processed_lidar_.time = ros::Time();
    return processed_lidar_;
  }

  CalculatePoses(raw_lidar.time, imu_queue);
  Undistort(raw_lidar);

  return processed_lidar_;
}

const StampedCloud& LidarUndistortor::UndistortCloud(
    const StampedCloud& raw_lidar,
    std::deque<nav_msgs::Odometry>& odom_queue) {
  static DataInterface& data_interface = DataInterface::GetSingleton();

  if (odom_queue.empty()) {
    data_interface.Log(LogLevel::kWarn, "Odom queue empty");
    processed_lidar_.time = ros::Time();
    return processed_lidar_;
  }

  if (odom_queue.front().header.stamp > raw_lidar.time) {
    data_interface.Log(LogLevel::kWarn, "Odom time error");
    processed_lidar_.time = ros::Time();
    return processed_lidar_;
  }

  CalculatePoses(raw_lidar.time, odom_queue);
  Undistort(raw_lidar);

  return processed_lidar_;
}

void LidarUndistortor::CalculatePoses(
    const ros::Time& lidar_time, std::deque<sensor_msgs::Imu>& imu_queue) {
  static double start_time = -2.0 * kparameter_sensor_.imu_period;
  static double keep_time =
      kparameter_sensor_.lidar_period - 8.0 * kparameter_sensor_.imu_period;
  static double end_time =
      kparameter_sensor_.lidar_period + kparameter_sensor_.imu_period;
  static std::vector<sensor_msgs::Imu> imu_list;
  imu_list.clear();
  pose_list_.clear();

  //---------------------------------------------------------------------------
  // pop imu data which is ahead of start time
  //---------------------------------------------------------------------------
  while ((imu_queue.front().header.stamp - lidar_time).toSec() < start_time) {
    imu_queue.pop_front();
  }

  //---------------------------------------------------------------------------
  // calculate poses which are ahead of end time
  //---------------------------------------------------------------------------
  // save first pose
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  StampedPose pose_temp;
  pose_temp.time = imu_queue.front().header.stamp;
  pose_temp.new_in_old = pose;
  pose_list_.emplace_back(pose_temp);

  // set up environment
  Eigen::Vector3d last_vel_ang(imu_queue.front().angular_velocity.x,
                               imu_queue.front().angular_velocity.y,
                               imu_queue.front().angular_velocity.z);
  imu_queue.pop_front();

  // calculate other poses which are ahead of end time
  while ((imu_queue.front().header.stamp - lidar_time).toSec() < end_time &&
         !imu_queue.empty()) {
    // calculate delta rotation
    double duration =
        (imu_queue.front().header.stamp - pose_temp.time).toSec();
    Eigen::Vector3d curr_vel_ang(imu_queue.front().angular_velocity.x,
                                 imu_queue.front().angular_velocity.y,
                                 imu_queue.front().angular_velocity.z);
    Eigen::Vector3d avg_vel_ang = 0.5 * (curr_vel_ang + last_vel_ang);
    Eigen::AngleAxisd delta_rotation = Eigen::AngleAxisd(
        avg_vel_ang.norm() * duration, avg_vel_ang.normalized());

    // calculate delta pose
    Eigen::Affine3d delta_pose = Eigen::Affine3d::Identity();
    delta_pose.matrix().block<3, 3>(0, 0) = delta_rotation.toRotationMatrix();

    // save current pose
    pose = pose * delta_pose;
    pose_temp.time = imu_queue.front().header.stamp;
    pose_temp.new_in_old = pose;
    pose_list_.emplace_back(pose_temp);

    // update environment
    last_vel_ang = curr_vel_ang;
    if ((imu_queue.front().header.stamp - lidar_time).toSec() > keep_time) {
      imu_list.emplace_back(imu_queue.front());
    }
    imu_queue.pop_front();
  }

  //---------------------------------------------------------------------------
  // finish last pose
  //---------------------------------------------------------------------------
  if (imu_queue.empty()) {
    // calculate delta rotation
    double duration = end_time - (pose_temp.time - lidar_time).toSec();
    Eigen::AngleAxisd delta_rotation = Eigen::AngleAxisd(
        last_vel_ang.norm() * duration, last_vel_ang.normalized());

    // calculate delta pose
    Eigen::Affine3d delta_pose = Eigen::Affine3d::Identity();
    delta_pose.matrix().block<3, 3>(0, 0) = delta_rotation.toRotationMatrix();

    // save final pose
    pose = pose * delta_pose;
    pose_temp.time = lidar_time + ros::Duration(end_time);
    pose_temp.new_in_old = pose;
    pose_list_.emplace_back(pose_temp);
  } else {
    // calculate delta rotation
    double duration =
        (imu_queue.front().header.stamp - pose_temp.time).toSec();
    Eigen::Vector3d curr_vel_ang(imu_queue.front().angular_velocity.x,
                                 imu_queue.front().angular_velocity.y,
                                 imu_queue.front().angular_velocity.z);
    Eigen::Vector3d avg_vel_ang = 0.5 * (curr_vel_ang + last_vel_ang);
    Eigen::AngleAxisd delta_rotation = Eigen::AngleAxisd(
        avg_vel_ang.norm() * duration, avg_vel_ang.normalized());

    // calculate delta pose
    Eigen::Affine3d delta_pose = Eigen::Affine3d::Identity();
    delta_pose.matrix().block<3, 3>(0, 0) = delta_rotation.toRotationMatrix();

    // save final pose
    pose = pose * delta_pose;
    pose_temp.time = imu_queue.front().header.stamp;
    pose_temp.new_in_old = pose;
    pose_list_.emplace_back(pose_temp);
  }

  //---------------------------------------------------------------------------
  // restore imu data
  //---------------------------------------------------------------------------
  for (auto it = imu_list.rbegin(); it != imu_list.rend(); ++it) {
    imu_queue.emplace_front(*it);
  }

  //---------------------------------------------------------------------------
  // transform poses from imu to lidar
  //---------------------------------------------------------------------------
  // Frame: i => imu, l => lidar
  // Time : 0 => lidar start time, s => imu start time, k => current time
  // sensor_in_pose_ => l_n_in_i_n
  Eigen::Affine3d i_s_in_l_0 =
      sensor_in_pose_.inverse() * CurrentTrans(lidar_time).inverse();
  for (auto& pose : pose_list_) {
    pose.new_in_old = i_s_in_l_0 * pose.new_in_old * sensor_in_pose_;
  }
}

void LidarUndistortor::CalculatePoses(
    const ros::Time& lidar_time,
    std::deque<nav_msgs::Odometry>& odom_queue) {
  static double start_time = -2.0 * kparameter_sensor_.odom_period;
  static double keep_time =
      kparameter_sensor_.lidar_period - 5.0 * kparameter_sensor_.odom_period;
  static double end_time =
      kparameter_sensor_.lidar_period + kparameter_sensor_.odom_period;
  static std::vector<nav_msgs::Odometry> odom_list;
  odom_list.clear();
  pose_list_.clear();

  //---------------------------------------------------------------------------
  // pop odom data which is ahead of start time
  //---------------------------------------------------------------------------
  while ((odom_queue.front().header.stamp - lidar_time).toSec() < start_time) {
    odom_queue.pop_front();
  }

  //---------------------------------------------------------------------------
  // calculate poses which are ahead of end time
  //---------------------------------------------------------------------------
  // calculate odom in base_0
  const auto& front_pose = odom_queue.front().pose.pose;
  Eigen::Affine3d front_base_in_odom = Eigen::Affine3d::Identity();
  front_base_in_odom.matrix().block<3, 3>(0, 0) =
      Eigen::Quaterniond(front_pose.orientation.w, front_pose.orientation.x,
                         front_pose.orientation.y, front_pose.orientation.z)
          .toRotationMatrix();
  front_base_in_odom.matrix().block<3, 1>(0, 3) =
      Eigen::Vector3d(front_pose.position.x, front_pose.position.y,
                      front_pose.position.z);
  Eigen::Affine3d odom_in_base_0 = front_base_in_odom.inverse();

  // save first pose
  StampedPose pose_temp;
  pose_temp.time = odom_queue.front().header.stamp;
  pose_temp.new_in_old = Eigen::Affine3d::Identity();
  pose_list_.emplace_back(pose_temp);

  // calculate other poses which are ahead of end time
  while ((odom_queue.front().header.stamp - lidar_time).toSec() < end_time &&
         !odom_queue.empty()) {
    // extract current base_in_odom
    const auto& curr_pose = odom_queue.front().pose.pose;
    Eigen::Affine3d curr_base_in_odom = Eigen::Affine3d::Identity();
    curr_base_in_odom.matrix().block<3, 3>(0, 0) =
        Eigen::Quaterniond(
            curr_pose.orientation.w, curr_pose.orientation.x,
            curr_pose.orientation.y, curr_pose.orientation.z)
            .toRotationMatrix();
    curr_base_in_odom.matrix().block<3, 1>(0, 3) =
        Eigen::Vector3d(curr_pose.position.x, curr_pose.position.y,
                        curr_pose.position.z);

    // save current pose
    pose_temp.time = odom_queue.front().header.stamp;
    pose_temp.new_in_old = odom_in_base_0 * curr_base_in_odom;
    pose_list_.emplace_back(pose_temp);

    // update environment
    if ((odom_queue.front().header.stamp - lidar_time).toSec() > keep_time) {
      odom_list.emplace_back(odom_queue.front());
    }
    odom_queue.pop_front();
  }

  //---------------------------------------------------------------------------
  // finish last pose
  //---------------------------------------------------------------------------
  if (odom_queue.empty()) {
    // calculate delta rotation and translation
    double duration = end_time - (pose_temp.time - lidar_time).toSec();
    Eigen::Vector3d vel_ang(odom_queue.front().twist.twist.angular.x,
                            odom_queue.front().twist.twist.angular.y,
                            odom_queue.front().twist.twist.angular.z);
    Eigen::Vector3d vel_lin(odom_queue.front().twist.twist.linear.x,
                            odom_queue.front().twist.twist.linear.y,
                            odom_queue.front().twist.twist.linear.z);
    Eigen::AngleAxisd delta_rotation =
        Eigen::AngleAxisd(vel_ang.norm() * duration, vel_ang.normalized());
    Eigen::Vector3d delta_translation = vel_lin * duration;

    // calculate delta pose
    Eigen::Affine3d delta_pose = Eigen::Affine3d::Identity();
    delta_pose.matrix().block<3, 3>(0, 0) = delta_rotation.toRotationMatrix();
    delta_pose.matrix().block<3, 1>(0, 3) = delta_translation;

    // save final pose
    pose_temp.time = lidar_time + ros::Duration(end_time);
    pose_temp.new_in_old = pose_list_.back().new_in_old * delta_pose;
    pose_list_.emplace_back(pose_temp);
  } else {
    // extract final base_in_odom
    const auto& last_pose = odom_queue.front().pose.pose;
    Eigen::Affine3d last_base_in_odom = Eigen::Affine3d::Identity();
    last_base_in_odom.matrix().block<3, 3>(0, 0) =
        Eigen::Quaterniond(last_pose.orientation.w, last_pose.orientation.x,
                           last_pose.orientation.y, last_pose.orientation.z)
            .toRotationMatrix();
    last_base_in_odom.matrix().block<3, 1>(0, 3) =
        Eigen::Vector3d(last_pose.position.x, last_pose.position.y,
                        last_pose.position.z);

    // save final pose
    pose_temp.time = odom_queue.front().header.stamp;
    pose_temp.new_in_old = odom_in_base_0 * last_base_in_odom;
    pose_list_.emplace_back(pose_temp);
  }

  //---------------------------------------------------------------------------
  // restore odom data
  //---------------------------------------------------------------------------
  for (auto it = odom_list.rbegin(); it != odom_list.rend(); ++it) {
    odom_queue.emplace_front(*it);
  }

  //---------------------------------------------------------------------------
  // transform poses from odom to lidar
  //---------------------------------------------------------------------------
  // Frame: b => base_link, l => lidar
  // Time : 0 => lidar start time, s => odom start time, k => current time
  // sensor_in_pose_ => l_n_in_b_n
  Eigen::Affine3d b_s_in_l_0 =
      sensor_in_pose_.inverse() * CurrentTrans(lidar_time).inverse();
  for (auto& pose : pose_list_) {
    pose.new_in_old = b_s_in_l_0 * pose.new_in_old * sensor_in_pose_;
  }
}

void LidarUndistortor::Undistort(const StampedCloud& raw_lidar) {
  static std::vector<pcl::PointCloud<PointXYZRTLT>::Ptr> cloud_list;

  processed_lidar_.time = raw_lidar.time;
  processed_lidar_.points->clear();

  size_t num_points = raw_lidar.points->size();
  cloud_list.clear();
  for (uint8_t i = 0; i < THREAD_NUM; i++) {
    pcl::PointCloud<PointXYZRTLT>::Ptr empty_cloud(
        new pcl::PointCloud<PointXYZRTLT>);
    cloud_list.emplace_back(empty_cloud);
  }

  omp_set_num_threads(THREAD_NUM);
#pragma omp parallel for
  for (size_t i = 0; i < num_points; ++i) {
    const auto& point = raw_lidar.points->at(i);
    Eigen::Affine3d l_k_in_l_0 =
        CurrentTrans(raw_lidar.time + ros::Duration(point.timestamp));
    Eigen::Vector3d point_in_l_k(point.x, point.y, point.z);
    Eigen::Vector3d point_in_l_0 = l_k_in_l_0 * point_in_l_k;

    PointXYZRTLT point_processed;
    point_processed.x = point_in_l_0.x();
    point_processed.y = point_in_l_0.y();
    point_processed.z = point_in_l_0.z();
    point_processed.reflectivity = point.reflectivity;
    point_processed.tag = point.tag;
    point_processed.line = point.line;
    point_processed.timestamp = point.timestamp;

    cloud_list.at(omp_get_thread_num())->push_back(point_processed);
  }

  for (uint8_t i = 0; i < THREAD_NUM; i++) {
    processed_lidar_.points->insert(processed_lidar_.points->end(),
                                    cloud_list.at(i)->begin(),
                                    cloud_list.at(i)->end());
  }
}

Eigen::Affine3d LidarUndistortor::CurrentTrans(const ros::Time& timestamp) {
  static DataInterface& data_interface = DataInterface::GetSingleton();

  if (pose_list_.empty()) {
    data_interface.Log(LogLevel::kError, "pose list empty");
    return Eigen::Affine3d::Identity();
  }

  if (timestamp < pose_list_.front().time) {
    data_interface.Log(LogLevel::kWarn,
                       "livox_preprocess: timestamp is too early: %fs earlier",
                       (timestamp - pose_list_.front().time).toSec());
    return Eigen::Affine3d::Identity();
  }

  if (timestamp > pose_list_.back().time) {
    size_t back_index = pose_list_.size() - 1;
    double alpha =
        (timestamp - pose_list_[back_index].time).toSec() /
        (pose_list_[back_index].time - pose_list_[back_index - 1].time)
            .toSec();
    return PredictTrans(pose_list_[back_index].new_in_old,
                        pose_list_[back_index - 1].new_in_old, alpha);
  }

  for (size_t i = 0; i < pose_list_.size() - 1; ++i) {
    if (timestamp >= pose_list_[i].time && timestamp < pose_list_[i + 1].time) {
      double alpha = (timestamp - pose_list_[i].time).toSec() /
                     (pose_list_[i + 1].time - pose_list_[i].time).toSec();

      return InterpolateTrans(pose_list_[i].new_in_old,
                              pose_list_[i + 1].new_in_old, alpha);
    }
  }

  return Eigen::Affine3d::Identity();
}

Eigen::Affine3d LidarUndistortor::InterpolateTrans(
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

Eigen::Affine3d LidarUndistortor::PredictTrans(const Eigen::Affine3d& trans_1,
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

}  // namespace preprocess
