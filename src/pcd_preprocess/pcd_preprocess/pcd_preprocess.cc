#include "pcd_preprocess/pcd_preprocess.h"
#include "pcd_preprocess/lidar_filter.h"
#include "pcd_preprocess/lidar_undistortor.h"

namespace preprocess {

bool PcdPreprocess::Init() {
  static DataInterface& data_interface = DataInterface::GetSingleton();
  static LidarUndistortor& lidar_undistortor = LidarUndistortor::GetSingleton();
  static LidarFilter& lidar_filter = LidarFilter::GetSingleton();

  // Parameter
  kparameter_sensor_ = data_interface.GetParameterSensor();
  kparameter_flag_ = data_interface.GetParameterFlag();

  // Variable
  raw_lidar_ = StampedCloud();
  processed_lidar_ = StampedCloud();
  imu_queue_.clear();
  odom_queue_.clear();

  // Other Components
  if (kparameter_flag_.mode != ModeUndistortion::kRawMode) {
    lidar_undistortor.Init();
  }
  lidar_filter.Init();

  return true;
}

bool PcdPreprocess::Work() {
  static DataInterface& data_interface = DataInterface::GetSingleton();

  switch (kparameter_flag_.mode) {
    case ModeUndistortion::kRawMode: {
      RawModeLoop();
      break;
    }
    case ModeUndistortion::kImuMode: {
      ImuModeLoop();
      break;
    }
    case ModeUndistortion::kOdomMode: {
      OdomModeLoop();
      break;
    }

    default: {
      data_interface.Log(LogLevel::kWarn, "### Wrong Mode ###");
      RawModeLoop();
      break;
    }
  }

  return true;
}

void PcdPreprocess::RawModeLoop() {
  static DataInterface& data_interface = DataInterface::GetSingleton();
  static LidarFilter& lidar_filter = LidarFilter::GetSingleton();

  if (data_interface.GetLidarFlag()) {
    raw_lidar_ = data_interface.GetLidar();
    data_interface.ResetLidarFlag();

    processed_lidar_ = raw_lidar_;
    if (!FilterCloud()) return;
    data_interface.PublishRawCloud(processed_lidar_);
  }
}

void PcdPreprocess::ImuModeLoop() {
  static DataInterface& data_interface = DataInterface::GetSingleton();
  static ros::Time imu_last_time;

  if (data_interface.GetImuFlag()) {
    std::vector<sensor_msgs::Imu> imu_vector(data_interface.GetImuBuffer());
    data_interface.ResetImuFlag();

    for (auto& imu : imu_vector) {
      if (imu.header.stamp < imu_last_time) {
        data_interface.Log(LogLevel::kWarn, "imu.time < imu_last_time");
        continue;
      }

      imu_last_time = imu.header.stamp;
      imu_queue_.emplace_back(imu);
    }
  }

  if (data_interface.GetLidarFlag()) {
    raw_lidar_ = data_interface.GetLidar();
    if (raw_lidar_.time == ros::Time()) {
      data_interface.Log(LogLevel::kError, "Lidar time error");
    }

    data_interface.ResetLidarFlag();

    if (!ImuUndistortCloud()) return;
    if (!FilterCloud()) return;
    data_interface.PublishProcessedCloud(processed_lidar_);
  }
}

void PcdPreprocess::OdomModeLoop() {
  static DataInterface& data_interface = DataInterface::GetSingleton();
  static ros::Time odom_last_time;

  if (data_interface.GetOdomFlag()) {
    std::vector<nav_msgs::Odometry> odom_vector(
        data_interface.GetOdomBuffer());
    data_interface.ResetOdomFlag();

    for (auto& odom : odom_vector) {
      if (odom.header.stamp < odom_last_time) {
        data_interface.Log(LogLevel::kWarn, "Odom time error");
        odom_queue_.clear();
      }

      odom_last_time = odom.header.stamp;
      odom_queue_.emplace_back(odom);
    }
  }

  if (data_interface.GetLidarFlag()) {
    raw_lidar_ = data_interface.GetLidar();
    data_interface.ResetLidarFlag();

    if (!OdomUndistortCloud()) return;
    if (!FilterCloud()) return;
    data_interface.PublishProcessedCloud(processed_lidar_);
  }
}

bool PcdPreprocess::ImuUndistortCloud() {
  static DataInterface& data_interface = DataInterface::GetSingleton();
  static LidarUndistortor& lidar_undistortor = LidarUndistortor::GetSingleton();

  processed_lidar_ = lidar_undistortor.UndistortCloud(raw_lidar_, imu_queue_);

  if (processed_lidar_.time == ros::Time()) {
    data_interface.Log(LogLevel::kWarn, "Undistort cloud failed");
    return false;
  }

  return true;
}

bool PcdPreprocess::OdomUndistortCloud() {
  static DataInterface& data_interface = DataInterface::GetSingleton();
  static LidarUndistortor& lidar_undistortor = LidarUndistortor::GetSingleton();

  processed_lidar_ = lidar_undistortor.UndistortCloud(raw_lidar_, odom_queue_);

  if (processed_lidar_.time == ros::Time()) {
    data_interface.Log(LogLevel::kWarn, "Undistort cloud failed");
    return false;
  }

  return true;
}

bool PcdPreprocess::FilterCloud() {
  static DataInterface& data_interface = DataInterface::GetSingleton();
  static LidarFilter& lidar_filter = LidarFilter::GetSingleton();

  processed_lidar_ = lidar_filter.FilterCloud(processed_lidar_);

  if (processed_lidar_.time == ros::Time()) {
    data_interface.Log(LogLevel::kWarn, "Filter cloud failed");
    return false;
  }

  return true;
}

}  // namespace preprocess
