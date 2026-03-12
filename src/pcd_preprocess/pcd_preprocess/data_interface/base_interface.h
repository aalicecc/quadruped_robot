#ifndef PCD_PREPROCESS_DATA_INTERFACE_BASE_INTERFACE_H_
#define PCD_PREPROCESS_DATA_INTERFACE_BASE_INTERFACE_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <deque>

struct PointXYZRTLT {
  PCL_ADD_POINT4D
  float reflectivity;
  uint8_t tag = 0;
  uint8_t line = 0;
  double timestamp = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZRTLT,
    (float, x, x)(float, y, y)(float, z, z)(float, reflectivity, reflectivity)(
        uint8_t, tag, tag)(uint8_t, line, line)(double, timestamp, timestamp))

namespace preprocess {

enum class ModeUndistortion { kRawMode = 0, kImuMode, kOdomMode };

struct ParameterFilter {
  Eigen::Vector2d height;
  Eigen::Vector2d fov;
  Eigen::Vector2d distance;
};

struct ParameterFlag {
  ModeUndistortion mode = ModeUndistortion::kRawMode;
};

struct ParameterSensor {
  std::string frame;
  double lidar_period;
  double imu_period;
  double odom_period;
  Eigen::Matrix3d imu_in_sensor;
  Eigen::Affine3d sensor_in_base;
};

struct StampedCloud {
  ros::Time time;
  pcl::PointCloud<PointXYZRTLT>::Ptr points;
  StampedCloud() {
    this->time = ros::Time();
    this->points =
        pcl::PointCloud<PointXYZRTLT>::Ptr(new pcl::PointCloud<PointXYZRTLT>);
  }
  StampedCloud& operator=(const StampedCloud& that) {
    this->time = that.time;
    this->points = pcl::PointCloud<PointXYZRTLT>::Ptr(
        new pcl::PointCloud<PointXYZRTLT>(*that.points));
    return *this;
  }
};

struct StampedPose {
  ros::Time time;
  Eigen::Affine3d new_in_old;
  StampedPose() {
    this->time = ros::Time();
    this->new_in_old = Eigen::Affine3d::Identity();
  }
  StampedPose& operator=(const StampedPose& that) {
    this->time = that.time;
    this->new_in_old = Eigen::Affine3d(that.new_in_old);
    return *this;
  }
};

enum class LogLevel { kDebug = 0, kInfo, kWarn, kError, kFatal };

class BaseInterface {
 public:
  BaseInterface() : lidar_flag_(false), imu_flag_(false), odom_flag_(false) {}
  virtual ~BaseInterface() {}
  BaseInterface(const BaseInterface&) = delete;
  BaseInterface& operator=(const BaseInterface&) = delete;

  // Interface Handle
  virtual void Log(LogLevel, const char*, ...) = 0;
  virtual void Shutdown() = 0;
  virtual bool Ok() = 0;
  virtual ros::Time GetTime() = 0;
  virtual void Work() = 0;

  // Publisher Handle
  virtual void PublishRawCloud(const StampedCloud&) = 0;
  virtual void PublishProcessedCloud(const StampedCloud&) = 0;

  // Initialization Handle
  virtual void Init(int, char*[], std::string, double, void (*)()) = 0;
  virtual void Deinit() = 0;

  // Parameter Handle
  inline const ParameterFilter& GetParameterFilter() const {
    return kparameter_filter_;
  }
  inline const ParameterFlag& GetParameterFlag() const {
    return kparameter_flag_;
  }
  inline const ParameterSensor& GetParameterSensor() const {
    return kparameter_sensor_;
  }

  // Subscriber Handle
  inline bool GetLidarFlag() { return lidar_flag_; }
  inline void ResetLidarFlag() { lidar_flag_ = false; }
  inline const StampedCloud& GetLidar() {
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    return lidar_;
  }
  inline bool GetImuFlag() { return imu_flag_; }
  inline void ResetImuFlag() { imu_flag_ = false; }
  inline const std::vector<sensor_msgs::Imu>& GetImuBuffer() {
    static std::vector<sensor_msgs::Imu> imu_buffer;
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_buffer = std::vector<sensor_msgs::Imu>(imu_buffer_);
    imu_buffer_.clear();
    return imu_buffer;
  }
  inline void ResetImuBuffer() { imu_buffer_.clear(); }
  inline bool GetOdomFlag() { return odom_flag_; }
  inline void ResetOdomFlag() { odom_flag_ = false; }
  inline const std::vector<nav_msgs::Odometry>& GetOdomBuffer() {
    static std::vector<nav_msgs::Odometry> odom_buffer;
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_buffer = std::vector<nav_msgs::Odometry>(odom_buffer_);
    odom_buffer_.clear();
    return odom_buffer;
  }

 protected:
  // Initialization Handle
  virtual void ParameterInit() = 0;
  virtual void VariableInit() = 0;
  virtual void PublisherInit() = 0;
  virtual void SubscriberInit() = 0;
  virtual void TimerInit(double, void (*)()) = 0;

  // Time Handle
  void (*timer_handle_)();

  // Parameters
  ParameterFilter kparameter_filter_;
  ParameterFlag kparameter_flag_;
  ParameterSensor kparameter_sensor_;

  // Variables
  std::atomic<bool> lidar_flag_;
  mutable std::mutex lidar_mutex_;
  StampedCloud lidar_;
  std::atomic<bool> imu_flag_;
  mutable std::mutex imu_mutex_;
  std::vector<sensor_msgs::Imu> imu_buffer_;
  std::atomic<bool> odom_flag_;
  mutable std::mutex odom_mutex_;
  std::vector<nav_msgs::Odometry> odom_buffer_;
};

}  // namespace preprocess

#endif  // PCD_PREPROCESS_DATA_INTERFACE_BASE_INTERFACE_H_
