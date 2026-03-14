#ifndef FREE_EDGE_DATA_INTERFACE_BASE_INTERFACE_H_
#define FREE_EDGE_DATA_INTERFACE_BASE_INTERFACE_H_

# include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

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

namespace postprocess {

struct ParameterFilter {
  Eigen::Vector2d height;
  Eigen::Vector2d fov;
  Eigen::Vector2d distance;
  int32_t fov_num;
  int32_t distance_num;
  double fov_step;
  double distance_step;
};

struct ParameterFlag {
  bool pub_cloud;
};

struct ParameterObstacle {
  int32_t cloud_num;
  double change_distance;
  double far_range;
  double far_gap;
  double near_constant;
  double near_factor;
};

struct ParameterSensor {
  std::string frame;
  std::string scan_frame;
  int32_t repeat_times;
  Eigen::Affine3d orientation;
  Eigen::Affine3d sensor_in_base;
  double lidar_period;
  double odom_period;
  double angle_increment;
};

struct StampedCloud {
  ros::Time time;
  pcl::PointCloud<pcl::PointXYZ>::Ptr points;
  // constructor
  StampedCloud() {
    this->time = ros::Time();
    this->points =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  }
  // copy operator
  StampedCloud& operator=(const StampedCloud& that) {
    this->time = that.time;
    this->points = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>(*that.points));
    return *this;
  }
};

enum class LogLevel { kDebug = 0, kInfo, kWarn, kError, kFatal };

class BaseInterface {
 public:
  BaseInterface() : lidar_flag_(false), odom_flag_(false) {}
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
  virtual void PublishObstacleCloud(const StampedCloud&) = 0;
  virtual void PublishObstacleScan(const sensor_msgs::LaserScan&) = 0;

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
  inline const ParameterObstacle& GetParameterObstacle() const {
    return kparameter_obstacle_;
  }
  inline const ParameterSensor& GetParameterSensor() const {
    return kparameter_sensor_;
  }

  // Subscriber Handle
  inline bool GetLidarFlag() { return lidar_flag_; }
  inline void ResetLidarFlag() { lidar_flag_ = false; }
  inline const std::vector<StampedCloud>& GetLidarBuffer() {
    static std::vector<StampedCloud> lidar_buffer;
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    lidar_buffer = std::vector<StampedCloud>(lidar_buffer_);
    lidar_buffer_.clear();
    return lidar_buffer;
  }
  inline void ResetLidarBuffer() { lidar_buffer_.clear(); }
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

  // Timer Handle
  void (*timer_handle_)();

  // Parameters
  ParameterSensor kparameter_sensor_;
  ParameterFilter kparameter_filter_;
  ParameterObstacle kparameter_obstacle_;
  ParameterFlag kparameter_flag_;

  // Variables
  std::atomic<bool> lidar_flag_;
  mutable std::mutex lidar_mutex_;
  std::vector<StampedCloud> lidar_buffer_;
  std::atomic<bool> odom_flag_;
  mutable std::mutex odom_mutex_;
  std::vector<nav_msgs::Odometry> odom_buffer_;
};

}  // namespace postprocess

#endif  // FREE_EDGE_DATA_INTERFACE_BASE_INTERFACE_H_
