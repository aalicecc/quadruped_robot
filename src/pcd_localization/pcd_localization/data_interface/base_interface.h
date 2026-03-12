#ifndef PCD_LOCALIZATION_DATA_INTERFACE_BASE_INTERFACE_H_
#define PCD_LOCALIZATION_DATA_INTERFACE_BASE_INTERFACE_H_

#include <ros/ros.h>

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

namespace localization {

struct ParametersFlag {
  bool auto_start;
  bool pure_lidar;
};

struct ParametersFrame {
  std::string map;
  std::string odom;
  std::string sensor;
};

struct ParametersInit {
  Eigen::Affine3f pose;
};

struct ParametersNdt {
  double trans_epsilon;
  double step_size;
  double resolution;
  int32_t max_iterations;
};

struct ParametersSource {
  double voxel_size;
  Eigen::Vector2d fov_threshold;
  Eigen::Vector2d distance_threshold;
};

struct ParametersTarget {
  std::string map_path;
  double voxel_size;
  double pcd_size;
  double update_distance;
};

struct StampedTrans {
  ros::Time time;
  Eigen::Affine3f transform;
  // constructor
  StampedTrans() {
    this->time = ros::Time();
    this->transform = Eigen::Affine3f::Identity();
  }
  StampedTrans(const ros::Time& time, const Eigen::Vector3f& translation,
                  const Eigen::Quaternionf& orientation) {
    this->time = time;
    this->transform = Eigen::Affine3f(orientation);
    this->transform.translation() = translation;
  }
  // copy operator
  StampedTrans& operator=(const StampedTrans& that) {
    this->time = that.time;
    this->transform = Eigen::Affine3f(that.transform);
    return *this;
  }
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
  BaseInterface() : init_flag_(false), lidar_flag_(false) {}
  virtual ~BaseInterface() = default;
  BaseInterface(const BaseInterface&) = delete;
  BaseInterface& operator=(const BaseInterface&) = delete;

  // Interface Handle
  virtual void Log(LogLevel, const char*, ...) = 0;
  virtual void Shutdown() = 0;
  virtual bool Ok() = 0;
  virtual ros::Time GetTime() = 0;
  virtual void Work() = 0;

  // Publisher Handle
  virtual void PublishSensorTrans(const StampedTrans&) = 0;
  virtual void PublishMapPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr&) = 0;
  virtual void PublishDebugPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr&,
                                  const std::string&) = 0;

  // Initialization Handle
  virtual void Init(int, char*[], std::string, double, void (*)()) = 0;
  virtual void Deinit() = 0;

  // Other Handle
  virtual void BroadcastMapToOdom(const Eigen::Affine3f&, ros::Time) = 0;
  virtual const StampedTrans& ListenFrameToSensor(const std::string&) = 0;

  // Parameter Handle
  inline const ParametersFlag& GetParametersFlag() const {
    return kparameters_flag_;
  }
  inline const ParametersFrame& GetParametersFrame() const {
    return kparameters_frame_;
  }
  inline const ParametersInit& GetParametersInit() const {
    return kparameters_init_;
  }
  inline const ParametersNdt& GetParametersNdt() const {
    return kparameters_ndt_;
  }
  inline const ParametersSource& GetParametersSource() const {
    return kparameters_source_;
  }
  inline const ParametersTarget& GetParametersTarget() const {
    return kparameters_target_;
  }

  // Subscriber Handle
  inline bool GetInitFlag() const { return init_flag_; }
  inline void ResetInitFlag() { init_flag_ = false; }
  inline const StampedTrans& GetInit() const {
    std::lock_guard<std::mutex> lock(init_mutex_);
    return init_;
  }
  inline bool GetLidarFlag() const { return lidar_flag_; }
  inline void ResetLidarFlag() { lidar_flag_ = false; }
  inline const StampedCloud& GetLidar() const {
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    return lidar_;
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
  ParametersFlag kparameters_flag_;
  ParametersFrame kparameters_frame_;
  ParametersInit kparameters_init_;
  ParametersNdt kparameters_ndt_;
  ParametersSource kparameters_source_;
  ParametersTarget kparameters_target_;

  // Variables
  std::atomic<bool> init_flag_;
  mutable std::mutex init_mutex_;
  StampedTrans init_;
  std::atomic<bool> lidar_flag_;
  mutable std::mutex lidar_mutex_;
  StampedCloud lidar_;
};

}  // namespace localization

#endif  // PCD_LOCALIZATION_DATA_INTERFACE_BASE_INTERFACE_H_
