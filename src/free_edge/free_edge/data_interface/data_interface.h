#ifndef FREE_EDGE_DATA_INTERFACE_DATA_INTERFACE_H_
#define FREE_EDGE_DATA_INTERFACE_DATA_INTERFACE_H_

#include "free_edge/data_interface/base_interface.h"
#include "sensor_msgs/PointCloud2.h"

namespace postprocess {

class DataInterface : public BaseInterface {
 public:
  static DataInterface& GetSingleton() {
    static DataInterface singleton;
    return singleton;
  }

  // Interface Handle
  void Log(LogLevel, const char*, ...) override;
  inline void Shutdown() override { ros::shutdown(); }
  inline bool Ok() override { return ros::ok(); }
  inline ros::Time GetTime() override { return ros::Time::now(); }
  inline void Work() override {
    timer_->reset();
    while (ros::ok()) {
      timer_handle_();
      timer_->sleep();
    }
  }

  // Publisher Handle
  void PublishObstacleCloud(const StampedCloud&) override;
  void PublishObstacleScan(const sensor_msgs::LaserScan&) override;

  // Initialization Handle
  void Init(int, char*[], std::string, double, void (*)()) override;
  void Deinit() override;

 protected:
  // Subscriber Handle
  void LidarHandle(const sensor_msgs::PointCloud2Ptr&);
  void OdomHandle(const nav_msgs::OdometryPtr&);

 private:
  DataInterface() = default;
  virtual ~DataInterface() = default;

  // Initialization Handle
  void ParameterInit() override;
  void VariableInit() override;
  void PublisherInit() override;
  void SubscriberInit() override;
  void TimerInit(double, void (*)()) override;

  // Node Handle
  ros::NodeHandle* nh_ptr_;
  ros::NodeHandle* nh_local_ptr_;

  // Timer Handle
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  std::unique_ptr<ros::Rate> timer_;

  // Publisher Handle
  ros::Publisher obstacle_scan_pub_;
  ros::Publisher obstacle_cloud_pub_;

  // Subscriber Handle
  ros::Subscriber lidar_sub_;
  ros::Subscriber odom_sub_;
};

}  // namespace postprocess

#endif  // FREE_EDGE_DATA_INTERFACE_DATA_INTERFACE_H_
