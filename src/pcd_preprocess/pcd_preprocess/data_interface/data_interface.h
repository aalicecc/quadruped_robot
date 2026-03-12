#ifndef PCD_PREPROCESS_DATA_INTERFACE_DATA_INTERFACE_H_
#define PCD_PREPROCESS_DATA_INTERFACE_DATA_INTERFACE_H_

#include "pcd_preprocess/data_interface/base_interface.h"
#include "livox_ros_driver2/CustomMsg.h"
#include "sensor_msgs/PointCloud2.h"

namespace preprocess {

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
  void PublishRawCloud(const StampedCloud&) override;
  void PublishProcessedCloud(const StampedCloud&) override;

  // Initialization Handle
  void Init(int, char*[], std::string, double, void (*)()) override;
  void Deinit() override;

 protected:
  // Subscriber Handle
  void LidarHandle(const livox_ros_driver2::CustomMsgPtr&);
  void ImuHandle(const sensor_msgs::ImuPtr&);
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
  ros::Publisher raw_cloud_pub_;
  ros::Publisher processed_cloud_pub_;

  // Subscriber Handle
  ros::Subscriber lidar_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber odom_sub_;
};

}  // namespace preprocess

#endif  // PCD_PREPROCESS_DATA_INTERFACE_DATA_INTERFACE_H_
