#ifndef PCD_LOCALIZATION_DATA_INTERFACE_DATA_INTERFACE_H_
#define PCD_LOCALIZATION_DATA_INTERFACE_DATA_INTERFACE_H_

#include "pcd_localization/data_interface/base_interface.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>

namespace localization {

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
  void PublishSensorTrans(const StampedTrans&) override;
  void PublishMapPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr&) override;
  void PublishDebugPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr&,
                          const std::string&) override;

  // Initialization Handle
  void Init(int, char*[], std::string, double, void (*)()) override;
  void Deinit() override;

  // Other Handle
  void BroadcastMapToOdom(const Eigen::Affine3f&, ros::Time) override;
  const StampedTrans& ListenFrameToSensor(const std::string&) override;

 protected:
  // Subscriber Handle
  void InitTransHandle(const geometry_msgs::PoseWithCovarianceStampedPtr&);
  void PointCloudHandle(const sensor_msgs::PointCloud2Ptr&);

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
  ros::Publisher sensor_trans_pub_;
  ros::Publisher map_points_pub_;
  ros::Publisher debug_points_pub_;

  // Subscriber Handle
  ros::Subscriber init_trans_sub_;
  ros::Subscriber lidar_points_sub_;
};

}  // namespace localization

#endif  // PCD_LOCALIZATION_DATA_INTERFACE_DATA_INTERFACE_H_
