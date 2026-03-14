bag_name=$1
if [ -z "$bag_name" ]; then
    bag_name="mark2_test_$(date -d today +%Y%m%d_%H%M%S)"
fi

if [ ! -d "$(rospack find quadruped_robot)/bags" ]; then
    mkdir $(rospack find quadruped_robot)/bags
fi

rosbag record -O $(rospack find quadruped_robot)/bags/$bag_name /livox/imu /livox/lidar /odom /imu_data /magnetic_data
