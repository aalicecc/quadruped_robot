cmake_minimum_required(VERSION 3.0.2)
project(pcd_localization)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    pcl_ros
    pcl_conversions
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
catkin_package()

include_directories(
    ../
    ./
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_executable(pcd_localization
    pcd_localization/main.cc
    pcd_localization/pcd_localization.cc
    pcd_localization/data_interface/data_interface.cc
)

target_link_libraries(pcd_localization
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

install(
    TARGETS pcd_localization
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/launch
)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/config
)