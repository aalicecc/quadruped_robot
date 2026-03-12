cmake_minimum_required(VERSION 3.0.2)
project(pcd_preprocess)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    sensor_msgs
    livox_ros_driver2
    pcl_ros
    pcl_conversions
)
find_package(OpenMP REQUIRED)
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

add_executable(pcd_preprocess
    pcd_preprocess/main.cc
    pcd_preprocess/pcd_preprocess.cc
    pcd_preprocess/lidar_undistortor.cc
    pcd_preprocess/lidar_filter.cc
    pcd_preprocess/data_interface/data_interface.cc
)

add_dependencies(pcd_preprocess livox_ros_driver2_generate_messages_cpp)
target_compile_options(pcd_preprocess PRIVATE ${OpenMP_CXX_FLAGS})

target_link_libraries(pcd_preprocess
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenMP_CXX_FLAGS}
)

install(
    TARGETS pcd_preprocess
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
