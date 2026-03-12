cmake_minimum_required(VERSION 3.0.2)
project(free_edge)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    sensor_msgs
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

add_executable(free_edge
    free_edge/main.cc
    free_edge/free_edge.cc
    free_edge/lidar_accumulate.cc
    free_edge/data_interface/data_interface.cc
)

target_compile_options(free_edge PRIVATE ${OpenMP_CXX_FLAGS})

target_link_libraries(free_edge
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${OpenMP_CXX_FLAGS}
)

install(
    TARGETS free_edge
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
