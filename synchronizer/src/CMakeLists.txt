cmake_minimum_required(VERSION 2.8.3)
project(synchronizer)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_filters #//ADD
 # OpenCV
  cv_bridge 
  image_transport
  pcl_ros
)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

#catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS cv_bridge roscpp sensor_msgs pcl_ros  std_msgs
#)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${OpenCV_INCLUDE_DIRS})
include_directories(include ${PCL_INCLUDE_DIRS})

add_executable(synchronizer synchronizer.cpp)
target_link_libraries (synchronizer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

