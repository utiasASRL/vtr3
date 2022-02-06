cmake_minimum_required(VERSION 3.16)
project(vtr_radar_msgs)

# Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(vtr_common_msgs REQUIRED)

# ROS messages
file(
  GLOB_RECURSE MSG_SRC
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  msg/*.msg
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_SRC}
  DEPENDENCIES builtin_interfaces sensor_msgs vtr_common_msgs
)

ament_export_dependencies(
  rosidl_default_runtime
  sensor_msgs
)

ament_package()