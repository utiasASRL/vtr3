cmake_minimum_required(VERSION 3.16)
project(vtr_messages)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ROS messages
file(
  GLOB_RECURSE ACTION_SRC
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  action/*.action
)
file(
  GLOB_RECURSE MSG_SRC
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  msg/*.msg
)
file(
  GLOB_RECURSE SRV_SRC
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  srv/*.srv
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${ACTION_SRC}
  ${MSG_SRC}
  ${SRV_SRC}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  std_srvs
)

ament_package()