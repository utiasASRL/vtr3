cmake_minimum_required(VERSION 3.16)
project(vtr_messages)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake.in")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ROS messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TimeStamp.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
