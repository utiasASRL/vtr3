cmake_minimum_required(VERSION 3.16)
project(vtr_mission_planning_msgs)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

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
  ${ACTION_SRC} ${MSG_SRC} ${SRV_SRC}
  DEPENDENCIES
    builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()