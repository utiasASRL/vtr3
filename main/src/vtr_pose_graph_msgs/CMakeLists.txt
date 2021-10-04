cmake_minimum_required(VERSION 3.16)
project(vtr_pose_graph_msgs)






## include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-march=native -O3 -pthread -Wall -Wextra -fPIC)
# add_compile_options(-fPIC)

## Common packages setup
# Boost requirement (by mission_planning but needed everywhere)
find_package(Boost REQUIRED COMPONENTS system thread)
# OpenMP - add flags in case we forget them somewhere
find_package(OpenMP)
if (OpenMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

## Compile for different ROS versions
if(DEFINED ENV{ROS_DISTRO})
  if("$ENV{ROS_DISTRO}" STREQUAL "foxy")
    add_definitions(-DVTR_ROS_FOXY)
  endif()
endif()

add_definitions(-DVTR_ENABLE_CAMERA)
add_definitions(-DVTR_ENABLE_LIDAR)
add_definitions(-DVTR_ENABLE_GPUSURF)  # set the available flag
set(OpenCV_DIR /usr/local/opencv_cuda/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)













# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ROS messages
file(
  GLOB_RECURSE MSG_SRC
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  msg/*.msg
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_SRC}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  std_srvs
)

ament_package()