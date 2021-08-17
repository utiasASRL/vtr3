cmake_minimum_required(VERSION 3.5)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Project name
project(asrl__gamepad_control)

# Find other ROS packages using CMake (this defines macros)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(joy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED) #asrl__common


add_executable(gamepad_control_node src/gamepad_control_node.cpp)
target_include_directories(gamepad_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# add_executable(gamepad_trigger_node src/gamepad_trigger_node.cpp)
# target_include_directories(gamepad_trigger_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

ament_target_dependencies(gamepad_control_node
  rclcpp
  joy
  geometry_msgs
  sensor_msgs
  std_msgs
)

install(TARGETS gamepad_control_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

#add_executable(gamepad_twist_node src/gamepad_twist_node.cpp)
#target_link_libraries(gamepad_twist_node ${catkin_LIBRARIES})

#add_executable(gamepad_scalar_node src/gamepad_scalar_node.cpp)
#target_link_libraries(gamepad_scalar_node ${catkin_LIBRARIES})

#cmake_minimum_required(VERSION 2.4.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

## Set the build type.  Options are:
##  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
##  Debug          : w/ debug symbols, w/o optimization
##  Release        : w/o debug symbols, w/ optimization
##  RelWithDebInfo : w/ debug symbols, w/ optimization
##  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Release)

#rosbuild_init()

##set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
##set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

##uncomment if you have defined messages
##rosbuild_genmsg()
##uncomment if you have defined services
##rosbuild_gensrv()

##common commands for building c++ executables and libraries
##rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
##target_link_libraries(${PROJECT_NAME} another_library)
##rosbuild_add_boost_directories()
##rosbuild_link_boost(${PROJECT_NAME} thread)
##rosbuild_add_executable(example examples/example.cpp)
##target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_executable(gamepad_control_node src/gamepad_control_node.cpp)
#rosbuild_add_executable(gamepad_twist_node src/gamepad_twist_node.cpp)
#rosbuild_add_executable(gamepad_scalar_node src/gamepad_scalar_node.cpp)










