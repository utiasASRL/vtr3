cmake_minimum_required(VERSION 3.16)
project(vtr_tactic)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

## Find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_python REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

find_package(rclcpp REQUIRED)
# find_package(rclcpp_action REQUIRED)
# find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(PCL COMPONENTS common REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)

find_package(lgmath REQUIRED)
find_package(steam REQUIRED)
find_package(gpusurf REQUIRED)  # currently assuming this is required
add_definitions(-DGPUSURF_ENABLED=1)

find_package(vtr_lidar REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_messages REQUIRED)
find_package(vtr_messages_lidar REQUIRED)
find_package(vtr_pose_graph REQUIRED)
find_package(vtr_path_tracker REQUIRED)
find_package(vtr_mission_planning REQUIRED)
find_package(vtr_vision REQUIRED)
find_package(vtr_steam_extensions REQUIRED)
find_package(vtr_lgmath_extensions REQUIRED)


## C++ Libraries
file(GLOB_RECURSE SRC
  src/modules/*.cpp
  src/pipelines/*.cpp
  src/memory_manager/*.cpp
  src/caches.cpp
  src/tactic.cpp
  src/visualize.cpp
)

add_library(${PROJECT_NAME} ${SRC})

ament_target_dependencies(${PROJECT_NAME}
  #
  rclcpp
  tf2 tf2_ros tf2_eigen
  # visualization
  OpenCV
  PCL pcl_conversions sensor_msgs nav_msgs
  #
  lgmath steam gpusurf
  # vtr
  vtr_lidar vtr_logging vtr_messages vtr_pose_graph vtr_messages_lidar
  vtr_vision vtr_steam_extensions vtr_lgmath_extensions vtr_mission_planning
  vtr_path_tracker
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  nav_msgs
  vtr_pose_graph
  vtr_path_tracker
  vtr_mission_planning
  vtr_lidar
  vtr_messages
  vtr_messages_lidar
  vtr_vision
  vtr_lgmath_extensions
  vtr_steam_extensions
)

# # main
# add_executable(${PROJECT_NAME} src/main.cpp)
# ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs)
# target_link_libraries(${PROJECT_NAME}
#   ${PROJECT_NAME}_module
# )
# target_include_directories(${PROJECT_NAME}
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>)

# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# # ament_export_dependencies()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# install(
#   TARGETS
#     ${PROJECT_NAME}
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
#   INCLUDES DESTINATION include
# )

# Python Package
# ament_python_install_package(${PROJECT_NAME})

# Python Scripts
# file(
#   GLOB_RECURSE PYTHON_SCRIPTS
#   scripts/*.py
# )
# install(
#   PROGRAMS ${PYTHON_SCRIPTS}
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install launch files.
# install(
#   DIRECTORY
#     launch
#     config
#   DESTINATION share/${PROJECT_NAME}/
# )

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(state_machine_tests test/state_machine_tests.cpp)
  # target_include_directories(state_machine_tests PUBLIC test)
  # target_link_libraries(state_machine_tests ${PROJECT_NAME}_state_machine)

  # C++ node tests

  # Python node tests

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
