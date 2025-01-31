cmake_minimum_required(VERSION 3.16)
project(vtr_virtual_teach)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(PCL 1.2 REQUIRED)

find_package(lgmath REQUIRED)
find_package(steam REQUIRED)
find_package(rclcpp REQUIRED)

find_package(vtr_common REQUIRED)
find_package(vtr_tactic)
find_package(vtr_logging REQUIRED)
find_package(vtr_storage REQUIRED)
find_package(vtr_storage REQUIRED)
find_package(vtr_pose_graph REQUIRED)
find_package(vtr_pose_graph_msgs REQUIRED)
find_package(vtr_lidar REQUIRED)
find_package(vtr_lidar_msgs)
find_package(Python3 REQUIRED COMPONENTS Development)

find_package(Boost REQUIRED COMPONENTS system chrono timer thread)

# Include PCL directories
include_directories(${PCL_INCLUDE_DIRS} ${Python3_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Libraries
include_directories(
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Libraries
file(GLOB_RECURSE SRC src/*.cpp)
add_library(${PROJECT_NAME} ${SRC})
ament_target_dependencies(${PROJECT_NAME}
  Boost
  lgmath steam
  vtr_common vtr_logging vtr_storage vtr_pose_graph vtr_pose_graph_msgs vtr_lidar vtr_tactic vtr_lidar_msgs
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  lgmath steam
  vtr_common vtr_logging vtr_storage vtr_pose_graph_msgs vtr_lidar vtr_tactic vtr_lidar_msgs
)

# Create the executable
add_executable(generate_global_map src/generate_global_map.cpp)
ament_target_dependencies(generate_global_map
  rclcpp
  vtr_lidar
  vtr_logging
  vtr_storage
  vtr_pose_graph
  vtr_tactic
  vtr_lidar_msgs
)
target_link_libraries(generate_global_map ${PCL_LIBRARIES} ${Python3_LIBRARIES})
install(TARGETS generate_global_map DESTINATION lib/${PROJECT_NAME})

# Add the new executable and its dependencies
add_executable(warthog_path_extractor src/warthog_path_extractor.cpp)
ament_target_dependencies(warthog_path_extractor
  rclcpp
  vtr_lidar
  vtr_logging
  vtr_pose_graph
  vtr_tactic
  vtr_lidar_msgs
)
install(TARGETS warthog_path_extractor DESTINATION lib/${PROJECT_NAME})


# Install header files and the executable
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()