cmake_minimum_required(VERSION 3.16)
project(vtr_tactic)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")


## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)

find_package(rclcpp REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(steam REQUIRED)

find_package(vtr_tactic_msgs REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_pose_graph REQUIRED)


## C++ Libraries
include_directories(
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

file(GLOB_RECURSE SRC
  src/pipeline_interface.cpp
  src/storables.cpp
  src/tactic.cpp
  src/task_queue.cpp
  src/types.cpp
)
add_library(${PROJECT_NAME}_tactic ${SRC})
ament_target_dependencies(${PROJECT_NAME}_tactic
  Boost
  rclcpp tf2 tf2_ros tf2_eigen
  sensor_msgs nav_msgs  # visualization
  lgmath steam
  vtr_tactic_msgs vtr_logging vtr_pose_graph
)

file(GLOB_RECURSE PIPELINES_SRC src/modules/*.cpp)
add_library(${PROJECT_NAME}_pipelines ${PIPELINES_SRC})
target_link_libraries(${PROJECT_NAME}_pipelines ${PROJECT_NAME}_tactic)

file(GLOB_RECURSE CALLBACK_SRC src/rviz_tactic_callback.cpp)
add_library(${PROJECT_NAME}_tactic_callback ${CALLBACK_SRC})
target_link_libraries(${PROJECT_NAME}_tactic_callback ${PROJECT_NAME}_tactic)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Boost
  rclcpp tf2 tf2_ros tf2_eigen
  sensor_msgs nav_msgs  # visualization
  lgmath steam
  vtr_tactic_msgs vtr_logging vtr_pose_graph
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_tactic
    ${PROJECT_NAME}_pipelines
    ${PROJECT_NAME}_tactic_callback
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # task queue tests
  ament_add_gtest(test_task_queue test/task_queues/test_task_queue.cpp)
  target_link_libraries(test_task_queue ${PROJECT_NAME}_pipelines)
  ament_add_gtest(test_task_executor test/task_queues/test_task_executor.cpp)
  target_link_libraries(test_task_executor ${PROJECT_NAME}_pipelines)

  # tactic tests
  ament_add_gtest(test_query_cache test/tactic/test_query_cache.cpp)
  target_link_libraries(test_query_cache ${PROJECT_NAME}_pipelines)
  ament_add_gtest(test_query_buffer test/tactic/test_query_buffer.cpp)
  target_link_libraries(test_query_buffer ${PROJECT_NAME}_pipelines)
  ament_add_gtest(test_tactic_concurrency test/tactic/test_tactic_concurrency.cpp)
  target_link_libraries(test_tactic_concurrency ${PROJECT_NAME}_pipelines)

  # pipeline and module tests
  ament_add_gtest(test_module test/pipeline/test_module.cpp)
  target_link_libraries(test_module ${PROJECT_NAME}_pipelines)

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
