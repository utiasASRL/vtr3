cmake_minimum_required(VERSION 3.16)
project(vtr_pose_graph)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake.in")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(lgmath REQUIRED)
find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_messages REQUIRED)
find_package(vtr_storage REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Libraries
# interface
file(
  GLOB_RECURSE INTERFACE_SRC
  src/interface/*.cpp
)
add_library(${PROJECT_NAME}_interface ${INTERFACE_SRC})
ament_target_dependencies(${PROJECT_NAME}_interface
  lgmath
  vtr_common
  vtr_logging
  vtr_messages
  vtr_storage
)
target_include_directories(${PROJECT_NAME}_interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
# index
file(
  GLOB_RECURSE INDEX_SRC
  src/index/*.cpp
  src/simple_graph/*.cpp
  src/evaluator/*.cpp
)
add_library(${PROJECT_NAME}_index ${INDEX_SRC})
ament_target_dependencies(${PROJECT_NAME}_index
  lgmath
  vtr_common
  vtr_logging
  vtr_messages
  vtr_storage
)
target_link_libraries(${PROJECT_NAME}_index ${PROJECT_NAME}_interface)
target_include_directories(${PROJECT_NAME}_index
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(lgmath)
ament_export_dependencies(vtr_common)
ament_export_dependencies(vtr_logging)
ament_export_dependencies(vtr_messages)
ament_export_dependencies(vtr_storage)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_interface
    ${PROJECT_NAME}_index
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(simple_graph_tests test/simple_graph_tests.cpp)
  # ament_target_dependencies(simple_graph_tests ${PROJECT_NAME}_index) # Does not work yet
  target_link_libraries(simple_graph_tests ${PROJECT_NAME}_index)
  ament_add_gtest(id_tests test/id_tests.cpp)
  target_link_libraries(id_tests ${PROJECT_NAME}_index)
  ament_add_gtest(read_write_tests test/read_write_tests.cpp)
  target_link_libraries(read_write_tests ${PROJECT_NAME}_index)
  ament_add_gtest(evaluator_tests test/evaluator_tests.cpp)
  target_link_libraries(evaluator_tests ${PROJECT_NAME}_index)
  ament_add_gtest(common_eval_tests test/common_eval_tests.cpp)
  target_link_libraries(common_eval_tests ${PROJECT_NAME}_index)
  ament_add_gtest(path_tests test/path_tests.cpp)
  target_link_libraries(path_tests ${PROJECT_NAME}_index)
  ament_add_gtest(subgraph_tests test/subgraph_tests.cpp)
  target_link_libraries(subgraph_tests ${PROJECT_NAME}_index)
  ament_add_gtest(composite_tests test/composite_tests.cpp)
  target_link_libraries(composite_tests ${PROJECT_NAME}_index)
  ament_add_gtest(serialization_tests test/serialization_tests.cpp)
  target_link_libraries(serialization_tests ${PROJECT_NAME}_index)

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml

  # Other
  add_executable(read_write_performance_tests
    test/read_write_performance_tests.cpp
  )
  ament_target_dependencies(read_write_performance_tests
    lgmath
    vtr_common
    vtr_logging
    vtr_messages
    vtr_storage
  )
  target_link_libraries(read_write_performance_tests ${PROJECT_NAME}_index)
  target_include_directories(read_write_performance_tests
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)

  add_executable(graph_read_write_performance_tests
    test/graph_read_write_performance_tests.cpp
  )
  ament_target_dependencies(graph_read_write_performance_tests
    lgmath
    vtr_common
    vtr_logging
    vtr_messages
    vtr_storage
  )
  target_link_libraries(graph_read_write_performance_tests ${PROJECT_NAME}_index)
  target_include_directories(graph_read_write_performance_tests
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)

  add_executable(read_write_tests_images
    test/read_write_tests_images.cpp
  )
  ament_target_dependencies(read_write_tests_images
    lgmath
    vtr_common
    vtr_logging
    vtr_messages
    vtr_storage
    cv_bridge
    sensor_msgs
    OpenCV
    rclcpp
    std_msgs
  )
  target_link_libraries(read_write_tests_images ${PROJECT_NAME}_index)
  target_include_directories(read_write_tests_images
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)

  install(
    TARGETS
      graph_read_write_performance_tests
      read_write_performance_tests
      read_write_tests_images
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
  )

endif()

ament_package()
