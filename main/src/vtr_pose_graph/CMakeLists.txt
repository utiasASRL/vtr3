cmake_minimum_required(VERSION 3.16)
project(vtr_pose_graph)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

# Find dependencies
find_package(Boost REQUIRED COMPONENTS system) # TODO required by steam, check if we can drop these dependencies
find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(lgmath REQUIRED)
find_package(steam REQUIRED)
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
  src/path/*.cpp
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
# relaxation
file(
  GLOB_RECURSE RELAXATION_SRC
  src/relaxation/*.cpp
)
add_library(${PROJECT_NAME}_relaxation ${RELAXATION_SRC})
ament_target_dependencies(${PROJECT_NAME}_relaxation
  lgmath
  steam
)
target_link_libraries(${PROJECT_NAME}_relaxation ${PROJECT_NAME}_index)
target_include_directories(${PROJECT_NAME}_relaxation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  lgmath
  steam
  vtr_common
  vtr_logging
  vtr_messages
  vtr_storage
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_interface
    ${PROJECT_NAME}_index
    ${PROJECT_NAME}_relaxation
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
  ament_add_gtest(data_read_write_tests test/data_read_write_tests.cpp)
  target_link_libraries(data_read_write_tests ${PROJECT_NAME}_index)
  ament_add_gtest(data_read_write_tests2 test/data_read_write_tests2.cpp)
  target_link_libraries(data_read_write_tests2 ${PROJECT_NAME}_index)
  ament_add_gtest(graph_read_write_tests test/graph_read_write_tests.cpp)
  target_link_libraries(graph_read_write_tests ${PROJECT_NAME}_index)
  ament_add_gtest(graph_read_write_tests2 test/graph_read_write_tests2.cpp)
  target_link_libraries(graph_read_write_tests2 ${PROJECT_NAME}_index)
  ament_add_gtest(graph_structure_tests test/graph_structure_tests.cpp)
  target_link_libraries(graph_structure_tests ${PROJECT_NAME}_index)
  ament_add_gtest(evaluator_tests test/evaluator_tests.cpp)
  target_link_libraries(evaluator_tests ${PROJECT_NAME}_index)
  ament_add_gtest(evaluator_common_tests test/evaluator_common_tests.cpp)
  target_link_libraries(evaluator_common_tests ${PROJECT_NAME}_index)
  ament_add_gtest(path_tests test/path_tests.cpp)
  target_link_libraries(path_tests ${PROJECT_NAME}_index)
  ament_add_gtest(subgraph_tests test/subgraph_tests.cpp)
  target_link_libraries(subgraph_tests ${PROJECT_NAME}_index)
  ament_add_gtest(composite_tests test/composite_tests.cpp)
  target_link_libraries(composite_tests ${PROJECT_NAME}_index)
  ament_add_gtest(relaxation_tests test/relaxation_tests.cpp)
  target_link_libraries(relaxation_tests ${PROJECT_NAME}_relaxation)
  ament_add_gtest(loc_retrieval_tests test/loc_retrieval_tests.cpp)
  target_link_libraries(loc_retrieval_tests ${PROJECT_NAME}_index)

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml

  # Other
  add_executable(data_read_write_performance_tests
    test/data_read_write_performance_tests.cpp
  )
  ament_target_dependencies(data_read_write_performance_tests
    lgmath
    vtr_common
    vtr_logging
    vtr_messages
    vtr_storage
  )
  target_link_libraries(data_read_write_performance_tests ${PROJECT_NAME}_index)
  target_include_directories(data_read_write_performance_tests
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

  add_executable(data_read_write_performance_tests2
    test/data_read_write_performance_tests2.cpp
  )
  ament_target_dependencies(data_read_write_performance_tests2
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
  target_link_libraries(data_read_write_performance_tests2 ${PROJECT_NAME}_index)
  target_include_directories(data_read_write_performance_tests2
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)

  install(
    TARGETS
      graph_read_write_performance_tests
      data_read_write_performance_tests
      data_read_write_performance_tests2
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
  )

endif()

ament_package()
