cmake_minimum_required(VERSION 3.16)
project(vtr_storage)

include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(test_msgs REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

find_package(vtr_logging REQUIRED)
find_package(vtr_messages REQUIRED)

# Libraries
file(
  GLOB_RECURSE SRC
  src/data_bubble.cpp
  src/data_stream_base.cpp
  src/random_access_reader.cpp
  src/sequential_append_writer.cpp
  src/data_stream_reader.cpp
)
add_library(${PROJECT_NAME} ${SRC})
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  pluginlib
  rcpputils
  rcutils
  rclcpp
  rosbag2_storage
  rosbag2_cpp
  test_msgs
  rosidl_runtime_c
  rosidl_runtime_cpp
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp
  vtr_logging
  vtr_messages
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  pluginlib
  rosbag2_storage
  rosbag2_cpp
  test_msgs
  rcutils
  rcpputils
  rclcpp
  rosidl_runtime_c
  rosidl_runtime_cpp
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp
  vtr_logging
  vtr_messages
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(read_write_append_test test/read_write_append_test.cpp)
  target_link_libraries(read_write_append_test ${PROJECT_NAME})
  ament_add_gtest(data_bubble_test test/data_bubble_test.cpp)
  target_link_libraries(data_bubble_test ${PROJECT_NAME})
  ament_add_gtest(read_write_calibration_test test/read_write_calibration_test.cpp)
  target_link_libraries(read_write_calibration_test ${PROJECT_NAME})
  ament_add_gtest(read_write_nullptr_test test/read_write_nullptr_test.cpp)
  target_link_libraries(read_write_nullptr_test ${PROJECT_NAME})
  ament_add_gtest(read_seek_test test/read_seek_test.cpp)
  target_link_libraries(read_seek_test ${PROJECT_NAME})
endif()

ament_package()
