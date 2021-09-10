cmake_minimum_required(VERSION 3.16)
project(vtr_storage)

include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(sqlite3_vendor REQUIRED)
find_package(SQLite3 REQUIRED)  # provided by sqlite3_vendor

find_package(yaml_cpp_vendor REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)

find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

find_package(rmw REQUIRED)
find_package(rmw_fastrtps_cpp REQUIRED)

find_package(vtr_logging REQUIRED)
find_package(vtr_messages REQUIRED)

# Libraries
# storage - sqlite, and converter - cdr
file(GLOB_RECURSE STORAGE_SRC
  src/storage/*.cpp
  src/converter/*.cpp
  src/ros_helper.cpp
  src/metadata_io.cpp
  src/typesupport_helpers.cpp
)
add_library(${PROJECT_NAME}_storage SHARED ${STORAGE_SRC})
ament_target_dependencies(${PROJECT_NAME}_storage
  ament_index_cpp
  rcpputils rcutils
  rosidl_runtime_c
  rosidl_runtime_cpp
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp
  rmw
  SQLite3 yaml_cpp_vendor
)
target_include_directories(${PROJECT_NAME}_storage
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# accessors
file(GLOB_RECURSE ACCESSOR_SRC src/accessor/*.cpp)
add_library(${PROJECT_NAME}_accessor SHARED ${ACCESSOR_SRC})
# ament_target_dependencies(${PROJECT_NAME}_accessor)
target_link_libraries(${PROJECT_NAME}_accessor ${PROJECT_NAME}_storage)
target_include_directories(${PROJECT_NAME}_accessor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# stream
file(GLOB_RECURSE STREAM_SRC src/stream/*.cpp)
add_library(${PROJECT_NAME}_stream ${STREAM_SRC})
ament_target_dependencies(${PROJECT_NAME}_stream
  rclcpp # serialization
  vtr_logging vtr_messages
)
target_link_libraries(${PROJECT_NAME}_stream
  ${PROJECT_NAME}_storage
  ${PROJECT_NAME}_accessor
)
target_include_directories(${PROJECT_NAME}_stream
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rcpputils rcutils
  rosidl_runtime_c
  rosidl_runtime_cpp
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp
  sqlite3_vendor SQLite3
  yaml_cpp_vendor
  vtr_logging vtr_messages
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_storage
    ${PROJECT_NAME}_accessor
    ${PROJECT_NAME}_stream
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(test_msgs REQUIRED)
  ament_add_gtest(data_bubble_test test/data_bubble_test.cpp)
  ament_target_dependencies(data_bubble_test test_msgs)
  target_link_libraries(data_bubble_test ${PROJECT_NAME}_stream)
  ament_add_gtest(read_write_calibration_test test/read_write_calibration_test.cpp)
  ament_target_dependencies(read_write_calibration_test test_msgs)
  target_link_libraries(read_write_calibration_test ${PROJECT_NAME}_stream)
  ament_add_gtest(read_write_append_test test/read_write_append_test.cpp)
  ament_target_dependencies(read_write_append_test test_msgs)
  target_link_libraries(read_write_append_test ${PROJECT_NAME}_stream)
  ament_add_gtest(read_write_nullptr_test test/read_write_nullptr_test.cpp)
  ament_target_dependencies(read_write_nullptr_test test_msgs)
  target_link_libraries(read_write_nullptr_test ${PROJECT_NAME}_stream)
  ament_add_gtest(read_seek_test test/read_seek_test.cpp)
  ament_target_dependencies(read_seek_test test_msgs)
  target_link_libraries(read_seek_test ${PROJECT_NAME}_stream)
endif()

ament_package()
