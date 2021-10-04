cmake_minimum_required(VERSION 3.16)
project(vtr_storage)

# Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(Boost REQUIRED thread)

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

# Libraries
# storage - sqlite
file(GLOB_RECURSE STORAGE_SRC src/storage/*.cpp)
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
# accessor
file(GLOB_RECURSE ACCESSOR_SRC src/accessor/*.cpp)
add_library(${PROJECT_NAME}_accessor SHARED ${ACCESSOR_SRC})
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
  Boost
  rclcpp # serialization
  vtr_logging
)
target_link_libraries(${PROJECT_NAME}_stream
  ${PROJECT_NAME}_accessor
)
target_include_directories(${PROJECT_NAME}_stream
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Boost
  rcpputils rcutils
  rosidl_runtime_c
  rosidl_runtime_cpp
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp
  sqlite3_vendor SQLite3
  yaml_cpp_vendor
  vtr_logging
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
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_sqlite_wrapper test/storage/sqlite/test_sqlite_wrapper.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_sqlite_wrapper ${PROJECT_NAME}_storage)
  ament_add_gmock(test_sqlite_wrapper2 test/storage/sqlite/test_sqlite_wrapper2.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_sqlite_wrapper2 ${PROJECT_NAME}_storage)
  ament_add_gmock(test_sqlite_storage test/storage/sqlite/test_sqlite_storage.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_sqlite_storage ${PROJECT_NAME}_storage)
  ament_add_gmock(test_sqlite_storage2 test/storage/sqlite/test_sqlite_storage2.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_sqlite_storage2 ${PROJECT_NAME}_storage)

  ament_add_gmock(test_storage_accessor test/accessor/test_storage_accessor.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_storage_accessor ${PROJECT_NAME}_accessor)

  find_package(std_msgs REQUIRED)

  ament_add_gmock(test_message test/stream/test_message.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_message ${PROJECT_NAME}_stream)

  ament_add_gmock(test_data_stream_accessor test/stream/test_data_stream_accessor.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  ament_target_dependencies(test_data_stream_accessor std_msgs)
  target_link_libraries(test_data_stream_accessor ${PROJECT_NAME}_stream)

  ament_add_gmock(test_data_bubble test/stream/test_data_bubble.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  ament_target_dependencies(test_data_bubble std_msgs)
  target_link_libraries(test_data_bubble ${PROJECT_NAME}_stream)

endif()

ament_package()
