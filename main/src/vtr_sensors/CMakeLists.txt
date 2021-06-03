cmake_minimum_required(VERSION 3.16)
project(vtr_sensors)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pgr_triclops REQUIRED)
find_package(vtr_messages REQUIRED)
find_package(vtr_storage REQUIRED)

# Libraries
file(GLOB_RECURSE INDEX_SRC src/*.cpp)
add_library(${PROJECT_NAME}_index ${INDEX_SRC})
ament_target_dependencies(${PROJECT_NAME}_index
  rclcpp
  sensor_msgs
  OpenCV
  pgr_triclops
  vtr_messages
  vtr_storage
)
target_include_directories(${PROJECT_NAME}_index
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        TARGETS ${PROJECT_NAME}_index
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)


add_executable(BumblebeeDriver src/BumblebeeMain.cpp)
target_link_libraries(BumblebeeDriver ${PROJECT_NAME}_index dc1394)

add_executable(BumblebeeRecorder src/xb3_recorder.cpp)
target_link_libraries(BumblebeeRecorder ${PROJECT_NAME}_index)

add_executable(BumblebeeReplay src/xb3_replay.cpp)
target_link_libraries(BumblebeeReplay ${PROJECT_NAME}_index)

add_executable(BumblebeeCalibration src/xb3_calibration.cpp)
target_link_libraries(BumblebeeCalibration ${PROJECT_NAME}_index)

install(TARGETS
        BumblebeeDriver
        BumblebeeRecorder
        BumblebeeReplay
        BumblebeeCalibration
        DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    # Linting
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
