cmake_minimum_required(VERSION 3.16)
project(electric_sheep)

# Common setup for vtr packages
## Compiler setup - assumed to be GNU
# Compile as C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Turn on as many warnings as possible by default.
add_compile_options(-march=native -O3 -pthread -Wall -Wextra)


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pgr_triclops REQUIRED)
find_package(vtr_messages REQUIRED)
find_package(vtr_storage REQUIRED)

# Libraries
# file(GLOB_RECURSE INDEX_SRC src/*.cpp)
# add_library(${PROJECT_NAME}_index ${INDEX_SRC})
# ament_target_dependencies(${PROJECT_NAME}_index
#   rclcpp
#   sensor_msgs
#   OpenCV
#   pgr_triclops
#   vtr_messages
#   vtr_storage
# )
# target_include_directories(${PROJECT_NAME}_index
#         PUBLIC
#         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#         $<INSTALL_INTERFACE:include>)

# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
        DIRECTORY include/
        DESTINATION include
)

# install(
#         TARGETS ${PROJECT_NAME}_index
#         EXPORT export_${PROJECT_NAME}
#         LIBRARY DESTINATION lib
#         ARCHIVE DESTINATION lib
#         RUNTIME DESTINATION bin
#         INCLUDES DESTINATION include
# )


add_executable(PCRecorder src/pc_recorder.cpp)
# target_link_libraries(BumblebeeDriver ${PROJECT_NAME}_index dc1394)
target_include_directories(PCRecorder
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(PCRecorder
  rclcpp
  sensor_msgs
  OpenCV
  pgr_triclops
  vtr_messages
  vtr_storage
)


add_executable(PCReplay src/pc_replay.cpp)
# target_link_libraries(BumblebeeDriver ${PROJECT_NAME}_index dc1394)
target_include_directories(PCReplay
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(PCReplay
  rclcpp
  sensor_msgs
  OpenCV
  pgr_triclops
  vtr_messages
  vtr_storage
)


install(TARGETS
        PCRecorder
        PCReplay
        DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    # Linting
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
