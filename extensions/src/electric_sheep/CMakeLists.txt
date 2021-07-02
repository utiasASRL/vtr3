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
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pgr_triclops REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

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


add_executable(DataRecorder src/data_recorder.cpp)
# target_link_libraries(BumblebeeDriver ${PROJECT_NAME}_index dc1394)
target_include_directories(DataRecorder
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(DataRecorder
  rclcpp
  sensor_msgs
  OpenCV
  pgr_triclops
  vtr_messages
  vtr_storage
)


add_executable(DataPlayer src/data_player.cpp)
target_include_directories(DataPlayer
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(DataPlayer
  rclcpp
  sensor_msgs
  OpenCV
  pgr_triclops
  tf2 tf2_ros tf2_eigen
  vtr_messages
  vtr_storage
)


add_executable(PCReplay src/pc_replay.cpp)
target_include_directories(PCReplay
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(PCReplay
  rclcpp
  sensor_msgs
  OpenCV
  pgr_triclops
  tf2 tf2_ros tf2_eigen
  vtr_messages
  vtr_storage
)


install(TARGETS
        DataRecorder
        DataPlayer
        PCReplay
        DESTINATION lib/${PROJECT_NAME})


# Python Package
# ament_python_install_package(${PROJECT_NAME})

# Python Scripts
file(
  GLOB_RECURSE PYTHON_SCRIPTS
  scripts/*.py
)
install(
  PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    # Linting
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
