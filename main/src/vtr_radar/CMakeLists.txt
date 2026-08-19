cmake_minimum_required(VERSION 3.16)
project(vtr_radar)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pybind11 CONFIG QUIET)

find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(lgmath REQUIRED)
find_package(steam REQUIRED)

find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_tactic REQUIRED)
find_package(vtr_radar_msgs REQUIRED)

find_package(navtech_msgs REQUIRED)
find_package(spinning_radar_doppler REQUIRED)

if(DEFINED VTR_ENABLE_RADAR)

## C++ Libraries
include_directories(PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# components
file(GLOB_RECURSE COMPONENTS_SRC
  src/data_types/*.cpp
  src/utils/*.cpp
)
add_library(${PROJECT_NAME}_components ${COMPONENTS_SRC})
ament_target_dependencies(${PROJECT_NAME}_components
  Eigen3 OpenCV
  cv_bridge pcl_conversions pcl_ros
  nav_msgs
  lgmath steam spinning_radar_doppler
  vtr_common vtr_logging vtr_tactic vtr_radar_msgs
)

add_subdirectory(../deps/cvnp "${CMAKE_CURRENT_BINARY_DIR}/cvnp_build")
add_library(${PROJECT_NAME}_dro src/dro/dro_wrapper.cpp)

ament_target_dependencies(${PROJECT_NAME}_dro
  rclcpp
  cv_bridge
  pybind11
  ament_index_cpp
  Eigen3
)
target_link_libraries(${PROJECT_NAME}_dro cvnp)


# pipelines and modules
file(GLOB_RECURSE PIPELINE_SRC
  src/modules/*.cpp
  src/icp_pipeline.cpp
  src/direct_pipeline.cpp
)
add_library(${PROJECT_NAME}_pipeline ${PIPELINE_SRC})
target_link_libraries(${PROJECT_NAME}_pipeline ${PROJECT_NAME}_components)
target_link_libraries(${PROJECT_NAME}_pipeline ${PROJECT_NAME}_dro)
ament_target_dependencies(${PROJECT_NAME}_pipeline
  Eigen3 OpenCV
  cv_bridge pcl_conversions pcl_ros
  nav_msgs visualization_msgs
  lgmath steam
  vtr_common vtr_logging vtr_tactic vtr_radar_msgs navtech_msgs
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3 OpenCV
  pybind11 ament_index_cpp
  cv_bridge pcl_conversions pcl_ros
  nav_msgs visualization_msgs
  lgmath steam spinning_radar_doppler
  vtr_common vtr_logging vtr_tactic vtr_radar_msgs navtech_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(FILES
  dro/dro.py
  dro/dro_motion_models.py
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    cvnp
    ${PROJECT_NAME}_components
    ${PROJECT_NAME}_pipeline
    ${PROJECT_NAME}_dro
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()


endif()
ament_package()
