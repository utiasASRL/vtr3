cmake_minimum_required(VERSION 3.16)
project(vtr_navigation)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(PROJ REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)  # load frames
find_package(tf2_ros REQUIRED)

find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_pose_graph REQUIRED)
find_package(vtr_tactic REQUIRED)
find_package(vtr_path_planning REQUIRED)
find_package(vtr_mission_planning REQUIRED)
find_package(vtr_navigation_msgs REQUIRED)
find_package(vtr_torch REQUIRED)

find_package(VTK REQUIRED COMPONENTS vtkCommonCore vtkCommonDataModel)


## TODO make these two optional (depending on which pipeline to use)
find_package(vtr_radar_lidar REQUIRED)
find_package(vtr_lidar REQUIRED)
find_package(vtr_radar REQUIRED)
find_package(vtr_vision REQUIRED)


## C++ Libraries
include_directories(
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

file(GLOB_RECURSE SERVER_SRC src/graph_map_server.cpp)
add_library(${PROJECT_NAME}_graph_map_server ${SERVER_SRC})
ament_target_dependencies(${PROJECT_NAME}_graph_map_server
  PROJ
  rclcpp tf2 tf2_ros
  vtr_common vtr_logging vtr_tactic vtr_navigation_msgs
)

file(GLOB_RECURSE SERVER_SRC src/ros_mission_server.cpp)
add_library(${PROJECT_NAME}_mission_server ${SERVER_SRC})
ament_target_dependencies(${PROJECT_NAME}_mission_server
  rclcpp
  vtr_common vtr_logging vtr_mission_planning vtr_navigation_msgs
)

file(GLOB_RECURSE SERVER_SRC src/command_publisher.cpp)
add_library(${PROJECT_NAME}_command_publisher ${SERVER_SRC})
ament_target_dependencies(${PROJECT_NAME}_command_publisher
  rclcpp
  vtr_common vtr_logging vtr_mission_planning
)

file(GLOB_RECURSE SERVER_SRC src/task_queue_server.cpp)
add_library(${PROJECT_NAME}_task_queue_server ${SERVER_SRC})
ament_target_dependencies(${PROJECT_NAME}_task_queue_server
  rclcpp
  vtr_common vtr_logging vtr_mission_planning vtr_navigation_msgs
)

file(GLOB_RECURSE SRC src/navigator.cpp)
add_library(${PROJECT_NAME}_navigator ${SRC})


find_package(vtr_radar_lidar REQUIRED)
ament_target_dependencies(${PROJECT_NAME}_navigator
  rclcpp tf2 tf2_ros
  sensor_msgs  # for visualization only
  vtr_common vtr_logging vtr_pose_graph vtr_tactic vtr_mission_planning
  vtr_radar_lidar vtr_radar vtr_lidar vtr_vision
)

target_link_libraries(${PROJECT_NAME}_navigator
  ${PROJECT_NAME}_graph_map_server
  ${PROJECT_NAME}_mission_server
  ${PROJECT_NAME}_command_publisher
  ${PROJECT_NAME}_task_queue_server
)
target_include_directories(${PROJECT_NAME}_navigator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(DEFINED VTR_ENABLE_LIDAR)
  ament_export_dependencies(
    rclcpp tf2 tf2_ros
    vtr_common vtr_logging vtr_pose_graph vtr_tactic vtr_mission_planning
    vtr_navigation_msgs
    vtr_lidar
    PROJ
  )
elseif(DEFINED VTR_ENABLE_VISION)
  ament_export_dependencies(
    rclcpp tf2 tf2_ros
    vtr_common vtr_logging vtr_pose_graph vtr_tactic vtr_mission_planning
    vtr_navigation_msgs
    vtr_vision
    PROJ
  )
endif()


# main
add_executable(${PROJECT_NAME} src/main.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp sensor_msgs
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_navigator)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)


install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_graph_map_server
    ${PROJECT_NAME}_mission_server
    ${PROJECT_NAME}_command_publisher
    ${PROJECT_NAME}_task_queue_server
    ${PROJECT_NAME}_navigator
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Python Package
ament_python_install_package(${PROJECT_NAME})

# Python Scripts
file(
  GLOB_RECURSE PYTHON_SCRIPTS
  scripts/*.py
)
install(
  PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
