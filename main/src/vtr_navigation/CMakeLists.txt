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
find_package(tf2_geometry_msgs REQUIRED)

find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_pose_graph REQUIRED)
find_package(vtr_tactic REQUIRED)
find_package(vtr_path_planning REQUIRED)
find_package(vtr_mission_planning REQUIRED)
find_package(vtr_navigation_msgs REQUIRED)
find_package(vtr_torch REQUIRED)
find_package(yaml-cpp REQUIRED)

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

find_package(vtr_radar_lidar REQUIRED)
find_package(vtr_route_planning REQUIRED)

# Thin wait-policy library (no Navigator / ROS sensors) for offline W* benches.
# POSITION_INDEPENDENT_CODE required so this static lib can link into the
# pybind shared module (vtr_wait_strategy_py).
add_library(${PROJECT_NAME}_wait_policy
  src/survival_model.cpp
  src/wait_strategy.cpp
  src/obstacle_memory.cpp
  src/obstacle_stats.cpp
)
set_target_properties(${PROJECT_NAME}_wait_policy PROPERTIES
  POSITION_INDEPENDENT_CODE ON)
ament_target_dependencies(${PROJECT_NAME}_wait_policy
  vtr_common vtr_logging vtr_tactic vtr_route_planning
)
target_link_libraries(${PROJECT_NAME}_wait_policy yaml-cpp)
target_include_directories(${PROJECT_NAME}_wait_policy
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# HSHMAT: Navigator library with survival model, wait strategy, obstacle memory, stats, and real-world logging
add_library(${PROJECT_NAME}_navigator
  src/navigator.cpp
  src/real_world_logger.cpp
)
ament_target_dependencies(${PROJECT_NAME}_navigator
  rclcpp tf2 tf2_ros
  sensor_msgs  # for visualization only
  vtr_common vtr_logging vtr_pose_graph vtr_tactic vtr_mission_planning
  vtr_radar_lidar vtr_radar vtr_lidar vtr_vision
  vtr_route_planning
)

target_link_libraries(${PROJECT_NAME}_navigator
  yaml-cpp
  ${PROJECT_NAME}_wait_policy
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
    vtr_navigation_msgs vtr_route_planning
    vtr_lidar
    PROJ
  )
elseif(DEFINED VTR_ENABLE_VISION)
  ament_export_dependencies(
    rclcpp tf2 tf2_ros
    vtr_common vtr_logging vtr_pose_graph vtr_tactic vtr_mission_planning
    vtr_navigation_msgs vtr_route_planning
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
    ${PROJECT_NAME}_wait_policy
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

# ------------------ Scalability W* microbench (offline) ------------------
add_executable(wait_strategy_scalability_bench src/wait_strategy_scalability_bench.cpp)
ament_target_dependencies(wait_strategy_scalability_bench
  vtr_common vtr_logging vtr_tactic vtr_route_planning)
target_link_libraries(wait_strategy_scalability_bench
  ${PROJECT_NAME}_wait_policy
  yaml-cpp)
target_include_directories(wait_strategy_scalability_bench
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
install(TARGETS wait_strategy_scalability_bench
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# ------------------ Python hybrid: episodes call C++ W* ------------------
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG QUIET)
if(NOT pybind11_FOUND)
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -m pybind11 --cmakedir
    OUTPUT_VARIABLE _pybind11_cmakedir
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  if(_pybind11_cmakedir)
    list(APPEND CMAKE_PREFIX_PATH "${_pybind11_cmakedir}")
    find_package(pybind11 CONFIG QUIET)
  endif()
endif()
if(NOT pybind11_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v2.11.1
  )
  FetchContent_MakeAvailable(pybind11)
endif()

if(TARGET pybind11::module OR pybind11_FOUND)
  # Compile wait-policy + EW-TDSP + easylogging into the MODULE with -fPIC.
  # Do not link workspace .a archives (built without -fPIC).
  set(_VTR_SRC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
  pybind11_add_module(vtr_wait_strategy_py MODULE
    src/wait_strategy_pybind.cpp
    src/survival_model.cpp
    src/wait_strategy.cpp
    src/obstacle_memory.cpp
    src/obstacle_stats.cpp
    ${_VTR_SRC_ROOT}/vtr_route_planning/src/ew_tdsp_planner.cpp
    ${_VTR_SRC_ROOT}/vtr_logging/src/easylogging++.cc
    ${_VTR_SRC_ROOT}/vtr_logging/src/configure.cpp
  )
  target_compile_definitions(vtr_wait_strategy_py PRIVATE
    ELPP_NO_DEFAULT_LOG_FILE
    ELPP_DISABLE_INFO_LOGS
    ELPP_DISABLE_DEBUG_LOGS
    ELPP_DISABLE_TRACE_LOGS
    ELPP_DISABLE_VERBOSE_LOGS)
  # Reuse the include graph already resolved for wait_policy (ROS + VTR headers).
  target_include_directories(vtr_wait_strategy_py PRIVATE
    $<TARGET_PROPERTY:${PROJECT_NAME}_wait_policy,INCLUDE_DIRECTORIES>
    ${_VTR_SRC_ROOT}/vtr_logging/include
    ${_VTR_SRC_ROOT}/vtr_route_planning/include
  )
  # Bring in ROS shared libs + their include dirs (rclcpp etc. are .so — safe).
  ament_target_dependencies(vtr_wait_strategy_py
    PUBLIC rclcpp vtr_common vtr_tactic vtr_pose_graph vtr_tactic_msgs)
  # Drop non-PIC VTR static archives that ament_target_dependencies may attach.
  get_target_property(_ws_libs vtr_wait_strategy_py LINK_LIBRARIES)
  if(_ws_libs)
    set(_ws_keep "")
    foreach(_lib IN LISTS _ws_libs)
      if(_lib MATCHES "\\.a$")
        continue()
      endif()
      if(_lib MATCHES "vtr_logging" OR _lib MATCHES "vtr_route_planning"
          OR _lib MATCHES "vtr_pose_graph" OR _lib MATCHES "vtr_tactic[^_]"
          OR _lib MATCHES "vtr_common" OR _lib MATCHES "vtr_storage")
        continue()
      endif()
      list(APPEND _ws_keep "${_lib}")
    endforeach()
    set_property(TARGET vtr_wait_strategy_py PROPERTY LINK_LIBRARIES "${_ws_keep}")
  endif()
  target_link_libraries(vtr_wait_strategy_py PRIVATE yaml-cpp)
  install(TARGETS vtr_wait_strategy_py
    LIBRARY DESTINATION local/lib/python3.10/dist-packages
    RUNTIME DESTINATION local/lib/python3.10/dist-packages)
  # Also drop a copy where some envs expect plain dist-packages.
  install(TARGETS vtr_wait_strategy_py
    LIBRARY DESTINATION lib/python3/dist-packages
    RUNTIME DESTINATION lib/python3/dist-packages)
  message(STATUS "vtr_wait_strategy_py: hybrid W* module enabled (PIC self-contained)")
else()
  message(WARNING "pybind11 not available — skipping vtr_wait_strategy_py")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()

# ------------------ Obstacle Monitor Node ------------------
add_executable(obstacle_monitor src/obstacle_monitor.cpp)
ament_target_dependencies(obstacle_monitor
  rclcpp nav_msgs geometry_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)
target_include_directories(obstacle_monitor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
install(TARGETS obstacle_monitor
  RUNTIME DESTINATION lib/${PROJECT_NAME})
