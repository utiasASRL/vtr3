###### HEADER #####
cmake_minimum_required(VERSION 3.16)
project(vtr_path_planning)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

# Find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(lgmath REQUIRED)
find_package(steam REQUIRED)

find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_tactic REQUIRED)


# # find python libraries
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)

# C++ Libraries
include_directories(
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PYTHON3_INCLUDE_DIRS}
    ${NumPy_INCLUDE_DIRS}
)

file(GLOB_RECURSE SRC src/base_path_planner.cpp src/stationary_planner.cpp)
add_library(${PROJECT_NAME}_base ${SRC})
ament_target_dependencies(${PROJECT_NAME}_base
  geometry_msgs
  vtr_common vtr_logging vtr_tactic
)

file(GLOB_RECURSE SRC src/mpc/*.cpp)
add_library(${PROJECT_NAME}_mpc ${SRC})
target_link_libraries(${PROJECT_NAME}_mpc ${PROJECT_NAME}_base)
ament_target_dependencies(${PROJECT_NAME}_mpc
  rclcpp tf2 tf2_ros tf2_eigen
  lgmath steam Python3
)




find_package(matplotlib_cpp REQUIRED)
include_directories(SYSTEM ${matplotlib_cpp_SOURCE_DIR})


file(GLOB_RECURSE SRC src/cbit/*.cpp)
add_library(${PROJECT_NAME}_cbit ${SRC})
target_link_libraries(${PROJECT_NAME}_cbit ${PROJECT_NAME}_mpc)
target_link_libraries(${PROJECT_NAME}_cbit ${PYTHON_LIBRARIES})

ament_target_dependencies(${PROJECT_NAME}_cbit
  rclcpp tf2 tf2_ros tf2_eigen Python3
  lgmath steam PythonLibs matplotlib_cpp
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp tf2 tf2_ros tf2_eigen geometry_msgs matplotlib_cpp
  vtr_common vtr_logging vtr_tactic
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_base
    ${PROJECT_NAME}_mpc
    ${PROJECT_NAME}_cbit
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
