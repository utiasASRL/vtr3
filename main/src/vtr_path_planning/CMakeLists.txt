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

# C++ Libraries
include_directories(
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

file(GLOB_RECURSE SRC src/base_path_planner.cpp)
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
  lgmath steam
)

file(GLOB_RECURSE SRC src/cbit/*.cpp)
add_library(${PROJECT_NAME}_cbit ${SRC})
target_link_libraries(${PROJECT_NAME}_cbit ${PROJECT_NAME}_base)
ament_target_dependencies(${PROJECT_NAME}_cbit
  rclcpp tf2 tf2_ros tf2_eigen
  lgmath steam
)

# Hacky way of declaring the helper scripts
file(GLOB_RECURSE SRC src/cbit/generate_pq.cpp)

file(GLOB_RECURSE SRC src/cbit/utils.cpp)

file(GLOB_RECURSE SRC src/cbit/cbit_path_planner.cpp)

# START OF JORDY EXPERIMENTAL MATPLOTLIB CHANGES (note also had to make similar changes in vtr_lidar and vtr_navigation cmakelist.txt also for some dumb reason)
# Declare the plotting tools
file(GLOB_RECURSE SRC src/cbit/cbit_plotting.cpp)

# find python libraries
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)
include_directories(${PYTHON3_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})

# populate matplotlib repository
include(FetchContent)
FetchContent_Declare(
    matplotlib
    GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
    GIT_TAG        f23347fca25219d1c42cbb91608b5556814bf572
)
FetchContent_GetProperties(matplotlib)
if(NOT matplotlib_POPULATED)
    FetchContent_Populate(matplotlib)
endif()
include_directories(SYSTEM ${matplotlib_SOURCE_DIR})

target_link_libraries(${PROJECT_NAME}_cbit ${PYTHON_LIBRARIES})


find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_cbit ${PYTHON_LIBRARIES})
# EXPERIMENTAL, JORDY ADDITION FOR MATPLOTLIBCPP PLOTTING
#target_link_libraries($vtr_path_planning ${PYTHON_LIBRARIES} Python3::NumPy)
# END OF EXPERIMENTAL MATPLOTLIBCPP 

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp tf2 tf2_ros tf2_eigen geometry_msgs
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
