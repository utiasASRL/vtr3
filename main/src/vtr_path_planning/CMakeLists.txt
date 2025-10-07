###### HEADER #####
cmake_minimum_required(VERSION 3.16)
project(vtr_path_planning)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")
# add_compile_options(-g -Og)

# Find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(lgmath REQUIRED)
find_package(casadi REQUIRED)

find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_tactic REQUIRED)
find_package(vtr_path_planning_msgs REQUIRED)
find_package(vtr_navigation_msgs REQUIRED)


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

file(GLOB_RECURSE SRC src/base_path_planner.cpp src/stationary_planner.cpp src/dynamic_path_planner.cpp)
add_library(${PROJECT_NAME}_base ${SRC})
ament_target_dependencies(${PROJECT_NAME}_base
  geometry_msgs
  vtr_common vtr_logging vtr_tactic
  vtr_path_planning_msgs
)

execute_process(COMMAND ${PYTHON_EXECUTABLE} scripts/export_mpc.py
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})


add_custom_command(OUTPUT "include/vtr_path_planning/mpc/generated/unicycle_mpc_solver.hpp"
                          "include/vtr_path_planning/mpc/generated/unicycle_follower_mpc_solver.hpp"
                          "include/vtr_path_planning/mpc/generated/bicycle_mpc_solver.hpp"
                          "include/vtr_path_planning/mpc/generated/bicycle_follower_mpc_solver.hpp"
  COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/scripts/export_mpc.py"
  DEPENDS "scripts/unicycle_solver.py"
          "scripts/unicycle_follower_solver.py"
          "scripts/bicycle_solver_pt.py"
          "scripts/bicycle_follower_solver_pt.py")

add_custom_target(generate_headers ALL
DEPENDS "include/vtr_path_planning/mpc/generated/unicycle_mpc_solver.hpp"
        "include/vtr_path_planning/mpc/generated/unicycle_follower_mpc_solver.hpp"
        "include/vtr_path_planning/mpc/generated/bicycle_mpc_solver.hpp"
        "include/vtr_path_planning/mpc/generated/bicycle_follower_mpc_solver.hpp")

file(GLOB_RECURSE SRC src/mpc/*.cpp EXCLUDE src/mpc/generated/*)
add_library(${PROJECT_NAME}_mpc ${SRC})
target_link_libraries(${PROJECT_NAME}_mpc ${PROJECT_NAME}_base)
target_link_libraries(${PROJECT_NAME}_mpc casadi)
ament_target_dependencies(${PROJECT_NAME}_mpc
  rclcpp tf2 tf2_ros tf2_eigen
  lgmath Python3 casadi
  vtr_path_planning_msgs vtr_navigation_msgs
)


add_library(solve_unicycle_mpc SHARED src/mpc/generated/unicycle_mpc_solver.cpp)
add_dependencies(solve_unicycle_mpc generate_headers)
target_link_libraries(solve_unicycle_mpc casadi)
target_compile_options(solve_unicycle_mpc PRIVATE -W)

add_library(solve_bicycle_mpc SHARED src/mpc/generated/bicycle_mpc_solver.cpp)
add_dependencies(solve_bicycle_mpc generate_headers)
target_link_libraries(solve_bicycle_mpc casadi)
target_compile_options(solve_bicycle_mpc PRIVATE -W)

add_library(solve_unicycle_follower_mpc SHARED src/mpc/generated/unicycle_follower_mpc_solver.cpp)
add_dependencies(solve_unicycle_follower_mpc generate_headers)
target_link_libraries(solve_unicycle_follower_mpc casadi)
target_compile_options(solve_unicycle_follower_mpc PRIVATE -W)

add_library(solve_bicycle_follower_mpc SHARED src/mpc/generated/bicycle_follower_mpc_solver.cpp)
add_dependencies(solve_bicycle_follower_mpc generate_headers)
target_link_libraries(solve_bicycle_follower_mpc casadi)
target_compile_options(solve_bicycle_follower_mpc PRIVATE  -W)

add_library(solve_bicycle_joint_mpc SHARED src/mpc/generated/bicycle_joint_mpc_solver.cpp)
add_dependencies(solve_bicycle_joint_mpc generate_headers)
target_link_libraries(solve_bicycle_joint_mpc casadi)
target_compile_options(solve_bicycle_joint_mpc PRIVATE  -W)

find_package(matplotlib_cpp REQUIRED)
include_directories(SYSTEM ${matplotlib_cpp_SOURCE_DIR})


file(GLOB_RECURSE SRC src/cbit/*.cpp)
add_library(${PROJECT_NAME}_cbit ${SRC})
target_link_libraries(${PROJECT_NAME}_cbit ${PROJECT_NAME}_mpc)
target_link_libraries(${PROJECT_NAME}_cbit ${PYTHON_LIBRARIES})

ament_target_dependencies(${PROJECT_NAME}_cbit
  rclcpp tf2 tf2_ros tf2_eigen Python3
  lgmath PythonLibs matplotlib_cpp
  vtr_path_planning_msgs vtr_navigation_msgs
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp tf2 tf2_ros tf2_eigen geometry_msgs matplotlib_cpp
  vtr_common vtr_logging vtr_tactic casadi
  vtr_path_planning_msgs vtr_navigation_msgs
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
    solve_unicycle_mpc
    solve_bicycle_mpc
    solve_unicycle_follower_mpc
    solve_bicycle_follower_mpc
    solve_bicycle_joint_mpc
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_unicycle test/closed_loop/unicycle_tests.cpp)
  target_link_libraries(test_unicycle ${PROJECT_NAME}_cbit)

  ament_add_gtest(test_casadi_solvers test/casadi_simple_tests.cpp)
  target_link_libraries(test_casadi_solvers ${PROJECT_NAME}_mpc)

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
