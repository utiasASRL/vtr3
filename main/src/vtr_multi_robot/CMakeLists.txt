cmake_minimum_required(VERSION 3.12)
project(vtr_multi_robot)


## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

# Enable extra warnings for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vtr_tactic REQUIRED) # remove or add other packages your node needs
find_package(vtr_navigation_msgs REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_route_planning REQUIRED)


add_executable(convoy_planning_node src/convoy_planning_node.cpp)

ament_target_dependencies(
    convoy_planning_node
    rclcpp
    vtr_tactic
    vtr_navigation_msgs
    vtr_logging
    vtr_route_planning
)

install(TARGETS
    convoy_planning_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Python Package
ament_python_install_package(${PROJECT_NAME})

# Python Scripts
file(
  GLOB_RECURSE PYTHON_SCRIPTS
  ${PROJECT_NAME}/*.py
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

ament_package()