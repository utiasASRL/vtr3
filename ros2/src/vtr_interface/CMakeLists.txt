cmake_minimum_required(VERSION 3.16)
project(vtr_interface)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake.in")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# C++ Libraries

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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
