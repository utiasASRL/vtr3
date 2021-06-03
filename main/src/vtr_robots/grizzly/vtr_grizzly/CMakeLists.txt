cmake_minimum_required(VERSION 3.16)
project(vtr_grizzly)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files.
install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
