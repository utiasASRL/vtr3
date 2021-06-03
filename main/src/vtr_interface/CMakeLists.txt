cmake_minimum_required(VERSION 3.16)
project(vtr_interface)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# C++ Libraries

# Build protobuf messages
file(
  GLOB PROTO_SCRIPTS
  vtr_interface/*.proto
)
add_custom_target(
  proto_python ALL
  protoc -I${CMAKE_CURRENT_SOURCE_DIR}/vtr_interface
    --python_out=${CMAKE_CURRENT_SOURCE_DIR}/vtr_interface
    ${PROTO_SCRIPTS}
  USES_TERMINAL
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

# Build vtr frontend
set(vtr_frontend_dir ${CMAKE_CURRENT_SOURCE_DIR}/vtr_frontend/vtr-ui)
add_custom_target(
  vtr_frontend ALL
  npm --prefix ${vtr_frontend_dir} install ${vtr_frontend_dir} # Install deps
  COMMAND
  npm --prefix ${vtr_frontend_dir} run build # Compile
  USES_TERMINAL
)
install(
  DIRECTORY vtr_frontend/vtr-ui/build
  DESTINATION lib/${PROJECT_NAME}/vtr-ui
)
install(
  FILES ${PROTO_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}/proto
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
