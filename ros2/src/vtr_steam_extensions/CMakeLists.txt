cmake_minimum_required(VERSION 3.16)
project(vtr_steam_extensions)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED system)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

find_package(lgmath REQUIRED)
find_package(steam REQUIRED)
find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_vision REQUIRED)

file(GLOB_RECURSE STEAM_EXTENSIONS_SRC  ${CMAKE_CURRENT_SOURCE_DIR}/src/evaluator/*.cpp)
add_library(${PROJECT_NAME}
  ${STEAM_EXTENSIONS_SRC}
)
ament_target_dependencies(${PROJECT_NAME}
  Eigen3
  OpenMP
  lgmath
  steam
  vtr_common
  vtr_logging
  vtr_vision
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3
  OpenMP
  lgmath
  steam
  vtr_common
  vtr_logging
  vtr_vision
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}
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
