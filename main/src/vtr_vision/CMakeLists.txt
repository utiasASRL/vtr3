cmake_minimum_required(VERSION 3.16)
project(vtr_vision)

# Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

# Find dependencies
find_package(ament_cmake REQUIRED)

find_package(OpenCV REQUIRED)

find_package(gpusurf REQUIRED) # currently assuming this is required
find_package(lgmath REQUIRED)
find_package(steam REQUIRED)

find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_tactic REQUIRED)

## C++ Libraries
include_directories(PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# utilities
file(GLOB_RECURSE UTILS_SRC
  src/conversions/*.cpp
  src/features/*.cpp
)
add_library(${PROJECT_NAME}_utils ${UTILS_SRC})
ament_target_dependencies(${PROJECT_NAME}_utils
  OpenCV
  gpusurf lgmath steam
  vtr_common vtr_logging vtr_tactic
)

file(GLOB_RECURSE PIPELINE_SRC
  src/modules/*.cpp
  pipeline.cpp
)
add_library(${PROJECT_NAME}_pipeline ${PIPELINE_SRC})
target_link_libraries(${PROJECT_NAME}_pipeline
  ${PROJECT_NAME}_utils
)
ament_target_dependencies(${PROJECT_NAME}_pipeline
  OpenCV
  gpusurf lgmath steam
  vtr_common vtr_logging vtr_tactic
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  OpenCV
  gpusurf lgmath steam
  vtr_common vtr_logging vtr_tactic
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_utils
    ${PROJECT_NAME}_pipeline
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

