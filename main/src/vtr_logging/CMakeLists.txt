cmake_minimum_required(VERSION 3.16)
project(vtr_logging)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

## Find dependencies
find_package(ament_cmake REQUIRED)

# find_package(easyloggingpp REQUIRED)

# Libraries
file(GLOB_RECURSE SRC src/*.cpp)
add_library(${PROJECT_NAME} ${SRC})
ament_target_dependencies(${PROJECT_NAME}
  # easyloggingpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  # easyloggingpp
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

ament_export_include_directories(include)

ament_package()
