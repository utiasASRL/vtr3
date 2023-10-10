cmake_minimum_required(VERSION 3.8)
project(vtr_torch)


## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(rclcpp REQUIRED)

find_package(vtr_common REQUIRED)
find_package(vtr_tactic REQUIRED)
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

# find dependencies
find_package(ament_cmake REQUIRED)



include_directories(PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${TORCH_INCLUDE_DIRS}
)

file(GLOB_RECURSE MODULE_SRC
  src/modules/torch_module.cpp
)
add_library(${PROJECT_NAME}_modules ${MODULE_SRC})
target_link_libraries(${PROJECT_NAME}_modules ${TORCH_LIBRARIES})

ament_target_dependencies(${PROJECT_NAME}_modules
  Eigen3 Torch
  vtr_common vtr_logging vtr_tactic
)


ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3 Torch
  vtr_common vtr_logging vtr_tactic
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_modules
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
