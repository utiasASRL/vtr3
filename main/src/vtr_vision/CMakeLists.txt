cmake_minimum_required(VERSION 3.16)
project(vtr_vision)

# Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(pcl_ros REQUIRED)      # not ported to ROS Foxy yet
find_package(CUDA REQUIRED)
find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
find_package(gpusurf REQUIRED)      # currently assuming this is required
add_definitions(-DGPUSURF_ENABLED=1)
find_package(cudpp REQUIRED)
find_package(lgmath REQUIRED)
find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_messages REQUIRED)
find_package(vtr_storage REQUIRED)

file(
  GLOB_RECURSE SRC
  src/*.cpp
)
add_library(${PROJECT_NAME} ${SRC})

ament_target_dependencies(${PROJECT_NAME}
  Eigen3
  OpenCV
  # pcl_ros
  CUDA
  cudpp
  gpusurf
  lgmath
  OpenMP
  vtr_common
  vtr_logging
  vtr_messages
  vtr_storage
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  gpusurf
  lgmath
  vtr_common
  vtr_logging
  vtr_messages
  vtr_storage
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
  #    ament_add_gtest(augmenter_tests test/features/augmenter_tests.cpp)
  #    target_link_libraries(augmenter_tests ${PROJECT_NAME})
  ament_add_gtest(bow_tests test/features/bow_tests.cpp)
  target_link_libraries(bow_tests ${PROJECT_NAME})
  ament_add_gtest(ransac_dataset_test test/integration/ransac_dataset_test.cpp)
  target_link_libraries(ransac_dataset_test ${PROJECT_NAME})
  ament_add_gtest(conversion_test test/messages/conversion_test.cpp)
  target_link_libraries(conversion_test ${PROJECT_NAME})
  ament_add_gtest(sampler_tests test/outliers/sampler_tests.cpp)
  target_link_libraries(sampler_tests ${PROJECT_NAME})
  ament_add_gtest(stereo_camera_model_tests test/outliers/stereo_camera_model_tests.cpp)
  target_link_libraries(stereo_camera_model_tests ${PROJECT_NAME})

  # Copy sample dataset from src to build
  file(COPY samples/data/ DESTINATION sample_data)

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml

  # Other
  add_executable(ransac_demo samples/ransac_demo.cpp)
  target_link_libraries(ransac_demo ${PROJECT_NAME})

endif()

ament_package()

