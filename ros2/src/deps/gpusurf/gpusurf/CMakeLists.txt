cmake_minimum_required(VERSION 3.16)
project(gpusurf)

# Compiler setup (assumed to be GNU)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-march=native -O3)
add_compile_options(-Wno-unused-variable -Wno-unused-parameter -Wno-narrowing -Wno-unused-result -Wno-write-strings -Wno-ignored-qualifiers)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(cudpp REQUIRED)
find_package(CUDA REQUIRED)
SET(CUDA_NVCC_FLAGS -arch=compute_61 -code=sm_61 --std c++17 --default-stream per-thread)
SET(CUDA_VERBOSE_BUILD OFF)
SET(CUDA_BUILD_TYPE Device)
SET(CUDA_GENERATED_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/intermediate)
find_package(OpenCV REQUIRED)

# Libraries
CUDA_ADD_LIBRARY(
  ${PROJECT_NAME} SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/GpuIntegralImage.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/GpuIntegralImageProcessor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/GpuIntegralImage_kernel.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/GpuSurfDetector.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/GpuSurfDetectorInternal.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/GpuSurfFeatures.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/GpuSurfOctave.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/gpusurf.cu
  ${CMAKE_CURRENT_SOURCE_DIR}/src/timing.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/detector.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/gpu_globals.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/gpu_area.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/GpuSurfStereoDetector.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/GpuSurfStereoDetectorInternal.cpp
)
ament_target_dependencies(${PROJECT_NAME} cudpp CUDA OpenCV)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# May need the following
# ament_export_dependencies(Boost)
# ament_export_dependencies(eigen3_cmake_module)
# ament_export_dependencies(Eigen3)
# ament_export_dependencies(OpenMP)
# ament_export_dependencies(lgmath)(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    # Libraries
    ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   # Unit-tests
#   ament_add_gtest(pattern_tests test/pattern_tests.cpp)
#   target_link_libraries(pattern_tests ${PROJECT_NAME})
#   ament_add_gtest(time_tests test/time_tests.cpp)
#   target_link_libraries(time_tests ${PROJECT_NAME})

#   # Linting
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
# endif()

ament_package()