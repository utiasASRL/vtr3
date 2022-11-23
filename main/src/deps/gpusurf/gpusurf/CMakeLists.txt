cmake_minimum_required(VERSION 3.16)
project(gpusurf)

# Compiler setup (assumed to be GNU)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-march=native -O3)
add_compile_options(-Wno-unused-variable -Wno-unused-parameter -Wno-narrowing -Wno-unused-result -Wno-write-strings -Wno-ignored-qualifiers)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(cudpp REQUIRED)
find_package(CUDA REQUIRED)
# go to https://developer.nvidia.com/cuda-gpus, look for the compute capability of your gpu
# replace the two "75" with the compute capability of your gpu
SET(CUDA_NVCC_FLAGS -arch=compute_86 -code=sm_86 --std c++11 --default-stream per-thread)
SET(CUDA_VERBOSE_BUILD OFF)
SET(CUDA_BUILD_TYPE Device)
SET(CUDA_GENERATED_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/intermediate)

set(OpenCV_DIR /usr/local/opencv_cuda/lib/cmake/opencv4)
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
ament_export_dependencies(
  cudpp CUDA OpenCV
)

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

ament_package()