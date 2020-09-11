cmake_minimum_required(VERSION 3.16)
project(vtr_vision)

# Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake.in")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(pcl_ros REQUIRED)     # ROS2 package still under development
find_package(CUDA REQUIRED)
set_cuda_flags()
find_package(OpenMP REQUIRED)
# set_openmp_flags()                # todo: sort out openMP flags
find_package(gpusurf REQUIRED)      # currently assuming this is required
add_definitions(-DGPUSURF_ENABLED=1)
find_package(cudpp REQUIRED)
find_package(lgmath REQUIRED)
find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_messages REQUIRED)

file(
        GLOB_RECURSE SRC
        src/*.cpp
)
add_library(${PROJECT_NAME} ${SRC})

ament_target_dependencies(${PROJECT_NAME}
        Eigen3
        OpenCV
        perception_pcl
        CUDA
        gpusurf
        # cudpp
        lgmath
        OpenMP
        vtr_common
        vtr_logging
        vtr_messages
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

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

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()

