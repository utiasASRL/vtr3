cmake_minimum_required(VERSION 3.16)
project(vtr_vision)

# Common setup for vtr packages
include(
  "${CMAKE_CURRENT_LIST_DIR}/../vtr_cmake/vtr_include.cmake.in"
  NO_POLICY_SCOPE
)

###### IMPORT DEPENDENCIES (build_depends) ######
## System ##
#find_package([system_dependency_1] REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Protobuf REQUIRED)

# Find LibElas
find_package(libelas QUIET) # Not required because we might not build with it
if(${libelas_FOUND})
  add_definitions(-DELAS_ENABLED=1)
else(${libelas_FOUND})
  add_definitions(-DELAS_ENABLED=0)
endif(${libelas_FOUND})

## Optional cuda dependency, if found cuda feature extracts will be compiled
find_package(CUDA)
set_cuda_flags()

# Find GPUSURF
find_package(gpusurf)
if(${gpusurf_FOUND})
  add_definitions(-DGPUSURF_ENABLED=1)
else(${gpusurf_FOUND})
  add_definitions(-DGPUSURF_ENABLED=0)
endif(${gpusurf_FOUND})

# find OpenMP - multithreading package
find_package(OpenMP REQUIRED)
set_openmp_flags()

# # Keep a record of the OpenCV major version number
# # Note:"INTERNAL" was taken out because a test script was failing to find the OPENCV_VERSION_NUM
# set(OPENCV_VERSION_NUM ${OpenCV_VERSION_MAJOR} CACHE STRING "A copy of the variable OpenCV_VERSION_MAJOR" )

###### ROS ######
# This is a top-level algorithm package that is ros-agnostic
# It should not have any ROS dependencies, but may have other ros-agnostic dependencies.
# The CATKIN_ENABLED flag is cached, and may be passed at run time to enable catkin exporting.
if (CATKIN_ENABLED)
  # Adding packages as components adds them to our ${catkin_INCLUDE_DIRS} and ${catkin_LIBRARIES} variables.
  find_package(
    catkin REQUIRED
    COMPONENTS
    roscpp
    asrl__common
    asrl__messages
    asrl__vision
    robochunk_base
    robochunk_msgs
    robochunk_util
    # OPTIONAL_COMPONENTS libelas
  )

  ##### EXPORT MYSELF TO CATKIN #####
  #   "INCLUDE_DIRS" specifies to location of your package's headers for inclusion in inheriting ${catkin_INCLUDE_DIRS}.
  #   "LIBRARIES" specifies the libraries your package provides for inclusion in inheriting ${catkin_LIBRARIES}. This should match all add_library directives.
  #   "DEPENDS" specifies the system dependencies
  #   "CATKIN_DEPENDS" specifies the ros dependencies
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    ##Dependencies (build_depends and run_depends)
    DEPENDS
    OpenCV Protobuf #Glog #[system_dependency]
    CATKIN_DEPENDS
    roscpp
    asrl__common
    asrl__messages
    asrl__vision
    robochunk_base
    robochunk_msgs
    robochunk_util
  )
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${date_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${gpusurf_INCLUDE_DIRS}
  ${LGMath_INCLUDE_DIRS}
  ${libelas_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# # Includes for Qt
# file(GLOB_RECURSE HEADERS include/*.hpp include/*.h)

# # if gpu surf isn't found, exclude the header in the extractor from the list
# if(NOT ${gpusurf_FOUND})
#   file(GLOB_RECURSE REMOVED_HEADERS include/vtr/vision/features/extractor/cuda/*.hpp include/vtr/vision/features/extractor/cuda/*.h)
#   list(REMOVE_ITEM HEADERS REMOVED_HEADERS)
# endif()

# add_custom_target(${PROJECT_NAME}__headers SOURCES ${HEADERS})

add_subdirectory(src)


# add_subdirectory(samples)

# add_subdirectory(test)
