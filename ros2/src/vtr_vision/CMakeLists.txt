cmake_minimum_required(VERSION 3.16)
project(vtr_vision)

# Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake.in")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED) # not installed with foxy yet todo
find_package(CUDA REQUIRED)
set_cuda_flags()
find_package(OpenMP REQUIRED)
# set_openmp_flags()          # todo: sort out openMP flags
find_package(gpusurf REQUIRED)
find_package(cudpp REQUIRED)
find_package(lgmath REQUIRED)
find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)


#file(GLOB MISC_SRC
#        "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
#file(GLOB SENSORS_SRC
#        "${CMAKE_CURRENT_SOURCE_DIR}/sensors/*.cpp")
#file(GLOB GEOMETRY_SRC
#        "${CMAKE_CURRENT_SOURCE_DIR}/geometry/*.cpp")
#file(GLOB MSG_BRIDGE_SRC
#        "${CMAKE_CURRENT_SOURCE_DIR}/messages/*.cpp")
#file(GLOB_RECURSE OUTLIERS_SRC
#        "${CMAKE_CURRENT_SOURCE_DIR}/outliers/*.cpp")
#file(GLOB FEATURES_SRC
#        "${CMAKE_CURRENT_SOURCE_DIR}/features/extractor/*.cpp"
#        "${CMAKE_CURRENT_SOURCE_DIR}/features/matcher/*.cpp"
#        )
#file(GLOB CUDA_SRC
#        "${CMAKE_CURRENT_SOURCE_DIR}/features/extractor/cuda/*.cpp")
#set(CUDA_LIBS ${gpusurf_LIBRARIES})     # todo: not sure what this doing
#
#add_library(
#        ${PROJECT_NAME}
#        ${CUDA_SRC}
#        ${FEATURES_SRC}
#        ${GEOMETRY_SRC}
#        ${MISC_SRC}
#        ${MSG_BRIDGE_SRC}
#        ${OUTLIERS_SRC}
#        ${SENSORS_SRC}
#)

file(
        GLOB_RECURSE SRC
        src/*.cpp
)
add_library(${PROJECT_NAME} ${SRC})

ament_target_dependencies(${PROJECT_NAME}
        Eigen3
        OpenCV
        CUDA
        gpusurf
        # cudpp
        lgmath
        OpenMP
        vtr_common
        vtr_logging
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

####### ROS ######
## This is a top-level algorithm package that is ros-agnostic
## It should not have any ROS dependencies, but may have other ros-agnostic dependencies.
## The CATKIN_ENABLED flag is cached, and may be passed at run time to enable catkin exporting.
#if (CATKIN_ENABLED)
#  # Adding packages as components adds them to our ${catkin_INCLUDE_DIRS} and ${catkin_LIBRARIES} variables.
#  find_package(
#    catkin REQUIRED
#    COMPONENTS
#    roscpp
#    asrl__common
#    asrl__messages
#    robochunk_base
#    robochunk_msgs
#    robochunk_util
#    # OPTIONAL_COMPONENTS libelas
#  )
#
#  ##### EXPORT MYSELF TO CATKIN #####
#  #   "INCLUDE_DIRS" specifies to location of your package's headers for inclusion in inheriting ${catkin_INCLUDE_DIRS}.
#  #   "LIBRARIES" specifies the libraries your package provides for inclusion in inheriting ${catkin_LIBRARIES}. This should match all add_library directives.
#  #   "DEPENDS" specifies the system dependencies
#  #   "CATKIN_DEPENDS" specifies the ros dependencies
#  catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES ${PROJECT_NAME}
#    ##Dependencies (build_depends and run_depends)
#    DEPENDS
#    OpenCV Protobuf #Glog #[system_dependency]
#    CATKIN_DEPENDS
#    roscpp
#    asrl__common
#    asrl__messages
#    robochunk_base
#    robochunk_msgs
#    robochunk_util
#  )
#endif()
#
#include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
#  ${date_INCLUDE_DIRS}
#  ${EIGEN3_INCLUDE_DIR}
#  ${gpusurf_INCLUDE_DIRS}
#  ${LGMath_INCLUDE_DIRS}
##  ${libelas_INCLUDE_DIRS}
#  ${OpenCV_INCLUDE_DIRS}
#  ${PCL_INCLUDE_DIRS}
#)
#
## # Includes for Qt
## file(GLOB_RECURSE HEADERS include/*.hpp include/*.h)
#
## # if gpu surf isn't found, exclude the header in the extractor from the list
## if(NOT ${gpusurf_FOUND})
##   file(GLOB_RECURSE REMOVED_HEADERS include/vtr/vision/features/extractor/cuda/*.hpp include/vtr/vision/features/extractor/cuda/*.h)
##   list(REMOVE_ITEM HEADERS REMOVED_HEADERS)
## endif()
#
## add_custom_target(${PROJECT_NAME}__headers SOURCES ${HEADERS})
#
#add_subdirectory(src)
#
#
## add_subdirectory(samples)
#
## add_subdirectory(test)
