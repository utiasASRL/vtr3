cmake_minimum_required(VERSION 3.10)
project(vtr_testing)

# Common setup for vtr packages
include(
  "${CMAKE_CURRENT_LIST_DIR}/../vtr_cmake/vtr_include.cmake.in"
  NO_POLICY_SCOPE
)

###### IMPORT DEPENDENCIES (build_depends) ######
## System ##
#find_package([system_dependency_1] REQUIRED)
find_package(Eigen3 REQUIRED)  # Note this used to be 3.2.2, but we have 3.4.x
find_package(Protobuf REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
# find_package(PROJ4 REQUIRED)  # cannot find in the system for some reason

# find OpenMP - multithreading package
find_package(OpenMP REQUIRED)
set_openmp_flags()

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
    rosbag
    cv_bridge
    actionlib
    message_generation
    tf
    image_transport
    asrl__common
    asrl__vision
    vtr_navigation
    # asrl__navigation
    # asrl__messages
    # asrl__pose_graph
    # asrl__planning
    # asrl__interface
    # asrl__terrain_assessment
    robochunk_base
    robochunk_msgs
    robochunk_util
    robochunk_comms
    babelfish_robochunk_translator
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
    OpenCV Protobuf  # LGMath??? # [system_dependency]
    CATKIN_DEPENDS
    actionlib
    message_runtime
    rosbag
    roscpp
    tf
    image_transport
    # lgmath  #???
    asrl__common
    asrl__vision
    vtr_navigation
    # asrl__pose_graph
    # asrl__navigation
    # asrl__interface
    # asrl__messages
    # asrl__planning
    # asrl__terrain_assessment
    babelfish_robochunk_translator
    robochunk_base
    robochunk_msgs
    robochunk_util
    robochunk_comms
  )

  # Note: find_package for gpusurf is done here to allow non-cuda machines to build
  find_package(gpusurf)
  if(${gpusurf_FOUND})
    include_directories(${gpusurf_INCLUDE_DIRS})
    add_definitions(-DGPUSURF_ENABLED=1)
  else(${gpusurf_FOUND})
    add_definitions(-DGPUSURF_ENABLED=0)
  endif(${gpusurf_FOUND})

endif()

# file(GLOB_RECURSE INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/include/ *.h *.hpp *.inl)
# add_custom_target(${PROJECT_NAME}__headers SOURCES ${INCLUDES})
#
# file(GLOB_RECURSE LAUNCHES ${CMAKE_CURRENT_SOURCE_DIR}/launch/ *.launch *.xml)
# add_custom_target(${PROJECT_NAME}__launches SOURCES ${LAUNCHES})
#
# file(GLOB_RECURSE PARAMS ${CMAKE_CURRENT_SOURCE_DIR}/param/ *.yaml)
# add_custom_target(${PROJECT_NAME}__params SOURCES ${PARAMS})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  # ${PROJ4_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(ModuleVO src/examples/module_vo.cpp)
target_link_libraries(
  ModuleVO
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${OpenCV_LIBRARIES}
)