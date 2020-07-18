###### HEADER #####
cmake_minimum_required(VERSION 3.16)
project(vtr_path_planning)

# Common setup for vtr packages
include(
  "${CMAKE_CURRENT_LIST_DIR}/../vtr_cmake/vtr_include.cmake.in"
  NO_POLICY_SCOPE
)

###### IMPORT DEPENDENCIES (build_depends) ######
## System ##
#find_package([system_dependency_1] REQUIRED)
find_package(Protobuf REQUIRED)
find_package(PROJ4 REQUIRED)
find_package(OpenMP REQUIRED)  # OpenMP - multithreading package
set_openmp_flags()

## ROS ##
# This is a top-level algorithm package that is ros-agnostic
# It should not have any ROS dependencies, but may have other ros-agnostic dependencies.
# The CATKIN_ENABLED flag is cached, and may be passed at run time to enable catkin exporting.
if (CATKIN_ENABLED)
  # Adding packages as components adds them to our ${catkin_INCLUDE_DIRS} and ${catkin_LIBRARIES} variables.
  find_package(
    catkin REQUIRED
    COMPONENTS
    actionlib
    actionlib_msgs
    message_generation
    roscpp
    std_msgs
    std_srvs
    tf
    asrl__planning
    asrl__pose_graph
    asrl__messages
    asrl__common
    asrl__interface
    robochunk_base
    robochunk_msgs
    robochunk_util
    robochunk_comms
    babelfish_robochunk_translator
  )

  # Export the python components of this package
  # catkin_python_setup()

  # file(GLOB msg_files RELATIVE ${PROJECT_SOURCE_DIR}/msg msg/*.msg)
  # add_message_files(FILES ${msg_files})

  # file(GLOB srv_files RELATIVE ${PROJECT_SOURCE_DIR}/srv srv/*.srv)
  # add_service_files(FILES ${srv_files})

  # file(GLOB act_files RELATIVE ${PROJECT_SOURCE_DIR}/action action/*.action)
  # add_action_files(FILES ${act_files})

  # generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

  ##### EXPORT MYSELF TO CATKIN #####
  #   "INCLUDE_DIRS" specifies to location of your package's headers for inclusion in inheriting ${catkin_INCLUDE_DIRS}.
  #   "LIBRARIES" specifies the libraries your package provides for inclusion in inheriting ${catkin_LIBRARIES}. This should match all add_library directives.
  #   "DEPENDS" specifies the system dependencies
  #   "CATKIN_DEPENDS" specifies the ros dependencies
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
    ${PROJECT_NAME}
    ##Dependencies (build_depends and run_depends)
    DEPENDS
    Protobuf
    CATKIN_DEPENDS
    asrl__planning
    asrl__pose_graph
    asrl__messages
    asrl__common
    asrl__interface
    actionlib
    std_msgs
    actionlib_msgs
    roscpp
    tf
    robochunk_base
    robochunk_msgs
    robochunk_util
    robochunk_comms
    babelfish_robochunk_translator
  )
  
  file(GLOB_RECURSE INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/include/)
  add_library(${PROJECT_NAME} src/time_delta_planner.cpp ${INCLUDES})
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
  add_dependencies(
    ${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

endif()

##### CONFIGURE INCLUDE DIRECTORIES #####
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJ4_INCLUDE_DIRS}
)

##### UNIT TESTS #####
#file(GLOB TEST_FILES "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp")
#catkin_add_gtest(${PROJECT_NAME}_tests ${TEST_FILES})
#target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME}__matching ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(${PROJECT_NAME}_tests ${catkin_EXPORTED_TARGETS})
