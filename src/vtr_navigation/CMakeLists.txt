cmake_minimum_required(VERSION 3.10)
project(vtr_navigation)

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
find_package(PROJ4 REQUIRED)

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
    pcl_conversions
    tf
    tf_conversions
    eigen_conversions
    asrl__common
    asrl__path_tracker
    asrl__pose_graph
    asrl__vision
    # asrl__messages
    # asrl__planning
    # asrl__interface
    # asrl__terrain_assessment
    robochunk_base
    robochunk_msgs
    robochunk_util
    robochunk_comms
    babelfish_robochunk_translator
  )

  # file(GLOB msg_files RELATIVE ${PROJECT_SOURCE_DIR}/msg msg/*.msg)
  # add_message_files(FILES ${msg_files})

  # file(GLOB srv_files RELATIVE ${PROJECT_SOURCE_DIR}/srv srv/*.srv)
  # add_service_files(FILES ${srv_files})

  # file(GLOB act_files RELATIVE ${PROJECT_SOURCE_DIR}/action action/*.action)
  # add_action_files(FILES ${act_files})

  # catkin_python_setup()

  # generate_messages(DEPENDENCIES std_msgs)

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
    OpenCV Protobuf # Robochunk LGMath#[system_dependency]
    CATKIN_DEPENDS
    actionlib
    message_runtime
    pcl_conversions
    rosbag
    roscpp
    tf
    tf_conversions
    eigen_conversions
    asrl__common
    asrl__vision
    asrl__path_tracker
    asrl__pose_graph
    # asrl__interface
    # asrl__messages
    # asrl__planning
    # asrl__terrain_assessment
    babelfish_robochunk_translator
    robochunk_base
    robochunk_msgs
    robochunk_util
    robochunk_comms
    lgmath
  )

  # Note: find_package for gpusurf is done here to allow non-cuda machines to build
  find_package(gpusurf)
  if(${gpusurf_FOUND})
    include_directories(${gpusurf_INCLUDE_DIRS})
    add_definitions(-DGPUSURF_ENABLED=1)
  else(${gpusurf_FOUND})
    add_definitions(-DGPUSURF_ENABLED=0)
  endif(${gpusurf_FOUND})

  # # Find LibElas
  # find_package(libelas QUIET) # Not required because we might not build with it
  # if(${libelas_FOUND})
  #   add_definitions(-DELAS_ENABLED=1)
  # else(${libelas_FOUND})
  #   add_definitions(-DELAS_ENABLED=0)
  # endif(${libelas_FOUND})

  # add_executable(FakeRobot ${CMAKE_CURRENT_SOURCE_DIR}/src/examples/FakeRobot.cpp)
  # target_link_libraries(FakeRobot ${catkin_LIBRARIES}
  #                                 ${PROTOBUF_LIBRARIES}
  # )
  # add_dependencies(FakeRobot ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

  # add_executable(Navigator src/Navigator.cpp src/NavigatorNode.cpp)
  # target_link_libraries(Navigator ${PROJECT_NAME}
  #                                 ${catkin_LIBRARIES}
  #                                 ${PROTOBUF_LIBRARIES}
  #                                 ${OpenCV_LIBRARIES})

  # add_dependencies(Navigator ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

  # # Path Massager Executable:
  # add_executable(following_path_massager src/FollowingPathMassager.cpp)
  # target_link_libraries(following_path_massager ${catkin_LIBRARIES})
  # add_dependencies(following_path_massager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

endif()

# file(GLOB_RECURSE INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/include/ *.h *.hpp *.inl)
# add_custom_target(${PROJECT_NAME}__headers SOURCES ${INCLUDES}) # QtCreator

# file(GLOB_RECURSE LAUNCHES ${CMAKE_CURRENT_SOURCE_DIR}/launch/ *.launch *.xml)
# add_custom_target(${PROJECT_NAME}__launches SOURCES ${LAUNCHES}) # QtCreator

# file(GLOB_RECURSE PARAMS ${CMAKE_CURRENT_SOURCE_DIR}/param/ *.yaml)
# add_custom_target(${PROJECT_NAME}__params SOURCES ${PARAMS}) # QtCreator

###########
## Build ##
###########

file(GLOB_RECURSE MODULE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/modules/*.cpp)
file(GLOB_RECURSE ASSEMBLY_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/assemblies/*.cpp)
file(GLOB_RECURSE PIPELINE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/pipelines/*.cpp)
file(GLOB_RECURSE TACTIC_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/tactics/*.cpp)
file(GLOB_RECURSE FACTORY_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/factories/*.cpp )
file(GLOB_RECURSE TA_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/terrain_assessment/*.cpp)
file(GLOB_RECURSE MEMORY_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/memory/MemoryManager.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/memory/MapMemoryManager.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/memory/LiveMemoryManager.cpp
)

add_library(${PROJECT_NAME}
  src/visualize.cpp
  src/caches.cpp
  ${FACTORY_SRC}
  ${PIPELINE_SRC}
  ${TACTIC_SRC}
  ${ASSEMBLY_SRC}
  ${MODULE_SRC}
  # # ${MEMORY_SRC}
  # # ${TA_SRC}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PROJ4_LIBRARIES}
  yaml-cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJ4_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  # ${libelas_INCLUDE_DIRS}
)

# add_dependencies(${PROJECT_NAME}
#   ${catkin_EXPORTED_TARGETS}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
# )

# add_executable(assembly_builder src/examples/assembly_builder.cpp)
# target_link_libraries(assembly_builder ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   zmq
#   ${PROTOBUF_LIBRARIES}
#   ${OpenCV_LIBRARIES})
# add_dependencies(assembly_builder ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

# add_executable(ChangeDetection src/examples/change_detection.cpp)
# target_link_libraries(ChangeDetection ${PROJECT_NAME}
#                                       ${catkin_LIBRARIES}
#                                       ${PROTOBUF_LIBRARIES})

# add_executable(StateMachineExample src/examples/StateMachineExample.cpp)
# target_link_libraries(StateMachineExample ${PROJECT_NAME}
#                                  ${catkin_LIBRARIES}
#                                  ${PROTOBUF_LIBRARIES}
#                                  ${Steam_LIBRARIES}
#                                  ${OpenCV_LIBRARIES})

# add_executable(GpsSplicer src/examples/GpsSplicer.cpp)
# target_link_libraries(GpsSplicer ${PROJECT_NAME}
#                                  ${catkin_LIBRARIES}
#                                  ${PROTOBUF_LIBRARIES}
#                                  ${Steam_LIBRARIES}
#                                  ${OpenCV_LIBRARIES})

# add_subdirectory(test)
