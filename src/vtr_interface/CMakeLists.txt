##### HEADER #####
cmake_minimum_required(VERSION 3.16)
project(vtr_interface)

# Common setup for vtr packages
include(
  "${CMAKE_CURRENT_LIST_DIR}/../vtr_cmake/vtr_include.cmake.in"
  NO_POLICY_SCOPE
)

###### IMPORT DEPENDENCIES (build_depends) ######
## System ##
#find_package([system_dependency_1] REQUIRED)
# find_package(easyloggingpp REQUIRED)
find_package(Protobuf REQUIRED)

## Define the protobuf generation function
include(GenerateProtoPython NO_POLICY_SCOPE)

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
    tf
    asrl__common
    asrl__messages
    asrl__planning
    asrl__pose_graph
    robochunk_msgs
  )

  # Export the python components of this package
  catkin_python_setup()

  # file(GLOB act_files RELATIVE ${PROJECT_SOURCE_DIR}/action action/*.action)
  # add_action_files(FILES ${act_files})

  # generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

  # This is really only here so that messages and python files are visible externally
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    tf
    asrl__common
    asrl__messages
    asrl__planning
    asrl__pose_graph
    robochunk_msgs
  )

  # add_library(${PROJECT_NAME} src/UserPrompt.cpp)
  # target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
  # add_dependencies(${PROJECT_NAME}
  #   ${catkin_EXPORTED_TARGETS}
  #   ${${PROJECT_NAME}_EXPORTED_TARGETS}
  # )

endif()

## Generate Protobuf messages for Python
file(GLOB
  ProtoFiles
  "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto"
)
GENERATE_PROTO_PYTHON(PROTO_PY
                      "${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/proto"
                      "${CMAKE_CURRENT_SOURCE_DIR}/proto"
                      "${ProtoFiles}")
add_custom_target(${PROJECT_NAME}_proto_py ALL DEPENDS ${PROTO_PY})

##### CONFIGURE INCLUDE DIRECTORIES #####
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${easyloggingpp_INCLUDE_DIRS}
# )
