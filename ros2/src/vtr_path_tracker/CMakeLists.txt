###### HEADER #####
cmake_minimum_required(VERSION 3.16)
project(vtr_path_tracker)

# Common setup for vtr packages
include(
        "${CMAKE_CURRENT_LIST_DIR}/../vtr_cmake/vtr_include.cmake.in"
        NO_POLICY_SCOPE
)

###### IMPORT DEPENDENCIES (build_depends) ######
## System ##
#find_package([system_dependency_1] REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(OpenMP REQUIRED)
set_openmp_flags()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")

add_definitions(-DEIGEN_DONT_VECTORIZE=1)  # vtr3 change : issue in ubuntu 20.04
add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT=1)

## ROS ##
# This is a top-level algorithm package that is ros-agnostic
# It should not have any ROS dependencies, but may have other ros-agnostic dependencies.
# The CATKIN_ENABLED flag is cached, and may be passed at run time to enable catkin exporting.
if (CATKIN_ENABLED)
    # Adding packages as components adds them to our ${catkin_INCLUDE_DIRS} and ${catkin_LIBRARIES} variables.
    find_package(
            catkin REQUIRED
            COMPONENTS
            asrl__common
            asrl__messages
            asrl__pose_graph
            angles
            geometry_msgs
            std_msgs
            roscpp # for rviz debugging
            # visualization_msgs # for rviz debugging
    )

    # Export the python components of this package
    # catkin_python_setup()

    ##### EXPORT MYSELF TO CATKIN #####
    #   "INCLUDE_DIRS" specifies to location of your package's headers for inclusion in inheriting ${catkin_INCLUDE_DIRS}.
    #   "LIBRARIES" specifies the libraries your package provides for inclusion in inheriting ${catkin_LIBRARIES}. This should match all add_library directives.
    #   "DEPENDS" specifies the system dependencies
    #   "CATKIN_DEPENDS" specifies the ros dependencies
    catkin_package(
            INCLUDE_DIRS include
            LIBRARIES
            ${PROJECT_NAME}
            DEPENDS
            Protobuf
            CATKIN_DEPENDS
            asrl__common
            asrl__messages
            asrl__pose_graph
            angles
            geometry_msgs
            std_msgs
            roscpp
    )

endif ()

### BUILD TARGETS ####

# add all files to a dummy target for Qt Creator
file(GLOB_RECURSE REPO_FILES "*")
add_custom_target(${PROJECT_NAME}__repo_files SOURCES ${REPO_FILES})

## main library
file(GLOB_RECURSE SOURCE_FILES "src/*.cpp")

add_library(${PROJECT_NAME}
        ${SOURCE_FILES}
        )

target_link_libraries(
        ${PROJECT_NAME}
        ${${PROJECT_NAME}_DEPEND_LIBRARIES}
        yaml-cpp
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

# Test using an artificial graph/path
# add_executable(path_tracker_base_test test/path_tracker_base_test.cpp)
# target_link_libraries(path_tracker_base_test ${PROJECT_NAME} ${PROTOBUF_LIBRARIES})

# test using a graph from disk
# add_executable(flatness_based_mpc_test test/flatness_based_mpc_test.cpp)
# target_link_libraries(flatness_based_mpc_test ${PROJECT_NAME} ${PROTOBUF_LIBRARIES})

# For running small tests
# add_executable(sandbox test/sandbox.cpp)
# target_link_libraries(sandbox ${PROJECT_NAME} ${PROTOBUF_LIBRARIES})

### UNIT TESTS ####

if (CATKIN_ENABLE_TESTING)

endif ()
