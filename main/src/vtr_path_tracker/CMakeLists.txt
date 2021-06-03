cmake_minimum_required(VERSION 3.16)
project(vtr_path_tracker)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

# Find dependencies
## System ##
#find_package([system_dependency_1] REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread) # TODO required by mission_planning and steam, check if we can drop these dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

## ROS ##
find_package(ament_cmake REQUIRED)
find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_messages REQUIRED)
find_package(vtr_pose_graph REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)


## Libraries
file(GLOB_RECURSE PATH_TRACKER_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_library(${PROJECT_NAME}
        ${PATH_TRACKER_SRC}
        )

ament_target_dependencies(${PROJECT_NAME}
        Eigen3
        vtr_common
        vtr_logging
        vtr_messages
        vtr_pose_graph
        geometry_msgs
        std_msgs
        rclcpp
        )
target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
        Eigen3
        vtr_common
        vtr_logging
        vtr_messages
        vtr_pose_graph
        geometry_msgs
        std_msgs
        rclcpp
)

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

# Install config file(s)
install(
        DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    # Linting
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
