cmake_minimum_required(VERSION 2.8)

# Name the project
project(asrl__safety_monitor)
include("${CMAKE_CURRENT_LIST_DIR}/../asrl__cmake/AsrlIncludes.cmake.in" NO_POLICY_SCOPE)

# TODO: Fix these warnings in the code
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-sign-compare")

###### ROS ######
# This is a top-level algorithm package that is ros-agnostic
# It should not have any ROS dependencies, but may have other ros-agnostic dependencies.
# The CATKIN_ENABLED flag is cached, and may be passed at run time to enable catkin exporting.
if (CATKIN_ENABLED)
    find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        asrl__common
        asrl__messages
        asrl__control__path_tracker
        # asrl__terrain_assessment
        #    asrl__vehicle_safety__grizzly_ambience
    )

    # Generate msgs and srvs
    file(GLOB msg_files RELATIVE ${PROJECT_SOURCE_DIR}/msg msg/*.msg)
    add_message_files(FILES ${msg_files})

#    file(GLOB srv_files RELATIVE ${PROJECT_SOURCE_DIR}/srv srv/*.srv)
#    add_service_files(FILES ${srv_files})

    generate_messages(DEPENDENCIES)

    # Includes
    include_directories(
      include
      ${catkin_INCLUDE_DIRS}
    )

    #add_library(${PROJECT_NAME}_lib
    #  src/asrl/safety_monitor_input/safety_monitor_input.cpp
    #)

    #target_link_libraries(${PROJECT_NAME}_lib
    #    ${catkin_LIBRARIES}
    #)
    #add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS})

    catkin_package(INCLUDE_DIRS include
                   LIBRARIES ${PROJECT_NAME}_lib
                   CATKIN_DEPENDS std_msgs rospy roscpp
                                 asrl__control__path_tracker
                                 asrl__common
                                 asrl__messages
                                #  asrl__terrain_assessment
            #                     asrl__terrain_assessment__messages
            #                     asrl__vehicle_safety__grizzly_ambience
                   )


   file(GLOB_RECURSE INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/include/ *.h *.hpp *.inl *.cpp)
   add_custom_target(${PROJECT_NAME}__headers SOURCES ${INCLUDES}) # QtCreator

    add_executable(safety_monitor_node src/safety_monitor_node.cpp)
    target_link_libraries(safety_monitor_node  ${catkin_LIBRARIES})
    add_dependencies(safety_monitor_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

#    add_executable(safety_monitor_user_interface src/safety_monitor_user_interface.cpp)
#    target_link_libraries(safety_monitor_user_interface  ${catkin_LIBRARIES})


    #add_executable(vtr_monitor_node src/vtr_monitor_node.cpp)
    #target_link_libraries(vtr_monitor_node  ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)

endif()
