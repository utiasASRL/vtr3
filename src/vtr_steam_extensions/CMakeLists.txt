###### HEADER #####
cmake_minimum_required(VERSION 3.16)
project(vtr_steam_extensions)

# Common setup for vtr packages
include(
  "${CMAKE_CURRENT_LIST_DIR}/../vtr_cmake/vtr_include.cmake.in"
  NO_POLICY_SCOPE
)

###### IMPORT DEPENDENCIES (build_depends) ######
## System ##
#find_package([system_dependency_1] REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(easyloggingpp REQUIRED)
# find OpenMP - multithreading package
find_package(OpenMP REQUIRED)
set_openmp_flags()

add_definitions(-DEIGEN_DONT_VECTORIZE=1)  # vtr3 change: issue in ubuntu 20.04
add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT=1)

###### ROS ######
# This is a top-level algorithm package that is ros-agnostic
# It should not have any ROS dependencies, but may have other ros-agnostic dependencies.
# The CATKIN_ENABLED flag is cached, and may be passed at run time to enable catkin exporting.
if (CATKIN_ENABLED)
    # Adding packages as components adds them to our ${catkin_INCLUDE_DIRS} and ${catkin_LIBRARIES} variables.
    find_package(catkin REQUIRED
                 COMPONENTS asrl__common roscpp vtr_vision tf steam robochunk_base robochunk_util robochunk_msgs
    )
    ##### EXPORT MYSELF TO CATKIN #####
    #   "INCLUDE_DIRS" specifies to location of your package's headers for inclusion in inheriting ${catkin_INCLUDE_DIRS}.
    #   "LIBRARIES" specifies the libraries your package provides for inclusion in inheriting ${catkin_LIBRARIES}. This should match all add_library directives.
    #   "DEPENDS" specifies the system dependencies
    #   "CATKIN_DEPENDS" specifies the ros dependencies
    catkin_package(
                   INCLUDE_DIRS include
                   LIBRARIES ${PROJECT_NAME}
                   #Dependencies (build_depends and run_depends)
                   DEPENDS #[system_dependency]
                   CATKIN_DEPENDS asrl__common vtr_vision roscpp steam robochunk_base robochunk_util robochunk_msgs
    )
endif()

##### CONFIGURE INCLUDE DIRECTORIES #####
include_directories(include
                    ${catkin_INCLUDE_DIRS}
)

##### SPECIFY BUILD TARGETS #####
##Libraries:
file(GLOB STEAM_EXTENSIONS_SRC  ${CMAKE_CURRENT_SOURCE_DIR}/src/evaluator/*.cpp)
add_library(${PROJECT_NAME} ${STEAM_EXTENSIONS_SRC})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${LGMath_LIBRARIES} ${Steam_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

##Executables:
#add_executable(${PROJECT_NAME} src/exe_src1.cpp src/exe_src2.cpp)
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${[system_dependency_1]_LIBRARIES})
#add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# Unit Tests
# add_subdirectory(test)

