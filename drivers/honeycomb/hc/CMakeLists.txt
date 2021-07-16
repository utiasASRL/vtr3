cmake_minimum_required(VERSION 2.8.3)

# Not using project version
if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

project(hc)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  dynamic_reconfigure
  roscpp
  sensor_msgs
  visualization_msgs
)

# Honeycomb api
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Honeycomb REQUIRED)
if(NOT Honeycomb_FOUND)
 message (FATAL_ERROR "Need Honeycomb API library to continue.")
endif()
include_directories(${Honeycomb_INCLUDE_DIR})

generate_dynamic_reconfigure_options(
  cfg/HcNode.cfg
)

catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
    sensor_msgs
    visualization_msgs
  INCLUDE_DIRS include
  LIBRARIES scan_image_producer pitch_table_utils scans
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

###########
## Build ##
###########

set(CMAKE_CXX_STANDARD 11)

# lib pitch_table_utils
add_library(pitch_table_utils src/pitch_table_utils.cc)
target_link_libraries(pitch_table_utils ${catkin_LIBRARIES})
add_dependencies(pitch_table_utils ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

# lib scans
add_library(scans src/scans.cc)
target_link_libraries(scans ${catkin_LIBRARIES} ${Honeycomb_LIBRARY})
add_dependencies(scans ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

# lib scan_image_producer
add_library(scan_image_producer src/scan_image_producer.cc)
target_link_libraries(scan_image_producer ${catkin_LIBRARIES} ${Honeycomb_LIBRARY})
add_dependencies(scan_image_producer ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

# hc_node
add_executable(hc_node src/hc_node.cc)
target_link_libraries(hc_node ${catkin_LIBRARIES} scans scan_image_producer pitch_table_utils
  ${Honeycomb_LIBRARY}
)
add_dependencies(hc_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

#############
## Install ##
#############

install(TARGETS scans scan_image_producer pitch_table_utils
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS hc_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  bin/csv_to_pc2
  bin/pc2_to_csv
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
