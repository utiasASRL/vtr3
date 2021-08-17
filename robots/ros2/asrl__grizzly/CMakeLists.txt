##### HEADER #####
cmake_minimum_required(VERSION 3.5)
project(asrl__grizzly)

##Set the GCC compiler options
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic -Wall -Wextra -Wno-long-long")
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

# file(GLOB_RECURSE LAUNCHES ${CMAKE_CURRENT_SOURCE_DIR}/launch/ *.launch)
# add_custom_target(${PROJECT_NAME}__launches SOURCES ${LAUNCHES}) # QtCreator

# file(GLOB_RECURSE XACRO ${CMAKE_CURRENT_SOURCE_DIR}/urdf/ *.xacro */*.xacro *.urdf.xml)
# add_custom_target(${PROJECT_NAME}__xacro SOURCES ${XACRO}) # QtCreator

