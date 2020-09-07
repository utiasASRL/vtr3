cmake_minimum_required(VERSION 3.16)
project(date)

## Find dependencies
find_package(ament_cmake REQUIRED)

## External easylogging++ project
include(ExternalProject)
ExternalProject_Add(
  date
  PREFIX external
  GIT_REPOSITORY https://github.com/HowardHinnant/date.git
  GIT_TAG v3.0.0
  INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  UPDATE_COMMAND ""
  PATCH_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND rsync -rm --include=*.h <SOURCE_DIR>/include/date/ <INSTALL_DIR>/include
)

install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

ament_export_include_directories(include)

ament_package()
