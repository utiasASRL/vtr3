cmake_minimum_required(VERSION 3.16)
project(vtr_documentation)

## find dependencies
find_package(ament_cmake REQUIRED)
find_package(Doxygen
             REQUIRED dot
             OPTIONAL_COMPONENTS mscgen dia)

## Build documentation
# Packages that we want to generate documentation for
set(INPUT_LIST
  vtr_common
  vtr_documentation
  vtr_lgmath_extensions
  vtr_lidar
  vtr_logging
  vtr_messages
  vtr_mission_planning
  vtr_navigation
  vtr_path_planning
  vtr_path_tracker
  vtr_pose_graph
  vtr_safety_monitor
  vtr_steam_extensions
  vtr_storage
  vtr_tactic
  # vtr_vision  # TODO caussing a weird PostScript error
)
# Get absolute paths to packages
foreach(PATH ${INPUT_LIST})
  set(ROOT_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/../${PATH}")
  if(EXISTS "${ROOT_FOLDER}/src")
    string(APPEND INPUT_PATHS " ${ROOT_FOLDER}/src")
  endif()
  if(EXISTS "${ROOT_FOLDER}/include")
    string(APPEND INPUT_PATHS " ${ROOT_FOLDER}/include")
  endif()
  if(EXISTS "${ROOT_FOLDER}/tutorial")
    string(APPEND INPUT_PATHS " ${ROOT_FOLDER}/tutorial")
  endif()
endforeach(PATH)
# Set mdfile for main page
file(GLOB MAINPAGE_MDFILE ${CMAKE_CURRENT_SOURCE_DIR}/mainpage.md)
# Build
if(${DOXYGEN_FOUND})
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in
    ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    @ONLY
  )
  add_custom_target(doc ALL
    ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM)
endif()

## Install to install directory
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/docs/
  DESTINATION docs
)

ament_package()
