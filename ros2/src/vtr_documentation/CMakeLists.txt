cmake_minimum_required(VERSION 3.16)
project(vtr_documentation)

## Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_cmake/vtr_include.cmake.in")

## find dependencies
find_package(ament_cmake REQUIRED)
find_package(Doxygen
             REQUIRED dot
             OPTIONAL_COMPONENTS mscgen dia)

## Build documentation
# Packages that we want to generate documentation for
set(INPUT_LIST
  vtr_documentation
)
# Get absolute paths to packages
foreach(PATH ${INPUT_LIST})
  set(ROOT_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/../${PATH}")
  if(EXISTS "${ROOT_FOLDER}/src")
    list(APPEND INPUT_PATHS "${ROOT_FOLDER}/src")
  endif()
  if(EXISTS "${ROOT_FOLDER}/include")
    list(APPEND INPUT_PATHS "${ROOT_FOLDER}/include")
  endif()
  if(EXISTS "${ROOT_FOLDER}/tutorial")
    list(APPEND INPUT_PATHS "${ROOT_FOLDER}/tutorial")
  endif()
endforeach(PATH)
# Set mdfile for main page
file(GLOB MAINPAGE_MDFILE ${CMAKE_CURRENT_SOURCE_DIR}/mainpage.md)
# Build
if(${DOXYGEN_FOUND})
  set(DOXYGEN_EXTRACT_ALL YES)
  set(DOXYGEN_EXTRACT_PRIVATE YES)
  set(DOXYGEN_EXTRACT_PACKAGE YES)
  set(DOXYGEN_EXTRACT_STATIC YES)
  set(DOXYGEN_EXTRACT_LOCAL_METHODS YES)
  set(DOXYGEN_EXTRACT_ANON_NSPACES YES)
  set(DOXYGEN_SORT_BRIEF_DOCS YES)
  set(DOXYGEN_RECURSIVE YES)
  set(DOXYGEN_SOURCE_BROWSER YES)
  doxygen_add_docs(doc
    ${MAINPAGE_MDFILE}
    ${INPUT_PATHS}
    ALL
  )
endif()

## Install to install directory
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html
  DESTINATION docs
)

ament_package()
