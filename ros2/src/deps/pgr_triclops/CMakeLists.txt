cmake_minimum_required(VERSION 3.16)
project(pgr_triclops)

# Compiler setup
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# build tool
find_package(ament_cmake REQUIRED)

if(NOT DEFINED CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
            Release Debug RelWithDebInfo MinSizeRel)
endif()

include_directories(include/triclops)

### External Libs : Find pre-built shared and static libraries
# Need to set the actual libraries first not the links for copy below
foreach(lib libflycapture2bridge.so.3.4.0.2 libtriclops.so.3.4.0.2 libflycapture2bridge.so.3 libtriclops.so.3 flycapture2bridge triclops pnmutils)
    # So if you have the same library name somewhere else, it will use that library instead.
    find_library(${lib}_library NAMES ${lib} PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/
            NO_DEFAULT_PATH NO_CMAKE_ENVIRONMENT_PATH NO_CMAKE_PATH NO_SYSTEM_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH NO_CMAKE_FIND_ROOT_PATH)
    set(ExternalSharedLibraries ${ExternalSharedLibraries} ${${lib}_library})
endforeach()

set(EXTERNAL_LIBS_INSTALL_PATH ${CMAKE_DEVEL_PREFIX}/lib)

### FIND SOURCE FILES ###
file(GLOB_RECURSE SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/src/triclops/src/pnmutils/*.cpp")

### TARGETS
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/)
add_library(${PROJECT_NAME} ${SOURCE})
target_link_libraries(${PROJECT_NAME}
        ${ExternalSharedLibraries}
        ${${PROJECT_NAME}_DEPEND_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/triclops>
        $<INSTALL_INTERFACE:include/triclops>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
        DIRECTORY include/triclops/
        DESTINATION include/triclops
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        INCLUDES DESTINATION include
)

install(
        PROGRAMS ${ExternalSharedLibraries}
        DESTINATION lib
)

if(BUILD_TESTING)
    # Linting
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()