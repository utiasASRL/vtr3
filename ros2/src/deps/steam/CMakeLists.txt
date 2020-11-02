cmake_minimum_required(VERSION 3.16)
project(steam)

# Compiler setup (assumed to be GNU)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-march=native -O3 -Wall -Wno-long-long -pedantic -Wno-unused-function)
add_definitions(-DSTEAM_DEFAULT_NUM_OPENMP_THREADS=4) # define number of threads to be used by OpenMP in steam

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED system)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
find_package(lgmath REQUIRED)

# TODO (yuchen) Check if the two flags are still needed.
# add_definitions(-DEIGEN_DONT_VECTORIZE=1)  # vtr3 change : issue in ubuntu 20.04
# add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT=1)

# Libraries
file(GLOB_RECURSE SOURCE src/*.cpp)
add_library(${PROJECT_NAME} ${SOURCE})
ament_target_dependencies(${PROJECT_NAME} Boost Eigen3 OpenMP lgmath)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Executables
add_executable(SimpleBAandTrajPrior sample/SimpleBAandTrajPrior.cpp)
target_link_libraries(SimpleBAandTrajPrior ${PROJECT_NAME})
target_include_directories(SimpleBAandTrajPrior
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(SimpleBundleAdjustment sample/SimpleBundleAdjustment.cpp)
target_link_libraries(SimpleBundleAdjustment ${PROJECT_NAME})
target_include_directories(SimpleBundleAdjustment
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(SimpleBundleAdjustmentFullRel sample/SimpleBundleAdjustmentFullRel.cpp)
target_link_libraries(SimpleBundleAdjustmentFullRel ${PROJECT_NAME})
target_include_directories(SimpleBundleAdjustmentFullRel
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(SimpleBundleAdjustmentRelLand sample/SimpleBundleAdjustmentRelLand.cpp)
target_link_libraries(SimpleBundleAdjustmentRelLand ${PROJECT_NAME})
target_include_directories(SimpleBundleAdjustmentRelLand
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(SimpleBundleAdjustmentRelLandX sample/SimpleBundleAdjustmentRelLandX.cpp)
target_link_libraries(SimpleBundleAdjustmentRelLandX ${PROJECT_NAME})
target_include_directories(SimpleBundleAdjustmentRelLandX
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(SimplePoseGraphRelax sample/SimplePoseGraphRelax.cpp)
target_link_libraries(SimplePoseGraphRelax ${PROJECT_NAME})
target_include_directories(SimplePoseGraphRelax
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(SimpleTrajectoryPrior sample/SimpleTrajectoryPrior.cpp)
target_link_libraries(SimpleTrajectoryPrior ${PROJECT_NAME})
target_include_directories(SimpleTrajectoryPrior
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(SpherePoseGraphRelax sample/SpherePoseGraphRelax.cpp)
target_link_libraries(SpherePoseGraphRelax ${PROJECT_NAME})
target_include_directories(SpherePoseGraphRelax
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(TrustRegionExample sample/TrustRegionExample.cpp)
target_link_libraries(TrustRegionExample ${PROJECT_NAME})
target_include_directories(TrustRegionExample
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# May need the following
ament_export_dependencies(
  Boost
  Eigen3
  OpenMP
  lgmath
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    # Libraries
    ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  TARGETS
    # Executables
    SimpleBAandTrajPrior
    SimpleBundleAdjustment
    SimpleBundleAdjustmentFullRel
    SimpleBundleAdjustmentRelLand
    SimpleBundleAdjustmentRelLandX
    SimplePoseGraphRelax
    SimpleTrajectoryPrior
    SpherePoseGraphRelax
    TrustRegionExample
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Unit-tests
  ament_add_gtest(pattern_tests test/pattern_tests.cpp)
  target_link_libraries(pattern_tests ${PROJECT_NAME})
  ament_add_gtest(time_tests test/time_tests.cpp)
  target_link_libraries(time_tests ${PROJECT_NAME})

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()