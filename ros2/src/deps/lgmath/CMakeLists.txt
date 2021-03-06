cmake_minimum_required(VERSION 3.16)
project(lgmath)

# Compiler setup
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-march=native -O3 -Wall -Wno-long-long -pedantic)
# x86 specific flags
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
  add_compile_options(-mno-avx)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Libraries
file(GLOB_RECURSE SOURCE src/*.cpp)
add_library(${PROJECT_NAME} ${SOURCE})
ament_target_dependencies(${PROJECT_NAME} Eigen3)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# May need the following
# ament_export_dependencies(eigen3_cmake_module)
# ament_export_dependencies(Eigen3)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Unit-tests
  ament_add_gtest(so3_tests test/SO3Tests.cpp)
  target_link_libraries(so3_tests ${PROJECT_NAME})
  ament_add_gtest(se3_tests test/SE3Tests.cpp)
  target_link_libraries(se3_tests ${PROJECT_NAME})
  ament_add_gtest(covariance_tests test/CovarianceTests.cpp)
  target_link_libraries(covariance_tests ${PROJECT_NAME})
  ament_add_gtest(rotation_tests test/RotationTests.cpp)
  target_link_libraries(rotation_tests ${PROJECT_NAME})
  ament_add_gtest(transform_tests test/TransformTests.cpp)
  target_link_libraries(transform_tests ${PROJECT_NAME})
  ament_add_gtest(transform_with_covariance_tests test/TransformWithCovarianceTests.cpp)
  target_link_libraries(transform_with_covariance_tests ${PROJECT_NAME})
  # Benchmarks (yuchen: temporarily disabled unless we have performance issue later on)
  # ament_add_gtest(so3_benchmarks benchmark/SO3SpeedTest.cpp)
  # target_link_libraries(so3_benchmarks ${PROJECT_NAME})
  # ament_add_gtest(se3_benchmarks benchmark/SE3SpeedTest.cpp)
  # target_link_libraries(se3_benchmarks ${PROJECT_NAME})
  # ament_add_gtest(rotation_benchmarks benchmark/RotationSpeedTest.cpp)
  # target_link_libraries(rotation_benchmarks ${PROJECT_NAME})
  # ament_add_gtest(transform_benchmarks benchmark/TransformSpeedTest.cpp)
  # target_link_libraries(transform_benchmarks ${PROJECT_NAME})
  # ament_add_gtest(transform_with_covariance_benchmarks benchmark/TransformWithCovarianceSpeedTest.cpp)
  # target_link_libraries(transform_with_covariance_benchmarks ${PROJECT_NAME})

  # Linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
