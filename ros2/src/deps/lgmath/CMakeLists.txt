cmake_minimum_required(VERSION 3.16)
project(lgmath)

## Compiler setup
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-march=native -pthread -Wall -Wextra)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# # Libraries
# file(
#   GLOB_RECURSE INDEX_SRC
#   src/index src/simple_graph
#   *.cpp
# )
# add_library(${PROJECT_NAME}_index ${INDEX_SRC})
# ament_target_dependencies(${PROJECT_NAME}_index
#   vtr_common
#   vtr_logging
#   vtr_messages
# )
# target_include_directories(${PROJECT_NAME}_index
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>)

# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# install(
#   DIRECTORY include/
#   DESTINATION include
# )

# install(
#   TARGETS ${PROJECT_NAME}_index
#   EXPORT export_${PROJECT_NAME}
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(simple_graph_tests test/simple_graph_tests.cpp)
#   # ament_target_dependencies(simple_graph_tests ${PROJECT_NAME}_index) # Does not work yet
#   target_link_libraries(simple_graph_tests ${PROJECT_NAME}_index)
#   ament_add_gtest(id_tests test/id_tests.cpp)
#   target_link_libraries(id_tests ${PROJECT_NAME}_index)
#   ament_add_gtest(read_write_tests test/read_write_tests.cpp)
#   target_link_libraries(read_write_tests ${PROJECT_NAME}_index)

#   # Linting
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
# endif()

ament_package()
