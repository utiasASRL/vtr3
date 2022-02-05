cmake_minimum_required(VERSION 3.16)
project(vtr_pose_graph)

# Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")

# Find dependencies
find_package(ament_cmake REQUIRED)

find_package(lgmath REQUIRED)
find_package(steam REQUIRED)

find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_storage REQUIRED)
find_package(vtr_pose_graph_msgs REQUIRED)

# Libraries
include_directories(
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# simple graph - generic graph structure with searching functions
file(GLOB_RECURSE SIMPLE_GRAPH_SRC src/simple_graph/*.cpp)
add_library(${PROJECT_NAME}_simple_graph ${SIMPLE_GRAPH_SRC})
ament_target_dependencies(${PROJECT_NAME}_simple_graph
  vtr_common vtr_logging
)

# index - spatio-temporal pose graph structure
file(GLOB_RECURSE INDEX_SRC src/index/*.cpp)
add_library(${PROJECT_NAME}_index ${INDEX_SRC})
ament_target_dependencies(${PROJECT_NAME}_index
  vtr_common vtr_logging
)
target_link_libraries(${PROJECT_NAME}_index
  ${PROJECT_NAME}_simple_graph
)

# serializable
file(GLOB_RECURSE SERIALIZABLE_SRC
  src/serializable/*.cpp
)
add_library(${PROJECT_NAME}_serializable ${SERIALIZABLE_SRC})
ament_target_dependencies(${PROJECT_NAME}_serializable
  vtr_common vtr_logging vtr_storage vtr_pose_graph_msgs
)
target_link_libraries(${PROJECT_NAME}_serializable
  ${PROJECT_NAME}_index
)

# # evaluator
# file(GLOB_RECURSE EVALUATOR_SRC
#   src/evaluator/*.cpp
# )
# add_library(${PROJECT_NAME}_evaluator ${EVALUATOR_SRC})
# ament_target_dependencies(${PROJECT_NAME}_evaluator
#   vtr_common vtr_logging vtr_storage vtr_pose_graph_msgs
# )
# target_link_libraries(${PROJECT_NAME}_evaluator
#   ${PROJECT_NAME}_serializable
# )

# # path (localization chain)
# file(GLOB_RECURSE PATH_SRC
#   src/path/*.cpp
# )
# add_library(${PROJECT_NAME}_path ${PATH_SRC})
# ament_target_dependencies(${PROJECT_NAME}_path
#   vtr_common vtr_logging vtr_storage vtr_pose_graph_msgs
# )
# target_link_libraries(${PROJECT_NAME}_path
#   ${PROJECT_NAME}_evaluator
# )

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  lgmath steam
  vtr_common vtr_logging vtr_storage vtr_pose_graph_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_simple_graph
    ${PROJECT_NAME}_index
    ${PROJECT_NAME}_serializable
#     ${PROJECT_NAME}_evaluator
#     ${PROJECT_NAME}_path
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  # id tests
  ament_add_gmock(test_id test/id/test_id.cpp)
  ament_target_dependencies(test_id vtr_logging vtr_common)

  # simple graph tests
  ament_add_gtest(test_simple_graph test/simple_graph/test_simple_graph.cpp)
  target_link_libraries(test_simple_graph ${PROJECT_NAME}_simple_graph)

  # index tests
  ament_add_gmock(test_edge_vertex_base test/index/test_edge_vertex_base.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_edge_vertex_base ${PROJECT_NAME}_index)
  ament_add_gmock(test_graph_structure test/index/test_graph_structure.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_graph_structure ${PROJECT_NAME}_index)
  ament_add_gmock(test_subgraph test/index/test_subgraph.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_subgraph ${PROJECT_NAME}_index)

  # serialization tests
  ament_add_gmock(test_serialization_vertex test/serializable/test_serialization_vertex.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_serialization_vertex ${PROJECT_NAME}_serializable)
  ament_add_gmock(test_serialization_edge test/serializable/test_serialization_edge.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_serialization_edge ${PROJECT_NAME}_serializable)
  ament_add_gmock(test_serialization_graph test/serializable/test_serialization_graph.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_serialization_graph ${PROJECT_NAME}_serializable)

  # data read write tests
  find_package(std_msgs REQUIRED)
  ament_add_gmock(test_data_read_write test/stream/test_data_read_write.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  ament_target_dependencies(test_data_read_write std_msgs)
  target_link_libraries(test_data_read_write ${PROJECT_NAME}_serializable)
  ament_add_gmock(test_vertex_stream_time test/stream/test_vertex_stream_time.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  ament_target_dependencies(test_vertex_stream_time std_msgs)
  target_link_libraries(test_vertex_stream_time ${PROJECT_NAME}_serializable)

  # evaluator tests
  # ament_add_gtest(test_evaluator_base test/evaluator/test_evaluator_base.cpp)
  # target_link_libraries(test_evaluator_base ${PROJECT_NAME}_evaluator)
  # ament_add_gtest(test_evaluator test/evaluator/test_evaluator.cpp)
  # target_link_libraries(test_evaluator ${PROJECT_NAME}_evaluator)
  # ament_add_gmock(test_graph_traversal test/evaluator/test_graph_traversal.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  # target_link_libraries(test_graph_traversal ${PROJECT_NAME}_evaluator)
  # ament_add_gmock(test_subgraph_evaluator test/evaluator/test_subgraph_evaluator.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  # target_link_libraries(test_subgraph_evaluator ${PROJECT_NAME}_evaluator)

  # # path tests
  # ament_add_gmock(test_path test/path/test_path.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  # target_link_libraries(test_path ${PROJECT_NAME}_path)
  # ament_add_gmock(test_localization_chain test/path/test_localization_chain.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  # target_link_libraries(test_localization_chain ${PROJECT_NAME}_path)
endif()

ament_package()
