# Library
file(GLOB_RECURSE GEOMETRY_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
add_library(${PROJECT_NAME}__geometry ${GEOMETRY_SRC})
target_link_libraries(${PROJECT_NAME}__geometry ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(${PROJECT_NAME}__geometry ${catkin_EXPORTED_TARGETS})
