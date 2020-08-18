# Bridge Library
file(GLOB MSG_BRIDGE_SRCS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
add_library(${PROJECT_NAME}__message_bridge ${MSG_BRIDGE_SRCS})
target_link_libraries(${PROJECT_NAME}__message_bridge
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
