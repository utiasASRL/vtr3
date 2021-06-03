cmake_minimum_required(VERSION 3.16)
project(cudpp)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(CUDA REQUIRED)

# Libraries
add_subdirectory(cudpp)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
  DIRECTORY 
    cudpp/cudpp/include/
    cudpp/common/inc/
  DESTINATION include
)

install(
  TARGETS
    # Libraries
    cudpp
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_package()