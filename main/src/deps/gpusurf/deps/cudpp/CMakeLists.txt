cmake_minimum_required(VERSION 3.16)
project(cudpp)

find_package(ament_cmake REQUIRED)

# Find dependencies
find_package(CUDA)


if (CUDA_FOUND)
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
endif()
ament_package()