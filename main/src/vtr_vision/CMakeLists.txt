cmake_minimum_required(VERSION 3.8)
project(vtr_vision)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()



# Common setup for vtr packages
include("${CMAKE_CURRENT_LIST_DIR}/../vtr_common/vtr_include.cmake")


# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.

find_package(OpenCV REQUIRED)
# find_package(Torch REQUIRED)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

find_package(gpusurf REQUIRED) # currently assuming this is required
find_package(lgmath REQUIRED)
find_package(steam REQUIRED)

find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_messages REQUIRED)
find_package(vtr_storage REQUIRED)
find_package(vtr_tactic REQUIRED)


file(GLOB_RECURSE SRC src/*.cpp)
add_library(${PROJECT_NAME} ${SRC})

# target_link_libraries(${PROJECT_NAME} ${TORCH_LIBRARIES})
#set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)

ament_target_dependencies(${PROJECT_NAME}
  OpenCV
  gpusurf lgmath steam
  vtr_common vtr_logging vtr_messages vtr_storage vtr_tactic
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  #${TORCH_INCLUDE_DIRS})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  gpusurf lgmath steam
  vtr_common vtr_logging vtr_messages vtr_storage vtr_tactic
  # Torch
)

#ament_export_include_directories(${TORCH_INCLUDE_DIRS})
#ament_export_libraries(${TORCH_LIBRARIES})


install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
