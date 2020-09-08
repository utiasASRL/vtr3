# Collect all the source files
 file(GLOB MISC_SRC
   "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

 file(GLOB SENSORS_SRC
   "${CMAKE_CURRENT_SOURCE_DIR}/sensors/*.cpp")
 file(GLOB GEOMETRY_SRC
   "${CMAKE_CURRENT_SOURCE_DIR}/geometry/*.cpp")
# file(GLOB INTERFACE_SRC
#   ${CMAKE_CURRENT_SOURCE_DIR}/interface/robochunk_conversions.cpp
#     )
file(GLOB MSG_BRIDGE_SRC
  "${CMAKE_CURRENT_SOURCE_DIR}/messages/*.cpp")
file(GLOB_RECURSE OUTLIERS_SRC
  "${CMAKE_CURRENT_SOURCE_DIR}/outliers/*.cpp")
file(GLOB FEATURES_SRC
  "${CMAKE_CURRENT_SOURCE_DIR}/features/extractor/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/features/matcher/*.cpp"
  # "${CMAKE_CURRENT_SOURCE_DIR}/features/augment/*.cpp"
  # "${CMAKE_CURRENT_SOURCE_DIR}/features/bow/*.cpp"
)
# file(GLOB STEREO_PROC_SRC
#   ${CMAKE_CURRENT_SOURCE_DIR}/stereo_proc/costs/SadCost.cpp
#   ${CMAKE_CURRENT_SOURCE_DIR}/stereo_proc/filters/BgrToGrayFilter.cpp
#   ${CMAKE_CURRENT_SOURCE_DIR}/stereo_proc/filters/BilateralFilter.cpp
#   ${CMAKE_CURRENT_SOURCE_DIR}/stereo_proc/filters/MedianFilter.cpp
#   ${CMAKE_CURRENT_SOURCE_DIR}/stereo_proc/filters/SpeckleFilter.cpp
#   ${CMAKE_CURRENT_SOURCE_DIR}/stereo_proc/matchers/CVStereoBM.cpp
#   ${CMAKE_CURRENT_SOURCE_DIR}/stereo_proc/matchers/CVStereoSGBM.cpp
#   ${CMAKE_CURRENT_SOURCE_DIR}/stereo_proc/matchers/ElasMatcher.cpp
#   ${CMAKE_CURRENT_SOURCE_DIR}/stereo_proc/reprojectors/CVReprojector.cpp
#   ${CMAKE_CURRENT_SOURCE_DIR}/stereo_proc/StereoMatcher.cpp
#   )

# # Check if OpenCV was compiled with CUDA support, meaning these variables exist:
# if(OPENCV_GPU_FOUND OR (OPENCV_CUDAIMGPROC_FOUND AND OPENCV_CUDASTEREO_FOUND))
#   file(GLOB GPU_STEREO_PROC_SRC
#   ${CMAKE_CURRENT_SOURCE_DIR}/stereo_proc/filters/GpuBilateralFilter.cpp
#   ${CMAKE_CURRENT_SOURCE_DIR}/stereo_proc/matchers/CVGpuStereoBM.cpp
#   ${CMAKE_CURRENT_SOURCE_DIR}/stereo_proc/reprojectors/CVGpuReprojector.cpp
#   )
#   message(STATUS "OpenCV CUDA support enabled, compiling gpu-based stereo_proc files")
# else()
#   message("OpenCV CUDA support not enabled, only compiling cpu-based stereo_proc files")
# endif()

if((CUDA_ENABLED) AND (${gpusurf_FOUND}))
  message(STATUS "CUDA found, compiling gpu-based feature extractors")
  file(GLOB CUDA_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/features/extractor/cuda/*.cpp")
  set(CUDA_LIBS ${gpusurf_LIBRARIES})
  add_definitions(-DCUDA_ENABLED)
else()
  message(STATUS ${CUDA_ENABLED})
  message(STATUS ${gpusurf_FOUND})
  message(STATUS "CUDA not found, only compiling cpu-based feature extractors")
endif()

# Add the library
add_library(
  ${PROJECT_NAME}
  ${CUDA_SRC}
  ${FEATURES_SRC}
  ${GEOMETRY_SRC}
  # ${GPU_STEREO_PROC_SRC}
  # ${INTERFACE_SRC}
  ${MISC_SRC}
  ${MSG_BRIDGE_SRC}
  ${OUTLIERS_SRC}
  ${SENSORS_SRC}
  # ${STEREO_PROC_SRC}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CUDA_LIBS}
  ${LGMath_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)
# Optional libelas dependency
#if(${libelas_FOUND})
#    target_link_libraries(${PROJECT_NAME} ${libelas_LIBRARIES})
#endif()

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
