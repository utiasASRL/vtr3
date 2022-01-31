## Compiler setup - assumed to be GNU
# Compile as C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Turn on as many warnings as possible by default.
add_compile_options(-march=native -O3 -pthread -Wall -Wextra)

# template instantiation help
# add_compile_options(-frepo)

# built time and memory report
# add_compile_options(-ftime-report -fmem-report)

# address sanitizer
# add_compile_options(-fsanitize=address)
# set(CMAKE_CXX_STANDARD_LIBRARIES -lasan)


## Common packages setup
# Boost requirement (by mission_planning but needed everywhere)
find_package(Boost REQUIRED COMPONENTS system thread chrono timer)
# OpenMP - add flags in case we forget them somewhere
find_package(OpenMP)
if (OpenMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

## Compile for different ROS versions
if(DEFINED ENV{ROS_DISTRO})
  if("$ENV{ROS_DISTRO}" STREQUAL "foxy")
    add_definitions(-DVTR_ROS_FOXY)
  endif()
endif()

## Make VT&R run deterministically
# Note: these flags disable multi-threading in VTR tactic and pipelines, use
# with care and for debugging only.
# add_definitions(-DVTR_DETERMINISTIC)
# add_definitions(-DSTEAM_DEFAULT_NUM_OPENMP_THREADS=1) # note: also change this flag in steam package

## Enable certain pipelines
# Note: these flags are used to remove sensor specific dependencies when certain
# pipelines are not used. They determines which pipelines will be available
# rather than which pipeline to use. Enabled by default; Disable by commenting
# out.
# add_definitions(-DVTR_ENABLE_CAMERA)
add_definitions(-DVTR_ENABLE_LIDAR)

# ## GPUSURF enable/disable flag (used by vision pipeline only)
# # Note: currently assume that gpusurf is always available, because we have no
# # other options, so do not disable (i.e. comment out) this flag
# add_definitions(-DVTR_ENABLE_GPUSURF)  # set the available flag
# set(OpenCV_DIR /usr/local/opencv_cuda/lib/cmake/opencv4)
# find_package(OpenCV REQUIRED)