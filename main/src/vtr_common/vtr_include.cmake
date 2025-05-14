## Compiler setup
# Compile as C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Turn on as many warnings as possible by default.
add_compile_options(-march=native -O3 -pthread -Wall -Wextra)

# template instantiation help
# add_compile_options(-frepo)

#Add debug symbols
#add_compile_options(-g -Og)

# built time and memory report
# add_compile_options(-ftime-report -fmem-report)

# address sanitizer
# add_compile_options(-fsanitize=address)
# set(CMAKE_CXX_STANDARD_LIBRARIES -lasan)
# add_compile_options(-g -Og)


## Common packages setup
# Boost requirement (by mission_planning but needed everywhere)
find_package(Boost REQUIRED COMPONENTS system thread chrono timer)
# OpenMP - add flags in case we forget them somewhere
find_package(OpenMP)
if (OpenMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


#Set to VTR_PIPELINE=VISION, LIDAR, RADAR, or RADAR-LIDAR
set(SelectedPipeline "$ENV{VTR_PIPELINE}")

if(SelectedPipeline STREQUAL "LIDAR")
  add_definitions(-DVTR_ENABLE_LIDAR)
  set(VTR_ENABLE_LIDAR true)
elseif(SelectedPipeline STREQUAL "RADAR")
  add_definitions(-DVTR_ENABLE_RADAR)
  set(VTR_ENABLE_RADAR true)
elseif(SelectedPipeline STREQUAL "RADAR-LIDAR")
  add_definitions(-DVTR_ENABLE_RADAR)
  set(VTR_ENABLE_RADAR true)
  add_definitions(-DVTR_ENABLE_LIDAR)
  set(VTR_ENABLE_LIDAR true)
elseif(SelectedPipeline STREQUAL "VISION")
  ## GPUSURF enable/disable flag (used by vision pipeline only)
  # Note: currently assume that gpusurf is always available, because we have no
  # other options, so do not disable (i.e. comment out) this flag
  add_definitions(-DVTR_ENABLE_GPUSURF)  # set the available flag
  add_definitions(-DVTR_ENABLE_VISION)
  add_definitions(-DVTR_VISION_LEARNED)
  set(VTR_ENABLE_VISION true)
else()
  add_definitions(-DVTR_ENABLE_RADAR)
  set(VTR_ENABLE_RADAR true)
  add_definitions(-DVTR_ENABLE_LIDAR)
  set(VTR_ENABLE_LIDAR true)
  add_definitions(-DVTR_ENABLE_GPUSURF)  # set the available flag
  add_definitions(-DVTR_ENABLE_VISION)
  add_definitions(-DVTR_VISION_LEARNED)
  set(VTR_ENABLE_VISION true)
  message(WARNING "VTR_PIPELINE not set! Compiling all! Save time by selecting VTR_PIPELINE=VISION, LIDAR, RADAR, or RADAR-LIDAR")
endif()
