add_definitions(-DBUILD_DLL)
include_directories(cudpp/include cudpp/src cudpp/src/kernel common/inc)

SET(CUDA_GENERATED_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/intermediate)

if(MSVC)
  set(CUTIL_PLATFORM_SOURCES common/src/stopwatch_win.cpp)
else(MSVC)
  set(CUTIL_PLATFORM_SOURCES common/src/stopwatch_linux.cpp)
endif(MSVC)

CUDA_ADD_LIBRARY(
  cudpp SHARED
  common/src/cmd_arg_reader.cpp
  common/src/bank_checker.cpp
  common/src/cutil.cpp
  ${CUTIL_PLATFORM_SOURCES}
  common/src/multithreading.cpp
  common/src/findFile.cpp
  common/src/param.cpp
  common/src/stopwatch.cpp
  cudpp/src/cudpp.cpp
  cudpp/src/cudpp_plan.cpp 
  cudpp/src/cudpp_plan_manager.cpp 
  cudpp/src/cudpp_maximal_launch.cpp
  cudpp/src/app/compact_app.cu 
  cudpp/src/app/scan_app.cu 
  cudpp/src/app/segmented_scan_app.cu 
  cudpp/src/app/spmvmult_app.cu 
  cudpp/src/app/radixsort_app.cu 
  cudpp/src/app/rand_app.cu 
)
