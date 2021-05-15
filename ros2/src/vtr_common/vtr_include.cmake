## Compiler setup - assumed to be GNU
# Compile as C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Turn on as many warnings as possible by default.
add_compile_options(-march=native -O3 -pthread -Wall -Wextra)

# TODO for now turn off some warnings
# add_compile_options(-Wno-unused-function -Wno-virtual-move-assign -Wno-terminate -Wno-deprecated-declarations -Wno-pragmas)
# add_compile_options(-Wno-expansion-to-defined -Wno-reorder)

# we suck at template instantiation. Help pls
# add_compile_options(-frepo)

# info please, why does the build suck?
# add_compile_options(-ftime-report -fmem-report)

# address sanitizer
# add_compile_options(-fsanitize=address)
# set(CMAKE_CXX_STANDARD_LIBRARIES -lasan)


## Common packages setup
# Boost requirement (by mission_planning but needed everywhere)
find_package(Boost REQUIRED COMPONENTS system thread)

# OpenMP - add flags in case we forget them somewhere
find_package(OpenMP)
if (OpenMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


## GPU setup
# GPUSURF enable/disable flag
find_package(gpusurf QUIET)  # Note: currently assume that gpusurf is always available
add_definitions(-DGPUSURF_ENABLED=1)  # set the available flag


## Easylogging++
# Disable loggings (use if needed)
#SET(EASYLOGGING_FLAGS "-DELPP_DISABLE_LOGS")
# Disable default log file
SET(EASYLOGGING_FLAGS "${EASYLOGGING_FLAGS} -DELPP_NO_DEFAULT_LOG_FILE")
# Disable crash handling so that we get the true crash reason.
SET(EASYLOGGING_FLAGS "${EASYLOGGING_FLAGS} -DELPP_DISABLE_DEFAULT_CRASH_HANDLING")
# Thread safety is required
SET(EASYLOGGING_FLAGS "${EASYLOGGING_FLAGS} -DELPP_THREAD_SAFE -DELPP_STACKTRACE_ON_CRASH -DELPP_FEATURE_CRASH_LOG")
# Uncomment to enable logging of STL containers
SET(EASYLOGGING_FLAGS "${EASYLOGGING_FLAGS} -DELPP_STL_LOGGING -DELPP_LOG_STD_ARRAY -DELPP_LOG_STD_UNORDERED_MAP -DELPP_LOG_STD_UNORDERED_SET")
# Remove leading and trailing whitespaces.
string(STRIP ${EASYLOGGING_FLAGS} EASYLOGGING_FLAGS)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EASYLOGGING_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EASYLOGGING_FLAGS}")

## Make VT&R run deterministically
add_definitions(-DDETERMINISTIC_VTR)
add_definitions(-DSTEAM_DEFAULT_NUM_OPENMP_THREADS=1)