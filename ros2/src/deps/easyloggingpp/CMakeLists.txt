cmake_minimum_required(VERSION 3.16)
project(easyloggingpp)

## Find dependencies
find_package(ament_cmake REQUIRED)
find_program(MAKE_EXE make)

## External easylogging++ project

# Compile options
# Disable loggings (use if needed)
#SET(EASYLOGGING_FLAGS "-DELPP_DISABLE_LOGS")
# Disable default log file
SET(EASYLOGGING_FLAGS "${EASYLOGGING_FLAGS} -DELPP_NO_DEFAULT_LOG_FILE")
# Disable crash handling so that we get the true crash reason.
SET(EASYLOGGING_FLAGS "${EASYLOGGING_FLAGS} -DELPP_DISABLE_DEFAULT_CRASH_HANDLING")
# Thread safety is required
SET(EASYLOGGING_FLAGS "${EASYLOGGING_FLAGS} -DELPP_THREAD_SAFE -DELPP_STACKTRACE_ON_CRASH")
# Uncomment to enable logging of STL containers
#SET(EASYLOGGING_FLAGS "${EASYLOGGING_FLAGS} -DELPP_STL_LOGGING -DELPP_LOG_STD_ARRAY -DELPP_LOG_STD_UNORDERED_MAP -DELPP_LOG_STD_UNORDERED_SET")
# Remove leading and trailing whitespaces.
string(STRIP ${EASYLOGGING_FLAGS} EASYLOGGING_FLAGS)

# Add easylogging++ as an external project
include(ExternalProject)
ExternalProject_Add(
  easyloggingpp
  PREFIX external
  GIT_REPOSITORY git@github.com:amrayn/easyloggingpp.git
  GIT_TAG v9.97.0
  INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  UPDATE_COMMAND ""
  PATCH_COMMAND ""
  # Use default cmake configure and build
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DCMAKE_CXX_FLAGS="${EASYLOGGING_FLAGS}" -Dbuild_static_lib=ON
)

install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib
  DESTINATION ${CMAKE_INSTALL_PREFIX}
)
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

ament_export_include_directories(include)
# Added in easyloggingpp cmake file, and is able to be found by ament here.
ament_export_libraries(easyloggingpp)

ament_package()
