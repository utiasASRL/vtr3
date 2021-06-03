// Note: include and only include this header in main.cpp.
#pragma once

// #include "easylogging++.h"
#include "vtr_logging/easylogging++.h"

namespace vtr {
namespace logging {

/// Sets up logging in a sane way
void configureLogging(const std::string& log_filename = "", bool debug = false);

}  // namespace logging
}  // namespace vtr
