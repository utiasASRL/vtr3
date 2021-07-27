// Note: include and only include this header in main.cpp.
#pragma once

#include "vtr_logging/easylogging++.h"

namespace vtr {
namespace logging {

/**
 * \brief Configures easylogging++ for use in vtr.
 * \param[in] log_filename filename to store logs
 * \param[in] debug enable debug logs
 * \param[in] enabled vector of loggers (string). If not empty then only the
 * specified loggers will be enabled; otherwise all loggers will be enabled
 */
void configureLogging(const std::string& log_filename = "", bool debug = false,
                      const std::vector<std::string>& enabled = {});

}  // namespace logging
}  // namespace vtr
