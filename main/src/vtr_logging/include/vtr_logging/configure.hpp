// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file configure.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
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
