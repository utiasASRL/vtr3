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
 * \file utils.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <cstdio>
#include <exception>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"

namespace vtr {
namespace storage {

constexpr const char CALIBRATION_FOLDER[] = "calibration";

class NoCalibration {};

struct NoBagExistsException : public std::runtime_error {
  NoBagExistsException(rcpputils::fs::path directory)
      : std::runtime_error(""), directory_(directory) {}
  const char* what() const throw() {
    return ("No bag exists at directory " + directory_.string()).c_str();
  }
  rcpputils::fs::path get_directory() { return directory_; }
  rcpputils::fs::path directory_;
};

}  // namespace storage
}  // namespace vtr