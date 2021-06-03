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