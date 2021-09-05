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
 * \file message.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <any>
#include <iostream>
#include <optional>

#include <rcutils/time.h>

namespace vtr {
namespace storage {

using Index = int32_t;
using TimeStamp = rcutils_time_point_value_t;

// timestamp value stored in sqlite database if message has no timestamps
constexpr TimeStamp NO_TIMESTAMP_VALUE = -1;

class VTRMessage {
public:
  VTRMessage() = default;
  template <class T> VTRMessage(T message) : message_{message} {
    static_assert(!std::is_same_v<std::any, T>,
                  "Attempted to initialize a VTRMessage with an std::any!");
  }

  template <class T> VTRMessage &operator=(const T &message) {
    message_ = std::make_any<T>(message);
    return *this;
  }

  template <class T> void set(T message) {
    message_ = std::make_any<T>(message);
  }

  template <class T> T get() const {
    try {
      return std::any_cast<T>(message_);
    } catch (const std::bad_any_cast &e) {
      throw std::runtime_error(
          "Any cast failed in retrieving data in VTR Storage");
    }
  }

  TimeStamp get_timestamp() const {
    if (!timestamp_.has_value()) {
      throw std::runtime_error(
          "Attempted to get uninitialized timestamp of a VTRMessage");
    } else {
      return timestamp_.value();
    }
  }

  bool has_timestamp() const { return timestamp_.has_value(); }

  void set_timestamp(TimeStamp new_timestamp) { timestamp_ = new_timestamp; }

  Index get_index() const {
    if (!database_index_.has_value()) {
      throw std::runtime_error(
          "Attempted to get uninitialized timestamp of a VTRMessage");
    } else {
      return database_index_.value();
    }
  }

  bool has_index() const { return database_index_.has_value(); }

  void set_index(Index new_index) { database_index_ = new_index; }

private:
  std::any message_;
  std::optional<Index> database_index_;
  std::optional<TimeStamp> timestamp_;
};

} // namespace storage
} // namespace vtr