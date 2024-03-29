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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <chrono>

#include "vtr_common/timing/date.h"  /// \todo remove this when moving to c++20

namespace vtr {
namespace common {
namespace timing {

// Bring in all of the typedefs so the prefix is consistent
using date::days;
using date::months;
using date::years;
using std::chrono::hours;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::minutes;
using std::chrono::nanoseconds;
using std::chrono::seconds;

using clock = std::chrono::high_resolution_clock;
using time_point = std::chrono::time_point<clock>;
using duration_ms = std::chrono::duration<double, std::milli>;

/** \brief Converts a unix stamp (ns since epoch, UTC) to a chrono time point */
time_point toChrono(const uint64_t& nano_since_epoch);

/** \brief Converts a chrono time point to a unix stamp (ns since epoch, UTC) */
uint64_t toUnix(const time_point& time);

/** \brief Generate a human-readable string of a chrono time point */
std::string toIsoString(const time_point& time);

/** \brief Generate a string of a chrono time point to be used in a file name */
std::string toIsoFilename(const time_point& time);

}  // namespace timing
}  // namespace common
}  // namespace vtr