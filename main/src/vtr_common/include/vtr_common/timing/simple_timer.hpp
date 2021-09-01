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
 * \file simple_timer.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <chrono>
#include <ostream>

#include <vtr_common/timing/time_utils.hpp>

namespace vtr {
namespace common {
namespace timing {

class SimpleTimer {
 public:
  SimpleTimer() { reset(); }
  ~SimpleTimer() {}
  void reset() { start_time_ = get_time(); }
  double elapsedMs() const {
    return duration_ms(get_time() - start_time_).count();
  }

  friend std::ostream& operator<<(std::ostream& os, const SimpleTimer& st) {
    return os << st.elapsedMs() << " ms";
  }

 private:
  static time_point get_time() { return clock::now(); }

  time_point start_time_;
};

}  // namespace timing
}  // namespace common
}  // namespace vtr
