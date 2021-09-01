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
 * \file stopwatch.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <chrono>

namespace vtr {
namespace common {
namespace timing {

class Stopwatch {
 public:
  Stopwatch(bool start = false)
      : started_(false),
        paused_(false),
        reference_(std::chrono::steady_clock::now()),
        accumulated_(std::chrono::duration<long double>(0)) {
    if (start) this->start();
  }

  Stopwatch(const Stopwatch& other) = default;
  Stopwatch(Stopwatch&& other) = default;

  virtual ~Stopwatch() = default;

  Stopwatch& operator=(const Stopwatch& other) = default;
  Stopwatch& operator=(Stopwatch&& other) = default;

  void start() {
    if (!started_) {
      started_ = true;
      paused_ = false;
      accumulated_ = std::chrono::duration<long double>(0);
      reference_ = std::chrono::steady_clock::now();
    } else if (paused_) {
      reference_ = std::chrono::steady_clock::now();
      paused_ = false;
    }
  }

  void stop() {
    if (started_ && !paused_) {
      std::chrono::steady_clock::time_point now =
          std::chrono::steady_clock::now();
      accumulated_ =
          accumulated_ +
          std::chrono::duration_cast<std::chrono::duration<long double> >(
              now - reference_);
      paused_ = true;
    }
  }

  void reset() {
    if (started_) {
      started_ = false;
      paused_ = false;
      reference_ = std::chrono::steady_clock::now();
      accumulated_ = std::chrono::duration<long double>(0);
    }
  }

  template <class duration_t = std::chrono::milliseconds>
  typename duration_t::rep count() const {
    if (started_) {
      if (paused_) {
        return std::chrono::duration_cast<duration_t>(accumulated_).count();
      } else {
        return std::chrono::duration_cast<duration_t>(
                   accumulated_ +
                   (std::chrono::steady_clock::now() - reference_))
            .count();
      }
    } else {
      return duration_t(0).count();
    }
  }

 private:
  bool started_;
  bool paused_;
  std::chrono::steady_clock::time_point reference_;
  std::chrono::duration<long double> accumulated_;
};

}  // namespace timing
}  // namespace common
}  // namespace vtr