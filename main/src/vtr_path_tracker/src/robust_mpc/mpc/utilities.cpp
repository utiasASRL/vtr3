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
 * \file utilities.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_path_tracker/robust_mpc/mpc/utilities.hpp>

namespace vtr {
namespace path_tracker {
namespace utils {

double thetaWrap(double th_in) {
  double th_out = th_in;
  if (th_in > M_PI) {
    th_out = th_in - 2 * M_PI;
  } else if (th_in < -M_PI) {
    th_out = th_in + 2 * M_PI;
  }
  return th_out;
}

double getSign(double number) {
  return (double)((0.0 < number) - (number < 0.0));
}

}  // namespace utils
}  // namespace path_tracker
}  // namespace vtr
