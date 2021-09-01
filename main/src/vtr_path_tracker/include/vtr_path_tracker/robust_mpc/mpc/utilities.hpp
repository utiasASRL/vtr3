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
 * \file utilities.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_path_tracker/robust_mpc/optimization/mpc_nominal_model.hpp>

namespace vtr {
namespace path_tracker {
namespace utils {

/**
 * \brief thetaWrap Wrap an angle to (-pi, pi]
 * \param th_in The angle
 * \note  Assumes abs(th_in) < 3pi
 * \return The equivalent angle between (-pi, pi]
 */
double thetaWrap(double th_in);

/** \brief Returns the sign of a number as a float (-1.,1.) */
double getSign(double number);

}  // namespace utils
}  // namespace path_tracker
}  // namespace vtr
