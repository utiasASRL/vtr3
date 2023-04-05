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
 * \file instantiation.cpp
 * \brief
 * \details This file instantiates the VanillaRansac class, which provides a
 * basic RANSAC algorithm
 *
 * \author Kirk MacTavish, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_vision/outliers/ransac/vanilla_ransac.hpp>
#include <vtr_vision/sensors/sensor_model_base.hpp>

namespace vtr {
namespace vision {

template class RansacBase<Eigen::Matrix3d>;
template class RansacBase<Eigen::Matrix4d>;

template class VanillaRansac<Eigen::Matrix3d>;
template class VanillaRansac<Eigen::Matrix4d>;

}  // namespace vision
}  // namespace vtr