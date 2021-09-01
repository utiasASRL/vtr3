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
 * \file instantiate.cpp
 * \brief
 * \details
 *
 * \author Kirk MacTavish, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_vision/sensors/sensor_model_base.hpp>
#include "vtr_vision/sensors/register_svd.hpp"

namespace vtr {
namespace vision {

// Explicit instantiation of the SVD registrar
template bool registerSVD<3>(
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& a,
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& b,
    Eigen::Matrix<double, 4, 4>* tf,
    const Eigen::Array<double, 1, Eigen::Dynamic>& weights, bool scaling);
template bool registerSVD<2>(
    const Eigen::Matrix<double, 2, Eigen::Dynamic>& a,
    const Eigen::Matrix<double, 2, Eigen::Dynamic>& b,
    Eigen::Matrix<double, 3, 3>* tf,
    const Eigen::Array<double, 1, Eigen::Dynamic>& weights, bool scaling);

template class SensorModelBase<Eigen::Matrix3d>;
template class SensorModelBase<Eigen::Matrix4d>;

}  // namespace vision
}  // namespace vtr
