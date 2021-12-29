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
 * \file conversions.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 * \brief lgmath classes to corresponding ROS messages conversion
 */
#pragma once

#include "lgmath.hpp"
#include "vtr_common_msgs/msg/lie_group_transform.hpp"

namespace vtr {

namespace common {

void toROSMsg(const lgmath::se3::TransformationWithCovariance& T,
              vtr_common_msgs::msg::LieGroupTransform& T_msg);

void fromROSMsg(const vtr_common_msgs::msg::LieGroupTransform& T_msg,
                lgmath::se3::TransformationWithCovariance& T);

}  // namespace common
}  // namespace vtr