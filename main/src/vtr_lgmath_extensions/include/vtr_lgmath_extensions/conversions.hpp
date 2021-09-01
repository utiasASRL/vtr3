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
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <geometry_msgs/msg/pose2_d.hpp>

#include <lgmath.hpp>

#include <vtr_logging/logging.hpp>
#include <vtr_messages/msg/lg_transform.hpp>

/** \brief Converting LGMath object to/from ROS LgTransform messages. */
namespace vtr_messages {
namespace msg {

using CovarianceType = Eigen::Matrix<double, 6, 6>;

/** \brief Load a Transformation into an LgTransform message */
LgTransform& operator<<(LgTransform& tf, const lgmath::se3::Transformation& T);

/** \brief Load a TransformationWithCovariance into an LgTransform message */
LgTransform& operator<<(LgTransform& tf,
                        const lgmath::se3::TransformationWithCovariance& T);

/** \brief Unpack a Transformation from an LgTransform message */
LgTransform& operator>>(LgTransform& tf, lgmath::se3::Transformation& T);

/** \brief Unpack a TransformationWithCovariance from an LgTransform message */
LgTransform& operator>>(LgTransform& tf,
                        lgmath::se3::TransformationWithCovariance& T);

/** \brief Print an LgTransform message */
std::ostream& operator<<(std::ostream& out, const LgTransform& tf);

}  // namespace msg
}  // namespace vtr_messages

namespace geometry_msgs {
namespace msg {

/** \brief Converting LGMath messages into "projected" 2D poses */
Pose2D& operator<<(Pose2D& p, const lgmath::se3::Transformation& T);

}  // namespace msg
}  // namespace geometry_msgs