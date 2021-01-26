#pragma once

#include <geometry_msgs/msg/pose2_d.hpp>

#include <lgmath.hpp>
#include <vtr_messages/msg/lg_transform.hpp>

/// Converting LGMath object to/from ROS LgTransform messages
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