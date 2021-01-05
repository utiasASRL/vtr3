#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <lgmath/se3/Transformation.hpp>
#include <lgmath/se3/TransformationWithCovariance.hpp>
#include <lgmath/so3/Rotation.hpp>

namespace vtr {
namespace common {
namespace rosutils {

Eigen::Matrix4d fromStampedTransformation(
    tf2::Stamped<tf2::Transform> const &t_base_child);

}

}  // namespace common
}  // namespace vtr
