#include <vtr_lgmath_extensions/conversions.hpp>
#include <vtr_logging/logging.hpp>

namespace vtr_messages {
namespace msg {

using CovarianceType = Eigen::Matrix<double, 6, 6>;

LgTransform& operator<<(LgTransform& tf, const lgmath::se3::Transformation& T) {
  auto tmp = T.vec();
  tf.xi = std::vector<double>(tmp.data(), tmp.data() + 6);

  tf.cov.clear();
  tf.cov_set = false;

  return tf;
}

LgTransform& operator<<(LgTransform& tf,
                        const lgmath::se3::TransformationWithCovariance& T) {
  auto tmp = T.vec();
  tf.xi = std::vector<double>(tmp.data(), tmp.data() + 6);

  if (T.covarianceSet()) {
    tf.cov = std::vector<double>(T.cov().data(), T.cov().data() + 36);
    tf.cov_set = true;
  } else {
    tf.cov.clear();
    tf.cov_set = false;
  }

  return tf;
}

LgTransform& operator>>(LgTransform& tf, lgmath::se3::Transformation& T) {
  if (tf.xi.size() != 6) {
    throw std::runtime_error(
        "Lie vector in ROS LgTransform did not have length 6!");
  }

  T = lgmath::se3::Transformation(Eigen::Matrix<double, 6, 1>(tf.xi.data()));
  return tf;
}

LgTransform& operator>>(LgTransform& tf,
                        lgmath::se3::TransformationWithCovariance& T) {
  if (tf.xi.size() != 6) {
    throw std::runtime_error(
        "Lie vector in ROS LgTransform did not have length 6!");
  }

  T = lgmath::se3::TransformationWithCovariance(
      Eigen::Matrix<double, 6, 1>(tf.xi.data()));

  if (tf.cov_set) {
    if (tf.cov.size() != 36) {
      LOG(ERROR) << "Covariance in ROS message did not have length 36; "
                    "proceeding with unset covariance.";
    } else {
      T.setCovariance(Eigen::Matrix<double, 6, 6>(tf.cov.data()));
    }
  }

  return tf;
}

std::ostream& operator<<(std::ostream& out, const LgTransform& tf) {
  out << Eigen::Matrix<double, 1, 6>(tf.xi.data());
  if (tf.cov_set) {
    out << std::endl;
    out << Eigen::Matrix<double, 6, 6>(tf.cov.data());
  }

  return out;
}
}  // namespace msg
}  // namespace vtr_messages

namespace geometry_msgs {
namespace msg {

Pose2D& operator<<(Pose2D& p, const lgmath::se3::Transformation& T) {
  // NOTE: this is a POSE, so we use the vector from a to b, expressed in a, as
  // well as the angle from a to b
  auto tmp = T.r_ba_ina();
  p.x = tmp(0);
  p.y = tmp(1);

  // This isn't quite equal to theta_z, but it's close enough for projection
  // purposes
  p.theta = T.vec()(5);

  return p;
}

}  // namespace msg
}  // namespace geometry_msgs