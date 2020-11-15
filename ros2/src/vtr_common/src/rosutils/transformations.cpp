#include <vtr_common/rosutils/transformations.hpp>

namespace vtr {
namespace common {
namespace rosutils {

Eigen::Matrix4d fromStampedTransformation(
    tf2::Stamped<tf2::Transform> const &t_base_child) {
  Eigen::Matrix4d T;
  T.setIdentity();
  tf2::Matrix3x3 C_bc(t_base_child.getRotation());
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col) T(row, col) = C_bc[row][col];
  T(0, 3) = t_base_child.getOrigin().x();
  T(1, 3) = t_base_child.getOrigin().y();
  T(2, 3) = t_base_child.getOrigin().z();
  /// LOG(DEBUG) << T << std::endl;
  return T;
}

}  // namespace rosutils

}  // namespace common
}  // namespace vtr
