#include <vtr_common/rosutils/transformations.hpp>

namespace vtr {
namespace common {
namespace rosutils {

Eigen::Matrix4d fromPoseMessage(const geometry_msgs::msg::Pose &pose) {
  Eigen::Vector3d p_b_c_b(pose.position.x, pose.position.y, pose.position.z);
  tf2::Matrix3x3 C_bc(tf2::Quaternion(pose.orientation.x, pose.orientation.y,
                                      pose.orientation.z, pose.orientation.w));
  Eigen::Matrix4d T_b_c = Eigen::Matrix4d::Identity();
  T_b_c.topRightCorner<3, 1>() = p_b_c_b;

  T_b_c.topLeftCorner<3, 3>() << C_bc[0][0], C_bc[0][1], C_bc[0][2], C_bc[1][0],
      C_bc[1][1], C_bc[1][2], C_bc[2][0], C_bc[2][1], C_bc[2][2];

  return T_b_c;
}

Eigen::Matrix3d fromPoseMessage(const geometry_msgs::msg::Pose2D &pose) {
  Eigen::Vector2d p_b_c_b(pose.x, pose.y);
  Eigen::Matrix3d T_b_c = Eigen::Matrix3d::Identity();
  T_b_c.topRightCorner<2, 1>() = p_b_c_b;

  T_b_c.topLeftCorner<2, 2>() << cos(pose.theta), -sin(pose.theta),
      sin(pose.theta), cos(pose.theta);

  return T_b_c;
}

geometry_msgs::msg::Vector3 toVector3Message(const Eigen::Vector3d &v) {
  geometry_msgs::msg::Vector3 v3;
  v3.x = v[0];
  v3.y = v[1];
  v3.z = v[2];

  return v3;
}

geometry_msgs::msg::Point toPointMessage(const Eigen::Vector3d &v) {
  geometry_msgs::msg::Point v3;
  v3.x = v[0];
  v3.y = v[1];
  v3.z = v[2];

  return v3;
}

geometry_msgs::msg::Quaternion toQuaternionMessage(const Eigen::Vector4d &v) {
  geometry_msgs::msg::Quaternion q_msg;
  q_msg.x = v[0];
  q_msg.y = v[1];
  q_msg.z = v[2];
  q_msg.w = v[3];

  return q_msg;
}

geometry_msgs::msg::Quaternion toQuaternionMessage(
    const Eigen::Quaterniond &v) {
  geometry_msgs::msg::Quaternion q;
  q.x = v.x();
  q.y = v.y();
  q.z = v.z();
  q.w = v.w();

  return q;
}

geometry_msgs::msg::Pose toPoseMessage(const Eigen::Matrix4d &T_base_pose) {
  Eigen::Quaterniond q_bp(T_base_pose.topLeftCorner<3, 3>());
  Eigen::Vector3d p_b_p_b = T_base_pose.topRightCorner<3, 1>();

  geometry_msgs::msg::Pose ts;
  ts.position = toPointMessage(p_b_p_b);
  ts.orientation = toQuaternionMessage(q_bp);

  return ts;
}

geometry_msgs::msg::Pose toPoseMessage(
    const lgmath::se3::Transformation &T_base_pose) {
  return toPoseMessage(T_base_pose.matrix());
}

geometry_msgs::msg::Transform toTransformMessage(
    const Eigen::Matrix4d &T_base_child) {
  Eigen::Quaterniond q_bc(T_base_child.topLeftCorner<3, 3>());
  Eigen::Vector3d p_b_c_b = T_base_child.topRightCorner<3, 1>();

  geometry_msgs::msg::Transform ts;
  ts.translation = toVector3Message(p_b_c_b);
  ts.rotation = toQuaternionMessage(q_bc);

  return ts;
}

geometry_msgs::msg::Transform toTransformMessage(
    const lgmath::se3::Transformation &T_base_child) {
  return toTransformMessage(T_base_child.matrix());
}

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
