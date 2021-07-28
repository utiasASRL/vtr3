#include <vtr_tactic/modules/lidar/keyframe_test_module.hpp>

namespace vtr {
namespace tactic {
namespace lidar {

void KeyframeTestModule::configFromROS(const rclcpp::Node::SharedPtr &node,
                                       const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->min_translation = node->declare_parameter<float>(param_prefix + ".min_translation", config_->min_translation);
  config_->min_rotation = node->declare_parameter<float>(param_prefix + ".min_rotation", config_->min_rotation);
  config_->max_translation = node->declare_parameter<float>(param_prefix + ".max_translation", config_->max_translation);
  config_->max_rotation = node->declare_parameter<float>(param_prefix + ".max_rotation", config_->max_rotation);
  config_->min_matched_points_ratio = node->declare_parameter<float>(param_prefix + ".min_matched_points_ratio", config_->min_matched_points_ratio);
  // clang-format on
}

void KeyframeTestModule::runImpl(QueryCache &qdata, MapCache &,
                                 const Graph::ConstPtr &) {
  // default to
  qdata.keyframe_test_result.fallback(KeyframeTestResult::DO_NOTHING);

  // input
  const auto &first_frame = *qdata.first_frame;
  const auto &T_r_m = *qdata.T_r_m_odo;
  const auto &success = *qdata.odo_success;
  // output
  auto &result = *qdata.keyframe_test_result;

  // check first frame
  if (first_frame) result = KeyframeTestResult::CREATE_VERTEX;

  if (!success) {
    result = KeyframeTestResult::FAILURE;
    return;
  }

  // check travel distance
  auto se3vec = T_r_m.vec();
  auto translation_distance = se3vec.head<3>().norm();
  auto rotation_distance = se3vec.tail<3>().norm() * 57.29577;  // 180/pi
  CLOG(DEBUG, "lidar.keyframe_test")
      << "Total translation: " << translation_distance
      << ", total rotation: " << rotation_distance;
  if (qdata.matched_points_ratio) {
    CLOG(DEBUG, "lidar.keyframe_test")
        << "Matched point ratio: " << *qdata.matched_points_ratio;
  }

  if (translation_distance >= config_->max_translation ||
      rotation_distance >= config_->max_rotation)
    result = KeyframeTestResult::CREATE_VERTEX;

  if (translation_distance <= config_->min_translation &&
      rotation_distance <= config_->min_rotation)
    return;

  // check matched points ratio
  if (qdata.matched_points_ratio &&
      *qdata.matched_points_ratio < config_->min_matched_points_ratio)
    result = KeyframeTestResult::CREATE_VERTEX;
}

}  // namespace lidar
}  // namespace tactic
}  // namespace vtr