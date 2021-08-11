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
  config_->max_num_frames = node->declare_parameter<int>(param_prefix + ".max_num_frames", config_->max_num_frames);
  // clang-format on
}

void KeyframeTestModule::runImpl(QueryCache &qdata, const Graph::ConstPtr &) {
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

  // check if we successfully register this frame
  if (!success) {
    result = KeyframeTestResult::FAILURE;
    return;
  }

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
  if (qdata.new_map) {
    CLOG(DEBUG, "lidar.keyframe_test")
        << "Number of frames since last keyframe: "
        << qdata.new_map->number_of_updates;
  }

  if (translation_distance >= config_->max_translation ||
      rotation_distance >= config_->max_rotation) {
    result = KeyframeTestResult::CREATE_VERTEX;
  } else if (translation_distance <= config_->min_translation &&
             rotation_distance <= config_->min_rotation) {
    // check number of frames
    /// \todo also check map points maybe
    if (qdata.new_map &&
        qdata.new_map->number_of_updates > config_->max_num_frames)
      result = KeyframeTestResult::CREATE_VERTEX;
  } else {
    // check matched points ratio
    if (qdata.matched_points_ratio &&
        *qdata.matched_points_ratio < config_->min_matched_points_ratio)
      result = KeyframeTestResult::CREATE_VERTEX;
  }
}

}  // namespace lidar
}  // namespace tactic
}  // namespace vtr