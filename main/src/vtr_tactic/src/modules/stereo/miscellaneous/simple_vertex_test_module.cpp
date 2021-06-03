#include <vtr_tactic/modules/stereo/miscellaneous/simple_vertex_test_module.hpp>

namespace vtr {
namespace tactic {

void SimpleVertexTestModule::setConfig(std::shared_ptr<Config> &config) {
  // Set the base module
  auto down_casted_config =
      std::dynamic_pointer_cast<VertexCreationModule::Config>(config);
  VertexCreationModule::setConfig(down_casted_config);
  simple_config_ = config;
}

void SimpleVertexTestModule::runImpl(QueryCache &qdata, MapCache &mdata,
                                     const Graph::ConstPtr &) {
  // default to creating candidate
  *qdata.keyframe_test_result = KeyframeTestResult::CREATE_CANDIDATE;

  /// \todo yuchen check first frame, is this appropriate? what if we do not
  /// have enough features here?
  if (*qdata.first_frame) {
    LOG(DEBUG) << "First frame encountered, make it a keyframe.";
    *qdata.keyframe_test_result = KeyframeTestResult::CREATE_VERTEX;
    return;
  }

  int32_t inlier_count = 0;
  if (qdata.ransac_matches.is_valid() == true) {
    auto &matches = *qdata.ransac_matches;
    for (auto &rig : matches) {
      for (auto &channel : rig.channels) {
        inlier_count += channel.matches.size();
      }
    }
  } else {
    // if we don't have ransac data, this cache probably doesn't have images, do
    // nothing
    *qdata.keyframe_test_result = KeyframeTestResult::DO_NOTHING;
    return;
  }
#if false
  if (qdata.triangulated_matches.is_valid() == true) {
    auto &matches = *qdata.triangulated_matches;
    for (auto &rig : matches) {
      for (auto &channel : rig.channels) {
        inlier_count += channel.matches.size();
      }
    }
  }
#endif
  if (inlier_count < simple_config_->match_threshold_fail_count) {
    LOG(ERROR) << "Uh oh, " << inlier_count << " is not enough inliers";
    *qdata.keyframe_test_result = KeyframeTestResult::FAILURE;
    *qdata.success = false;
    return;
  }

  if (*qdata.steam_failure == true || *qdata.success == false) {
    LOG(ERROR) << "Uh oh, state estimation failed";
    *qdata.success = false;
    return;
  }

  if (qdata.T_r_m.is_valid() == true) {
    // Inputs, Query Frame, Map Frame, Inliers, Initial Guess
    const auto &T_query_map = *qdata.T_r_m;

    // extract the translational component of the distance
    auto se3Vec = T_query_map.vec();

    // The maximum component of the translation
    double translation_distance = se3Vec.head<3>().norm();

    // The maximum component of the rotation.
    double rotation_distance = se3Vec.tail<3>().norm() * 57.29577;

    // If we have not moved enough to create a vertex, then just return
    if (translation_distance < simple_config_->min_distance &&
        rotation_distance < .1) {
      return;
    }

    else if (translation_distance > simple_config_->max_creation_distance) {
      LOG(ERROR) << "Uh oh, we have a huge translation " << translation_distance
                 << " m";
      *qdata.keyframe_test_result = KeyframeTestResult::FAILURE;
      *qdata.success = false;
      return;
    } else if (rotation_distance > simple_config_->rotation_threshold_max) {
      LOG(ERROR) << "Uh oh, we have a huge rotation " << rotation_distance
                 << " deg";
      *qdata.keyframe_test_result = KeyframeTestResult::FAILURE;
      *qdata.success = false;
      return;
    }

    // check if to see if we have met any candidate creation criteria
    if (translation_distance > simple_config_->min_creation_distance) {
      *qdata.keyframe_test_result = KeyframeTestResult::CREATE_VERTEX;
      return;
    } else if (rotation_distance > simple_config_->rotation_threshold_min) {
      *qdata.keyframe_test_result = KeyframeTestResult::CREATE_VERTEX;
      return;
    } else if (inlier_count < simple_config_->match_threshold_min_count) {
      *qdata.keyframe_test_result = KeyframeTestResult::CREATE_VERTEX;
      return;
    }
  } else {
    LOG(ERROR) << "QVO did not estimate T_r_m";
    *qdata.keyframe_test_result = KeyframeTestResult::FAILURE;
    *qdata.success = false;
  }

  LOG(DEBUG) << "Simple vertex test result: "
             << static_cast<int>(*qdata.keyframe_test_result);
}

}  // namespace tactic
}  // namespace vtr
