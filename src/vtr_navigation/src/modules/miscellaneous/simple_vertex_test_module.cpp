#include <vtr/navigation/modules/miscellaneous/simple_vertex_test_module.h>

namespace vtr {
namespace navigation {

void SimpleVertexTestModule::setConfig(std::shared_ptr<Config> &config) {
  // Set the base module
  auto down_casted_config =
      std::dynamic_pointer_cast<VertexCreationModule::Config>(config);
  VertexCreationModule::setConfig(down_casted_config);
  simple_config_ = config;
}

void SimpleVertexTestModule::run(QueryCache &qdata, MapCache &mdata,
                                 const std::shared_ptr<const Graph> &) {
  // default to creating candidate
  qdata.new_vertex_flag = CREATE_CANDIDATE;

  int32_t inlier_count = 0;
  if (mdata.ransac_matches.is_valid() == true) {
    auto &matches = *mdata.ransac_matches;
    for (auto &rig : matches) {
      for (auto &channel : rig.channels) {
        inlier_count += channel.matches.size();
      }
    }
  } else {
    // if we don't have ransac data, this cache probably doesn't have images, do
    // nothing
    qdata.new_vertex_flag = DO_NOTHING;
    return;
  }
  if (mdata.triangulated_matches.is_valid() == true) {
    auto &matches = *mdata.triangulated_matches;
    for (auto &rig : matches) {
      for (auto &channel : rig.channels) {
        inlier_count += channel.matches.size();
      }
    }
  }
  if (inlier_count < simple_config_->match_threshold_fail_count) {
    LOG(ERROR) << "Uh oh, " << inlier_count << " is not enough inliers";
    qdata.new_vertex_flag = FAILURE;
    mdata.success = false;
    return;
  }

  if (*mdata.steam_failure.fallback(true) == true || *mdata.success == false) {
    LOG(ERROR) << "Uh oh, state estimation failed";
    mdata.success = false;
    return;
  }

  if (mdata.T_q_m.is_valid() == true) {
    // Inputs, Query Frame, Map Frame, Inliers, Initial Guess
    const auto &T_query_map = *mdata.T_q_m;

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
      qdata.new_vertex_flag = FAILURE;
      mdata.success = false;
      return;
    } else if (rotation_distance > simple_config_->rotation_threshold_max) {
      LOG(ERROR) << "Uh oh, we have a huge rotation " << rotation_distance
                 << " deg";
      qdata.new_vertex_flag = FAILURE;
      mdata.success = false;
      return;
    }

    // check if to see if we have met any candidate creation criteria
    if (translation_distance > simple_config_->min_creation_distance) {
      qdata.new_vertex_flag = CREATE_VERTEX;
      return;
    } else if (rotation_distance > simple_config_->rotation_threshold_min) {
      qdata.new_vertex_flag = CREATE_VERTEX;
      return;
    } else if (inlier_count < simple_config_->match_threshold_min_count) {
      qdata.new_vertex_flag = CREATE_VERTEX;
      return;
    }
  } else {
    LOG(ERROR) << "QVO did not estimate T_q_m";
    qdata.new_vertex_flag = FAILURE;
    mdata.success = false;
  }
}

void SimpleVertexTestModule::updateGraph(QueryCache &, MapCache &,
                                         const std::shared_ptr<Graph> &,
                                         VertexId) {}

}  // namespace navigation
}  // namespace asrl
