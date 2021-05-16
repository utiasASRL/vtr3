#include <vtr_tactic/modules/lidar/keyframe_test_module.hpp>

namespace vtr {
namespace tactic {

void KeyframeTestModule::runImpl(QueryCache &qdata, MapCache &mdata,
                                 const Graph::ConstPtr &graph) {
  // default to
  qdata.keyframe_test_result.fallback(KeyframeTestResult::DO_NOTHING);

  // input
  auto &first_frame = *qdata.first_frame;
  auto &T_r_m = *mdata.T_r_m;
  // output
  auto &result = *qdata.keyframe_test_result;

  // check first frame
  if (first_frame)
    result = KeyframeTestResult::CREATE_VERTEX;

  // check travel distance
  auto se3vec = T_r_m.vec();
  auto translation_distance = se3vec.head<3>().norm();
  auto rotation_distance = se3vec.tail<3>().norm() * 57.29577; // 180/pi
  LOG(DEBUG) << "Total translation so far is: " << translation_distance;
  LOG(DEBUG) << "Total rotation so far is: " << rotation_distance;

  if (translation_distance >= config_->min_translation)
    result = KeyframeTestResult::CREATE_VERTEX;
  if (rotation_distance >= config_->min_rotation)
    result = KeyframeTestResult::CREATE_VERTEX;
}

}  // namespace tactic
}  // namespace vtr