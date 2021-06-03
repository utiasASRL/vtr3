#include <vtr_tactic/modules/lidar/icp_module.hpp>

namespace vtr {
namespace tactic {

void ICPModule::runImpl(QueryCache &qdata, MapCache &mdata,
                        const Graph::ConstPtr &graph) {
  if (config_->source == "live" && !qdata.current_map_odo) {
    LOG(INFO) << "First keyframe, simply return.";
    return;
  }

  // Get input and output data
  // input
  auto &sub_pts = *qdata.preprocessed_pointcloud;
  auto &icp_scores = *qdata.icp_scores;
  auto &T_s_r = *qdata.T_s_r;
  /// \todo (yuchen) check source in live and map
  auto &map = config_->source == "live" ? *qdata.current_map_odo
                                        : *qdata.current_map_loc;
  // output
  auto &T_r_m = config_->source == "live" ? *qdata.T_r_m_odo : *qdata.T_r_m_loc;

  // Create result containers
  ICP_results icp_results;
  auto T_m_s = T_r_m.inverse() * T_s_r.inverse();
  config_->init_transform = T_m_s.matrix();
  PointToMapICP(sub_pts, icp_scores, map, *config_, icp_results);

  T_r_m = lgmath::se3::TransformationWithCovariance(
      Eigen::Matrix4d((icp_results.transform * T_s_r.matrix()).inverse()));
  /// \todo need to output covariance from icp.
  T_r_m.setZeroCovariance();
}
}  // namespace tactic
}  // namespace vtr