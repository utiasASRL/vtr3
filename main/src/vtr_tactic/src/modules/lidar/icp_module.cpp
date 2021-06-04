#include <vtr_tactic/modules/lidar/icp_module.hpp>

namespace vtr {
namespace tactic {
namespace lidar {

void ICPModule::configFromROS(const rclcpp::Node::SharedPtr &node,
                              const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->source = node->declare_parameter<std::string>(param_prefix + ".source", config_->source);
  // icp params
  config_->n_samples = node->declare_parameter<int>(param_prefix + ".n_samples", config_->n_samples);
  config_->max_pairing_dist = node->declare_parameter<float>(param_prefix + ".max_pairing_dist", config_->max_pairing_dist);
  config_->max_planar_dist = node->declare_parameter<float>(param_prefix + ".max_planar_dist", config_->max_planar_dist);
  config_->max_iter = node->declare_parameter<int>(param_prefix + ".max_iter", config_->max_iter);
  config_->avg_steps = node->declare_parameter<int>(param_prefix + ".avg_steps", config_->avg_steps);
  config_->rotDiffThresh = node->declare_parameter<float>(param_prefix + ".rotDiffThresh", config_->rotDiffThresh);
  config_->transDiffThresh = node->declare_parameter<float>(param_prefix + ".transDiffThresh", config_->transDiffThresh);
  // clang-format on
}

void ICPModule::runImpl(QueryCache &qdata, MapCache &,
                        const Graph::ConstPtr &) {
  if (config_->source == "live" && !qdata.current_map_odo) {
    LOG(INFO) << "First keyframe, simply return.";
    return;
  }

  // Get input data
  auto &points = *qdata.preprocessed_pointcloud;
  auto &icp_scores = *qdata.icp_scores;
  auto &T_s_r = *qdata.T_s_r;
  /// check source in live and map
  auto &map = config_->source == "live" ? *qdata.current_map_odo
                                        : *qdata.current_map_loc;
  // Output
  auto &T_r_m = config_->source == "live" ? *qdata.T_r_m_odo : *qdata.T_r_m_loc;

  // Create result containers
  ICP_results icp_results;
  auto T_m_s = T_r_m.inverse() * T_s_r.inverse();
  config_->init_transform = T_m_s.matrix();
  PointToMapICP(points, icp_scores, map, *config_, icp_results);

  T_r_m = lgmath::se3::TransformationWithCovariance(
      Eigen::Matrix4d((icp_results.transform * T_s_r.matrix()).inverse()));

  /// \todo Compute the covariance from icp.
  T_r_m.setZeroCovariance();
}

}  // namespace lidar
}  // namespace tactic
}  // namespace vtr