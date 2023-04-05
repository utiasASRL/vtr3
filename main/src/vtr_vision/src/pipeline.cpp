#include <vtr_vision/pipeline.hpp>
#include "vtr_tactic/modules/factory.hpp"


namespace vtr {
namespace vision {

using namespace tactic;
auto StereoPipeline::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                   const std::string &param_prefix) 
    -> ConstPtr {
  auto config = std::make_shared<Config>();
                                   
  // clang-format off
  config->preprocessing = node->declare_parameter<std::vector<std::string>>(param_prefix + ".preprocessing", config->preprocessing);
  config->odometry = node->declare_parameter<std::vector<std::string>>(param_prefix + ".odometry", config->odometry);
  config->bundle_adjustment = node->declare_parameter<std::vector<std::string>>(param_prefix + ".bundle_adjustment", config->bundle_adjustment);
  config->localization = node->declare_parameter<std::vector<std::string>>(param_prefix + ".localization", config->localization);
  // clang-format on
  return config;
}


StereoPipeline::StereoPipeline(
    const Config::ConstPtr &config,
    const std::shared_ptr<ModuleFactory> &module_factory,
    const std::string &name)
    : BasePipeline(module_factory, name), config_(config) {
    // preprocessing
    
  for (auto module : config_->preprocessing)
    preprocessing_.push_back(factory()->get("preprocessing." + module));

//   // odometry
  for (auto module : config_->odometry)
    odometry_.push_back(factory()->get("odometry." + module));
//   // localization
//   for (auto module : config_->localization)
//     localization_.push_back(factory()->get("localization." + module));
}

StereoPipeline::~StereoPipeline() {}

tactic::OutputCache::Ptr StereoPipeline::createOutputCache() const {
  return std::make_shared<tactic::OutputCache>();
}


void StereoPipeline::preprocess_(const tactic::QueryCache::Ptr &qdata0, const tactic::OutputCache::Ptr &output0,
                   const tactic::Graph::Ptr &graph,
                   const std::shared_ptr<tactic::TaskExecutor> &executor) {
  auto qdata = std::dynamic_pointer_cast<CameraQueryCache>(qdata0);
  for (auto module : preprocessing_) module->run(*qdata0, *output0, graph, executor);
}

void StereoPipeline::runOdometry_(const tactic::QueryCache::Ptr &qdata0, const tactic::OutputCache::Ptr &output0,
                   const tactic::Graph::Ptr &graph,
                   const std::shared_ptr<tactic::TaskExecutor> &executor) {
  auto qdata = std::dynamic_pointer_cast<CameraQueryCache>(qdata0);
  
  if (!*qdata->first_frame)
    setOdometryPrior(qdata, graph);

  for (auto module : odometry_) module->run(*qdata0, *output0, graph, executor);
}

void StereoPipeline::setOdometryPrior(CameraQueryCache::Ptr &qdata,
                                      const tactic::Graph::Ptr &graph) {

  auto T_r_m_est = estimateTransformFromKeyframe(*qdata->timestamp_odo, *qdata->stamp,
                                                 qdata->rig_images.valid());

  *qdata->T_r_m_prior = T_r_m_est;
}

tactic::EdgeTransform StereoPipeline::estimateTransformFromKeyframe(
    const tactic::Timestamp &kf_stamp, const tactic::Timestamp &curr_stamp,
    bool check_expiry) {
  tactic::EdgeTransform T_q_m;
  // The elapsed time since the last keyframe
  // auto curr_time_point = common::timing::toChrono(curr_stamp);
  // auto dt_duration = curr_time_point - common::timing::toChrono(kf_stamp);
  // double dt = std::chrono::duration<double>(dt_duration).count();

  double dt = (curr_stamp - kf_stamp) / 1e9; //convert to seconds

  // Make sure the trajectory is current
  if (check_expiry && trajectory_) {
    //auto traj_dt_duration = curr_time_point - trajectory_time_point_;
    //double traj_dt = std::chrono::duration<double>(traj_dt_duration).count();
    if (dt > 1.0 /* tactic->config().extrapolate_timeout */) {
      CLOG(WARNING, "stereo.pipeline")
          << "The trajectory expired after " << dt
          << " s for estimating the transform from keyframe at "
          << kf_stamp;
      trajectory_.reset();
    }
  }

  // we need to update the new T_q_m prediction
  Eigen::Matrix<double, 6, 6> cov =
      Eigen::Matrix<double, 6, 6>::Identity() * pow(dt, 2.0);
  // scale the rotational uncertainty to be one order of magnitude lower than
  // the translational uncertainty.
  cov.block(3, 3, 3, 3) /= 10;
  if (trajectory_ != nullptr) {
    // Query the saved trajectory estimator we have with the candidate frame
    // time
    auto candidate_time =
        steam::traj::Time(static_cast<int64_t>(kf_stamp));
    auto candidate_eval = trajectory_->getPoseInterpolator(candidate_time);
    // Query the saved trajectory estimator we have with the current frame time
    auto query_time =
        steam::traj::Time(static_cast<int64_t>(curr_stamp));
    auto curr_eval = trajectory_->getPoseInterpolator(query_time);

    // find the transform between the candidate and current in the vehicle frame
    T_q_m = candidate_eval->evaluate().inverse() * curr_eval->evaluate();
    // give it back to the caller, TODO: (old) We need to get the covariance out
    // of the trajectory.

    // This ugliness of setting the time is because we don't have a reliable and
    // tested way of predicting the covariance. This is used by the stereo
    // matcher to decide how tight it should set its pixel search
    T_q_m.setCovariance(cov);

    CLOG(DEBUG, "stereo.pipeline")
        << "Estimated T_q_m (based on keyframe) from steam trajectory.";
  } else {
    // since we don't have a trajectory, we can't accurately estimate T_q_m
    T_q_m.setCovariance(4 * cov);

    CLOG(DEBUG, "stereo.pipeline")
        << "Estimated T_q_m is identity with high covariance.";
  }
  return T_q_m;
}





} //vision
} //vtr