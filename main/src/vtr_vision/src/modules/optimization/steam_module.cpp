// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file steam_module.cpp
 * \brief SteamModule class method definitions
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_vision/modules/optimization/steam_module.hpp>

namespace vtr {
namespace vision {

using namespace tactic;

namespace {
bool checkDiagonal(Eigen::Matrix<double, 6, 1> &diag) {
  for (int idx = 0; idx < 6; ++idx) {
    if (diag(idx, 0) <= 0) return false;
  }
  return true;
}
bool checkDiagonal(Eigen::Array<double, 1, 6> &diag) {
  for (int idx = 0; idx < 6; ++idx) {
    if (diag(idx) <= 0) return false;
  }
  return true;
}
}  // namespace

auto SteamModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                const std::string &param_prefix) -> ConstPtr {
  auto config = std::make_shared<SteamModule::Config>();
  // clang-format off
  config->solver_type = node->declare_parameter<std::string>(param_prefix + ".solver_type", config->solver_type);
  config->loss_function = node->declare_parameter<std::string>(param_prefix + ".loss_function", config->loss_function);
  config->verbose = node->declare_parameter<bool>(param_prefix + ".verbose", config->verbose);
  config->use_T_q_m_prior = node->declare_parameter<bool>(param_prefix + ".use_T_q_m_prior", config->use_T_q_m_prior);

  config->iterations = node->declare_parameter<int>(param_prefix + ".iterations", config->iterations);
  config->absoluteCostThreshold = node->declare_parameter<double>(param_prefix + ".absoluteCostThreshold", config->absoluteCostThreshold);
  config->absoluteCostChangeThreshold = node->declare_parameter<double>(param_prefix + ".absoluteCostChangeThreshold", config->absoluteCostChangeThreshold);
  config->relativeCostChangeThreshold = node->declare_parameter<double>(param_prefix + ".relativeCostChangeThreshold", config->relativeCostChangeThreshold);

  config->ratioThresholdShrink = node->declare_parameter<double>(param_prefix + ".ratioThresholdShrink", config->ratioThresholdShrink);
  config->ratioThresholdGrow = node->declare_parameter<double>(param_prefix + ".ratioThresholdGrow", config->ratioThresholdGrow);
  config->shrinkCoeff = node->declare_parameter<double>(param_prefix + ".shrinkCoeff", config->shrinkCoeff);
  config->growCoeff = node->declare_parameter<double>(param_prefix + ".growCoeff", config->growCoeff);
  config->maxShrinkSteps = node->declare_parameter<int>(param_prefix + ".maxShrinkSteps", config->maxShrinkSteps);
  config->backtrackMultiplier = node->declare_parameter<double>(param_prefix + ".backtrackMultiplier", config->backtrackMultiplier);
  config->maxBacktrackSteps = node->declare_parameter<int>(param_prefix + ".maxBacktrackSteps", config->maxBacktrackSteps);

  // validity checking
  config->perform_planarity_check = node->declare_parameter<bool>(param_prefix + ".perform_planarity_check", config->perform_planarity_check);
  config->plane_distance = node->declare_parameter<double>(param_prefix + ".plane_distance", config->plane_distance);
  config->min_point_depth = node->declare_parameter<double>(param_prefix + ".min_point_depth", config->min_point_depth);
  config->max_point_depth = node->declare_parameter<double>(param_prefix + ".max_point_depth", config->max_point_depth);

  // trajectory stuff.
  config->save_trajectory = node->declare_parameter<bool>(param_prefix + ".save_trajectory", config->save_trajectory);
  config->trajectory_smoothing = node->declare_parameter<bool>(param_prefix + ".trajectory_smoothing", config->trajectory_smoothing);
  config->lin_acc_std_dev_x = node->declare_parameter<double>(param_prefix + ".lin_acc_std_dev_x", config->lin_acc_std_dev_x);
  config->lin_acc_std_dev_y = node->declare_parameter<double>(param_prefix + ".lin_acc_std_dev_y", config->lin_acc_std_dev_y);
  config->lin_acc_std_dev_z = node->declare_parameter<double>(param_prefix + ".lin_acc_std_dev_z", config->lin_acc_std_dev_z);
  config->ang_acc_std_dev_x = node->declare_parameter<double>(param_prefix + ".ang_acc_std_dev_x", config->ang_acc_std_dev_x);
  config->ang_acc_std_dev_y = node->declare_parameter<double>(param_prefix + ".ang_acc_std_dev_y", config->ang_acc_std_dev_y);
  config->ang_acc_std_dev_z = node->declare_parameter<double>(param_prefix + ".ang_acc_std_dev_z", config->ang_acc_std_dev_z);
  config->disable_solver = node->declare_parameter<bool>(param_prefix + ".disable_solver", config->disable_solver);
  // velocity prior
  config->velocity_prior = node->declare_parameter<bool>(param_prefix + ".velocity_prior", config->velocity_prior);
  config->lin_vel_mean_x = node->declare_parameter<double>(param_prefix + ".lin_vel_mean_x", config->lin_vel_mean_x);
  config->lin_vel_mean_y = node->declare_parameter<double>(param_prefix + ".lin_vel_mean_y", config->lin_vel_mean_y);
  config->lin_vel_mean_z = node->declare_parameter<double>(param_prefix + ".lin_vel_mean_z", config->lin_vel_mean_z);
  config->ang_vel_mean_x = node->declare_parameter<double>(param_prefix + ".ang_vel_mean_x", config->ang_vel_mean_x);
  config->ang_vel_mean_y = node->declare_parameter<double>(param_prefix + ".ang_vel_mean_y", config->ang_vel_mean_y);
  config->ang_vel_mean_z = node->declare_parameter<double>(param_prefix + ".ang_vel_mean_z", config->ang_vel_mean_z);

  config->lin_vel_std_dev_x = node->declare_parameter<double>(param_prefix + ".lin_vel_std_dev_x", config->lin_vel_std_dev_x);
  config->lin_vel_std_dev_y = node->declare_parameter<double>(param_prefix + ".lin_vel_std_dev_y", config->lin_vel_std_dev_y);
  config->lin_vel_std_dev_z = node->declare_parameter<double>(param_prefix + ".lin_vel_std_dev_z", config->lin_vel_std_dev_z);
  config->ang_vel_std_dev_x = node->declare_parameter<double>(param_prefix + ".ang_vel_std_dev_x", config->ang_vel_std_dev_x);
  config->ang_vel_std_dev_y = node->declare_parameter<double>(param_prefix + ".ang_vel_std_dev_y", config->ang_vel_std_dev_y);
  config->ang_vel_std_dev_z = node->declare_parameter<double>(param_prefix + ".ang_vel_std_dev_z", config->ang_vel_std_dev_z);
  // clang-format on

  return config;
}

void SteamModule::setConfig() {
  backup_params_.verbose = config_->verbose;
  backup_params_.max_iterations = config_->iterations;
  backup_params_.absolute_cost_threshold = config_->absoluteCostThreshold;
  backup_params_.absolute_cost_change_threshold =
      config_->absoluteCostChangeThreshold;
  backup_params_.relative_cost_change_threshold =
      config_->relativeCostChangeThreshold;

  backup_params_.shrink_coeff = config_->shrinkCoeff;
  backup_params_.grow_coeff = config_->growCoeff;
  backup_params_.max_shrink_steps = config_->maxShrinkSteps;

  // Eigen::Matrix<double, 1, 6> Qc_diag;
  Eigen::Matrix<double, 6, 1> Qc_diag;
  Qc_diag << config_->lin_acc_std_dev_x, config_->lin_acc_std_dev_y,
      config_->lin_acc_std_dev_z, config_->ang_acc_std_dev_x,
      config_->ang_acc_std_dev_y, config_->ang_acc_std_dev_z;
  if (checkDiagonal(Qc_diag) == false && config_->trajectory_smoothing) {
    throw std::runtime_error(
        "Elements of the smoothing factor must be greater than zero!");
  }
  // Make Qc_inv
  // smoothing_factor_information_.setZero();
  // smoothing_factor_information_.diagonal() = 1.0 / Qc_diag;
  // smoothing_factor_information_.setZero();
  smoothing_factor_information_ = Qc_diag;

  // Setup velocity prior
  velocity_prior_ << config_->lin_vel_mean_x, config_->lin_vel_mean_y,
      config_->lin_vel_mean_z, config_->ang_vel_mean_x, config_->ang_vel_mean_y,
      config_->ang_vel_mean_z;

  Eigen::Array<double, 1, 6> Qv_diag;
  // Eigen::Matrix<double, 6, 1> Qv_diag;
  Qv_diag << config_->lin_vel_std_dev_x, config_->lin_vel_std_dev_y,
      config_->lin_vel_std_dev_z, config_->ang_vel_std_dev_x,
      config_->ang_vel_std_dev_y, config_->ang_vel_std_dev_z;

  if (checkDiagonal(Qv_diag) == false && config_->trajectory_smoothing) {
    throw std::runtime_error(
        "Error: elements of the velocity prior noise must be greater than "
        "zero!");
  }
  // velocity_prior_cov_.setZero();
  // velocity_prior_cov_ = Qv_diag;
  velocity_prior_cov_.setZero();
  velocity_prior_cov_.diagonal() = 1.0 / Qv_diag;
}

std::shared_ptr<steam::SolverBase> SteamModule::generateSolver(
    steam::OptimizationProblem problem) {

  // Setup Solver
  std::shared_ptr<steam::SolverBase> solver;
  if (config_->solver_type == "LevenburgMarquardt") {
    steam::LevMarqGaussNewtonSolver::Params params;
    params.verbose = config_->verbose;
    params.max_iterations = config_->iterations;
    params.absolute_cost_threshold = config_->absoluteCostThreshold;
    params.absolute_cost_change_threshold = config_->absoluteCostChangeThreshold;
    params.relative_cost_change_threshold = config_->relativeCostChangeThreshold;

    params.shrink_coeff = config_->shrinkCoeff;
    params.grow_coeff = config_->growCoeff;
    params.max_shrink_steps = config_->maxShrinkSteps;



    solver.reset(new steam::LevMarqGaussNewtonSolver(problem, params));
        CLOG(DEBUG, "stereo.keyframe_optimization") << "Made LMQ";

  } else if (config_->solver_type == "DoglegGaussNewton") {
    steam::DoglegGaussNewtonSolver::Params params;
    params.verbose = config_->verbose;
    params.max_iterations = config_->iterations;
    params.absolute_cost_threshold = config_->absoluteCostThreshold;
    params.absolute_cost_change_threshold = config_->absoluteCostChangeThreshold;
    params.relative_cost_change_threshold = config_->relativeCostChangeThreshold;

    params.ratio_threshold_shrink = config_->ratioThresholdShrink;
    params.ratio_threshold_grow = config_->ratioThresholdGrow;
    params.shrink_coeff = config_->shrinkCoeff;
    params.grow_coeff = config_->growCoeff;
    params.max_shrink_steps = config_->maxShrinkSteps;
    CLOG(DEBUG, "stereo.keyframe_optimization") << "Constructing Dogleg";
    solver = std::make_shared<steam::DoglegGaussNewtonSolver>(problem, params);
    CLOG(DEBUG, "stereo.keyframe_optimization") << "Made Dogleg";

  } else if (config_->solver_type == "VanillaGaussNewton") {
    steam::GaussNewtonSolver::Params params;
    params.verbose = config_->verbose;
    params.max_iterations = config_->iterations;
    params.absolute_cost_threshold = config_->absoluteCostThreshold;
    params.absolute_cost_change_threshold = config_->absoluteCostChangeThreshold;
    params.relative_cost_change_threshold = config_->relativeCostChangeThreshold;
    solver.reset(new steam::GaussNewtonSolver(problem, params));
  } else {
    CLOG(ERROR, "stereo.optimization") << "Unknown solver type: " << config_->solver_type;
  }
  return solver;
}

bool SteamModule::forceLM(steam::OptimizationProblem &problem) {
  try {
    backup_lm_solver_.reset(new steam::LevMarqGaussNewtonSolver(problem, backup_params_));
    backup_lm_solver_->optimize();
  } catch (std::runtime_error &re) {
    CLOG(ERROR, "stereo.optimization") << "Back up LM failed, abandon hope....";
    return false;
  }
  backup_lm_solver_used_ = true;
  return true;
}

void SteamModule::run_(tactic::QueryCache &qdata0, tactic::OutputCache &output, const tactic::Graph::Ptr &graph,
                const std::shared_ptr<tactic::TaskExecutor> &executor) {
  auto &qdata = dynamic_cast<CameraQueryCache &>(qdata0);

  // *qdata.steam_failure = false;
  backup_lm_solver_used_ = false;

  // basic sanity check
  if (!qdata.rig_features.valid() ||
      (qdata.success.valid() && *qdata.success == false)) {
      CLOG(WARNING, "stereo.optimization") << "No features";
    return;
  }

  /// \todo yuchen find a better place for this, or the following transformation
  /// code.
  if (!verifyInputData(qdata)) return;

  // Construct a transform evaluator that takes points from the vehicle frame
  // into the sensor frame.
  if (qdata.T_s_r.valid()) {
    tf_sensor_vehicle_ = steam::se3::SE3StateVar::MakeShared(*qdata.T_s_r);
  } else {
    tf_sensor_vehicle_ = steam::se3::SE3StateVar::MakeShared(lgmath::se3::Transformation());
  }
  tf_sensor_vehicle_->locked() = true;

  for (auto it = qdata.T_sensor_vehicle_map->begin();
       it != qdata.T_sensor_vehicle_map->end(); ++it) {
    tf_sensor_vehicle_map_[it->first] =
        steam::se3::SE3StateVar::MakeShared(it->second);
    tf_sensor_vehicle_map_[it->first]->locked() = true;
  }

  tf_identity_ = steam::se3::SE3StateVar::MakeShared(
      lgmath::se3::Transformation());
  tf_identity_->locked() = true;

  
  try {
    // PROBLEM SPECIFIC
    steam::OptimizationProblem problem = generateOptimizationProblem(qdata, graph);
    CLOG(DEBUG, "stereo.keyframe_optimization") << "Generated optimization problem";

    CLOG(DEBUG, "stereo.keyframe_optimization") << "Problem cost terms" << problem.getNumberOfCostTerms();

    solver_ = generateSolver(problem);
    CLOG(DEBUG, "stereo.keyframe_optimization") << "Generated solver";


    // default to success
    bool success = true;

    // attempt to run the solver
    try {
      if (!config_->disable_solver) solver_->optimize();
    } catch (std::logic_error &e) {
      LOG(ERROR) << "Forced Gradient-Descent, running in LM..." << e.what();
      success = forceLM(problem);
      // *qdata.steam_failure = !success;
      *qdata.success = success;
    } catch (steam::unsuccessful_step &e) {
      // did any successful steps occur?
      if (solver_->curr_iteration() <= 1) {
        // no: something is very wrong
        CLOG(ERROR, "stereo.optimization")
            << "Steam has failed to optimise the problem. This is an error";
        success = false;
        // *qdata.steam_failure = !success;
        *qdata.success = success;
      } else {
        // yes: just a marginal problem, let's use what we got
        CLOG(WARNING, "stereo.optimization") << "Steam has failed due to an unsuccessful step. This "
                        "should be OK if it happens infrequently.";
      }
    } catch (std::runtime_error &e) {
      CLOG(ERROR, "stereo.optimization") << "Steam has failed with an unusual error: " << e.what();
      success = false;
      // *qdata.steam_failure = !success;
      *qdata.success = success;
    }

    success = success && verifyOutputData(qdata);
    if (success) updateCaches(qdata);

  } catch (...) {
    // *qdata.steam_failure = true;
    LOG(ERROR) << " bailing on steam problem!";
    *qdata.success = false;
  }

  if (config_->use_T_q_m_prior && qdata.T_r_m_prior.valid() &&
      (*qdata.T_r_m_prior).covarianceSet() && (*qdata.T_r_m).covarianceSet()) {
    double prior_ct_sigma = sqrt((*qdata.T_r_m_prior).cov()(1, 1));
    double ct_sigma = sqrt((*qdata.T_r_m).cov()(1, 1));
    if (ct_sigma > prior_ct_sigma) {
      LOG(WARNING) << "Loc. added uncertainty, bailing.";
      *qdata.success = false;
      // *qdata.steam_failure = true;
    }
  }
}


StereoCalibPtr SteamModule::toStereoSteamCalibration(
    const vision::RigCalibration &calibration) {
  StereoCalibPtr sharedStereoIntrinsics(new steam::stereo::CameraIntrinsics);
  sharedStereoIntrinsics->b = -calibration.extrinsics[1].matrix()(0, 3);
  sharedStereoIntrinsics->fu = calibration.intrinsics[0](0, 0);
  sharedStereoIntrinsics->fv = calibration.intrinsics[0](1, 1);
  sharedStereoIntrinsics->cu = calibration.intrinsics[0](0, 2);
  sharedStereoIntrinsics->cv = calibration.intrinsics[0](1, 2);
  return sharedStereoIntrinsics;
}

}  // namespace vision
}  // namespace vtr
