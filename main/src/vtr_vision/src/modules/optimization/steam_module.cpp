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
bool checkDiagonal(Eigen::Array<double, 1, 6> &diag) {
  for (int idx = 0; idx < 6; ++idx) {
    if (diag(idx) <= 0) return false;
  }
  return true;
}
}  // namespace

void SteamModule::configFromROS(const rclcpp::Node::SharedPtr &node,
                                const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->solver_type = node->declare_parameter<std::string>(param_prefix + ".solver_type", config_->solver_type);
  config_->loss_function = node->declare_parameter<std::string>(param_prefix + ".loss_function", config_->loss_function);
  config_->verbose = node->declare_parameter<bool>(param_prefix + ".verbose", config_->verbose);
  config_->use_T_q_m_prior = node->declare_parameter<bool>(param_prefix + ".use_T_q_m_prior", config_->use_T_q_m_prior);

  config_->iterations = node->declare_parameter<int>(param_prefix + ".iterations", config_->iterations);
  config_->absoluteCostThreshold = node->declare_parameter<double>(param_prefix + ".absoluteCostThreshold", config_->absoluteCostThreshold);
  config_->absoluteCostChangeThreshold = node->declare_parameter<double>(param_prefix + ".absoluteCostChangeThreshold", config_->absoluteCostChangeThreshold);
  config_->relativeCostChangeThreshold = node->declare_parameter<double>(param_prefix + ".relativeCostChangeThreshold", config_->relativeCostChangeThreshold);

  config_->ratioThresholdShrink = node->declare_parameter<double>(param_prefix + ".ratioThresholdShrink", config_->ratioThresholdShrink);
  config_->ratioThresholdGrow = node->declare_parameter<double>(param_prefix + ".ratioThresholdGrow", config_->ratioThresholdGrow);
  config_->shrinkCoeff = node->declare_parameter<double>(param_prefix + ".shrinkCoeff", config_->shrinkCoeff);
  config_->growCoeff = node->declare_parameter<double>(param_prefix + ".growCoeff", config_->growCoeff);
  config_->maxShrinkSteps = node->declare_parameter<int>(param_prefix + ".maxShrinkSteps", config_->maxShrinkSteps);
  config_->backtrackMultiplier = node->declare_parameter<double>(param_prefix + ".backtrackMultiplier", config_->backtrackMultiplier);
  config_->maxBacktrackSteps = node->declare_parameter<int>(param_prefix + ".maxBacktrackSteps", config_->maxBacktrackSteps);

  // validity checking
  config_->perform_planarity_check = node->declare_parameter<bool>(param_prefix + ".perform_planarity_check", config_->perform_planarity_check);
  config_->plane_distance = node->declare_parameter<double>(param_prefix + ".plane_distance", config_->plane_distance);
  config_->min_point_depth = node->declare_parameter<double>(param_prefix + ".min_point_depth", config_->min_point_depth);
  config_->max_point_depth = node->declare_parameter<double>(param_prefix + ".max_point_depth", config_->max_point_depth);

  // trajectory stuff.
  config_->save_trajectory = node->declare_parameter<bool>(param_prefix + ".save_trajectory", config_->save_trajectory);
  config_->trajectory_smoothing = node->declare_parameter<bool>(param_prefix + ".trajectory_smoothing", config_->trajectory_smoothing);
  config_->lin_acc_std_dev_x = node->declare_parameter<double>(param_prefix + ".lin_acc_std_dev_x", config_->lin_acc_std_dev_x);
  config_->lin_acc_std_dev_y = node->declare_parameter<double>(param_prefix + ".lin_acc_std_dev_y", config_->lin_acc_std_dev_y);
  config_->lin_acc_std_dev_z = node->declare_parameter<double>(param_prefix + ".lin_acc_std_dev_z", config_->lin_acc_std_dev_z);
  config_->ang_acc_std_dev_x = node->declare_parameter<double>(param_prefix + ".ang_acc_std_dev_x", config_->ang_acc_std_dev_x);
  config_->ang_acc_std_dev_y = node->declare_parameter<double>(param_prefix + ".ang_acc_std_dev_y", config_->ang_acc_std_dev_y);
  config_->ang_acc_std_dev_z = node->declare_parameter<double>(param_prefix + ".ang_acc_std_dev_z", config_->ang_acc_std_dev_z);
  config_->disable_solver = node->declare_parameter<bool>(param_prefix + ".disable_solver", config_->disable_solver);
  // velocity prior
  config_->velocity_prior = node->declare_parameter<bool>(param_prefix + ".velocity_prior", config_->velocity_prior);
  config_->lin_vel_mean_x = node->declare_parameter<double>(param_prefix + ".lin_vel_mean_x", config_->lin_vel_mean_x);
  config_->lin_vel_mean_y = node->declare_parameter<double>(param_prefix + ".lin_vel_mean_y", config_->lin_vel_mean_y);
  config_->lin_vel_mean_z = node->declare_parameter<double>(param_prefix + ".lin_vel_mean_z", config_->lin_vel_mean_z);
  config_->ang_vel_mean_x = node->declare_parameter<double>(param_prefix + ".ang_vel_mean_x", config_->ang_vel_mean_x);
  config_->ang_vel_mean_y = node->declare_parameter<double>(param_prefix + ".ang_vel_mean_y", config_->ang_vel_mean_y);
  config_->ang_vel_mean_z = node->declare_parameter<double>(param_prefix + ".ang_vel_mean_z", config_->ang_vel_mean_z);

  config_->lin_vel_std_dev_x = node->declare_parameter<double>(param_prefix + ".lin_vel_std_dev_x", config_->lin_vel_std_dev_x);
  config_->lin_vel_std_dev_y = node->declare_parameter<double>(param_prefix + ".lin_vel_std_dev_y", config_->lin_vel_std_dev_y);
  config_->lin_vel_std_dev_z = node->declare_parameter<double>(param_prefix + ".lin_vel_std_dev_z", config_->lin_vel_std_dev_z);
  config_->ang_vel_std_dev_x = node->declare_parameter<double>(param_prefix + ".ang_vel_std_dev_x", config_->ang_vel_std_dev_x);
  config_->ang_vel_std_dev_y = node->declare_parameter<double>(param_prefix + ".ang_vel_std_dev_y", config_->ang_vel_std_dev_y);
  config_->ang_vel_std_dev_z = node->declare_parameter<double>(param_prefix + ".ang_vel_std_dev_z", config_->ang_vel_std_dev_z);
  // clang-format on
  setConfig();
}

void SteamModule::setConfig() {
  backup_params_.verbose = config_->verbose;
  backup_params_.maxIterations = config_->iterations;
  backup_params_.absoluteCostThreshold = config_->absoluteCostThreshold;
  backup_params_.absoluteCostChangeThreshold =
      config_->absoluteCostChangeThreshold;
  backup_params_.relativeCostChangeThreshold =
      config_->relativeCostChangeThreshold;

  backup_params_.shrinkCoeff = config_->shrinkCoeff;
  backup_params_.growCoeff = config_->growCoeff;
  backup_params_.maxShrinkSteps = config_->maxShrinkSteps;

  Eigen::Array<double, 1, 6> Qc_diag;
  Qc_diag << config_->lin_acc_std_dev_x, config_->lin_acc_std_dev_y,
      config_->lin_acc_std_dev_z, config_->ang_acc_std_dev_x,
      config_->ang_acc_std_dev_y, config_->ang_acc_std_dev_z;
  if (checkDiagonal(Qc_diag) == false && config_->trajectory_smoothing) {
    throw std::runtime_error(
        "Elements of the smoothing factor must be greater than zero!");
  }
  // Make Qc_inv
  smoothing_factor_information_.setZero();
  smoothing_factor_information_.diagonal() = 1.0 / Qc_diag;

  // Setup velocity prior
  velocity_prior_ << config_->lin_vel_mean_x, config_->lin_vel_mean_y,
      config_->lin_vel_mean_z, config_->ang_vel_mean_x, config_->ang_vel_mean_y,
      config_->ang_vel_mean_z;

  Eigen::Array<double, 1, 6> Qv_diag;
  Qv_diag << config_->lin_vel_std_dev_x, config_->lin_vel_std_dev_y,
      config_->lin_vel_std_dev_z, config_->ang_vel_std_dev_x,
      config_->ang_vel_std_dev_y, config_->ang_vel_std_dev_z;

  if (checkDiagonal(Qv_diag) == false && config_->trajectory_smoothing) {
    throw std::runtime_error(
        "Error: elements of the velocity prior noise must be greater than "
        "zero!");
  }
  velocity_prior_cov_.setZero();
  velocity_prior_cov_.diagonal() = 1.0 / Qv_diag;
}

std::shared_ptr<steam::SolverBase> SteamModule::generateSolver(
    std::shared_ptr<steam::OptimizationProblem> &problem) {
  // Setup Solver
  std::shared_ptr<steam::SolverBase> solver;
  if (config_->solver_type == "LevenburgMarquardt") {
    steam::LevMarqGaussNewtonSolver::Params params;
    params.verbose = config_->verbose;
    params.maxIterations = config_->iterations;
    params.absoluteCostThreshold = config_->absoluteCostThreshold;
    params.absoluteCostChangeThreshold = config_->absoluteCostChangeThreshold;
    params.relativeCostChangeThreshold = config_->relativeCostChangeThreshold;

    params.shrinkCoeff = config_->shrinkCoeff;
    params.growCoeff = config_->growCoeff;
    params.maxShrinkSteps = config_->maxShrinkSteps;

    solver.reset(new steam::LevMarqGaussNewtonSolver(problem.get(), params));
  } else if (config_->solver_type == "DoglegGaussNewton") {
    steam::DoglegGaussNewtonSolver::Params params;
    params.verbose = config_->verbose;
    params.maxIterations = config_->iterations;
    params.absoluteCostThreshold = config_->absoluteCostThreshold;
    params.absoluteCostChangeThreshold = config_->absoluteCostChangeThreshold;
    params.relativeCostChangeThreshold = config_->relativeCostChangeThreshold;

    params.ratioThresholdShrink = config_->ratioThresholdShrink;
    params.ratioThresholdGrow = config_->ratioThresholdGrow;
    params.shrinkCoeff = config_->shrinkCoeff;
    params.growCoeff = config_->growCoeff;
    params.maxShrinkSteps = config_->maxShrinkSteps;

    solver.reset(new steam::DoglegGaussNewtonSolver(problem.get(), params));
  } else if (config_->solver_type == "VanillaGaussNewton") {
    steam::VanillaGaussNewtonSolver::Params params;
    params.verbose = config_->verbose;
    params.maxIterations = config_->iterations;
    params.absoluteCostThreshold = config_->absoluteCostThreshold;
    params.absoluteCostChangeThreshold = config_->absoluteCostChangeThreshold;
    params.relativeCostChangeThreshold = config_->relativeCostChangeThreshold;
    solver.reset(new steam::VanillaGaussNewtonSolver(problem.get(), params));
  } else {
    LOG(ERROR) << "Unknown solver type: " << config_->solver_type;
  }
  return solver;
}

bool SteamModule::forceLM(
    std::shared_ptr<steam::OptimizationProblem> &problem) {
  try {
    backup_lm_solver_.reset(
        new steam::LevMarqGaussNewtonSolver(problem.get(), backup_params_));
    backup_lm_solver_->optimize();
  } catch (std::runtime_error &re) {
    LOG(ERROR) << "Back up LM failed, abandon hope....";
    return false;
  }
  backup_lm_solver_used_ = true;
  return true;
}

void SteamModule::runImpl(QueryCache &qdata0, const Graph::ConstPtr &graph) {
  auto &qdata = dynamic_cast<CameraQueryCache &>(qdata0);

  *qdata.steam_failure = false;
  backup_lm_solver_used_ = false;

  // basic sanity check
  if (!qdata.rig_features.is_valid() ||
      (qdata.success.is_valid() && *qdata.success == false)) {
    return;
  }

  /// \todo yuchen find a better place for this, or the following transformation
  /// code.
  if (!verifyInputData(qdata)) return;

  // Construct a transform evaluator that takes points from the vehicle frame
  // into the sensor frame.
  if (qdata.T_sensor_vehicle.is_valid()) {
    tf_sensor_vehicle_ = steam::se3::FixedTransformEvaluator::MakeShared(
        *qdata.T_sensor_vehicle);
  } else {
    tf_sensor_vehicle_ = steam::se3::FixedTransformEvaluator::MakeShared(
        lgmath::se3::Transformation());
  }

  for (auto it = qdata.T_sensor_vehicle_map->begin();
       it != qdata.T_sensor_vehicle_map->end(); ++it) {
    tf_sensor_vehicle_map_[it->first] =
        steam::se3::FixedTransformEvaluator::MakeShared(it->second);
  }

  tf_identity_ = steam::se3::FixedTransformEvaluator::MakeShared(
      lgmath::se3::Transformation());

  std::shared_ptr<steam::OptimizationProblem> problem;
  try {
    // PROBLEM SPECIFIC
    if (!verifyInputData(qdata)) return;
    problem = generateOptimizationProblem(qdata, graph);
    if (problem == nullptr) {
      LOG(ERROR) << "Couldn't generate optimization problem!" << std::endl;
      *qdata.steam_failure = true;
      return;
    }

    solver_ = generateSolver(problem);

    // default to success
    bool success = true;

    // attempt to run the solver
    try {
      std::lock_guard<std::mutex> iteration_lock(*qdata.steam_mutex);
      if (!config_->disable_solver) solver_->optimize();
    } catch (std::logic_error &e) {
      LOG(ERROR) << "Forced Gradient-Descent, running in LM..." << e.what();
      std::lock_guard<std::mutex> iteration_lock(*qdata.steam_mutex);
      success = forceLM(problem);
      *qdata.steam_failure = !success;
      *qdata.success = success;
    } catch (steam::unsuccessful_step &e) {
      // did any successful steps occur?
      if (solver_->getCurrIteration() <= 1) {
        // no: something is very wrong
        LOG(ERROR)
            << "Steam has failed to optimise the problem. This is an error";
        success = false;
        *qdata.steam_failure = !success;
        *qdata.success = success;
      } else {
        // yes: just a marginal problem, let's use what we got
        LOG(WARNING) << "Steam has failed due to an unsuccessful step. This "
                        "should be OK if it happens infrequently.";
      }
    } catch (std::runtime_error &e) {
      LOG(ERROR) << "Steam has failed with an unusual error: " << e.what();
      success = false;
      *qdata.steam_failure = !success;
      *qdata.success = success;
    }

    success = success && verifyOutputData(qdata);
    if (success == true) updateCaches(qdata);

  } catch (...) {
    *qdata.steam_failure = true;
    LOG(ERROR) << " bailing on steam problem!";
    *qdata.success = false;
  }

  if (config_->use_T_q_m_prior && qdata.T_r_m_prior.is_valid() == true &&
      (*qdata.T_r_m_prior).covarianceSet() && (*qdata.T_r_m).covarianceSet()) {
    double prior_ct_sigma = sqrt((*qdata.T_r_m_prior).cov()(1, 1));
    double ct_sigma = sqrt((*qdata.T_r_m).cov()(1, 1));
    if (ct_sigma > prior_ct_sigma) {
      LOG(WARNING) << "Loc. added uncertainty, bailing.";
      *qdata.success = false;
      *qdata.steam_failure = true;
    }
  }
}

MonoCalibPtr SteamModule::toMonoSteamCalibration(
    const vision::RigCalibration &calibration) {
  MonoCalibPtr sharedMonoIntrinsics(
      new vtr::steam_extensions::mono::CameraIntrinsics);
  sharedMonoIntrinsics->fu = calibration.intrinsics[0](0, 0);
  sharedMonoIntrinsics->fv = calibration.intrinsics[0](1, 1);
  sharedMonoIntrinsics->cu = calibration.intrinsics[0](0, 2);
  sharedMonoIntrinsics->cv = calibration.intrinsics[0](1, 2);
  return sharedMonoIntrinsics;
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
