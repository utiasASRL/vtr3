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
 * \file steam_module.hpp
 * \brief SteamModule class definition
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <lgmath.hpp>
#include <steam.hpp>

#include <vtr_steam_extensions/evaluator/mono_camera_error_eval.hpp>
#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_vision/cache.hpp>

namespace vtr {
namespace vision {

using MonoCalibPtr = vtr::steam_extensions::mono::CameraIntrinsics::Ptr;
using StereoCalibPtr = steam::stereo::CameraIntrinsics::Ptr;

/** \brief Reject outliers and estimate a preliminary transform */
class SteamModule : public tactic::BaseModule {
 public:
  /**
   * \brief Static module identifier.
   * \todo change this to static_name
   */
  static constexpr auto static_name = "steam";

  /** \brief Collection of config parameters */
  struct Config {
    /** \brief Initialize sane defaults as required */
    Config() : trajectory_smoothing(false){};

    /**
     * \brief STEAM solver type. options: LevenburgMarquardt,
     * DoglegGaussNewton, VanillaGaussNewton
     */
    std::string solver_type;

    /** \brief STEAM loss function, options: Huber, L2, DCS */
    std::string loss_function;

    /** \brief STEAM verbose printing flag. */
    bool verbose;

    /** \brief Whether to use the prior. */
    bool use_T_q_m_prior;

    /** \brief Whether to ucheck the points against a plane fit */
    bool perform_planarity_check;

    /** \brief Allowable distance from the plane to be an inlier */
    float plane_distance;

    /** \brief Maximum iterations for the STEAM solve. */
    int iterations;

    /// Absolute cost threshold to trigger convergence (cost is less than x)
    double absoluteCostThreshold;  // 0.0

    /// Change in cost threshold to trigger convergence (cost went down by less
    /// than x)
    double absoluteCostChangeThreshold;  // 1e-4

    /// Relative cost threshold to trigger convergence (costChange/oldCost is
    /// less than x)
    double relativeCostChangeThreshold;  // 1e-4

    /// Minimum ratio of actual to predicted cost reduction, shrink trust region
    /// if lower (range: 0.0-1.0)
    double ratioThresholdShrink;

    /// Grow trust region if ratio of actual to predicted cost reduction above
    /// this (range: 0.0-1.0)
    double ratioThresholdGrow;

    /// Amount to shrink by (range: <1.0)
    double shrinkCoeff;

    /// Amount to grow by (range: >1.0)
    double growCoeff;

    /// Maximum number of times to shrink trust region before giving up
    int maxShrinkSteps;

    /// Minimum/Maximum depth of a valid point
    double min_point_depth;
    double max_point_depth;

    // line search
    double backtrackMultiplier;

    int maxBacktrackSteps;

    /** \brief flag to enable sampling and saving the trajectory to a stream. */
    bool save_trajectory;

    /** \brief flag to enable smoothing via a STEAM trajectory. */
    bool trajectory_smoothing;

    /**
     * \brief flag to disable running smoother (we might just want the
     * trajectory)
     */
    bool disable_solver;

    /** \brief Smoothing factors for the trajectory. */
    double lin_acc_std_dev_x;
    double lin_acc_std_dev_y;
    double lin_acc_std_dev_z;
    double ang_acc_std_dev_x;
    double ang_acc_std_dev_y;
    double ang_acc_std_dev_z;

    /** \brief Velocity prior for the trajectory */
    bool velocity_prior;
    double lin_vel_mean_x;
    double lin_vel_mean_y;
    double lin_vel_mean_z;
    double ang_vel_mean_x;
    double ang_vel_mean_y;
    double ang_vel_mean_z;

    double lin_vel_std_dev_x;
    double lin_vel_std_dev_y;
    double lin_vel_std_dev_z;
    double ang_vel_std_dev_x;
    double ang_vel_std_dev_y;
    double ang_vel_std_dev_z;
  };

  SteamModule(const std::string &name = static_name)
      : tactic::BaseModule{name}, config_(std::make_shared<Config>()) {
    backup_lm_solver_used_ = false;
  };

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 protected:
  /**
   * \brief Given two frames and matches detects the inliers that fit the given
   * model, and provides an initial guess at transform T_q_m.
   */
  void runImpl(tactic::QueryCache &qdata,
               const tactic::Graph::ConstPtr &graph) override;

  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;

  std::shared_ptr<steam::SolverBase> generateSolver(
      std::shared_ptr<steam::OptimizationProblem> &problem);

  steam::se3::FixedTransformEvaluator::Ptr tf_sensor_vehicle_;
  std::map<tactic::VertexId, steam::se3::FixedTransformEvaluator::Ptr>
      tf_sensor_vehicle_map_;

  steam::se3::FixedTransformEvaluator::Ptr tf_identity_;

  /** \brief Given two frames, builds a sensor specific optimization problem. */
  virtual std::shared_ptr<steam::OptimizationProblem>
  generateOptimizationProblem(CameraQueryCache &qdata,
                              const tactic::Graph::ConstPtr &graph) = 0;

  /** \brief Attempts to run the problem with the backup LM solver. */
  bool forceLM(std::shared_ptr<steam::OptimizationProblem> &problem);

  /** \brief Updates the caches with any optimized variables. */
  virtual void updateCaches(CameraQueryCache &qdata) = 0;

  /**
   * \brief Verifies the input data being used in the optimization problem
   * \param qdata The query data.
   */
  virtual bool verifyInputData(CameraQueryCache &qdata) = 0;

  /**
   * \brief Verifies the output data generated byt the optimization problem
   * \param qdata The query data.
   */
  virtual bool verifyOutputData(CameraQueryCache &qdata) = 0;

  /** \brief Generate a mono steam calibration */
  MonoCalibPtr toMonoSteamCalibration(const RigCalibration &calibration);

  /** \brief Generate a stereo steam calibration */
  StereoCalibPtr toStereoSteamCalibration(const RigCalibration &calibration);

  steam::LevMarqGaussNewtonSolver::Params backup_params_;
  std::shared_ptr<steam::SolverBase> backup_lm_solver_;
  bool backup_lm_solver_used_;
  std::shared_ptr<steam::SolverBase> solver_;

  /**
   * \brief The steam trajectory, allows smoothing factors, velocity priors and
   * pose extrapolation.
   */
  std::shared_ptr<steam::se3::SteamTrajInterface> trajectory_;

  Eigen::Matrix<double, 6, 6> smoothing_factor_information_;
  Eigen::Matrix<double, 6, 1> velocity_prior_;
  Eigen::Matrix<double, 6, 6> velocity_prior_cov_;

 private:
  void setConfig();

};  // namespace tactic

}  // namespace vision
}  // namespace vtr
