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
 * \file odometry_gyro_module.cpp
 * \author Sven Lilge, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_radar/modules/odometry/odometry_gyro_module.hpp"

#include "vtr_radar/utils/nanoflann_utils.hpp"

#include "steam/evaluable/p2p/yaw_vel_error_evaluator.hpp"

namespace vtr {
namespace radar {

using namespace tactic;
using namespace steam;
using namespace steam::se3;
using namespace steam::traj;
using namespace steam::vspace;

auto OdometryGyroModule::Config::fromROS(const rclcpp::Node::SharedPtr &node,
                                        const std::string &param_prefix)
    -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  // motion compensation
  const auto qcd = node->declare_parameter<std::vector<double>>(param_prefix + ".traj_qc_diag", std::vector<double>());
  if (qcd.size() != 6) {
    std::string err{"Qc diagonal malformed. Must be 6 elements!"};
    CLOG(ERROR, "radar.odometry_icp") << err;
    throw std::invalid_argument{err};
  }
  config->traj_qc_diag << qcd[0], qcd[1], qcd[2], qcd[3], qcd[4], qcd[5];
  config->gyro_cov = node->declare_parameter<double>(param_prefix + ".gyro_cov", config->gyro_cov);

  // optimization params
  config->verbose = node->declare_parameter<bool>(param_prefix + ".verbose", config->verbose);
  config->max_iterations = (unsigned int)node->declare_parameter<int>(param_prefix + ".max_iterations", config->max_iterations);

  // clang-format on
  return config;
}

void OdometryGyroModule::run_(QueryCache &qdata0, OutputCache &,
                             const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  // Do nothing if qdata does not contain any gyro data (was populated by radar)
  // Also do nothing, if odometry has not been initialized (we will wait until radar did this)
  if(!qdata.gyro_msg || !qdata.sliding_map_odo)
  {
    return;
  }

  CLOG(DEBUG, "radar.odometry_gyro")
      << "Retrieve input data and setup evaluators.";

  // Inputs
  const auto &query_stamp = *qdata.stamp;
  const auto &T_s_r = *qdata.T_s_r_gyro;
  const auto &timestamp_odo = *qdata.timestamp_odo;
  const auto &T_r_m_odo = *qdata.T_r_m_odo;
  const auto &w_m_r_in_r_odo = *qdata.w_m_r_in_r_odo;
  const auto &gyro_msg = *qdata.gyro_msg;

  // clang-format off
  /// Create robot to sensor transform variable, fixed.
  const auto T_s_r_var = SE3StateVar::MakeShared(T_s_r);
  T_s_r_var->locked() = true;

  /// trajectory smoothing
  Evaluable<lgmath::se3::Transformation>::ConstPtr T_r_m_eval = nullptr;
  Evaluable<Eigen::Matrix<double, 6, 1>>::ConstPtr w_m_r_in_r_eval = nullptr;
  const_vel::Interface::Ptr trajectory = nullptr;
  std::vector<StateVarBase::Ptr> state_vars;
  trajectory = const_vel::Interface::MakeShared(config_->traj_qc_diag);

  /// last frame state
  Time prev_time(static_cast<int64_t>(timestamp_odo)); // get previous odometry timestamp
  auto prev_T_r_m_var = SE3StateVar::MakeShared(T_r_m_odo); // get previous odometry pose
  auto prev_w_m_r_in_r_var = VSpaceStateVar<6>::MakeShared(w_m_r_in_r_odo); // get previous odometry velocity
  prev_T_r_m_var->locked() = true; // Lock previous pose
  prev_w_m_r_in_r_var->locked() = true; // Lock previous velocity
  trajectory->add(prev_time, prev_T_r_m_var, prev_w_m_r_in_r_var); // add stuff to the trajectory
  state_vars.emplace_back(prev_T_r_m_var);
  state_vars.emplace_back(prev_w_m_r_in_r_var);


  // frame state at measurement time

  Time query_time(static_cast<int64_t>(query_stamp));

  Eigen::Matrix<double,6,1> xi_m_r_in_r_odo((query_time - prev_time).seconds() *w_m_r_in_r_odo);
  auto T_r_m_odo_extp = tactic::EdgeTransform(xi_m_r_in_r_odo) * T_r_m_odo;
  auto T_r_m_var = SE3StateVar::MakeShared(T_r_m_odo_extp);
  //
  auto w_m_r_in_r_var = VSpaceStateVar<6>::MakeShared(w_m_r_in_r_odo);
  //
  trajectory->add(query_stamp, T_r_m_var, w_m_r_in_r_var);
  state_vars.emplace_back(T_r_m_var);
  state_vars.emplace_back(w_m_r_in_r_var);

  auto w_m_s_in_s_var = compose_velocity(T_s_r_var, w_m_r_in_r_var);

  // initialize problem
  OptimizationProblem problem;

  // add variables
  for (const auto &var : state_vars)
    problem.addStateVariable(var);

  // add prior cost terms
  trajectory->addPriorCostTerms(problem);

  // Lets define a velocity measurement cost term
  double gyro_measurement = -1*gyro_msg.angular_velocity.z;

  const auto loss_func = L2LossFunc::MakeShared();
  const auto noise_model = StaticNoiseModel<1>::MakeShared(Eigen::Matrix<double, 1, 1>::Identity()*config_->gyro_cov);
  const auto error_func = p2p::YawVelErrorEvaluator::MakeShared(Eigen::Matrix<double, 1, 1>::Identity()*gyro_measurement,w_m_s_in_s_var);
  const auto measurement_cost = WeightedLeastSqCostTerm<1>::MakeShared(error_func, noise_model, loss_func);


  CLOG(DEBUG, "radar.odometry_gyro") << "Adding yaw velocity measurement: " << gyro_measurement;
  CLOG(DEBUG, "radar.odometry_gyro") << "Time between states: " << (query_time - prev_time).seconds();

  problem.addCostTerm(measurement_cost);

  CLOG(DEBUG, "radar.odometry_gyro") << "Pose at previous timestep: " << prev_T_r_m_var->value();
  CLOG(DEBUG, "radar.odometry_gyro") << "Velocity at previous timestep: " << prev_w_m_r_in_r_var->value();
  CLOG(DEBUG, "radar.odometry_gyro") << "Pose at new timestep before optimization: " << T_r_m_var->value();
  CLOG(DEBUG, "radar.odometry_gyro") << "Velocity at new timestep before optimization: " << w_m_r_in_r_var->value();

  // optimize
  GaussNewtonSolver::Params params;
  params.verbose = config_->verbose;
  params.max_iterations = (unsigned int)config_->max_iterations;
  GaussNewtonSolver solver(problem, params);
  solver.optimize();
  Covariance covariance(solver);

  CLOG(DEBUG, "radar.odometry_gyro") << "Pose at new timestep after optimization: " << T_r_m_var->value();
  CLOG(DEBUG, "radar.odometry_gyro") << "Velocity at new timestep after optimization: " << w_m_r_in_r_var->value();

  // Get the odometry results
  *qdata.T_r_m_odo = T_r_m_var->value();
  *qdata.timestamp_odo = query_stamp;
  *qdata.w_m_r_in_r_odo = w_m_r_in_r_var->value();
  *qdata.odo_success = true;

  // Do I need to set these??
  EdgeTransform T_r_m_gyro = EdgeTransform(T_r_m_var->value(), covariance.query(T_r_m_var));
  auto &sliding_map_odo = *qdata.sliding_map_odo;
  *qdata.T_r_v_odo = T_r_m_gyro * sliding_map_odo.T_vertex_this().inverse();
  *qdata.w_v_r_in_r_odo = *qdata.w_m_r_in_r_odo;

  CLOG(DEBUG, "radar.odometry_gyro") << "T_r_v_odo: " << *qdata.T_r_v_odo;
}

}  // namespace radar
}  // namespace vtr