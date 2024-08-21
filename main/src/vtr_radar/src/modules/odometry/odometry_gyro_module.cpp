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
 * \author Yuchen Wu, Keenan Burnett, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_radar/modules/odometry/odometry_gyro_module.hpp"

#include "vtr_radar/utils/nanoflann_utils.hpp"

#include "steam/problem/cost_term/gyro_super_cost_term.hpp"

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

  // optimization params
  config->verbose = node->declare_parameter<bool>(param_prefix + ".verbose", false);
  config->max_iterations = (unsigned int)node->declare_parameter<int>(param_prefix + ".max_iterations", 1);

  // clang-format on
  return config;
}

void OdometryGyroModule::run_(QueryCache &qdata0, OutputCache &,
                             const Graph::Ptr &, const TaskExecutor::Ptr &) {
  auto &qdata = dynamic_cast<RadarQueryCache &>(qdata0);

  // Do nothing if qdata does not contain any gyro data (was populated by radar)
  // Also do nothing, if odometry has not been initialized (we will wait until radar did this)
  if(!qdata.gyro_msg.valid() || !qdata.sliding_map_odo)
  {
    return;
  }

  CLOG(DEBUG, "radar.odometry_gyro")
      << "Retrieve input data and setup evaluators.";

  // Inputs
  const auto &query_stamp = *qdata.stamp;
  const auto &T_s_r = *qdata.T_s_r;
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

  const Eigen::Matrix<double,6,1> xi_m_r_in_r_odo((query_time - prev_time).seconds() * w_m_r_in_r_odo);
  const auto T_r_m_odo_extp = tactic::EdgeTransform(xi_m_r_in_r_odo) * T_r_m_odo;
  const auto T_r_m_var = SE3StateVar::MakeShared(T_r_m_odo_extp);
  //
  const auto w_m_r_in_r_var = VSpaceStateVar<6>::MakeShared(w_m_r_in_r_odo);
  //
  trajectory->add(query_stamp, T_r_m_var, w_m_r_in_r_var);
  state_vars.emplace_back(T_r_m_var);
  state_vars.emplace_back(w_m_r_in_r_var);

  // initialize problem
  OptimizationProblem problem;

  // add variables
  for (const auto &var : state_vars)
    problem.addStateVariable(var);

  // add prior cost terms
  trajectory->addPriorCostTerms(problem);

  // Lets define a velocity measurement cost term

  const auto loss_func = L2LossFunc::MakeShared();

  Eigen::Matrix<double,3,1> gyro_measurement;
  gyro_measurement(0) = -1*gyro_msg.angular_velocity.x; // Need to use negative value due to Tim's convention? 
  gyro_measurement(1) = -1*gyro_msg.angular_velocity.y;
  gyro_measurement(2) = -1*gyro_msg.angular_velocity.z;

  // Define the bias values
  Eigen::Matrix<double, 6, 1> bias_value1 = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> bias_value2 = Eigen::Matrix<double, 6, 1>::Zero();

  Evaluable<GyroSuperCostTerm::BiasType>::ConstPtr bias1 = std::make_shared<Evaluable<GyroSuperCostTerm::BiasType>>(bias_value1);
  Evaluable<GyroSuperCostTerm::BiasType>::ConstPtr bias2 = std::make_shared<Evaluable<GyroSuperCostTerm::BiasType>>(bias_value2);


  GyroSuperCostTerm::Options options;
  options.num_threads = 1;
  options.gyro_loss_func = GyroSuperCostTerm::LOSS_FUNC::L2;
  options.gyro_loss_sigma = 0.1; //gyro_msg.angular_velocity_covariance[8] ??
  options.r_imu_ang = gyro_measurement;
  options.se2 = true;

  const auto measurement_cost = GyroSuperCostTerm::MakeShared(trajectory, prev_time, query_time, bias1, bias2, options);
  problem.addCostTerm(measurement_cost);

  // optimize
  GaussNewtonSolver::Params params;
  params.verbose = config_->verbose;
  params.max_iterations = (unsigned int)config_->max_iterations;
  GaussNewtonSolver solver(problem, params);
  solver.optimize();
  Covariance covariance(solver);

  // Get the odometry results
  *qdata.T_r_m_odo = T_r_m_var->value();
  *qdata.timestamp_odo = query_stamp;
  *qdata.w_m_r_in_r_odo = w_m_r_in_r_var->value();
  *qdata.odo_success = true;

  // Do I need to set these??
  // qdata.T_r_v_odo
  // qdata.w_v_r_in_r_odo
}

}  // namespace radar
}  // namespace vtr