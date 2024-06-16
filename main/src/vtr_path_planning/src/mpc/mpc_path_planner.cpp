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
 * \file mpc_path_planner.cpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */

#include <ranges>

#include "vtr_path_planning/mpc/mpc_path_planner.hpp"
#include "vtr_path_planning/mpc/lateral_error_evaluators.hpp"
#include "vtr_path_planning/mpc/scalar_log_barrier_evaluator.hpp"



// Main MPC problem solve function - TODO: dump all these arguments into an mpc config class
struct MPCResult SolveMPC(const MPCConfig& config, const tactic::LocalizationChain::Ptr chain) {
  using namespace steam;

  // Access configuration parameters from the config structure
  Eigen::Vector2d previous_vel = config.previous_vel;
  lgmath::se3::Transformation T0 = config.T0;
  std::vector<lgmath::se3::Transformation> tracking_reference_poses = config.tracking_reference_poses;
  std::vector<double> barrier_q_left = config.barrier_q_left;
  std::vector<double> barrier_q_right = config.barrier_q_right;
  int rollout_window = config.K;
  double DT = config.DT;
  double VF = config.VF;
  Eigen::Matrix<double, 1, 1> lat_noise_vect = config.lat_noise_vect;
  Eigen::Matrix<double, 2, 2> vel_noise_vect = config.vel_noise_vect;
  bool point_stabilization = config.point_stabilization;
  double vel_error_weight = config.vel_error_weight;
  double lat_error_weight = config.lat_error_weight;
  bool verbosity = config.verbosity;
  bool homotopy_mode = config.homotopy_mode;

  // Conduct an MPC Iteration given current configurations

  // Invert the extrapolated robot state and use this as the state initialization
  auto T_init = steam::se3::SE3StateVar::MakeShared(T0);
  T_init->locked() = true;



  const Eigen::Vector2d V_REF{VF, 0.0};
  const Eigen::Vector2d V_INIT = previous_vel;

  const Eigen::Vector2d V_MAX{1.5, 1.0};
  const Eigen::Vector2d V_MIN{-1.5, -1.0};

  const Eigen::Vector2d ACC_MAX{0.25, 0.5};
  const Eigen::Vector2d ACC_MIN{-0.25, -0.5};

  Eigen::Matrix<double, 6, 2> P_tran;
  P_tran << 1, 0,
            0, 0,
            0, 0,
            0, 0,
            0, 0,
            0, 1;


  // Setup shared loss functions and noise models for all cost terms
  const auto l2Loss = L2LossFunc::MakeShared();
  Eigen::Matrix2d vel_cov = Eigen::Matrix2d::Zero();
  vel_cov.diagonal() << 1.0, 1e5;
  const auto sharedVelNoiseModel = steam::StaticNoiseModel<2>::MakeShared(
      vel_cov);

  Eigen::Matrix<double, 6, 6> pose_cov =
      Eigen::Matrix<double, 6, 6>::Identity();
  pose_cov.diagonal() << 1.0, 1.0, 1e5, 1e5, 1e5, 1e2;
  const auto finalPoseNoiseModel =
      steam::StaticNoiseModel<6>::MakeShared(pose_cov);

  std::vector<vspace::VSpaceStateVar<2>::Ptr> vel_state_vars;
  std::vector<Eigen::Vector2d> last_valid_vels;


  for (unsigned i = 0; i < rollout_window; i++) {
    vel_state_vars.push_back(
        vspace::VSpaceStateVar<2>::MakeShared(V_INIT));
    std::cout << "Initial velo " << vel_state_vars.back()->value() << std::endl;
  }

  steam::Timer timer;

  std::vector<Evaluable<lgmath::se3::Transformation>::Ptr> pose_vars;
  std::vector<Evaluable<lgmath::se3::Transformation>::Ptr> path_vars;
  std::vector<lgmath::se3::Transformation> pose_vals;

  lgmath::se3::Transformation Tf_accum = T_init->value();

    // Create STEAM variables
    for (unsigned i = 0; i < rollout_window; i++) {
      

      Eigen::VectorXd vel_proj = DT * P_tran * V_REF;
      auto deltaTf = lgmath::se3::Transformation(vel_proj);
      Tf_accum = Tf_accum * deltaTf;
      pose_vals.push_back(Tf_accum);

    }

  double final_cost = std::numeric_limits<double>::max();

  for (double weight = 10.0; weight > 1e-2; weight *= 0.8) {
    pose_vars.clear();
    path_vars.clear();

    std::cout << "Weight is: " << weight;

    // Setup the optimization problem
    OptimizationProblem opt_problem;

    Evaluable<lgmath::se3::Transformation>::Ptr Tf_acc = T_init;

    // Create STEAM variables
    for (unsigned i = 0; i < rollout_window; i++) {
      auto& vel_var = vel_state_vars[i];

      auto vel_proj =
          vspace::MatrixMultEvaluator<6, 2>::MakeShared(vel_var, DT * P_tran);
      auto deltaTf = se3::ExpMapEvaluator::MakeShared(vel_proj);
      Tf_acc = se3::compose(Tf_acc, deltaTf);
      pose_vars.push_back(Tf_acc);

    }


    const auto pose_refs = generateHomotopyReference(pose_vals, chain);

    try{
    for (unsigned i = 0; i < rollout_window; i++) {
      auto& vel_var = vel_state_vars[i];
      auto& Tf_k = pose_vars[i];

      const auto interp_state = se3::SE3StateVar::MakeShared(pose_refs.poses[i]);
      // const auto interp_state = se3::SE3StateVar::MakeShared(chain->pose(std::min(chain->trunkSequenceId() + i, unsigned(chain->size()) - 1)));
      interp_state->locked() = true;
      path_vars.push_back(interp_state);

      opt_problem.addStateVariable(vel_var);
      const auto vel_cost_term = WeightedLeastSqCostTerm<2>::MakeShared(
          vspace::vspace_error<2>(vel_var, V_REF), sharedVelNoiseModel, l2Loss);
      opt_problem.addCostTerm(vel_cost_term);

      const auto path_cost = WeightedLeastSqCostTerm<6>::MakeShared(se3::tran2vec(se3::compose(
        se3::inverse(interp_state), Tf_k)), finalPoseNoiseModel, l2Loss);
      opt_problem.addCostTerm(path_cost);


      opt_problem.addCostTerm(vspace::LogBarrierCostTerm<2>::MakeShared(vspace::vspace_error<2>(vel_var,
      V_MAX), weight));
      opt_problem.addCostTerm(vspace::LogBarrierCostTerm<2>::MakeShared(vspace::neg<2>(vspace::vspace_error<2>(vel_var,
      V_MIN)), weight));


      if (i > 0) {
        const auto accel_term = vspace::add<2>(
            vel_state_vars[i], vspace::neg<2>(vel_state_vars[i - 1]));
        opt_problem.addCostTerm(vspace::LogBarrierCostTerm<2>::MakeShared(vspace::vspace_error<2>(accel_term,
        DT*ACC_MAX), weight));
        opt_problem.addCostTerm(vspace::LogBarrierCostTerm<2>::MakeShared(vspace::neg<2>(vspace::vspace_error<2>(accel_term,
        DT*ACC_MIN)), weight));
      } else {
        const auto accel_term = vspace::vspace_error<2>(
            vspace::neg<2>(vel_state_vars[i]), -previous_vel);
        opt_problem.addCostTerm(vspace::LogBarrierCostTerm<2>::MakeShared(vspace::vspace_error<2>(accel_term,
        DT*ACC_MAX), weight));
        opt_problem.addCostTerm(vspace::LogBarrierCostTerm<2>::MakeShared(vspace::neg<2>(vspace::vspace_error<2>(accel_term,
        DT*ACC_MIN)), weight));
      }
    }

    } catch(std::logic_error &e) {
      CLOG(WARNING, "mpc.solver") << "Last update was unsuccessful! Falling back on previous solution!";
      break;
    }


    using SolverType = LevMarqGaussNewtonSolver;

    // Initialize solver parameters
    SolverType::Params params;
    params.verbose = true;  // Makes the output display for debug when true
    params.max_iterations = 100;
    params.ratio_threshold = 0.1;

    double initial_cost = opt_problem.cost();
    // Check the cost, disregard the result if it is unreasonable (i.e if its
    // higher then the initial cost)
    std::cout << "The Initial Solution Cost is:" << initial_cost << std::endl;

    SolverType solver(opt_problem, params);

    try{
      solver.optimize();
    } catch (steam::unsuccessful_step &e) {
      break;
    }
    last_valid_vels.clear();
    for (const auto& vel_var : vel_state_vars) {
      last_valid_vels.push_back(vel_var->value());
    }
  }


  std::cout << "Total time: " << timer.milliseconds() << "ms" << std::endl;
  for (const auto& vel_var : vel_state_vars) {
    std::cout << "Final velo " << vel_var->value() << std::endl;
  }

  // First check if any of the values are nan, if so we return a zero velocity and flag the error
  if (last_valid_vels.size() == 0)
  {
    CLOG(ERROR, "mpc.solver") << "MPC failed!";
    Eigen::Vector2d nan_vel {0.0, 0.0};

    // if (verbosity) {
    //   throw std::runtime_error("Steam failed Crashing for debug!");
    // }

    // if we do detect nans, return the mpc_poses as all being the robot's current pose (not moving across the horizon as we should be stopped)
    std::vector<lgmath::se3::Transformation> mpc_poses;
    for (size_t i = 0; i < rollout_window; i++)
    {
      mpc_poses.push_back(T0);
    }
    return {nan_vel, mpc_poses};
  } else {

    // Store the velocity command to apply
    Eigen::Vector2d applied_vel = last_valid_vels[0];

    // if no nan values, return the applied velocity and mpc pose predictions as normal
    // Store the sequence of resulting mpc prediction horizon poses for visualization
    std::vector<lgmath::se3::Transformation> mpc_poses;
    for (const auto& pose_var : pose_vars) {
      mpc_poses.push_back(pose_var->value());
    }

    // Return the resulting structure
    return {applied_vel, mpc_poses};
  }
  
}




// helper function for computing the optimization reference poses T_ref based on the current path solution
// This is specifically for the tracking mpc, but is also used to generate the warm start poses for the corridor mpc
struct PoseResultTracking GenerateTrackingReference(std::shared_ptr<std::vector<Pose>> cbit_path_ptr, std::tuple<double, double, double, double, double, double> robot_pose, int K, double DT, double VF)
{

    // Save a copy of the current path solution to work on
    auto cbit_path = *cbit_path_ptr;

    // PSEUDO CODE:
    // 1. Find the closest point on the cbit path to the current state of the robot
    // 2. Using K, DT, VF, we generate a vector of "p" values that we want to create Euclidean Poses for (we know these up front)
    // 3. After we have our starting closest point on the path, assign that point a p value of 0. Compute the p values for each subsequent point in the lookahead window
    // 4. using the desired p values, and the known p values, interpolate a, x,y,z,yaw, value each measurement
    // 5. Create the proper measurement transform for each measurement and get it ready for the using with the optimization problem

    // Limiting the size of the cbit path based on the sliding window and then assigning p values
    double lookahead_dist = 0.0;
    double p_dist = 0.0;

    double min_dist = INFINITY;
    double new_dist;
    double dx;
    double dy;
    double p_correction = 0.0;
    bool min_flag = true;

    std::vector<double> cbit_p;
    cbit_p.reserve(cbit_path.size());
    cbit_p.push_back(0.0);
    for (size_t i = 0; i < (cbit_path.size()-2); i++) // the last value of vector is size()-1, so second to last will be size-2
    { 
      // calculate the p value for the point
      p_dist = sqrt((((cbit_path)[i].x - (cbit_path)[i+1].x) * ((cbit_path)[i].x - (cbit_path)[i+1].x)) + (((cbit_path)[i].y - (cbit_path)[i+1].y) * ((cbit_path)[i].y - (cbit_path)[i+1].y)));
      lookahead_dist = lookahead_dist + p_dist;
      cbit_p.push_back(lookahead_dist);

      // Keep track of the closest point to the robot state
      dx = (cbit_path)[i].x - std::get<0>(robot_pose);
      dy = (cbit_path)[i].y - std::get<1>(robot_pose);
      new_dist = sqrt((dx * dx) + (dy * dy));
      if ((new_dist < min_dist) && (min_flag == true))
      {
        p_correction = lookahead_dist;
        min_dist = new_dist;
      }
      else
      {
        if (new_dist > min_dist + 0.1)
        {
          min_flag = false;
        }
      }
      
      // Stop once we get about 12m ahead of the robot (magic number for now, but this is just a conservative estimate of any reasonable lookahead window and mpc horizon)
      if (lookahead_dist > 12.0)
      {
        break;
      }
    }

    // Determine the p_values we need for our measurement horizon, corrected for the p value of the closest point on the path to the current robot state
    std::vector<double> p_meas_vec;
    std::vector<lgmath::se3::Transformation> homotopy_reference_poses;
    std::vector<double> p_interp_vec;
    std::vector<double> q_interp_vec;

    p_meas_vec.reserve(K);
    for (int i = 0; i < K; i++)
    {

      p_meas_vec.push_back((i * DT * VF) + p_correction);
    }
    
    // Iterate through the p_measurements and interpolate euclidean poses from the cbit_path and the corresponding cbit_p values
    // Note this could probably just be combined in the previous loop too
    bool point_stabilization = false;
    for (size_t i = 0; i < p_meas_vec.size(); i++)
    {
      // handle end of path case:
      // if the p meas we would have needed exceeds the final measurement pose, set it equal to our last p value in the path
      // This will cause the intepolation to return the final cbit_path pose for all poses past this point

      if (p_meas_vec[i] > cbit_p[cbit_p.size()-1])
      {
        p_meas_vec[i] = cbit_p[cbit_p.size()-1];
        point_stabilization = true; // Enable point stabilization configs
        CLOG(INFO, "mpc.solver") << "Approaching End of Path, Converting MPC to Point Stabilization Problem";
      }
      struct InterpResult interp_pose = InterpolatePose(p_meas_vec[i], cbit_p, cbit_path);

      // add to measurement vector
      homotopy_reference_poses.push_back(interp_pose.pose);
      p_interp_vec.push_back(interp_pose.p_interp);
      q_interp_vec.push_back(interp_pose.q_interp);

    }
    return {homotopy_reference_poses, point_stabilization, p_interp_vec, q_interp_vec};
}

using Segment = std::pair<unsigned, unsigned>;

Segment findClosestSegment(const lgmath::se3::Transformation& T_wr, const tactic::LocalizationChain::Ptr chain, unsigned sid_start=0) {

    double best_distance = std::numeric_limits<double>::max();
    double max_distance = -1.;
    unsigned best_sid = sid_start;
    const unsigned end_sid = std::min(sid_start + 20 + 1,
                                    unsigned(chain->size()));

    // Explicit casting to avoid numerical underflow when near the beginning of
    // the chain
    const unsigned begin_sid = unsigned(std::max(int(sid_start) - 5, 0));

    // Find the closest vertex to the input
    for (auto path_it = chain->begin(begin_sid); unsigned(path_it) < end_sid;
        ++path_it) {

      // Calculate the distance
      double distance = (T_wr.inverse() * chain->pose(path_it)).r_ab_inb().norm();
      // CLOG(DEBUG, "mpc.cost_function") << "Dist: " << distance << " sid: " << unsigned(path_it);

      // Record the best distance
      max_distance = std::max(distance, max_distance);
      if (distance < best_distance) {
        best_distance = distance;
        best_sid = unsigned(path_it);
      }

      // This block detects direction switches, and prevents searching across them
      // It's only enabled in safe-search mode (no searching backwards as well),
      // it only tries if the current sid is not an extremum of the search range,
      // and it only stops at cusps that pass X m in 'distance' from the current
      // position
      // if (max_distance > config_.min_cusp_distance &&
      //     unsigned(path_it) > begin_sid && unsigned(path_it) + 1 < end_sid) {
      //   Eigen::Matrix<double, 6, 1> vec_prev_cur = path_it->T().vec();
      //   Eigen::Matrix<double, 6, 1> vec_cur_next = (path_it + 1)->T().vec();
      //   // + means they are in the same direction (note the negative at the front
      //   // to invert one of them)
      //   double r_dot = vec_prev_cur.head<3>().dot(vec_cur_next.head<3>());
      //   // + means they are in the same direction
      //   double C_dot = vec_prev_cur.tail<3>().dot(vec_cur_next.tail<3>());
      //   // combine the translation and rotation components using the angle weight
      //   double T_dot = r_dot + config_.angle_weight * C_dot;
      //   // If this is negative, they are in the 'opposite direction', and we're at
      //   // a cusp
      //   if (T_dot < 0) {
      //     CLOG_EVERY_N(1, DEBUG, "pose_graph")
      //         << "Not searching past the cusp at " << path_it->to() << ", "
      //         << distance << " (m/8degress) away.";
      //     break;
      //   }
      // }
    }

    //Handle end of path exceptions
    if(best_sid == 0)
      return std::make_pair(best_sid, best_sid + 1);
    if(best_sid == chain->size() - 1)
      return std::make_pair(best_sid - 1, best_sid);

    double next_distance = (T_wr.inverse() * chain->pose(best_sid + 1)).r_ab_inb().norm();
    double past_distance = (T_wr.inverse() * chain->pose(best_sid - 1)).r_ab_inb().norm();

    if(next_distance < past_distance)
      return std::make_pair(best_sid, best_sid + 1);
    else
      return std::make_pair(best_sid - 1, best_sid);
  }

struct CurvatureInfo
{
  Eigen::Vector3d center;
  double radius;

  inline double curvature() const {
    return 1 / radius;
  }

  static CurvatureInfo fromTransform(const lgmath::se3::Transformation &T_12);
};

CurvatureInfo CurvatureInfo::fromTransform(const lgmath::se3::Transformation& T) {
  // Note that this is only along a relative path with an origin at 0,0
  // Using the base tf is still required to move into the world frame
  auto aang = lgmath::so3::rot2vec(T.inverse().C_ba());
  double roc = T.r_ba_ina().norm() / 2 / (sin(aang(2) / 2) + 1e-6);

  static Eigen::Matrix3d rotm_perp;
  rotm_perp << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  auto dist = T.r_ba_ina().norm();
  auto lin_dir = T.r_ba_ina() / dist;

  Eigen::Vector3d coc = T.r_ba_ina() / 2 + sqrt(roc * roc - dist * dist / 4) *
                                               sgn(roc) * rotm_perp * lin_dir;
  return {coc, roc};
}

lgmath::se3::Transformation interpolatePath(const lgmath::se3::Transformation& T_wr,
                const lgmath::se3::Transformation& seq_start, const lgmath::se3::Transformation& seq_end,
                 double& interp) {
  const lgmath::se3::Transformation edge = seq_start.inverse() * seq_end;
  const auto& [coc, roc] = CurvatureInfo::fromTransform(edge);
  Eigen::Vector4d coc_h{0, 0, 0, 1};
  coc_h.head<3>() = coc;

  coc_h = seq_start.inverse().matrix() * coc_h;

  const auto interp_ang =
      acos((T_wr.r_ab_inb() - coc_h.head<3>())
               .normalized()
               .dot((seq_start.r_ab_inb() - coc_h.head<3>()).normalized()));
  const auto interp_full =
      acos((seq_end.r_ab_inb() - coc_h.head<3>())
               .normalized()
               .dot((seq_start.r_ab_inb() - coc_h.head<3>()).normalized()));
  interp = interp_ang / interp_full;
  const auto val = seq_start * lgmath::se3::Transformation(interp * edge.vec(), 0);
  return val;
}

// For generating VT&R teach path poses used in the corridor mpc (new version which directly uses the interpolated p measurements from the cbit path trajectory tracking)
PoseResultHomotopy generateHomotopyReference(const std::vector<lgmath::se3::Transformation>& rolled_out_poses, tactic::LocalizationChain::Ptr chain) {

    // Initialize vectors storing the barrier values:
    std::vector<double> barrier_q_left;
    std::vector<double> barrier_q_right;

    std::vector<lgmath::se3::Transformation> tracking_reference_poses;

    unsigned last_sid = chain->trunkSequenceId();

    // Iterate through the interpolated p_measurements and make interpolate euclidean poses from the teach path
    for (const auto& T_wrk : rolled_out_poses) {
      Segment closestSegment = findClosestSegment(T_wrk, chain, last_sid);
      last_sid = closestSegment.first;

      double interp;
      auto interpTf = interpolatePath(T_wrk, chain->pose(closestSegment.first), chain->pose(closestSegment.second), interp);

      // add to measurement vector
      tracking_reference_poses.push_back(interpTf);

      // Find the corresponding left and right barrier q values to pass to the mpc
      auto width1 = pose_graph::BasicPathBase::terrian_type_corridor_width(chain->query_terrain_type(closestSegment.first));
      auto width2 = pose_graph::BasicPathBase::terrian_type_corridor_width(chain->query_terrain_type(closestSegment.second));
      barrier_q_left.push_back((1-interp) * width1 + interp * width2);
      barrier_q_right.push_back((1-interp) * width1 + interp * width2);
    }

    return {tracking_reference_poses, barrier_q_left, barrier_q_right};
}

lgmath::se3::Transformation poseToTransformation(const Pose &p) {
  Eigen::Vector3d position {p.x, p.y, p.z};
  Eigen::Matrix3d rot_m = lgmath::so3::vec2rot({0, 0, -p.yaw});

  return lgmath::se3::Transformation{rot_m, position};
}


// function takes in the cbit path solution with a vector defining the p axis of the path, and then a desired p_meas
// Then tries to output a euclidean pose interpolated for the desired p_meas.
struct InterpResult InterpolatePose(double p_val, std::vector<double> cbit_p, std::vector<Pose> cbit_path)
{
  //If the path is length 0, return identity.
  if (cbit_path.size() == 0){
    return {tactic::EdgeTransform(true), 0, 0};
  }

  // Find the lower bound of the p values
  for (size_t i = 0; i < cbit_p.size(); i++)
  {
    if (cbit_p[i] < p_val)
    {
      continue;
    }
    else
    {
      double p_lower;
      double p_upper;
      Pose pose_lower;
      Pose pose_upper;
      // Handle potential for seg faults when p_val is before the cbit_p first value
      if (i == 0)
      {
        // means we need to back extrapolate
        p_lower = cbit_p[i];
        p_upper = cbit_p[i+1];
        pose_lower = cbit_path[i];
        pose_upper = cbit_path[i+1];
      }
      else
      {
        p_lower = cbit_p[i-1];
        p_upper = cbit_p[i];
        pose_lower = cbit_path[i-1];
        pose_upper = cbit_path[i];
      }

      double interp_percent = ((p_val - p_lower) / (p_upper - p_lower));


      // we also want to interpolate p and q values based on the original p,q from the cbit_path. We use this afterwards for finding appropriate corridor mpc
      // reference poses on the teach path
      double p_int = pose_lower.p +  interp_percent * (pose_upper.p - pose_lower.p);
      double q_int = pose_lower.q + interp_percent * (pose_upper.q - pose_lower.q);

      auto T_upper = poseToTransformation(pose_upper);
      auto T_lower = poseToTransformation(pose_lower);

      auto T_rel = T_lower.inverse() * T_upper;

      lgmath::se3::Transformation meas = T_lower * lgmath::se3::Transformation(interp_percent * T_rel.vec(), 0);

      CLOG(DEBUG, "mpc.debug") << "The measurement Euclidean state is - x: " << meas;
      CLOG(DEBUG, "mpc.debug") << "The measurement P,Q value is - p: " << p_int << " q: " << q_int;
      return {meas, p_int, q_int};
    }
  }
  //If we've reached here, the p value is longer than the path
  //return the last element.

  return {poseToTransformation(cbit_path.back()), cbit_p.back(), 0};
}


// Simple function for checking that the current output velocity command is saturated between our mechanical velocity limits
Eigen::Vector2d SaturateVel(const Eigen::Vector2d applied_vel, double v_lim, double w_lim)
{
    double command_lin_x;
    double command_ang_z;

    if (abs(applied_vel(0)) >= v_lim) {
      command_lin_x = sgn(applied_vel(0)) * v_lim;
    } else {
      command_lin_x = applied_vel(0) ;
    }

    if (abs(applied_vel(1)) >= w_lim)
    {
      command_ang_z = sgn(applied_vel(1)) * w_lim;
    } else {
      command_ang_z = applied_vel(1) ;
    }

    return {command_lin_x, command_ang_z};
}