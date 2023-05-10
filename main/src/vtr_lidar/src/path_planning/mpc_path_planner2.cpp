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
 * \file mpc.cpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_path_planning/mpc/mpc_path_planner2.hpp"
#include "vtr_path_planning/mpc/lateral_error_evaluators.hpp"
#include "vtr_path_planning/mpc/custom_loss_functions.hpp"
#include "vtr_path_planning/mpc/scalar_log_barrier_evaluator.hpp"
#include "vtr_lidar/cache.hpp"

// Updated version using corridor constrained MPC

// This file is used to generate a tracking mpc output velocity command given a discretized path to follow and optimization parameters
// It is used in cbit.cpp in both the vtr_lidar package (obstacle avoidance) and vtr_path_planning packages (obstacle free) in the computeCommand function


// Main MPC problem solve function - TODO: dump all these arguments into an mpc config class
struct mpc_result SolveMPC2(Eigen::Matrix<double, 2, 1> previous_vel, lgmath::se3::Transformation T0, std::vector<lgmath::se3::Transformation> measurements, std::vector<lgmath::se3::Transformation> measurements_cbit, std::vector<double> barrier_q_left, std::vector<double> barrier_q_right, int K, double DT, double VF, Eigen::Matrix<double, 1, 1> lat_noise_vect, Eigen::Matrix<double, 6, 6> pose_noise_vect, Eigen::Matrix<double, 2, 2> vel_noise_vect, Eigen::Matrix<double, 2, 2> accel_noise_vect, Eigen::Matrix<double, 6, 6> kin_noise_vect, bool point_stabilization, double pose_error_weight, double vel_error_weight, double acc_error_weight, double kin_error_weight, double lat_error_weight)
{
    
    // Pre-process the measurements to correct the cbit path yaw values depending on if the robot should be moving in forward or reverse
    //Eigen::Matrix<double, 4, 4> T_ROT_YAW;
    //T_ROT_YAW << -1, 0, 0, 0,
    //            0, -1, 0, 0,
    //            0, 0, 1, 0,
    //            0, 0, 0, 1;
    //lgmath::se3::Transformation T_ROT_YAW2(T_ROT_YAW);
    //CLOG(ERROR, "mpc_debug.cbit") << "THE PI ROTATION MATRIX IS: " << T_ROT_YAW2;

    //for (int i = 0; i < measurements_cbit.size(); i++)
    //{
    //  auto test1 = measurements[i].matrix().block(0, 0, 3, 3);
    //  auto test2 = measurements_cbit[i].matrix().block<3, 1>(0, 3);
    //  CLOG(ERROR, "mpc_debug.cbit") << "TEST2 IS: " << test2;
    //  auto cbit_pose = measurements_cbit[i].matrix().block(0, 0, 3, 3);
    //  auto teach_pose = measurements[i].matrix().block(0, 0, 3, 3);
    //  CLOG(ERROR, "mpc_debug.cbit") << "THE CBIT MATRIX IS: " << measurements_cbit[i];
    //  CLOG(ERROR, "mpc_debug.cbit") << "THE TEACH MATRIX IS: " << measurements[i];
      //measurements_cbit[i] = measurements[i];
    //  lgmath::se3::Transformation new_meas_cbit(test1,test2);
    //  measurements_cbit[i] = new_meas_cbit;

    //  CLOG(ERROR, "mpc_debug.cbit") << "THE ROTATED CBIT MATRIX IS: " << measurements_cbit[i];
      //auto rel_pose = teach_pose.transpose() * cbit_pose;
      //CLOG(ERROR, "mpc_debug.cbit") << "THE CBIT ROTATION MATRIX IS: " << cbit_pose;
      //CLOG(ERROR, "mpc_debug.cbit") << "THE TEACH ROTATION MATRIX IS: " << teach_pose;
      //CLOG(ERROR, "mpc_debug.cbit") << "THE RELATIVE ROTATION MATRIX IS: " << teach_pose.transpose() * cbit_pose;
      //double relative_yaw = atan2(rel_pose(1,0),rel_pose(0,0));
      //CLOG(ERROR, "mpc_debug.cbit") << "THE RELATIVE YAW IS: " << relative_yaw;
      //if (fabs(relative_yaw) >= 1.57075)
      //{
      //  measurements_cbit[i] = measurements_cbit[i] * T_ROT_YAW2;
      //  CLOG(ERROR, "mpc_debug.cbit") << "CORRECTED THE YAW";
      //}
      //CLOG(ERROR, "mpc_debug.cbit") << "THE ROTATED CBIT MATRIX IS: " << measurements_cbit[i];
    //}
    // End of pre-pre-processing measurement rotations



    // Conduct an MPC Iteration given current configurations

    // Velocity set-points (desired forward velocity and angular velocity), here we set a static forward target velocity, and try to minimize rotations (0rad/sec)
    Eigen::Matrix<double, 2, 1> v_ref;
    v_ref << VF,
                0;


    // Kinematic projection Matrix for Unicycle Model (note its -1's because our varpi lie algebra vector is of a weird frame)
    // TODO, make the choice of projection matrix a configurable param for the desired vehicle model.
    Eigen::Matrix<double, 6, 2> P_tran;
    P_tran << -1, 0,
                0, 0,
                0, 0,
                0, 0,
                0, 0,
                0, -1;

    // Lateral constraint projection matrices (Experimental)
    Eigen::Matrix<double, 3, 1> I_4; // In steam, the homopoint vars automatically add the 4th row 1, so representing I_4 just needs the 3 zeros
    I_4 << 0,
           0,
           0;


    // Setup shared loss functions and noise models for all cost terms
    //const auto sharedLossFunc = steam::L2LossFunc::MakeShared(); // The default L2 loss function weights all cost terms with a value of 1.0 (not using this one anymore)

    // The custom L2WeightedLossFunc allows you to dynamically set the weights of cost terms by providing the value as an argument
    const auto poseLossFunc = steam::L2WeightedLossFunc::MakeShared(pose_error_weight);
    const auto velLossFunc = steam::L2WeightedLossFunc::MakeShared(vel_error_weight);
    const auto accelLossFunc = steam::L2WeightedLossFunc::MakeShared(acc_error_weight);
    const auto kinLossFunc = steam::L2WeightedLossFunc::MakeShared(kin_error_weight);
    const auto latLossFunc = steam::L2WeightedLossFunc::MakeShared(lat_error_weight); 

    // Cost term Noise Covariance Initialization
    const auto sharedPoseNoiseModel = steam::StaticNoiseModel<6>::MakeShared(pose_noise_vect);
    const auto sharedVelNoiseModel = steam::StaticNoiseModel<2>::MakeShared(vel_noise_vect);
    const auto sharedAccelNoiseModel = steam::StaticNoiseModel<2>::MakeShared(accel_noise_vect);
    const auto sharedKinNoiseModel = steam::StaticNoiseModel<6>::MakeShared(kin_noise_vect);
    const auto sharedLatNoiseModel = steam::StaticNoiseModel<1>::MakeShared(lat_noise_vect);



    // Generate STEAM States for the velocity vector and SE3 state transforms
    std::vector<lgmath::se3::Transformation> pose_states;
    std::vector<Eigen::Matrix<double,2,1>> vel_states;

    // Invert the extrapolated robot state and use this as the state initialization
    lgmath::se3::Transformation T0_inv = T0.inverse();
    Eigen::Vector2d v0(0.0, 0.0);

    // Push back the initial states (current robot state)
    pose_states.push_back(T0_inv);
    vel_states.push_back(v0);

    //CLOG(DEBUG, "mpc_debug.cbit") << "Verifying that the initial robot state is about the same as the initial reference measurement";
    //CLOG(DEBUG, "mpc_debug.cbit") << "T0_inv: " << T0_inv;
    //CLOG(ERROR, "mpc_debug.cbit") << "1st meas: " << measurements_cbit[0];

    // Set the remaining states using a warm start from the cbit solution
    for (int i=1; i<K; i++)
    {
        //pose_states.push_back(lgmath::se3::Transformation());
        //pose_states.push_back(T0_inv); // old initialization - set all initial states to that of the current robot state
        pose_states.push_back(measurements_cbit[i]); // New initialization - use the reference measurements from the cbit solution as our initialization - the first one is the same as our initial state
        vel_states.push_back(v0);
    }

    // Create Steam states
    std::vector<steam::se3::SE3StateVar::Ptr> pose_state_vars;
    std::vector<steam::vspace::VSpaceStateVar<2>::Ptr> vel_state_vars;
    std::vector<steam::se3::SE3StateVar::Ptr> measurement_vars; // This one is for storing measurements as locked evaluators for barrier constraints
    
    // Create a locked state var for the 4th column of the identity matrix (used in state constraint)
    steam::stereo::HomoPointStateVar::Ptr I_4_eval = steam::stereo::HomoPointStateVar::MakeShared(I_4);
    I_4_eval->locked() = true;

    // Create Steam variables
    for (int i = 0; i < K; i++)
    {
        pose_state_vars.push_back(steam::se3::SE3StateVar::MakeShared(pose_states[i])); 
        vel_state_vars.push_back(steam::vspace::VSpaceStateVar<2>::MakeShared(vel_states[i])); 
    }

    // Lock the first (current robot) state from being modified during the optimization
    pose_state_vars[0]->locked() = true;



    // Setup the optimization problem
    steam::OptimizationProblem opt_problem;
    for (int i=1; i<K; i++) // start at 1 so as to not add the locked state variable
    {
        opt_problem.addStateVariable(pose_state_vars[i]);
    }

    // The velocity states should have one less variable then the pose states
    for (int i=0; i<K-1; i++)
    {
        opt_problem.addStateVariable(vel_state_vars[i]);
    }

    // Generate the cost terms using combinations of the built-in steam evaluators
    for (int i = 0; i < K; i++)
    {
      // Pose Error
      if (i > 0)
      {
        const auto pose_error_func = steam::se3::SE3ErrorEvaluator::MakeShared(pose_state_vars[i], measurements[i]);
        const auto pose_cost_term = steam::WeightedLeastSqCostTerm<6>::MakeShared(pose_error_func, sharedPoseNoiseModel, poseLossFunc);
        opt_problem.addCostTerm(pose_cost_term);
      }

      // Kinematic constraints (softened but penalized heavily)
      if (i < (K-1))
      {
        const auto lhs = steam::se3::ComposeInverseEvaluator::MakeShared(pose_state_vars[i+1], pose_state_vars[i]);
        const auto vel_proj = steam::vspace::MatrixMultEvaluator<6,2>::MakeShared(vel_state_vars[i], P_tran); // TODO, I guess this version of steam doesnt have this one, will need to do it myself
        const auto scaled_vel_proj = steam::vspace::ScalarMultEvaluator<6>::MakeShared(vel_proj, DT);
        const auto rhs = steam::se3::ExpMapEvaluator::MakeShared(scaled_vel_proj);
        const auto kin_error_func = steam::se3::LogMapEvaluator::MakeShared(steam::se3::ComposeInverseEvaluator::MakeShared(lhs, rhs));
        const auto kin_cost_term = steam::WeightedLeastSqCostTerm<6>::MakeShared(kin_error_func, sharedKinNoiseModel, kinLossFunc);
        opt_problem.addCostTerm(kin_cost_term);

        // Non-Zero Velocity Penalty (penalty of non resting control effort helps with point stabilization)
        const auto vel_cost_term = steam::WeightedLeastSqCostTerm<2>::MakeShared(vel_state_vars[i], sharedVelNoiseModel, velLossFunc);
        opt_problem.addCostTerm(vel_cost_term);

        // Velocity set-point constraint - No longer using this due to complications when repeating a path in reverse
        // Only add this cost term if we are not in point stabilization mode (end of path)
        
        //if (point_stabilization == false)
        //{
        //  const auto vel_cost_term = steam::WeightedLeastSqCostTerm<2>::MakeShared(steam::vspace::VSpaceErrorEvaluator<2>::MakeShared(vel_state_vars[i],v_ref), sharedVelNoiseModel, velLossFunc);
        //  opt_problem.addCostTerm(vel_cost_term);
        //}
        
        
        // Acceleration Constraints
        if (i == 0)
        {
        // On the first iteration, we need to use an error with the previously applied control command state
        const auto accel_cost_term = steam::WeightedLeastSqCostTerm<2>::MakeShared(steam::vspace::VSpaceErrorEvaluator<2>::MakeShared(vel_state_vars[i], previous_vel), sharedAccelNoiseModel, accelLossFunc);
        opt_problem.addCostTerm(accel_cost_term);
        } 
        else
        {
        // Subsequent iterations we make an error between consecutive velocities. We penalize large changes in velocity between time steps
        const auto accel_diff = steam::vspace::AdditionEvaluator<2>::MakeShared(vel_state_vars[i], steam::vspace::NegationEvaluator<2>::MakeShared(vel_state_vars[i-1]));
        const auto accel_cost_term = steam::WeightedLeastSqCostTerm<2>::MakeShared(accel_diff, sharedAccelNoiseModel, accelLossFunc);
        opt_problem.addCostTerm(accel_cost_term);
        }   
        
      }

      // Laterial Barrier State Constraints
      if (i >= 0)
      {
        // Generate a locked transform evaluator to store the current measurement for state constraints
        // The reason we make it a variable and lock it is so we can use the built in steam evaluators which require evaluable inputs
        measurement_vars.push_back(steam::se3::SE3StateVar::MakeShared(measurements[i]));
        measurement_vars[i]->locked() = true;

        // Take the compose inverse of the locked measurement w.r.t the state transforms
        const auto compose_inv = steam::se3::ComposeInverseEvaluator::MakeShared(measurement_vars[i], pose_state_vars[i]);

        // Use the ComposeLandmarkEvaluator to right multiply the 4th column of the identity matrix to create a 4x1 homogenous point vector with lat,lon,alt error components
        const auto error_vec = steam::stereo::ComposeLandmarkEvaluator::MakeShared(compose_inv, I_4_eval);

        // Build lateral barrier terms by querying the current cbit corridor
        Eigen::Matrix<double, 4, 1> barrier_right;
        barrier_right <<  0,
                          barrier_q_right[i],
                          0,
                          1;
        Eigen::Matrix<double, 4, 1> barrier_left;
        barrier_left <<   0.0,
                          barrier_q_left[i],
                          0,
                          1;

        //CLOG(INFO, "mpc_debug.cbit") << "Left Barrier for this meas is: " << barrier_q_left[i];
        //CLOG(INFO, "mpc_debug.cbit") << "Right Barrier for tis meas is: " << barrier_q_right[i];
        //CLOG(INFO, "mpc_debug.cbit") << "The Initaial Pose is:" << T0;
        //CLOG(INFO, "mpc_debug.cbit") << "The cbit measurement for this value is: " << measurements_cbit[i].inverse();
        //CLOG(INFO, "mpc_debug.cbit") << "The vtr measurement for this value is: " << measurements[i].inverse();

        // compute the lateral error using a custom Homogenous point error STEAM evaluator
        const auto lat_error_right = steam::LateralErrorEvaluatorRight::MakeShared(error_vec, barrier_right); // TODO, rename this evaluator to something else
        const auto lat_error_left = steam::LateralErrorEvaluatorLeft::MakeShared(error_vec, barrier_left);

        // Previously used Log barriers, however due to instability switch to using inverse squared barriers
        //const auto lat_barrier_right = steam::vspace::ScalarLogBarrierEvaluator<1>::MakeShared(lat_error_right);
        //const auto lat_barrier_left = steam::vspace::ScalarLogBarrierEvaluator<1>::MakeShared(lat_error_left);

        // For each side of the barrier, compute a scalar inverse barrier term to penalize being close to the bound
        const auto lat_barrier_right = steam::vspace::ScalarInverseBarrierEvaluator<1>::MakeShared(lat_error_right);
        const auto lat_barrier_left = steam::vspace::ScalarInverseBarrierEvaluator<1>::MakeShared(lat_error_left);

        // Generate least square cost terms and add them to the optimization problem
        const auto lat_cost_term_right = steam::WeightedLeastSqCostTerm<1>::MakeShared(lat_barrier_right, sharedLatNoiseModel, latLossFunc);
        opt_problem.addCostTerm(lat_cost_term_right);
        const auto lat_cost_term_left = steam::WeightedLeastSqCostTerm<1>::MakeShared(lat_barrier_left, sharedLatNoiseModel, latLossFunc);
        opt_problem.addCostTerm(lat_cost_term_left);
      
      }
    }

    // Solve the optimization problem with GuassNewton solver
    //using SolverType = steam::GaussNewtonSolver; // Old solver, does not have back stepping capability
    using SolverType = steam::LineSearchGaussNewtonSolver;
    //using SolverType = steam::DoglegGaussNewtonSolver;
    // Initialize solver parameters
    SolverType::Params params;
    params.verbose = true; // Makes the output display for debug when true
    params.relative_cost_change_threshold = 1e-6;
    params.max_iterations = 100;
    params.absolute_cost_change_threshold = 1e-6;
    params.backtrack_multiplier = 0.5; // Line Search Specific Params, will fail to build if using GaussNewtonSolver
    params.max_backtrack_steps = 200; // Line Search Specific Params, will fail to build if using GaussNewtonSolver
    //params.ratio_threshold_shrink = 0.1; // Dogleg Specific Params, will fail to build if using GaussNewtonSolver
    /// Grow trust region if ratio of actual to predicted cost reduction above
    /// this (range: 0.0-1.0)
    //params.ratio_threshold_grow = 0.9; // Dogleg Specific Params, will fail to build if using GaussNewtonSolver
    /// Amount to shrink by (range: <1.0)
    //params.shrink_coeff = 0.9; // Dogleg Specific Params, will fail to build if using GaussNewtonSolver
    /// Amount to grow by (range: >1.0)
    //params.grow_coeff = 1.1; // Dogleg Specific Params, will fail to build if using GaussNewtonSolver
    /// Maximum number of times to shrink trust region before giving up
    //params.max_shrink_steps = 200; // Dogleg Specific Params, will fail to build if using GaussNewtonSolver

    SolverType solver(opt_problem, params);

    // Solve the optimization problem
    solver.optimize();

    // DEBUG: Display the several of the prediction horizon results
    /*
    CLOG(DEBUG, "mpc_debug.cbit") << "Trying to Display the Optimization Results for the isolated MPC";
    CLOG(DEBUG, "mpc_debug.cbit") << "The First State is: " << pose_state_vars[0]->value().inverse();
    CLOG(DEBUG, "mpc_debug.cbit") << "The Second State is: " << pose_state_vars[1]->value().inverse();
    CLOG(DEBUG, "mpc_debug.cbit") << "The Third State is: " << pose_state_vars[2]->value().inverse();
    CLOG(DEBUG, "mpc_debug.cbit") << "The Forth State is: " << pose_state_vars[3]->value().inverse();
    CLOG(DEBUG, "mpc_debug.cbit") << "The Fifth State is: " << pose_state_vars[4]->value().inverse();
    CLOG(DEBUG, "mpc_debug.cbit") << "The Sixth State is: " << pose_state_vars[5]->value().inverse();
    CLOG(DEBUG, "mpc_debug.cbit") << "The Seventh State is: " << pose_state_vars[6]->value().inverse();
    CLOG(DEBUG, "mpc_debug.cbit") << "The Eighth State is: " << pose_state_vars[7]->value().inverse();
    CLOG(DEBUG, "mpc_debug.cbit") << "The Ninth State is: " << pose_state_vars[8]->value().inverse();
    CLOG(DEBUG, "mpc_debug.cbit") << "The Tenth State is: " << pose_state_vars[9]->value().inverse();
    CLOG(DEBUG, "mpc_debug.cbit") << "The First Velocity is: " << vel_state_vars[0]->value();
    CLOG(DEBUG, "mpc_debug.cbit") << "The Second Velocity is: " << vel_state_vars[1]->value();
    CLOG(DEBUG, "mpc_debug.cbit") << "The Third Velocity is: " << vel_state_vars[2]->value();
    CLOG(DEBUG, "mpc_debug.cbit") << "The Forth Velocity is: " << vel_state_vars[3]->value();
    CLOG(DEBUG, "mpc_debug.cbit") << "The Fifth Velocity is: " << vel_state_vars[4]->value();
    CLOG(DEBUG, "mpc_debug.cbit") << "The Sixth Velocity is: " << vel_state_vars[5]->value();
    CLOG(DEBUG, "mpc_debug.cbit") << "The Seventh Velocity is: " << vel_state_vars[6]->value();
    CLOG(DEBUG, "mpc_debug.cbit") << "The Eighth Velocity is: " << vel_state_vars[7]->value();
    CLOG(DEBUG, "mpc_debug.cbit") << "The Ninth Velocity is: " << vel_state_vars[8]->value();
    CLOG(DEBUG, "mpc_debug.cbit") << "The Tenth Velocity is: " << vel_state_vars[9]->value();
    

    CLOG(DEBUG, "mpc_debug.cbit") << "Linear Component to Return is: " << (vel_state_vars[0]->value())[0];
    CLOG(DEBUG, "mpc_debug.cbit") << "Angular Component to Return is: " << (vel_state_vars[0]->value())[1];
    */

    // Store the velocity command to apply
    Eigen::Matrix<double, 2, 1> applied_vel = vel_state_vars[0]->value();

    // First check if any of the values are nan, if so we return a zero velocity and flag the error
    Eigen::Matrix<double, 2, 1> nan_vel;
    if (std::isnan(applied_vel(0)) || std::isnan(applied_vel(1)))
    {
      CLOG(ERROR, "mpc.cbit") << "NAN values detected, mpc optimization failed. Returning zero velocities";
      nan_vel(0) = 0.0;
      nan_vel(1) = 0.0;

      // if we do detect nans, return the mpc_poses as all being the robots current pose (not moving across the horizon as we should be stopped)
      std::vector<lgmath::se3::Transformation> mpc_poses;
      for (int i = 0; i<pose_state_vars.size(); i++)
      {
        mpc_poses.push_back(T0);
      }
      return {nan_vel, mpc_poses};
    }

    // if no nan values, return the applied velocity and mpc pose predictions as normal
    else
    {
    // Store the sequence of resulting mpc prediction horizon poses for visualization
    std::vector<lgmath::se3::Transformation> mpc_poses;
    for (int i = 0; i<pose_state_vars.size(); i++)
    {
      mpc_poses.push_back(pose_state_vars[i]->value().inverse());
    }

    
    // Return the resulting structure
    return {applied_vel, mpc_poses};
    }
}



// helper function for computing the optimization reference poses T_ref based on the current path solution
// This is specifically for the tracking mpc, but is also used to generate the warm start poses for the corridor mpc
struct meas_result GenerateReferenceMeas2(std::shared_ptr<std::vector<Pose>> cbit_path_ptr, std::tuple<double, double, double, double, double, double> robot_pose, int K, double DT, double VF)
{


    // Save a copy of the current path solution to work on
    auto cbit_path = *cbit_path_ptr;

    // PSEUDO CODE:
    // 1. Find the closest point on the cbit path to the current state of the robot
    // 2. Using K, DT, VF, we generate a vector of "p" values that we want to create Euclidean measurements for (we know these up front)
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

    std::vector<double> cbit_p;
    cbit_p.reserve(cbit_path.size());
    cbit_p.push_back(0.0);
    for (int i = 0; i < (cbit_path.size()-2); i++) // the last value of vector is size()-1, so second to last will be size-2
    { 
      // calculate the p value for the point
      p_dist = sqrt((((cbit_path)[i].x - (cbit_path)[i+1].x) * ((cbit_path)[i].x - (cbit_path)[i+1].x)) + (((cbit_path)[i].y - (cbit_path)[i+1].y) * ((cbit_path)[i].y - (cbit_path)[i+1].y)));
      lookahead_dist = lookahead_dist + p_dist;
      cbit_p.push_back(lookahead_dist);

      // Keep track of the closest point to the robot state
      dx = (cbit_path)[i].x - std::get<0>(robot_pose);
      dy = (cbit_path)[i].y - std::get<1>(robot_pose);
      new_dist = sqrt((dx * dx) + (dy * dy));
      if (new_dist < min_dist)
      {
        CLOG(DEBUG, "mpc_debug.cbit") << "Minimum Distance: " << new_dist;
        p_correction = lookahead_dist;
        min_dist = new_dist;
      }
      
      // Stop once we get about 12m ahead of the robot (magic number for now, but this is just a conservative estimate of any reasonable lookahead window and mpc horizon)
      if (lookahead_dist > 12.0)
      {
        break;
      }
    }
    //CLOG(DEBUG, "mpc_debug.cbit") << "cbit_p is: " << cbit_p;

    // Determine the p_values we need for our measurement horizon, corrected for the p value of the closest point on the path to the current robot state
    std::vector<double> p_meas_vec;
    std::vector<lgmath::se3::Transformation> measurements;
    std::vector<double> p_interp_vec;
    std::vector<double> q_interp_vec;

    p_meas_vec.reserve(K);
    for (int i = 0; i < K; i++)
    {

      p_meas_vec.push_back((i * DT * VF) + p_correction);
    }
    //CLOG(DEBUG, "mpc_debug.cbit") << "p_meas_vec is: " << p_meas_vec;
    
    // Iterate through the p_measurements and interpolate euclidean measurements from the cbit_path and the corresponding cbit_p values
    // Note this could probably just be combined in the previous loop too
    bool point_stabilization = false;
    for (int i = 0; i < p_meas_vec.size(); i++)
    {
      // handle end of path case:
      // if the p meas we would have needed exceeds the final measurement pose, set it equal to our last p value in the path
      // This will cause the intepolation to return the final cbit_path pose for all measurements past this point
      //CLOG(INFO, "mpc_debug.cbit") << "The specific p_meas_vec[i] is: " << p_meas_vec[i];
      //CLOG(INFO, "mpc_debug.cbit") << "The size of cbit_p is:" << cbit_p.size();
      //CLOG(INFO, "mpc_debug.cbit") << "The final cbit_p value is:" << cbit_p[cbit_p.size()-1];

      if (p_meas_vec[i] > cbit_p[cbit_p.size()-1])
      {
        p_meas_vec[i] = cbit_p[cbit_p.size()-1];
        point_stabilization = true; // Enable point stabilization configs
        CLOG(INFO, "mpc.cbit") << "Approaching End of Path, Converting MPC to Point Stabilization Problem";
      }
      struct interp_result meas = InterpolateMeas2(p_meas_vec[i], cbit_p, cbit_path);

      // add to measurement vector
      measurements.push_back(meas.measurement);
      p_interp_vec.push_back(meas.p_interp);
      q_interp_vec.push_back(meas.q_interp);

    }

    //CLOG(WARNING, "mpc.cbit") << "The Approximate Robot State P value is (ref meas): " << p_correction;
    //CLOG(WARNING, "mpc.cbit") << "The p_interp_vec is: " << p_interp_vec;
    //CLOG(WARNING, "mpc.cbit") << "The q_interp_vec is: " << q_interp_vec;

    return {measurements, point_stabilization, p_interp_vec, q_interp_vec};
}

// For generating VT&R teach path measurements used in the corridor mpc
struct meas_result3 GenerateReferenceMeas3(std::shared_ptr<CBITPath> global_path_ptr, std::shared_ptr<CBITCorridor> corridor_ptr, std::tuple<double, double, double, double, double, double> robot_pose, int K, double DT, double VF, int current_sid)
{
    // Initialize vectors storing the barrier values:
    std::vector<double> barrier_q_left;
    std::vector<double> barrier_q_right;

    // load the teach path
    std::vector<Pose> teach_path = global_path_ptr->disc_path;
    
    //auto teach_p = global_path_ptr->p;
    std::vector<Pose> cbit_path;

    // Get rid of any of the path before the current sid
    // handle case at start of path 
    if (current_sid - 1 < 0)
    {
      current_sid = 1;
    }

    // Store the global p value of the previous sid. This is the point I use as my zero reference for the remaining p's
    double sid_p = global_path_ptr->p[current_sid-1];
    //CLOG(DEBUG, "mpc_debug.cbit") << "The global reference p is: " << sid_p;

    // I use sid -1 to be conservative, because I think its possible the robot pose is being localized in the frame ahead of the robot
    //CLOG(WARNING, "corridor_mpc_debug.cbit") << "The size of the teach_path is: " << teach_path.size();
    for (int i = (current_sid-1); i <= teach_path.size(); i++)
    {
      cbit_path.push_back(teach_path[i]);
    }

    //DEBUG:
    //CLOG(WARNING, "mpc_debug.cbit") << "The robot is at: x: " << std::get<0>(robot_pose) << " y: " << std::get<1>(robot_pose);
    //CLOG(WARNING, "mpc_debug.cbit") << "Teach Path 0 x: " << teach_path[0].x << " y: " << teach_path[0].y;
    //CLOG(WARNING, "mpc_debug.cbit") << "Teach Path 1 x: " << teach_path[1].x << " y: " << teach_path[1].y;
    //CLOG(WARNING, "mpc_debug.cbit") << "Teach Path 2 x: " << teach_path[2].x << " y: " << teach_path[2].y;
    //CLOG(WARNING, "mpc_debug.cbit") << "Teach Path 3 x: " << teach_path[3].x << " y: " << teach_path[3].y;
    //CLOG(WARNING, "mpc_debug.cbit") << "P[0] is: " << global_path_ptr->p[0];
    //CLOG(WARNING, "mpc_debug.cbit") << "P[1] is: " << global_path_ptr->p[1];
    //CLOG(WARNING, "mpc_debug.cbit") << "P[2] is: " << global_path_ptr->p[2];
    //CLOG(WARNING, "mpc_debug.cbit") << "P[3] is: " << global_path_ptr->p[3];


    // Efficient way of calculating the current p value of the robot w.r.t the teach path
    double x2;
    double x1;
    double y2;
    double y1;
    double p_val_ref;

    if (current_sid == 0)
    {
      x2 = teach_path[1].x;
      x1 = teach_path[0].x;
      y2 = teach_path[1].y;
      y1 =teach_path[0].x;
      p_val_ref = global_path_ptr->p[0];
    }
    else
    {
      x2 = teach_path[current_sid+1].x;
      x1 = teach_path[current_sid-1].x;
      y2 = teach_path[current_sid+1].y;
      y1 = teach_path[current_sid-1].y;
      p_val_ref = global_path_ptr->p[current_sid-1];
    }
    double xp = std::get<0>(robot_pose);
    double yp = std::get<1>(robot_pose);
    double total_dist = sqrt(((x2-x1) * (x2-x1)) + ((y2-y1) * (y2-y1)));

    double t = (((xp-x1)*(x2-x1)) + ((yp-y1)*(y2-y1))) / total_dist;
    double p_robot = t;

    // Check if p is negative, (robot starting behind path, if so, make it 0.0, otherwise this will cause optimization problems)
    if (p_robot < 0.0)
    {
      p_robot = 0.0;
    }

    CLOG(WARNING, "mpc_debug.cbit") << "The p value for the robot is: " << p_robot + sid_p;

    // PSEUDO CODE:
    // 1. Find the closest point on the cbit path to the current state of the robot
    // 2. Using K, DT, VF, we generate a vector of "p" values that we want to create Euclidean measurements for (we know these up front)
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

    std::vector<double> cbit_p;
    cbit_p.reserve(cbit_path.size());
    cbit_p.push_back(0.0);
    for (int i = 0; i < (cbit_path.size()-2); i++) // the last value of vector is size()-1, so second to last will be size-2
    { 
      // calculate the p value for the point
      p_dist = sqrt((((cbit_path)[i].x - (cbit_path)[i+1].x) * ((cbit_path)[i].x - (cbit_path)[i+1].x)) + (((cbit_path)[i].y - (cbit_path)[i+1].y) * ((cbit_path)[i].y - (cbit_path)[i+1].y)));
      lookahead_dist = lookahead_dist + p_dist;
      cbit_p.push_back(lookahead_dist);

      // Keep track of the closest point to the robot state
      dx = (cbit_path)[i].x - std::get<0>(robot_pose);
      dy = (cbit_path)[i].y - std::get<1>(robot_pose);
      new_dist = sqrt((dx * dx) + (dy * dy));
      if (new_dist < min_dist)
      {
        //CLOG(DEBUG, "mpc_debug.cbit") << "Minimum Distance: " << new_dist;
        p_correction = lookahead_dist;
        min_dist = new_dist;
      }
      
      // Stop once we get about 12m ahead of the robot (magic number for now, but this is just a conservative estimate of any reasonable lookahead window and mpc horizon)
      if (lookahead_dist > 12.0)
      {
        break;
      }
    }
    //CLOG(DEBUG, "mpc_debug.cbit") << "cbit_p is: " << cbit_p;
    // Determine the p_values we need for our measurement horizon, corrected for the p value of the closest point on the path to the current robot state
    std::vector<double> p_meas_vec;
    std::vector<lgmath::se3::Transformation> measurements;
    p_meas_vec.reserve(K);
    for (int i = 0; i < K; i++)
    {
      p_meas_vec.push_back((i * DT * VF) + p_robot);
    }

    // Iterate through the p_measurements and interpolate euclidean measurements from the cbit_path and the corresponding cbit_p values
    // Note this could probably be combined in the previous loop too
    bool point_stabilization = false;
    for (int i = 0; i < p_meas_vec.size(); i++)
    {
      // handle end of path case:
      // if the p meas we would have needed exceeds the final measurement pose, set it equal to our last p value in the path
      // This will cause the intepolation to return the final cbit_path pose for all measurements past this point
      //CLOG(INFO, "mpc_debug.cbit") << "The specific p_meas_vec[i] is: " << p_meas_vec[i];
      //CLOG(INFO, "mpc_debug.cbit") << "The size of cbit_p is:" << cbit_p.size();
      //CLOG(INFO, "mpc_debug.cbit") << "The final cbit_p value is:" << cbit_p[cbit_p.size()-1];

      if (p_meas_vec[i] > cbit_p[cbit_p.size()-1])
      {
        p_meas_vec[i] = cbit_p[cbit_p.size()-1];
        point_stabilization = true; // Enable point stabilization configs
        CLOG(INFO, "mpc.cbit") << "Approaching End of Path, Converting MPC to Point Stabilization Problem";
      }

      struct interp_result meas = InterpolateMeas2(p_meas_vec[i], cbit_p, cbit_path);
      //CLOG(WARNING, "corridor_mpc_debug.cbit") << "Adding Measurement: " << meas;

      // add to measurement vector
      measurements.push_back(meas.measurement);


      // Find the corresponding left and right barrier q values to pass to the mpc

      // The corridor_ptr points to the stored barrier values for the entire teach trajectory (0,p_len)
      // To find the corresponding values, we just need to query the corridor_ptr given the current sid_p + p_meas_vec[i], and return the q values for that bin
      double p_query = sid_p + p_meas_vec[i];
      // this isnt the most efficient way of doing this, but it should be fine, we really only need to run this loop 10-20 times and the size is likely less then 1000 each
      int p_ind = 0;
      while (corridor_ptr->p_bins[p_ind] <= p_query)
      {
        p_ind++;
      }
      barrier_q_left.push_back(corridor_ptr->q_left[p_ind-1]);
      barrier_q_right.push_back(corridor_ptr->q_right[p_ind-1]);
      //CLOG(WARNING, "mpc_debug.cbit") << "The left barrier is: " << corridor_ptr->q_left[p_ind-1];
      //CLOG(WARNING, "mpc_debug.cbit") << "The right barrier is: " << corridor_ptr->q_right[p_ind-1];
    }

    return {measurements, point_stabilization, barrier_q_left, barrier_q_right};
}

// For generating VT&R teach path measurements used in the corridor mpc (new version which directly uses the interpolated p measurements from the cbit path trajectory tracking)
struct meas_result3 GenerateReferenceMeas4(std::shared_ptr<CBITPath> global_path_ptr, std::shared_ptr<CBITCorridor> corridor_ptr, std::tuple<double, double, double, double, double, double> robot_pose, int K, double DT, double VF, int current_sid, std::vector<double> p_interp_vec)
{
    // Set point stabilization, but just note if we use this function in the cbit.cpp file we need to use the Tracking reference pose point stabilization instead
    bool point_stabilization = false;

    // Initialize vectors storing the barrier values:
    std::vector<double> barrier_q_left;
    std::vector<double> barrier_q_right;
    
    // load the teach path
    std::vector<Pose> teach_path = global_path_ptr->disc_path;
    std::vector<double> teach_path_p = global_path_ptr->p;
    

    std::vector<lgmath::se3::Transformation> measurements;
    
    //CLOG(ERROR, "mpc_debug.cbit") << "The p_interp_vec is: " << p_interp_vec;
    //CLOG(ERROR, "mpc_debug.cbit") << "The Global Path p is: " << teach_path_p;

    // Iterate through the interpolated p_measurements and make interpolate euclidean measurements from the teach path
    for (int i = 0; i < p_interp_vec.size(); i++)
    {

      struct interp_result meas = InterpolateMeas2(p_interp_vec[i], teach_path_p, teach_path);
      //CLOG(WARNING, "corridor_mpc_debug.cbit") << "Adding Measurement: " << meas;

      // add to measurement vector
      measurements.push_back(meas.measurement);


      // Find the corresponding left and right barrier q values to pass to the mpc

      // The corridor_ptr points to the stored barrier values for the entire teach trajectory (0,p_len)
      // To find the corresponding values, we just need to query the corridor_ptr given the current sid_p + p_meas_vec[i], and return the q values for that bin
      double p_query = p_interp_vec[i];
      // this isnt the most efficient way of doing this, but it should be fine, we really only need to run this loop 10-20 times and the size is likely less then 1000 each
      int p_ind = 0;
      while (corridor_ptr->p_bins[p_ind] <= p_query)
      {
        p_ind++;
      }
      barrier_q_left.push_back(corridor_ptr->q_left[p_ind-1]);
      barrier_q_right.push_back(corridor_ptr->q_right[p_ind-1]);
      //CLOG(WARNING, "mpc_debug.cbit") << "The left barrier is: " << corridor_ptr->q_left[p_ind-1];
      //CLOG(WARNING, "mpc_debug.cbit") << "The right barrier is: " << corridor_ptr->q_right[p_ind-1];
    }

    return {measurements, point_stabilization, barrier_q_left, barrier_q_right};
}

// function takes in the cbit path solution with a vector defining the p axis of the path, and then a desired p_meas
// Then tries to output a euclidean pose interpolated for the desired p_meas.
struct interp_result InterpolateMeas2(double p_val, std::vector<double> cbit_p, std::vector<Pose> cbit_path)
{
  // Find the lower bound of the p values
  for (int i = 0; i < cbit_p.size(); i++)
  {
    if (cbit_p[i] < p_val)
    {
      continue;
    }
    else
    {
      double p_lower = cbit_p[i-1];
      double p_upper = cbit_p[i];
      Pose pose_lower = cbit_path[i-1];
      Pose pose_upper = cbit_path[i];

    
      double x_int = pose_lower.x + ((p_val - p_lower) / (p_upper - p_lower)) * (pose_upper.x - pose_lower.x);
      double y_int = pose_lower.y + ((p_val - p_lower) / (p_upper - p_lower)) * (pose_upper.y - pose_lower.y);
      double z_int = pose_lower.z + ((p_val - p_lower) / (p_upper - p_lower)) * (pose_upper.z - pose_lower.z);

      // For yaw we need to be abit careful about sign and angle wrap around
      // Derive the yaw by creating the vector connecting the pose_upp and pose_lower pts
      // TODO: There is a problem here for reverse planning, will need to rotate the yaw 180 degrees in that case.
      // For normal forward planning this is fine though

      // This interpolation if we do have yaw available (when the input path is the teach path as it is for corridor mpc)
      double yaw_int;
      //yaw_int = std::atan2((pose_upper.y - pose_lower.y), (pose_upper.x - pose_lower.x));
      if ((pose_lower.yaw == 0.0) && (pose_upper.yaw == 0.0))
      {
        // Yaw interpolation when we dont have yaw available explicitly (i.e from cbit path euclid conversion)
        yaw_int = std::atan2((pose_upper.y - pose_lower.y), (pose_upper.x - pose_lower.x));
        //CLOG(ERROR, "mpc_debug.cbit") << "The Yaw For CBIT PATH is: " << yaw_int;
      }
      else
      {
        // if this is used on the cbit path, it will just return 0.0 every time (note this will break tracking control if yaw covariance set low)
        //yaw_int = pose_lower.yaw + ((p_val - p_lower) / (p_upper - p_lower)) * (pose_upper.yaw - pose_lower.yaw);
        yaw_int = pose_lower.yaw;
        //CLOG(ERROR, "mpc_debug.cbit") << "The Yaw For TEACH PATH is: " << yaw_int;
      }
      

      // we also want to interpolate p and q values based on the original p,q from the cbit_path. We use this afterwards for finding appropriate corridor mpc
      // reference poses on the teach path
      double p_int = pose_lower.p + ((p_val - p_lower) / (p_upper - p_lower)) * (pose_upper.p - pose_lower.p);
      double q_int = pose_lower.q + ((p_val - p_lower) / (p_upper - p_lower)) * (pose_upper.q - pose_lower.q);

      // Build the transformation matrix
      Eigen::Matrix4d T_ref;
      T_ref << std::cos(yaw_int),-1*std::sin(yaw_int),0, x_int,
              std::sin(yaw_int),   std::cos(yaw_int),0, y_int,
              0,               0,            1, z_int,
              0,               0,            0,                    1;
      T_ref = T_ref.inverse().eval();

      lgmath::se3::Transformation meas = lgmath::se3::Transformation(T_ref);

      CLOG(DEBUG, "mpc_debug.cbit") << "The measurement Euclidean state is - x: " << x_int << " y: " << y_int << " z: " << z_int << " yaw: " << yaw_int;
      CLOG(DEBUG, "mpc_debug.cbit") << "The measurement P,Q value is - p: " << p_int << " q: " << q_int;
      return {meas, p_int, q_int};
    }
  }
}


// Simple function for checking that the current output velocity command is saturated between our mechanical velocity limits
Eigen::Matrix<double, 2, 1> SaturateVel2(Eigen::Matrix<double, 2, 1> applied_vel, double v_lim, double w_lim)
{
    double command_lin_x;
    double command_ang_z;
    Eigen::Matrix<double, 2, 1> saturated_vel;


    if (abs(applied_vel(0)) >= v_lim)
    {
      if (applied_vel(0) > 0.0)
      {
        command_lin_x = v_lim;
      }
      else if (applied_vel(0)  < 0.0)
      {
        command_lin_x = -1.0* v_lim;
      }
    }

    else
    {
      command_lin_x = applied_vel(0) ;
    }

    if (abs(applied_vel(1)) >= w_lim)
    {
      if (applied_vel(1) > 0.0)
      {
        command_ang_z = w_lim;
      }
      else if (applied_vel(1)  < 0.0)
      {
        command_ang_z = -1.0 * w_lim;
      }
    }

    else
    {
      command_ang_z = applied_vel(1) ;
    }

    saturated_vel << command_lin_x, command_ang_z;
    return saturated_vel;
}