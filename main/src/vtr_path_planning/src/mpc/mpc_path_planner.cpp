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

#include "vtr_path_planning/mpc/mpc_path_planner.hpp"
#include "vtr_path_planning/mpc/lateral_error_evaluators.hpp"
#include "vtr_path_planning/mpc/custom_loss_functions.hpp"
#include "vtr_path_planning/mpc/scalar_log_barrier_evaluator.hpp"



// Main MPC problem solve function - TODO: dump all these arguments into an mpc config class
struct MPCResult SolveMPC(const MPCConfig& config)
{
    // Access configuration parameters from the config structure
    Eigen::Matrix<double, 2, 1> previous_vel = config.previous_vel;
    lgmath::se3::Transformation T0 = config.T0;
    std::vector<lgmath::se3::Transformation> homotopy_reference_poses = config.homotopy_reference_poses;
    std::vector<lgmath::se3::Transformation> tracking_reference_poses = config.tracking_reference_poses;
    std::vector<double> barrier_q_left = config.barrier_q_left;
    std::vector<double> barrier_q_right = config.barrier_q_right;
    int K = config.K;
    double DT = config.DT;
    double VF = config.VF;
    Eigen::Matrix<double, 1, 1> lat_noise_vect = config.lat_noise_vect;
    Eigen::Matrix<double, 6, 6> pose_noise_vect = config.pose_noise_vect;
    Eigen::Matrix<double, 2, 2> vel_noise_vect = config.vel_noise_vect;
    Eigen::Matrix<double, 2, 2> accel_noise_vect = config.accel_noise_vect;
    Eigen::Matrix<double, 6, 6> kin_noise_vect = config.kin_noise_vect;
    bool point_stabilization = config.point_stabilization;
    double pose_error_weight = config.pose_error_weight;
    double vel_error_weight = config.vel_error_weight;
    double acc_error_weight = config.acc_error_weight;
    double kin_error_weight = config.kin_error_weight;
    double lat_error_weight = config.lat_error_weight;
    bool verbosity = config.verbosity;
    bool homotopy_mode = config.homotopy_mode;

    // Conduct an MPC Iteration given current configurations

    // Velocity set-points (desired forward velocity and angular velocity), here we set a static forward target velocity, and try to minimize rotations (0rad/sec)
    Eigen::Matrix<double, 2, 1> v_ref;
    v_ref << VF,
                0;


    // Kinematic projection Matrix for Unicycle Model (note its -1's because our varpi lie algebra vector is of a weird frame)
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
    Eigen::Vector2d v0(VF, 0.0);

    // Push back the initial states (current robot state)
    pose_states.push_back(T0_inv);
    vel_states.push_back(v0);

    // Set the remaining states using a warm start from the cbit solution
    for (int i=1; i < K; i++)
    {
        pose_states.push_back(tracking_reference_poses[i]); // New initialization - use the reference measurements from the cbit solution as our initialization - the first one is the same as our initial state
        vel_states.push_back(v0);
    }

    // Create STEAM states
    std::vector<steam::se3::SE3StateVar::Ptr> pose_state_vars;
    std::vector<steam::vspace::VSpaceStateVar<2>::Ptr> vel_state_vars;
    std::vector<steam::se3::SE3StateVar::Ptr> measurement_vars; // This one is for storing measurements as locked evaluators for barrier constraints
    
    // Create a locked state var for the 4th column of the identity matrix (used in state constraint)
    steam::stereo::HomoPointStateVar::Ptr I_4_eval = steam::stereo::HomoPointStateVar::MakeShared(I_4);
    I_4_eval->locked() = true;

    // Create STEAM variables
    for (int i = 0; i < K; i++)
    {
        pose_state_vars.push_back(steam::se3::SE3StateVar::MakeShared(pose_states[i])); 
        vel_state_vars.push_back(steam::vspace::VSpaceStateVar<2>::MakeShared(vel_states[i])); 
    }

    // Lock the first (current robot) state from being modified during the optimization
    pose_state_vars[0]->locked() = true;



    // Setup the optimization problem
    steam::OptimizationProblem opt_problem;
    for (int i=1; i<K; i++) // start at 1 so as to not add the first locked state variable to the problem
    {
        opt_problem.addStateVariable(pose_state_vars[i]);
    }

    // The velocity states should have one less variable then the pose states
    for (int i=0; i<K-1; i++)
    {
        opt_problem.addStateVariable(vel_state_vars[i]);
    }

    // Generate the cost terms using combinations of the built-in steam evaluators
    double dynamic_pose_error_weight = pose_error_weight;
    for (int i = 0; i < K; i++)
    {
      // Pose Error
      if (i > 0)
      {
        const auto pose_error_func = steam::se3::SE3ErrorEvaluator::MakeShared(pose_state_vars[i], homotopy_reference_poses[i]);
        auto dynamicposeLossFunc = steam::L2WeightedLossFunc::MakeShared(dynamic_pose_error_weight);
        const auto pose_cost_term = steam::WeightedLeastSqCostTerm<6>::MakeShared(pose_error_func, sharedPoseNoiseModel, dynamicposeLossFunc);
        opt_problem.addCostTerm(pose_cost_term);
        //dynamic_pose_error_weight = dynamic_pose_error_weight * 0.95;
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
        // Only add this cost term if we are not in point stabilization mode (end of path)
        

        if (point_stabilization == false) {
          const auto vel_cost_term = steam::WeightedLeastSqCostTerm<2>::MakeShared(vel_state_vars[i], sharedVelNoiseModel, velLossFunc);
          opt_problem.addCostTerm(vel_cost_term);
        }
        
        
        // Acceleration Constraints
        if (i == 0) {
          // On the first iteration, we need to use an error with the previously applied control command state
          const auto accel_cost_term = steam::WeightedLeastSqCostTerm<2>::MakeShared(steam::vspace::VSpaceErrorEvaluator<2>::MakeShared(vel_state_vars[i], previous_vel), sharedAccelNoiseModel, accelLossFunc);
          opt_problem.addCostTerm(accel_cost_term);
        } else {
          // Subsequent iterations we make an error between consecutive velocities. We penalize large changes in velocity between time steps
          const auto accel_diff = steam::vspace::AdditionEvaluator<2>::MakeShared(vel_state_vars[i], steam::vspace::NegationEvaluator<2>::MakeShared(vel_state_vars[i-1]));
          const auto accel_cost_term = steam::WeightedLeastSqCostTerm<2>::MakeShared(accel_diff, sharedAccelNoiseModel, accelLossFunc);
          opt_problem.addCostTerm(accel_cost_term);
        }  
      }

      // Laterial Barrier State Constraints (only when using homotopy guided MPC)
      if (i >= 0)
      {
        // Generate a locked transform evaluator to store the current measurement for state constraints
        // The reason we make it a variable and lock it is so we can use the built in steam evaluators which require evaluable inputs
        measurement_vars.push_back(steam::se3::SE3StateVar::MakeShared(homotopy_reference_poses[i]));
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

        CLOG(DEBUG, "mpc.debug") << "Left Barrier for this meas is: " << barrier_q_left[i];
        CLOG(DEBUG, "mpc.debug") << "Right Barrier for tis meas is: " << barrier_q_right[i];

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
        const auto lat_cost_term_left = steam::WeightedLeastSqCostTerm<1>::MakeShared(lat_barrier_left, sharedLatNoiseModel, latLossFunc);

        // If using homotopy class based control, apply barrier constraints. Else ignore them (more stable but potentially more aggressive)
        if (homotopy_mode == true)
        {
          opt_problem.addCostTerm(lat_cost_term_right);
          opt_problem.addCostTerm(lat_cost_term_left);
        }
      }
    }

    // Solve the optimization problem with GuassNewton solver
    //using SolverType = steam::GaussNewtonSolver; // Old solver, does not have back stepping capability
    //using SolverType = steam::LineSearchGaussNewtonSolver;
    using SolverType = steam::DoglegGaussNewtonSolver;

    // Initialize solver parameters
    SolverType::Params params;
    params.verbose = verbosity; // Makes the output display for debug when true
    // params.relative_cost_change_threshold = 1e-4;
    params.max_iterations = 100;
    params.absolute_cost_change_threshold = 1e-2;
    //params.backtrack_multiplier = 0.95; // Line Search Specific Params, will fail to build if using GaussNewtonSolver
    //params.max_backtrack_steps = 1000; // Line Search Specific Params, will fail to build if using GaussNewtonSolver

    SolverType solver(opt_problem, params);

    double initial_cost = opt_problem.cost();
    // Check the cost, disregard the result if it is unreasonable (i.e if its higher then the initial cost)
    CLOG(DEBUG, "mpc.solver") << "The Initial Solution Cost is:" << initial_cost;


    // Solve the optimization problem
    solver.optimize();

    double final_cost = opt_problem.cost();
    // Check the cost, disregard the result if it is unreasonable (i.e if its higher then the initial cost)
    CLOG(DEBUG, "mpc.solver") << "The Final Solution Cost is:" << final_cost;

    if (final_cost > initial_cost)
    {
      CLOG(ERROR, "mpc.solver") << "The final cost was > initial cost, something went wrong. Commanding the vehicle to stop";
      Eigen::Matrix<double, 2, 1> bad_cost_vel;
      
      bad_cost_vel(0) = 0.0;
      bad_cost_vel(1) = 0.0;

      // Return the mpc_poses as all being the robots current pose (not moving across the horizon as we should be stopped)
      std::vector<lgmath::se3::Transformation> mpc_poses;
      for (size_t i = 0; i < pose_state_vars.size(); i++)
      {
        mpc_poses.push_back(T0);
      }
      return {bad_cost_vel, mpc_poses};
    }

    // DEBUG: Display the prediction horizon results
    /*
    CLOG(DEBUG, "mpc.solver") << "Trying to Display the Optimization Results for the isolated MPC";
    CLOG(DEBUG, "mpc.solver") << "The First State is: " << pose_state_vars[0]->value().inverse();
    CLOG(DEBUG, "mpc.solver") << "The Second State is: " << pose_state_vars[1]->value().inverse();
    CLOG(DEBUG, "mpc.solver") << "The Third State is: " << pose_state_vars[2]->value().inverse();
    CLOG(DEBUG, "mpc.solver") << "The Forth State is: " << pose_state_vars[3]->value().inverse();
    CLOG(DEBUG, "mpc.solver") << "The Fifth State is: " << pose_state_vars[4]->value().inverse();
    CLOG(DEBUG, "mpc.solver") << "The Sixth State is: " << pose_state_vars[5]->value().inverse();
    CLOG(DEBUG, "mpc.solver") << "The Seventh State is: " << pose_state_vars[6]->value().inverse();
    CLOG(DEBUG, "mpc.solver") << "The Eighth State is: " << pose_state_vars[7]->value().inverse();
    CLOG(DEBUG, "mpc.solver") << "The Ninth State is: " << pose_state_vars[8]->value().inverse();
    CLOG(DEBUG, "mpc.solver") << "The Tenth State is: " << pose_state_vars[9]->value().inverse();
    CLOG(DEBUG, "mpc.solver") << "The First Velocity is: " << vel_state_vars[0]->value();
    CLOG(DEBUG, "mpc.solver") << "The Second Velocity is: " << vel_state_vars[1]->value();
    CLOG(DEBUG, "mpc.solver") << "The Third Velocity is: " << vel_state_vars[2]->value();
    CLOG(DEBUG, "mpc.solver") << "The Forth Velocity is: " << vel_state_vars[3]->value();
    CLOG(DEBUG, "mpc.solver") << "The Fifth Velocity is: " << vel_state_vars[4]->value();
    CLOG(DEBUG, "mpc.solver") << "The Sixth Velocity is: " << vel_state_vars[5]->value();
    CLOG(DEBUG, "mpc.solver") << "The Seventh Velocity is: " << vel_state_vars[6]->value();
    CLOG(DEBUG, "mpc.solver") << "The Eighth Velocity is: " << vel_state_vars[7]->value();
    CLOG(DEBUG, "mpc.solver") << "The Ninth Velocity is: " << vel_state_vars[8]->value();
    CLOG(DEBUG, "mpc.solver") << "The Tenth Velocity is: " << vel_state_vars[9]->value();
    CLOG(DEBUG, "mpc.solver") << "Linear Component to Return is: " << (vel_state_vars[0]->value())[0];
    CLOG(DEBUG, "mpc.solver") << "Angular Component to Return is: " << (vel_state_vars[0]->value())[1];
    */

    // Store the velocity command to apply
    Eigen::Matrix<double, 2, 1> applied_vel = vel_state_vars[0]->value();

    // First check if any of the values are nan, if so we return a zero velocity and flag the error
    Eigen::Matrix<double, 2, 1> nan_vel;
    if (std::isnan(applied_vel(0)) || std::isnan(applied_vel(1)))
    {
      CLOG(ERROR, "mpc.solver") << "NAN values detected, mpc optimization failed. Returning zero velocities";
      nan_vel(0) = 0.0;
      nan_vel(1) = 0.0;

      if (verbosity) {
        throw std::runtime_error("NAN values detected in MPC! Crashing for debug!");
      }

      // if we do detect nans, return the mpc_poses as all being the robots current pose (not moving across the horizon as we should be stopped)
      std::vector<lgmath::se3::Transformation> mpc_poses;
      for (size_t i = 0; i < pose_state_vars.size(); i++)
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
    for (size_t i = 0; i < pose_state_vars.size(); i++)
    {
      mpc_poses.push_back(pose_state_vars[i]->value().inverse());
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


// For generating VT&R teach path poses used in the corridor mpc (new version which directly uses the interpolated p measurements from the cbit path trajectory tracking)
struct PoseResultHomotopy GenerateHomotopyReference(std::shared_ptr<CBITPath> global_path_ptr, std::shared_ptr<CBITCorridor> corridor_ptr, std::tuple<double, double, double, double, double, double> robot_pose, const std::vector<double> &p_interp_vec) {
    // Set point stabilization, but just note if we use this function in the cbit.cpp file we need to use the Tracking reference pose point stabilization instead
    bool point_stabilization = false;

    // Initialize vectors storing the barrier values:
    std::vector<double> barrier_q_left;
    std::vector<double> barrier_q_right;
    
    // load the teach path
    std::vector<Pose> teach_path = global_path_ptr->disc_path;
    std::vector<double> teach_path_p = global_path_ptr->p;
    

    std::vector<lgmath::se3::Transformation> tracking_reference_poses;

    // Iterate through the interpolated p_measurements and make interpolate euclidean poses from the teach path
    for (size_t i = 0; i < p_interp_vec.size(); i++)
    {

      struct InterpResult interp_pose = InterpolatePose(p_interp_vec[i], teach_path_p, teach_path);

      // add to measurement vector
      tracking_reference_poses.push_back(interp_pose.pose);


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
    }

    return {tracking_reference_poses, point_stabilization, barrier_q_left, barrier_q_right};
}

// function takes in the cbit path solution with a vector defining the p axis of the path, and then a desired p_meas
// Then tries to output a euclidean pose interpolated for the desired p_meas.
struct InterpResult InterpolatePose(double p_val, std::vector<double> cbit_p, std::vector<Pose> cbit_path)
{
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
      }
      else
      {
        yaw_int = pose_lower.yaw;
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

      CLOG(DEBUG, "mpc.debug") << "The measurement Euclidean state is - x: " << x_int << " y: " << y_int << " z: " << z_int << " yaw: " << yaw_int;
      CLOG(DEBUG, "mpc.debug") << "The measurement P,Q value is - p: " << p_int << " q: " << q_int;
      return {meas, p_int, q_int};
    }
  }
}


// Simple function for checking that the current output velocity command is saturated between our mechanical velocity limits
Eigen::Matrix<double, 2, 1> SaturateVel(Eigen::Matrix<double, 2, 1> applied_vel, double v_lim, double w_lim)
{
    double command_lin_x;
    double command_ang_z;
    Eigen::Matrix<double, 2, 1> saturated_vel;


    if (abs(applied_vel(0)) >= v_lim) {
      if (applied_vel(0) > 0.0)
      {
        command_lin_x = v_lim;
      }
      else if (applied_vel(0)  < 0.0)
      {
        command_lin_x = -1.0* v_lim;
      } else {
        command_lin_x = 0;
      }
    } else {
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
      } else {
        command_ang_z = 0;
      }
    } else {
      command_ang_z = applied_vel(1) ;
    }

    saturated_vel << command_lin_x, command_ang_z;
    return saturated_vel;
}