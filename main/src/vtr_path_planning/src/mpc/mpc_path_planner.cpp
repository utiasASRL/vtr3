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
#include "vtr_path_planning/mpc/mpc_path_planner.hpp"

// This file is used to generate a tracking mpc output velocity command given a discretized path to follow and optimization parameters
// It is used in cbit.cpp in both the vtr_lidar package (obstacle avoidance) and vtr_path_planning packages (obstacle free) in the computeCommand function


// Main MPC problem solve function
struct mpc_result SolveMPC(Eigen::Matrix<double, 2, 1> previous_vel, lgmath::se3::Transformation T0, std::vector<lgmath::se3::Transformation> measurements, int K, double DT, double VF, Eigen::Matrix<double, 6, 6> pose_noise_vect, Eigen::Matrix<double, 2, 2> vel_noise_vect, Eigen::Matrix<double, 2, 2> accel_noise_vect, Eigen::Matrix<double, 6, 6> kin_noise_vect, bool point_stabilization)
{
    
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
    Eigen::Matrix<double, 4, 1> I_4;
    I_4 << 0,
           0,
           0,
           1;
    Eigen::Matrix<double, 1, 4> I_2_tran;
    I_2_tran << 0, 1, 0, 0;


    // Setup shared loss functions and noise models for all cost terms
    const auto sharedLossFunc = steam::L2LossFunc::MakeShared();

    const auto sharedPoseNoiseModel =
        steam::StaticNoiseModel<6>::MakeShared(pose_noise_vect);

    const auto sharedVelNoiseModel =
        steam::StaticNoiseModel<2>::MakeShared(vel_noise_vect);

    const auto sharedAccelNoiseModel =
        steam::StaticNoiseModel<2>::MakeShared(accel_noise_vect);

    const auto sharedKinNoiseModel =
        steam::StaticNoiseModel<6>::MakeShared(kin_noise_vect);

    // Experimental lat constraint (hardcoded for now)
    Eigen::Matrix<double, 1, 1> lat_noise_vect;
    lat_noise_vect << 1.0;
    const auto sharedLatNoiseModel =
        steam::StaticNoiseModel<1>::MakeShared(lat_noise_vect);



    // Generate STEAM States for the velocity vector and SE3 state transforms
    std::vector<lgmath::se3::Transformation> pose_states;
    std::vector<Eigen::Matrix<double,2,1>> vel_states;

    // Invert the extrapolated robot state and use this as the state initialization
    lgmath::se3::Transformation T0_inv = T0.inverse();
    Eigen::Vector2d v0(0.0, 0.0);

    // Pushback the initial states (current robot state)
    pose_states.push_back(T0_inv); // Change this to T0 when implementing on robot, T0_1 for debug
    //vel_states.push_back(std::vector<double> {0.0, 0.0}); //I think a single line way t odo this is something like Eigen::Matrix<double, 2, 1>::Zero()
    vel_states.push_back(v0);

    // Set the remaining states
    for (int i=0; i<K-1; i++)
    {
        //pose_states.push_back(lgmath::se3::Transformation()); // I wonder if here I should initialize all the states as the initial state
        pose_states.push_back(T0_inv);
        vel_states.push_back(v0);
    }

    // Create Steam states
    std::vector<steam::se3::SE3StateVar::Ptr> pose_state_vars;
    std::vector<steam::vspace::VSpaceStateVar<2>::Ptr> vel_state_vars;

    for (int i = 0; i < K; i++)
    {
        pose_state_vars.push_back(steam::se3::SE3StateVar::MakeShared(pose_states[i]));
        vel_state_vars.push_back(steam::vspace::VSpaceStateVar<2>::MakeShared(vel_states[i])); 
    }

    // Lock the first (current robot) state from being able to be modified during the optimization
    pose_state_vars[0]->locked() = true;


    // Setup the optimization problem
    steam::OptimizationProblem opt_problem;
    for (int i=0; i<K; i++)
    {
        opt_problem.addStateVariable(pose_state_vars[i]);
        //opt_problem.addStateVariable(vel_state_vars[i]);
    }

    for (int i=0; i<K-1; i++)
    {
        //opt_problem.addStateVariable(pose_state_vars[i]);
        opt_problem.addStateVariable(vel_state_vars[i]);
    }

    // Generate the cost terms using combinations of the built-in steam evaluators
    for (int i = 0; i < K; i++)
    {
        // Pose Error
        const auto pose_error_func = steam::se3::SE3ErrorEvaluator::MakeShared(pose_state_vars[i], measurements[i]);
        const auto pose_cost_term = steam::WeightedLeastSqCostTerm<6>::MakeShared(pose_error_func, sharedPoseNoiseModel, sharedLossFunc);
        opt_problem.addCostTerm(pose_cost_term);

        // Non-Zero Velocity Penalty (OLD, not using this way any more, though might change to this when approaching end of path)
        //const auto vel_cost_term = steam::WeightedLeastSqCostTerm<2>::MakeShared(vel_state_vars[i], sharedVelNoiseModel, sharedLossFunc);
        //opt_problem.addCostTerm(vel_cost_term);

        

        // Kinematic constraints (softened but penalized heavily)
        if (i < (K-1))
        {
        const auto lhs = steam::se3::ComposeInverseEvaluator::MakeShared(pose_state_vars[i+1], pose_state_vars[i]);
        const auto vel_proj = steam::vspace::MatrixMultEvaluator<6,2>::MakeShared(vel_state_vars[i], P_tran); // TODO, I guess this version of steam doesnt have this one, will need to do it myself
        const auto scaled_vel_proj = steam::vspace::ScalarMultEvaluator<6>::MakeShared(vel_proj, DT);
        const auto rhs = steam::se3::ExpMapEvaluator::MakeShared(scaled_vel_proj);
        const auto kin_error_func = steam::se3::LogMapEvaluator::MakeShared(steam::se3::ComposeInverseEvaluator::MakeShared(lhs, rhs));
        const auto kin_cost_term = steam::WeightedLeastSqCostTerm<6>::MakeShared(kin_error_func, sharedKinNoiseModel, sharedLossFunc);
        opt_problem.addCostTerm(kin_cost_term);


        // Experimental velocity set-point constraint (instead of non zero velocity penalty)
        // Only add this cost term if we are not in point stabilization mode (end of path)
        //if (point_stabilization == false)
        //{
        //  const auto vel_cost_term = steam::WeightedLeastSqCostTerm<2>::MakeShared(steam::vspace::VSpaceErrorEvaluator<2>::MakeShared(vel_state_vars[i],v_ref), sharedVelNoiseModel, sharedLossFunc);
        //  opt_problem.addCostTerm(vel_cost_term);
        //}


        // Experimental acceleration limits
        
        if (i == 0)
        {
        // On the first iteration, we need to use an error with the previously applied control command state
        const auto accel_cost_term = steam::WeightedLeastSqCostTerm<2>::MakeShared(steam::vspace::VSpaceErrorEvaluator<2>::MakeShared(vel_state_vars[i], previous_vel), sharedAccelNoiseModel, sharedLossFunc);
        opt_problem.addCostTerm(accel_cost_term);
        } 
        else
        {
        // Subsequent iterations we make an error between consecutive velocities. We penalize large changes in velocity between time steps
        const auto accel_diff = steam::vspace::AdditionEvaluator<2>::MakeShared(vel_state_vars[i], steam::vspace::NegationEvaluator<2>::MakeShared(vel_state_vars[i-1]));
        const auto accel_cost_term = steam::WeightedLeastSqCostTerm<2>::MakeShared(accel_diff, sharedAccelNoiseModel, sharedLossFunc);
        opt_problem.addCostTerm(accel_cost_term);
        }   
        
        }



        // Experimental Hard Lateral Constraints
        /*
        CLOG(DEBUG, "debug") << "Attempting to add a lateral cost term";
        const auto inv_state = steam::se3::InverseEvaluator::MakeShared(pose_state_vars[i]);
        const auto pose_err = steam::vspace::MatrixMultEvaluator<4,4>::MakeShared(inv_state, measurements[i]); // I think this line is the problem, we cannot really convert the se3 evaluators to vspace evaluators
        const auto left_mul = steam::vspace::MatrixMultEvaluator<1,4>::MakeShared(pose_err, I_2_tran);
        const auto right_mul = steam::vspace::MatrixMultEvaluator<4,1>::MakeShared(pose_err, I_4);
        const auto lat_cost_term = steam::WeightedLeastSqCostTerm<6>::MakeShared(right_mul, sharedLatNoiseModel, sharedLossFunc);
        opt_problem.addCostTerm(lat_cost_term);
        CLOG(DEBUG, "debug") << "Successfully added Lat cost term";
        */
    }

    // Solve the optimization problem with GuassNewton solver
    using SolverType = steam::GaussNewtonSolver;
    // Initialize parameters (enable verbose mode)
    SolverType::Params params;
    params.verbose = false; // Makes the output display for debug when true

    SolverType solver(opt_problem, params);
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
struct meas_result GenerateReferenceMeas(std::shared_ptr<std::vector<Pose>> cbit_path_ptr, std::tuple<double, double, double, double, double, double> robot_pose, int K, double DT, double VF)
{


    // Save a copy of the current path solution to work on
    auto cbit_path = *cbit_path_ptr;

    // Experimental Changes to improve controller instability (completed but not rigourously field tested yet)

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
    //CLOG(DEBUG, "debug") << "cbit_p is: " << cbit_p;

    // Determine the p_values we need for our measurement horizon, corrected for the p value of the closest point on the path to the current robot state
    std::vector<double> p_meas_vec;
    std::vector<lgmath::se3::Transformation> measurements;
    p_meas_vec.reserve(K);
    for (int i = 0; i < K; i++)
    {

      p_meas_vec.push_back((i * DT * VF) + p_correction);
    }
    //CLOG(DEBUG, "debug") << "p_meas_vec is: " << p_meas_vec;

    // todo: Iterate through the p_measurements and interpolate euclidean measurements from the cbit_path and the corresponding cbit_p values
    // Note this could be combined in the previous loop too
    bool point_stabilization = false;
    for (int i = 0; i < p_meas_vec.size(); i++)
    {
      // handle end of path case:
      // if the p meas we would have needed exceeds the final measurement pose, set it equal to our last p value in the path
      // This will cause the intepolation to return the final cbit_path pose for all measurements past this point
      //CLOG(INFO, "debug") << "The specific p_meas_vec[i] is: " << p_meas_vec[i];
      //CLOG(INFO, "debug") << "The size of cbit_p is:" << cbit_p.size();
      //CLOG(INFO, "debug") << "The final cbit_p value is:" << cbit_p[cbit_p.size()-1];

      if (p_meas_vec[i] > cbit_p[cbit_p.size()-1])
      {
        p_meas_vec[i] = cbit_p[cbit_p.size()-1];
        point_stabilization = true; // Enable point stabilization configs
        CLOG(INFO, "mpc.cbit") << "Approaching End of Path, Converting MPC to Point Stabilization Problem";
      }
      lgmath::se3::Transformation meas = InterpolateMeas(p_meas_vec[i], cbit_p, cbit_path);

      // add to measurement vector
      measurements.push_back(meas);
    }

    return {measurements, point_stabilization};


    //End of Experimental Changes

}

// function takes in a the cbit path solution with a vector defining hte p axis of the path, and then a desired p_meas
// Then tries to output a euclidean pose interpolated for the desired p_meas.
lgmath::se3::Transformation InterpolateMeas(double p_val, std::vector<double> cbit_p, std::vector<Pose> cbit_path)
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

      double x_int = pose_lower.x + ((p_val - p_lower) / (p_upper - p_lower)) * (pose_upper .x - pose_lower.x);
      double y_int = pose_lower.y + ((p_val - p_lower) / (p_upper - p_lower)) * (pose_upper .y - pose_lower.y);
      double z_int = pose_lower.z + ((p_val - p_lower) / (p_upper - p_lower)) * (pose_upper .z - pose_lower.z);

      // For yaw we need to be abit careful about sign and angle wrap around
      // Derive the yaw by creating the vector connecting the pose_upp and pose_lower pts

      double yaw_int = std::atan2((pose_upper.y - pose_lower.y), (pose_upper.x - pose_lower.x));


      // Build the transformation matrix
      Eigen::Matrix4d T_ref;
      T_ref << std::cos(yaw_int),-1*std::sin(yaw_int),0, x_int,
              std::sin(yaw_int),   std::cos(yaw_int),0, y_int,
              0,               0,            1, z_int,
              0,               0,            0,                    1;
      T_ref = T_ref.inverse().eval();

      lgmath::se3::Transformation meas = lgmath::se3::Transformation(T_ref);

      CLOG(DEBUG, "mpc_debug.cbit") << "The measurement Euclidean state is - x: " << x_int << " y: " << y_int << " z: " << z_int << " yaw: " << yaw_int;
      return meas;
    }
  }
}


// Simple function for checking that the current output velocity command is saturated between our mechanical velocity limits
Eigen::Matrix<double, 2, 1> SaturateVel(Eigen::Matrix<double, 2, 1> applied_vel, double v_lim, double w_lim)
{
    double command_lin_x;
    double command_ang_z;
    Eigen::Matrix<double, 2, 1> saturated_vel;

    // Moved nan check to the main mpc solver function
    /*
    // First check if any of the values are nan, if so we return a zero velocity and flag the error
    if (std::isnan(applied_vel(0)) || std::isnan(applied_vel(1)))
    {
      CLOG(ERROR, "mpc.cbit") << "NAN values detected, mpc optimization failed. Returning zero velocities";
      saturated_vel(0) = 0.0;
      saturated_vel(1) = 0.0;
      return saturated_vel;
    }
    */

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
    // Removed for bi-directional control purposes
    //else if (applied_vel(0)  <= 0.0)
    //{
    //  command_lin_x = 0.0;
    //}
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
    //else if (applied_vel(1) <= -1*w_lim)
    //{
    //  command_ang_z = -1*w_lim;
    //}
    else
    {
      command_ang_z = applied_vel(1) ;
    }

    // Changes for Bi-directional path traversal, we no longer want to saturate commands less than 0.0

    saturated_vel << command_lin_x, command_ang_z;
    return saturated_vel;
}
































  