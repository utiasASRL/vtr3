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
#include "vtr_lidar/path_planning/mpc.hpp"
#include "vtr_lidar/cache.hpp"

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
        pose_states.push_back(lgmath::se3::Transformation());
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

        // Experimental velocity set-point constraint (instead of non zero velocity penalty)
        // Only add this cost term if we are not in point stabilization mode (end of path)
        if (point_stabilization == false)
        {
          const auto vel_cost_term = steam::WeightedLeastSqCostTerm<2>::MakeShared(steam::vspace::VSpaceErrorEvaluator<2>::MakeShared(vel_state_vars[i],v_ref), sharedVelNoiseModel, sharedLossFunc);
          opt_problem.addCostTerm(vel_cost_term);
        }


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


        // Kinematic constraints (softened but penalized heavily)
        if (i < (K-1))
        {
        const auto lhs = steam::se3::ComposeInverseEvaluator::MakeShared(pose_state_vars[i+1], pose_state_vars[i]);
        const auto vel_proj = steam::vspace::MatrixMultEvaluator<2>::MakeShared(vel_state_vars[i], P_tran); // TODO, I guess this version of steam doesnt have this one, will need to do it myself
        const auto scaled_vel_proj = steam::vspace::ScalarMultEvaluator<6>::MakeShared(vel_proj, DT);
        const auto rhs = steam::se3::ExpMapEvaluator::MakeShared(scaled_vel_proj);
        const auto kin_error_func = steam::se3::LogMapEvaluator::MakeShared(steam::se3::ComposeInverseEvaluator::MakeShared(lhs, rhs));
        const auto kin_cost_term = steam::WeightedLeastSqCostTerm<6>::MakeShared(kin_error_func, sharedKinNoiseModel, sharedLossFunc);
        opt_problem.addCostTerm(kin_cost_term);
        }
    }

    // Solve the optimization problem with GuassNewton solver
    using SolverType = steam::VanillaGaussNewtonSolver;
    // Initialize parameters (enable verbose mode)
    SolverType::Params params;
    params.verbose = false; // Makes the output display for debug

    SolverType solver(&opt_problem, params);
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


    // Take in the current euclidean path solution from the cbit planner in the world frame, the current robot state, and determine
    // which measurements we wish to track to follow the path at the desired target velocity
    CLOG(DEBUG, "mpc_debug.cbit") << "Reference Path Points:";
    std::vector<lgmath::se3::Transformation> measurements;
    double starting_dist = INFINITY;
    double new_dist;
    double dx;
    double dy;
    double delta_dist = 0;
    double index_counter = 0;

    // Find closest point on the path to the current state (new version, experimental)
    double min_dist = INFINITY;
    CLOG(DEBUG, "mpc_debug.cbit") << "Total CBIT Path Size: " << cbit_path.size();
    for (int i = 0; i < 200; i++) // magic number for now, corresponds to about a 5m lookahead (need to remove dependency on this)
    {
      // handle end of path case:
      if (i >= cbit_path.size())
      {
        break;
      }
      CLOG(DEBUG, "mpc_debug.cbit") << "x: " << (cbit_path)[i].x << "y: " << (cbit_path)[i].y << "z: " << (cbit_path)[i].z;
      dx = (cbit_path)[i].x - std::get<0>(robot_pose);
      dy = (cbit_path)[i].y - std::get<1>(robot_pose);
      new_dist = sqrt((dx * dx) + (dy * dy));
      CLOG(DEBUG, "mpc_debug.cbit") << "Dist to Pt: " << new_dist;
      if (new_dist < min_dist)
      {
        CLOG(DEBUG, "mpc_debug.cbit") << "Minimum Distance: " << new_dist;
        index_counter = i; // I wonder if we should add some buffer, as this point could still be behind the current robot
        min_dist = new_dist;
      }
    }
    /*
    // Find closest point on the path to the current state (Old version, mostly worked, but I think if the cbit path was not smooth there was a chance it would exit early before finding closest point)
    while (delta_dist >= 0)
    {
      CLOG(DEBUG, "mpc_debug.cbit") << "x: " << (cbit_path)[index_counter].x << "y: " << (cbit_path)[index_counter].y << "z: " << (cbit_path)[index_counter].z;
      dx = (cbit_path)[index_counter].x - std::get<0>(robot_pose);
      dy = (cbit_path)[index_counter].y - std::get<1>(robot_pose);
      new_dist = sqrt((dx * dx) + (dy * dy));
      delta_dist = starting_dist - new_dist;
      CLOG(DEBUG, "mpc_debug.cbit") << "Dist to Pt: " << new_dist;
      CLOG(DEBUG, "mpc_debug.cbit") << "Delta Dist: " << delta_dist;
      if (delta_dist >= 0)
      {
        starting_dist = new_dist;
        index_counter = index_counter + 1;
      }
      else
      {
        CLOG(DEBUG, "mpc_debug.cbit") << "Delta Dist Negative, Return i = " << index_counter-1;
        index_counter = index_counter - 1;
      }
    }
    */

    // Keep iterating through the rest of the path, storing points in the path as measurements if they maintain an approximate
    // forward path velocity of VF (//TODO, may need to also interpolate in some instances if we want to be very particular)
    for (int i = index_counter; i < (cbit_path).size(); i++)
    {
      // The first iteration we need to add the closest point to the initial position as a measurement
      // Subesequent iterations we want to select points on the path to track carefully based on the desired velocity we want to meet.
      
      // Reset the measurement distance
      double delta_dist = 0.0;
      if (index_counter != i)
      {
        // scan the path into the future until we proceed approximately VF*DT meters forward longitudinally long the path
        // The resulting indice of the path will be the one we use for the next measurement
        while ((delta_dist <= (VF*DT)) && (i < (cbit_path).size())) // Need to add this and condition to handle end of path case
        {
          
          double prev_x = (cbit_path)[i-1].x;
          double prev_y = (cbit_path)[i-1].y;
          double next_x = (cbit_path)[i].x;
          double next_y = (cbit_path)[i].y;
          delta_dist = delta_dist + sqrt(((next_x-prev_x) * (next_x-prev_x)) + ((next_y-prev_y) * (next_y-prev_y)));
          i = i + 1;
        }

        // With the above setup, pretty sure the resulting i will be 1 too far when we break the loop, so we need to decrement it once at the end
        i = i-1; 
      }




      // Derive a yaw direction for each point based on the vector formed between the current point and the previous point on the path
      double yaw = std::atan2(((cbit_path)[i].y - (cbit_path)[i-1].y), ((cbit_path)[i].x - (cbit_path)[i-1].x));
      //CLOG(DEBUG, "mpc_debug.cbit") << "The yaw of the path pt is: " << yaw;

      // Generate a reference Transformation matrix (ignores roll/pitch)
      Eigen::Matrix4d T_ref;
      T_ref << std::cos(yaw),-1*std::sin(yaw),0,(cbit_path)[i].x,
              std::sin(yaw),   std::cos(yaw),0,(cbit_path)[i].y,
              0,               0,            1,(cbit_path)[i].z,
              0,               0,            0,                    1;
      T_ref = T_ref.inverse().eval();

      measurements.push_back(lgmath::se3::Transformation(T_ref));

      // Early break condition when the number of measurements we need is satisfied based on the horizon
      if (measurements.size() == K)
      {
        break;
      }
      // TODO handle end of path case => will want to repeat the final measurements and turn problem into a point stabilization MPC.

    }

    // If we reach the end of the path without generating enough measurements (approaching end of path), populate the measurements
    // with the final point in the path (point stabilization problem)
    if (measurements.size() < K)
    {
      CLOG(WARNING, "mpc.cbit") << "Approaching End of Path, Converting MPC to Point Stabilization Problem";
      
      // debug logging messages
      /*
      CLOG(ERROR, "mpc.cbit") << "Displaying all measurements so far:";
      CLOG(ERROR, "mpc.cbit") << "Size of cbit path is: " << cbit_path.size();
      CLOG(ERROR, "mpc.cbit") << "Size of measurements is: " << measurements.size();
      for (int k = 0; k < measurements.size(); k++)
      {
        CLOG(ERROR, "mpc.cbit") << "Measurement k: " << k << " is: " << measurements[k];
      }
      
      CLOG(ERROR, "mpc.cbit") << "The last valid transform was: " << measurements[measurements.size()-1];
      */
      for (int j = measurements.size(); j < K; j++)
      {
        measurements.push_back(measurements[j-1]);
        //CLOG(ERROR, "mpc.cbit") << "Appending this transform to the measurements: " << measurements[j-1];
      }
      
      return {measurements, true};
    }

    else
    {
      return {measurements, false};
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

    if (applied_vel(0) >= v_lim)
    {
      command_lin_x = v_lim;
    }
    else if (applied_vel(0)  <= 0.0)
    {
      command_lin_x = 0.0;
    }
    else
    {
      command_lin_x = applied_vel(0) ;
    }

    if (applied_vel(1)  >= w_lim)
    {
      command_ang_z = w_lim;
    }
    else if (applied_vel(1) <= -1*w_lim)
    {
      command_ang_z = -1*w_lim;
    }
    else
    {
      command_ang_z = applied_vel(1) ;
    }

    saturated_vel << command_lin_x, command_ang_z;
    return saturated_vel;
}
































  