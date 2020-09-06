//////////////////////////////////////////////////////////////////////////////////////////////
/// \file TrajectoryPrior.cpp
/// \brief A sample usage of the STEAM Engine library for a trajectory prior problem.
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <math.h>

#include <lgmath.hpp>
#include <steam.hpp>

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Structure to store trajectory state variables
//////////////////////////////////////////////////////////////////////////////////////////////
struct TrajStateVar {
  steam::Time time;
  steam::se3::TransformStateVar::Ptr pose;
  steam::VectorSpaceStateVar::Ptr velocity;
};

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Example that uses a simple constant-velocity prior over a trajectory.
//////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {

  ///
  /// Setup Problem
  ///

  // Number of state times
  unsigned int numPoses = 100;

  // Setup velocity prior
  Eigen::Matrix<double,6,1> velocityPrior;
  double v_x = -1.0;
  double omega_z = 0.01;
  velocityPrior << v_x, 0.0, 0.0, 0.0, 0.0, omega_z;

  // Calculate time to do one circle
  double totalTime = 2.0 * M_PI / omega_z;

  // Calculate time between states
  double delT = totalTime/(numPoses-1);

  // Smoothing factor diagonal
  Eigen::Array<double,1,6> Qc_diag;
  Qc_diag << 1.0, 0.001, 0.001, 0.001, 0.001, 1.0;

  // Make Qc_inv
  Eigen::Matrix<double,6,6> Qc_inv; Qc_inv.setZero();
  Qc_inv.diagonal() = 1.0/Qc_diag;

  //
  // Setup initial conditions
  //

  // Pose
  Eigen::Matrix<double,6,1> initPoseVec;
  initPoseVec << 1,2,3,4,5,6;
  lgmath::se3::Transformation initPose(initPoseVec);

  // Zero velocity
  Eigen::Matrix<double,6,1> initVelocity; initVelocity.setZero();

  ///
  /// Setup States
  ///

  // Steam state variables
  std::vector<TrajStateVar> states;

  // Setup state variables - initialized at identity / zero
  for (unsigned int i = 0; i < numPoses; i++) {
    TrajStateVar temp;
    temp.time = steam::Time(i*delT);
    temp.pose = steam::se3::TransformStateVar::Ptr(new steam::se3::TransformStateVar(initPose));
    temp.velocity = steam::VectorSpaceStateVar::Ptr(new steam::VectorSpaceStateVar(initVelocity));
    states.push_back(temp);
  }

  // Setup Trajectory
  steam::se3::SteamTrajInterface traj(Qc_inv);

  for (unsigned int i = 0; i < states.size(); i++) {
    TrajStateVar& state = states.at(i);
    steam::se3::TransformStateEvaluator::Ptr temp =
        steam::se3::TransformStateEvaluator::MakeShared(state.pose);
    traj.add(state.time, temp, state.velocity);
  }

  ///
  /// Setup Cost Terms
  ///

  // Add priors (alternatively, we could lock pose variable)
  traj.addPosePrior(steam::Time(0.0), initPose, Eigen::Matrix<double,6,6>::Identity());
  traj.addVelocityPrior(steam::Time(0.0), velocityPrior, Eigen::Matrix<double,6,6>::Identity());

  steam::ParallelizedCostTermCollection::Ptr costTerms(new steam::ParallelizedCostTermCollection());
  traj.appendPriorCostTerms(costTerms);

  ///
  /// Make Optimization Problem
  ///

  // Initialize problem
  steam::OptimizationProblem problem;

  // Add state variables
  for (unsigned int i = 0; i < states.size(); i++) {
    const TrajStateVar& state = states.at(i);
    problem.addStateVariable(state.pose);
    problem.addStateVariable(state.velocity);
  }

  // Add cost terms
  problem.addCostTerm(costTerms);

  ///
  /// Setup Solver and Optimize
  ///

  typedef steam::DoglegGaussNewtonSolver SolverType;

  // Initialize parameters (enable verbose mode)
  SolverType::Params params;
  params.verbose = true;

  // Make solver
  SolverType solver(&problem, params);

  // Optimize
  solver.optimize();

  // Get velocity at interpolated time
  auto curr_vel = traj.getVelocity(states.at(0).time+0.5*delT);

  ///
  /// Print results
  ///

  std::cout << std::endl
            << "First Pose:                  " << states.at(0).pose->getValue()
            << "Second Pose:                 " << states.at(1).pose->getValue()
            << "Last Pose (full circle):     "  << states.back().pose->getValue()
            << "First Vel:                   " << states.at(0).velocity->getValue().transpose() << std::endl
            << "Second Vel:                  " << states.at(1).velocity->getValue().transpose() << std::endl
            << "Last Vel:                    " << states.back().velocity->getValue().transpose() << std::endl
            << "Interp. Vel (t=t0+0.5*delT): " << curr_vel.transpose() << std::endl;

  return 0;
}

