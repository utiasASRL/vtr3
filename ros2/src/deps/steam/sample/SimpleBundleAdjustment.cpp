//////////////////////////////////////////////////////////////////////////////////////////////
/// \file SimpleBundleAdjustment.cpp
/// \brief A sample usage of the STEAM Engine library for a bundle adjustment problem
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <iostream>

#include <lgmath.hpp>
#include <steam.hpp>
#include <steam/data/ParseBA.hpp>

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Example that loads and solves simple bundle adjustment problems
//////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {

  ///
  /// Parse Dataset
  ///

  // Get filename
  std::string filename;
  if (argc < 2) {
    filename = "../../include/steam/data/stereo_simulated.txt";
    //filename = "../../include/steam/data/stereo_simulated_window1.txt";
    //filename = "../../include/steam/data/stereo_simulated_window2.txt";
    std::cout << "Parsing default file: " << filename << std::endl << std::endl;
  } else {
    filename = argv[1];
    std::cout << "Parsing file: " << filename << std::endl << std::endl;
  }

  // Load dataset
  steam::data::SimpleBaDataset dataset = steam::data::parseSimpleBaDataset(filename);
  std::cout << "Problem has: " << dataset.frames_gt.size() << " poses" << std::endl;
  std::cout << "             " << dataset.land_gt.size() << " landmarks" << std::endl;
  std::cout << "            ~" << double(dataset.meas.size())/dataset.frames_gt.size() << " meas per pose" << std::endl << std::endl;

  ///
  /// Setup and Initialize States
  ///

  // Ground truth
  std::vector<steam::se3::TransformStateVar::Ptr> poses_gt_k_0;
  std::vector<steam::se3::LandmarkStateVar::Ptr> landmarks_gt;

  // State variable containers (and related data)
  std::vector<steam::se3::TransformStateVar::Ptr> poses_ic_k_0;
  std::vector<steam::se3::LandmarkStateVar::Ptr> landmarks_ic;

  // Setup ground-truth poses
  for (unsigned int i = 0; i < dataset.frames_gt.size(); i++) {
    steam::se3::TransformStateVar::Ptr temp(new steam::se3::TransformStateVar(dataset.frames_gt[i].T_k0));
    poses_gt_k_0.push_back(temp);
  }

  // Setup ground-truth landmarks
  for (unsigned int i = 0; i < dataset.land_gt.size(); i++) {
    steam::se3::LandmarkStateVar::Ptr temp(new steam::se3::LandmarkStateVar(dataset.land_gt[i].point));
    landmarks_gt.push_back(temp);
  }

  // Setup poses with initial condition
  for (unsigned int i = 0; i < dataset.frames_ic.size(); i++) {
    steam::se3::TransformStateVar::Ptr temp(new steam::se3::TransformStateVar(dataset.frames_ic[i].T_k0));
    poses_ic_k_0.push_back(temp);
  }

  // Lock first pose (otherwise entire solution is 'floating')
  //  **Note: alternatively we could add a prior (UnaryTransformError) to the first pose.
  poses_ic_k_0[0]->setLock(true);

  // Setup landmarks with initial condition
  for (unsigned int i = 0; i < dataset.land_ic.size(); i++) {
    steam::se3::LandmarkStateVar::Ptr temp(new steam::se3::LandmarkStateVar(dataset.land_ic[i].point));
    landmarks_ic.push_back(temp);
  }

  ///
  /// Setup Cost Terms
  ///

  // steam cost terms
  steam::ParallelizedCostTermCollection::Ptr stereoCostTerms(new steam::ParallelizedCostTermCollection());

  // Setup shared noise and loss function
  steam::BaseNoiseModel<4>::Ptr sharedCameraNoiseModel(new steam::StaticNoiseModel<4>(dataset.noise));
  steam::L2LossFunc::Ptr sharedLossFunc(new steam::L2LossFunc());

  // Setup camera intrinsics
  steam::stereo::CameraIntrinsics::Ptr sharedIntrinsics(
        new steam::stereo::CameraIntrinsics());
  sharedIntrinsics->b  = dataset.camParams.b;
  sharedIntrinsics->fu = dataset.camParams.fu;
  sharedIntrinsics->fv = dataset.camParams.fv;
  sharedIntrinsics->cu = dataset.camParams.cu;
  sharedIntrinsics->cv = dataset.camParams.cv;

  // Generate cost terms for camera measurements
  for (unsigned int i = 0; i < dataset.meas.size(); i++) {

    // Get pose reference
    steam::se3::TransformStateVar::Ptr& poseVar = poses_ic_k_0[dataset.meas[i].frameID];

    // Get landmark reference
    steam::se3::LandmarkStateVar::Ptr& landVar = landmarks_ic[dataset.meas[i].landID];

    // Construct transform evaluator between landmark frame (inertial) and camera frame
    steam::se3::TransformEvaluator::Ptr pose_c_v = steam::se3::FixedTransformEvaluator::MakeShared(dataset.T_cv);
    steam::se3::TransformEvaluator::Ptr pose_v_0 = steam::se3::TransformStateEvaluator::MakeShared(poseVar);
    steam::se3::TransformEvaluator::Ptr pose_c_0 = steam::se3::compose(pose_c_v, pose_v_0);

    // Construct error function
    steam::StereoCameraErrorEval::Ptr errorfunc(new steam::StereoCameraErrorEval(
            dataset.meas[i].data, sharedIntrinsics, pose_c_0, landVar));

    // Construct cost term
    steam::WeightedLeastSqCostTerm<4,6>::Ptr cost(new steam::WeightedLeastSqCostTerm<4,6>(errorfunc, sharedCameraNoiseModel, sharedLossFunc));
    stereoCostTerms->add(cost);
  }

  ///
  /// Make Optimization Problem
  ///

  // Initialize problem
  steam::OptimizationProblem problem;

  // Add pose variables
  for (unsigned int i = 1; i < poses_ic_k_0.size(); i++) {
    problem.addStateVariable(poses_ic_k_0[i]);
  }

  // Add landmark variables
  for (unsigned int i = 0; i < landmarks_ic.size(); i++) {
    problem.addStateVariable(landmarks_ic[i]);
  }

  // Add cost terms
  problem.addCostTerm(stereoCostTerms);

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

  return 0;
}
