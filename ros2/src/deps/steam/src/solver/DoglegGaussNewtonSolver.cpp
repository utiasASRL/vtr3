//////////////////////////////////////////////////////////////////////////////////////////////
/// \file DoglegGaussNewtonSolver.cpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/solver/DoglegGaussNewtonSolver.hpp>

#include <iostream>

#include <steam/common/Timer.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
DoglegGaussNewtonSolver::DoglegGaussNewtonSolver(OptimizationProblem* problem, const Params& params)
  : GaussNewtonSolverBase(problem), params_(params) {

  trustRegionSize = 0.0; // a trust region of 0.0 indicates it is uninitialized
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Build the system, solve for a step size and direction, and update the state
//////////////////////////////////////////////////////////////////////////////////////////////
bool DoglegGaussNewtonSolver::linearizeSolveAndUpdate(double* newCost, double* gradNorm) {

  if (newCost == NULL) {
    throw std::invalid_argument("Null pointer provided to return-input "
                                "'newCost' in linearizeSolveAndUpdate");
  }

  // Logging variables
  steam::Timer iterTimer;
  steam::Timer timer;
  double buildTime = 0;
  double solveTime = 0;
  double updateTime = 0;
  double actualToPredictedRatio;
  std::string doglegSegment;
  unsigned int numTrDecreases = 0;

  // Initialize new cost with old cost incase of failure
  *newCost = this->getPrevCost();

  // The 'left-hand-side' of the Gauss-Newton problem, generally known as the
  // approximate Hessian matrix (note we only store the upper-triangular elements)
  Eigen::SparseMatrix<double> approximateHessian;

  // The 'right-hand-side' of the Gauss-Newton problem, generally known as the gradient vector
  Eigen::VectorXd gradientVector;

  // Construct system of equations
  timer.reset();
  this->buildGaussNewtonTerms(&approximateHessian, &gradientVector);
  *gradNorm = gradientVector.norm();
  buildTime = timer.milliseconds();

  // Solve system
  timer.reset();

  // Get gradient descent step
  Eigen::VectorXd gradDescentStep = this->getCauchyPoint(approximateHessian, gradientVector);
  double gradDescentNorm = gradDescentStep.norm();

  // Get Gauss-Newton step
  Eigen::VectorXd gaussNewtonStep = this->solveGaussNewton(approximateHessian, gradientVector);
  double gaussNewtonNorm = gaussNewtonStep.norm();

  solveTime = timer.milliseconds();

  // Apply update (w line search)
  timer.reset();

  // Initialize trust region size (if first time)
  if (trustRegionSize == 0.0) {
    trustRegionSize = gaussNewtonNorm;
  }

  // Perform dogleg step
  unsigned int nBacktrack = 0;
  Eigen::VectorXd dogLegStep;   // initialize outside of loop to avoid reallocation
  Eigen::VectorXd gdToGnVector; // initialize outside of loop to avoid reallocation
  for (; nBacktrack < params_.maxShrinkSteps; nBacktrack++) {

    // Get step
    if (gaussNewtonNorm <= trustRegionSize) {

      // Trust region larger than Gauss Newton step
      dogLegStep = gaussNewtonStep;
      doglegSegment = "Gauss Newton";
    } else if (gradDescentNorm >= trustRegionSize) {

      // Trust region smaller than Gradient Descent step (Cauchy point)
      dogLegStep = (trustRegionSize/gradDescentNorm)*gradDescentStep;

      // For verbose
      doglegSegment = "Grad Descent";

    } else {

      // Trust region lies between the GD and GN steps, use interpolation
      if (gaussNewtonStep.rows() != gradDescentStep.rows()) {
        throw std::logic_error("Gauss-Newton and gradient descent dimensions did not match.");
      }

      // Get interpolation direction
      gdToGnVector = gaussNewtonStep - gradDescentStep;

      // Calculate interpolation constant
      double gdDotProdGdToGn = gradDescentStep.transpose()*gdToGnVector;
      double gdToGnSqrNorm = gdToGnVector.squaredNorm();
      double interpConst = (- gdDotProdGdToGn
                     + sqrt(gdDotProdGdToGn*gdDotProdGdToGn + (trustRegionSize*trustRegionSize
                     - gradDescentNorm*gradDescentNorm)*gdToGnSqrNorm) ) / gdToGnSqrNorm;

      // Interpolate step
      dogLegStep = gradDescentStep + interpConst*gdToGnVector;
      doglegSegment = "Interp GN&GD";
    }

    // Calculate the predicted reduction; note that a positive value denotes a reduction in cost
    double proposedCost = this->proposeUpdate(dogLegStep);
    double actualReduc = this->getPrevCost() - proposedCost;
    double predictedReduc = this->predictedReduction(approximateHessian, gradientVector, dogLegStep);
    actualToPredictedRatio = actualReduc/predictedReduc;

    // Check ratio of predicted reduction to actual reduction achieved
    if (actualToPredictedRatio > params_.ratioThresholdShrink) {

      // Good enough ratio to accept proposed state
      this->acceptProposedState();
      *newCost = proposedCost;
      if (actualToPredictedRatio > params_.ratioThresholdGrow) {
        // Ratio is strong enough to increase trust region size
        // Note: we take the max below, so that if the trust region is already much larger
        //   than the steps we are taking, we do not grow it unnecessarily
        trustRegionSize = std::max(trustRegionSize, params_.growCoeff*dogLegStep.norm());
      }
      break;
    } else {

      // Cost did not reduce enough, or possibly increased,
      // reject proposed state and reduce the size of the trust region
      this->rejectProposedState(); // Restore old state vector
      trustRegionSize *= params_.shrinkCoeff; // Reduce step size (backtrack)
      numTrDecreases++; // Count number of shrinks for logging
    }
  }

  updateTime = timer.milliseconds();

  // Print report line if verbose option enabled
  if (params_.verbose) {
    if (this->getCurrIteration() == 1) {
        std::cout  << std::right << std::setw( 4) << std::setfill(' ') << "iter"
                   << std::right << std::setw(12) << std::setfill(' ') << "cost"
                   << std::right << std::setw(12) << std::setfill(' ') << "build (ms)"
                   << std::right << std::setw(12) << std::setfill(' ') << "solve (ms)"
                   << std::right << std::setw(13) << std::setfill(' ') << "update (ms)"
                   << std::right << std::setw(11) << std::setfill(' ') << "time (ms)"
                   << std::right << std::setw(11) << std::setfill(' ') << "TR shrink"
                   << std::right << std::setw(11) << std::setfill(' ') << "AvP Ratio"
                   << std::right << std::setw(16) << std::setfill(' ') << "dogleg segment"
                   << std::endl;
    }

    std::cout << std::right << std::setw(4)  << std::setfill(' ') << this->getCurrIteration()
              << std::right << std::setw(12) << std::setfill(' ') << std::setprecision(5) << *newCost
              << std::right << std::setw(12) << std::setfill(' ') << std::setprecision(3) << std::fixed << buildTime << std::resetiosflags(std::ios::fixed)
              << std::right << std::setw(12) << std::setfill(' ') << std::setprecision(3) << std::fixed << solveTime << std::resetiosflags(std::ios::fixed)
              << std::right << std::setw(13) << std::setfill(' ') << std::setprecision(3) << std::fixed << updateTime << std::resetiosflags(std::ios::fixed)
              << std::right << std::setw(11) << std::setfill(' ') << std::setprecision(3) << std::fixed << iterTimer.milliseconds() << std::resetiosflags(std::ios::fixed)
              << std::right << std::setw(11) << std::setfill(' ') << numTrDecreases
              << std::right << std::setw(11) << std::setfill(' ') << std::setprecision(3) << std::fixed << actualToPredictedRatio << std::resetiosflags(std::ios::fixed)
              << std::right << std::setw(16) << std::setfill(' ') << doglegSegment
              << std::endl;
  }

  // Return successfulness
  if (nBacktrack < params_.maxShrinkSteps) {
    return true;
  } else {
    return false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Casts parameters to base type (for SolverBase class)
//////////////////////////////////////////////////////////////////////////////////////////////
const SolverBase::Params& DoglegGaussNewtonSolver::getSolverBaseParams() const {
  return params_;
}

} // steam
