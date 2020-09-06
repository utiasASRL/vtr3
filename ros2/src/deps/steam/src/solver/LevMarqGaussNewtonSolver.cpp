//////////////////////////////////////////////////////////////////////////////////////////////
/// \file LevMarqGaussNewtonSolver.cpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/solver/LevMarqGaussNewtonSolver.hpp>

#include <iostream>

#include <steam/common/Timer.hpp>


namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
LevMarqGaussNewtonSolver::LevMarqGaussNewtonSolver(OptimizationProblem* problem, const Params& params)
  : GaussNewtonSolverBase(problem), params_(params) {

  // a small diagonal coefficient means that it is closer to using just a gauss newton step
  diagCoeff = 1e-7;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Solve the Levenberg–Marquardt system of equations:
///        A*x = b, A = (J^T*J + diagonalCoeff*diag(J^T*J))
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd LevMarqGaussNewtonSolver::solveLevMarq(const Eigen::VectorXd& gradientVector,
                                                       double diagonalCoeff) {

  // Augment diagonal of the 'hessian' matrix
  for (int i = 0; i < approximateHessian_.outerSize(); i++) {
    approximateHessian_.coeffRef(i,i) *= (1.0 + diagonalCoeff);
  }

  // Solve system
  Eigen::VectorXd levMarqStep;
  try {

    // Solve for the LM step using the Cholesky factorization (fast)
    levMarqStep = this->solveGaussNewton(approximateHessian_, gradientVector, true);

  } catch (const decomp_failure& ex) {

    // Revert diagonal of the 'hessian' matrix
    for (int i = 0; i < approximateHessian_.outerSize(); i++) {
      approximateHessian_.coeffRef(i,i) /= (1.0 + diagonalCoeff);
    }

    // Throw up again
    throw ex;
  }

  // Revert diagonal of the 'hessian' matrix
  for (int i = 0; i < approximateHessian_.outerSize(); i++) {
    approximateHessian_.coeffRef(i,i) /= (1.0 + diagonalCoeff);
  }

  return levMarqStep;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Perform a plain LLT decomposition on the approx. Hessian matrix in
///        order to solve for the proper covariances (unmodified by the LM diagonal)
//////////////////////////////////////////////////////////////////////////////////////////////
void LevMarqGaussNewtonSolver::solveCovariances() {

  // Factorize the unaugmented hessian (the covariance matrix)
  this->factorizeHessian(approximateHessian_, false);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Build the system, solve for a step size and direction, and update the state
//////////////////////////////////////////////////////////////////////////////////////////////
bool LevMarqGaussNewtonSolver::linearizeSolveAndUpdate(double* newCost, double* gradNorm) {

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
  double actualToPredictedRatio = 0;
  unsigned int numTrDecreases = 0;

  // Initialize new cost with old cost incase of failure
  *newCost = this->getPrevCost();

  // The 'right-hand-side' of the Gauss-Newton problem, generally known as the gradient vector
  Eigen::VectorXd gradientVector;

  // Construct system of equations
  timer.reset();
  this->buildGaussNewtonTerms(&approximateHessian_, &gradientVector);
  *gradNorm = gradientVector.norm();
  buildTime = timer.milliseconds();

  // Perform LM Search
  unsigned int nBacktrack = 0;
  for (; nBacktrack < params_.maxShrinkSteps; nBacktrack++) {

    // Solve system
    timer.reset();
    bool decompSuccess = true;
    Eigen::VectorXd levMarqStep;
    try {

      // Solve system
      levMarqStep = this->solveLevMarq(gradientVector, diagCoeff);
    } catch (const decomp_failure& e) {
      decompSuccess = false;
    }
    solveTime += timer.milliseconds();

    // Test new cost
    timer.reset();

    // If decomposition was successful, calculate step quality
    double proposedCost = 0;
    if (decompSuccess) {

      // Calculate the predicted reduction; note that a positive value denotes a reduction in cost
      proposedCost = this->proposeUpdate(levMarqStep);
      double actualReduc = this->getPrevCost() - proposedCost;
      double predictedReduc = this->predictedReduction(approximateHessian_, gradientVector, levMarqStep);
      actualToPredictedRatio = actualReduc/predictedReduc;
    }

    // Check ratio of predicted reduction to actual reduction achieved
    if (actualToPredictedRatio > params_.ratioThreshold && decompSuccess) {

      // Good enough ratio to accept proposed state
      this->acceptProposedState();
      *newCost = proposedCost;
      diagCoeff = std::max(diagCoeff*params_.shrinkCoeff, 1e-7); // move towards gauss newton

      // Timing
      updateTime += timer.milliseconds();
      break;
    } else {

      // Cost did not reduce enough, possibly increased, or decomposition failed.
      // Reject proposed state and reduce the size of the trust region
      if (decompSuccess) {
        // Restore old state vector
        this->rejectProposedState();
      }
      diagCoeff = std::min(diagCoeff*params_.growCoeff, 1e7); // Move towards gradient descent
      numTrDecreases++; // Count number of shrinks for logging

      // Timing
      updateTime += timer.milliseconds();
    }
  }

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
const SolverBase::Params& LevMarqGaussNewtonSolver::getSolverBaseParams() const {
  return params_;
}

} // steam

