//////////////////////////////////////////////////////////////////////////////////////////////
/// \file LineSearchGaussNewtonSolver.cpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/solver/LineSearchGaussNewtonSolver.hpp>

#include <iostream>

#include <steam/common/Timer.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
LineSearchGaussNewtonSolver::LineSearchGaussNewtonSolver(OptimizationProblem* problem, const Params& params)
  : GaussNewtonSolverBase(problem), params_(params) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Build the system, solve for a step size and direction, and update the state
//////////////////////////////////////////////////////////////////////////////////////////////
bool LineSearchGaussNewtonSolver::linearizeSolveAndUpdate(double* newCost, double* gradNorm) {

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
  Eigen::VectorXd perturbation = this->solveGaussNewton(approximateHessian, gradientVector);
  solveTime = timer.milliseconds();

  // Apply update (w line search)
  timer.reset();

  // Perform line search
  double backtrackCoeff = 1.0; // step size multiplier
  unsigned int nBacktrack = 0;
  for (; nBacktrack < params_.maxBacktrackSteps; nBacktrack++) {
    // Test new cost
    double proposedCost = this->proposeUpdate(backtrackCoeff*perturbation);
    if (proposedCost <= this->getPrevCost()) {
      // cost went down (or is the same, x = 0)
      this->acceptProposedState();
      *newCost = proposedCost;
      break;
    } else {
      // cost went up
      this->rejectProposedState(); // restore old state vector
      backtrackCoeff = params_.backtrackMultiplier*backtrackCoeff; // reduce step size (backtrack)
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
                   << std::right << std::setw(13) << std::setfill(' ') << "searchCoeff"
                   << std::endl;
    }
    std::cout << std::right << std::setw(4)  << std::setfill(' ') << this->getCurrIteration()
              << std::right << std::setw(12) << std::setfill(' ') << std::setprecision(5) << *newCost
              << std::right << std::setw(12) << std::setfill(' ') << std::setprecision(3) << std::fixed << buildTime << std::resetiosflags(std::ios::fixed)
              << std::right << std::setw(12) << std::setfill(' ') << std::setprecision(3) << std::fixed << solveTime << std::resetiosflags(std::ios::fixed)
              << std::right << std::setw(13) << std::setfill(' ') << std::setprecision(3) << std::fixed << updateTime << std::resetiosflags(std::ios::fixed)
              << std::right << std::setw(11) << std::setfill(' ') << std::setprecision(3) << std::fixed << iterTimer.milliseconds() << std::resetiosflags(std::ios::fixed)
              << std::right << std::setw(13) << std::setfill(' ') << std::setprecision(3) << std::fixed << backtrackCoeff << std::resetiosflags(std::ios::fixed)
              << std::endl;
  }

  // Return successfulness
  if (nBacktrack < params_.maxBacktrackSteps) {
    return true;
  } else {
    return false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Casts parameters to base type (for SolverBase class)
//////////////////////////////////////////////////////////////////////////////////////////////
const SolverBase::Params& LineSearchGaussNewtonSolver::getSolverBaseParams() const {
  return params_;
}

} // steam
