//////////////////////////////////////////////////////////////////////////////////////////////
/// \file SolverBase.cpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/solver/SolverBase.hpp>

#include <iostream>

#include <steam/common/Timer.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
SolverBase::SolverBase(OptimizationProblem* problem) : problem_(problem),
    firstBackup_(true), pendingProposedState_(false),
    currIteration_(0), solverConverged_(false),
    term_(TERMINATE_NOT_YET_TERMINATED) {

  // Set current cost from initial problem
  currCost_ = prevCost_ = problem_->cost();

  // Set up state vector -- add all states that are not locked to vector
  const std::vector<StateVariableBase::Ptr>& stateRef = problem_->getStateVariables();
  for (unsigned int i = 0; i < stateRef.size(); i++) {
    const StateVariableBase::Ptr& stateVarRef = stateRef.at(i);
    if (!stateVarRef->isLocked()) {
      stateVec_.addStateVariable(stateVarRef);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not the solver is converged
//////////////////////////////////////////////////////////////////////////////////////////////
bool SolverBase::converged() const {
  return solverConverged_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Perform an iteration of the solver
//////////////////////////////////////////////////////////////////////////////////////////////
void SolverBase::iterate() {

  // Check is solver has already converged
  if (solverConverged_) {
    std::cout << "[STEAM WARN] Requested an iteration when solver has already converged, iteration ignored.";
    return;
  }

  // Log on first iteration
  if (this->getSolverBaseParams().verbose && currIteration_ == 0) {
    std::cout << "Begin Optimization" << std::endl;
    std::cout << "------------------" << std::endl;
    std::cout << "Number of States: " << this->getStateVector().getNumberOfStates() << std::endl;
    std::cout << "Number of Cost Terms: " << problem_->getNumberOfCostTerms() << std::endl;
    std::cout << "Initial Cost: " << currCost_ << std::endl;
  }

  // Update iteration number
  currIteration_++;

  // Record previous iteration cost
  prevCost_ = currCost_;

  // Perform an iteration of the implemented solver-type
  double gradientNorm = 0.0;
  bool stepSuccess = linearizeSolveAndUpdate(&currCost_, &gradientNorm);

  // Check termination criteria
  if (!stepSuccess && fabs(gradientNorm) < 1e-6) {
    term_ = TERMINATE_CONVERGED_ZERO_GRADIENT;
    solverConverged_ = true;
  } else if (!stepSuccess) {
    term_ = TERMINATE_STEP_UNSUCCESSFUL;
    solverConverged_ = true;
    throw unsuccessful_step("The steam solver terminated due to being unable to produce a "
                            "'successful' step. If this occurs, it is likely that your problem "
                            "is very nonlinear and poorly initialized, or is using incorrect "
                            "analytical Jacobians.");
  } else if (currIteration_ >= this->getSolverBaseParams().maxIterations) {
    term_ = TERMINATE_MAX_ITERATIONS;
    solverConverged_ = true;
  } else if ( currCost_ <= this->getSolverBaseParams().absoluteCostThreshold ) {
    term_ = TERMINATE_CONVERGED_ABSOLUTE_ERROR;
    solverConverged_ = true;
  } else if ( fabs(prevCost_ - currCost_) <= this->getSolverBaseParams().absoluteCostChangeThreshold ) {
    term_ = TERMINATE_CONVERGED_ABSOLUTE_CHANGE;
    solverConverged_ = true;
  } else if ( fabs(prevCost_ - currCost_)/prevCost_ <= this->getSolverBaseParams().relativeCostChangeThreshold ) {
    term_ = TERMINATE_CONVERGED_RELATIVE_CHANGE;
    solverConverged_ = true;
  }

  // Log on final iteration
  if (this->getSolverBaseParams().verbose && solverConverged_) {
    std::cout << "Termination Cause: " << term_ << std::endl;
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Perform iterations until convergence
///        This function is made to be simple and require no private methods so that users
///        can choose to control the loop themselves.
//////////////////////////////////////////////////////////////////////////////////////////////
void SolverBase::optimize() {
  // Timer
  steam::Timer timer;

  // Optimization loop
  while(!this->converged()) {
    this->iterate();
  }

  // Log
  if (this->getSolverBaseParams().verbose) {
    std::cout << "Total Optimization Time: " << timer.milliseconds() << " ms" << std::endl;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Return termination cause
//////////////////////////////////////////////////////////////////////////////////////////////
SolverBase::Termination SolverBase::getTerminationCause() {
  return term_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Return current iteration number
//////////////////////////////////////////////////////////////////////////////////////////////
unsigned int SolverBase::getCurrIteration() const {
  return currIteration_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Return previous iteration cost evaluation
//////////////////////////////////////////////////////////////////////////////////////////////
double SolverBase::getPrevCost() const {
  return prevCost_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Get reference to optimization problem
//////////////////////////////////////////////////////////////////////////////////////////////
OptimizationProblem& SolverBase::getProblem() {
  return *problem_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Get const reference to optimization problem
//////////////////////////////////////////////////////////////////////////////////////////////
const OptimizationProblem& SolverBase::getProblem() const {
  return *problem_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Get a reference to the state vector
//////////////////////////////////////////////////////////////////////////////////////////////
const StateVector& SolverBase::getStateVector() const {
  return stateVec_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Propose an update to the state vector.
//////////////////////////////////////////////////////////////////////////////////////////////
double SolverBase::proposeUpdate(const Eigen::VectorXd& stateStep) {

  // Check that an update is not already pending
  if (pendingProposedState_) {
    throw std::runtime_error("There is already a pending update, accept "
                             "or reject before proposing a new one.");
  }

  // Make copy of state vector
  if (firstBackup_) {
    stateVectorBackup_ = stateVec_;
    firstBackup_ = false;
  } else {
    stateVectorBackup_.copyValues(stateVec_);
  }

  // Update copy with perturbation
  stateVec_.update(stateStep);
  pendingProposedState_ = true;

  // Test new cost
  return problem_->cost();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Confirm the proposed state update
//////////////////////////////////////////////////////////////////////////////////////////////
void SolverBase::acceptProposedState() {

  // Check that an update has been proposed
  if (!pendingProposedState_) {
    throw std::runtime_error("You must call proposeUpdate before accept.");
  }

  // Switch flag, accepting the update
  pendingProposedState_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Reject the proposed state update and revert to the previous values
//////////////////////////////////////////////////////////////////////////////////////////////
void SolverBase::rejectProposedState() {

  // Check that an update has been proposed
  if (!pendingProposedState_) {
    throw std::runtime_error("You must call proposeUpdate before rejecting.");
  }

  // Revert to previous state
  stateVec_.copyValues(stateVectorBackup_);

  // Switch flag, ready for new proposal
  pendingProposedState_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Print termination cause
//////////////////////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<(std::ostream& out, const SolverBase::Termination& T) {
  switch(T) {
    case SolverBase::TERMINATE_NOT_YET_TERMINATED        : out << "NOT YET TERMINATED"; break;
    case SolverBase::TERMINATE_STEP_UNSUCCESSFUL         : out << "STEP UNSUCCESSFUL"; break;
    case SolverBase::TERMINATE_MAX_ITERATIONS            : out << "MAX ITERATIONS"; break;
    case SolverBase::TERMINATE_CONVERGED_ABSOLUTE_ERROR  : out << "CONVERGED ABSOLUTE ERROR"; break;
    case SolverBase::TERMINATE_CONVERGED_ABSOLUTE_CHANGE : out << "CONVERGED ABSOLUTE CHANGE"; break;
    case SolverBase::TERMINATE_CONVERGED_RELATIVE_CHANGE : out << "CONVERGED RELATIVE CHANGE"; break;
    case SolverBase::TERMINATE_CONVERGED_ZERO_GRADIENT   : out << "CONVERGED GRADIENT IS ZERO"; break;
  }
  return out;
}

} // steam
