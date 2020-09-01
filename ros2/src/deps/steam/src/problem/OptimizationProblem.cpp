//////////////////////////////////////////////////////////////////////////////////////////////
/// \file OptimizationProblem.cpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/problem/OptimizationProblem.hpp>

#include <iomanip>
#include <steam/common/Timer.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Default Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
OptimizationProblem::OptimizationProblem() {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Add an 'active' state variable
//////////////////////////////////////////////////////////////////////////////////////////////
void OptimizationProblem::addStateVariable(const StateVariableBase::Ptr& state) {
  stateVariables_.push_back(state);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Add a cost term (should depend on active states that were added to the problem)
//////////////////////////////////////////////////////////////////////////////////////////////
void OptimizationProblem::addCostTerm(const CostTermBase::ConstPtr& costTerm) {

  if (!costTerm->isImplParallelized()) {
    // Add single-threaded cost term to parallelizer
    singleCostTerms_.add(costTerm);
  } else {
    // Add parallelized cost terms to diff collection
    parallelizedCostTerms_.push_back(costTerm);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Compute the cost from the collection of cost terms
//////////////////////////////////////////////////////////////////////////////////////////////
double OptimizationProblem::cost() const {

  double cost = 0;

  // Add cost of the default dynamic cost terms
  cost += singleCostTerms_.cost();

  // Add cost of the custom cost-term collections
  for (unsigned int c = 0; c < parallelizedCostTerms_.size(); c++) {
    cost += parallelizedCostTerms_[c]->cost();
  }

  return cost;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Get reference to state variables
//////////////////////////////////////////////////////////////////////////////////////////////
const std::vector<StateVariableBase::Ptr>& OptimizationProblem::getStateVariables() const {
  return stateVariables_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Get the total number of cost terms
//////////////////////////////////////////////////////////////////////////////////////////////
unsigned int OptimizationProblem::getNumberOfCostTerms() const {

  unsigned int size = 0;

  // Add number of the default dynamic cost terms
  size += singleCostTerms_.numCostTerms();

  // Add number from the custom cost-term collections
  for (unsigned int c = 0; c < parallelizedCostTerms_.size(); c++) {
    size += parallelizedCostTerms_[c]->numCostTerms();
  }

  return size;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Fill in the supplied block matrices
//////////////////////////////////////////////////////////////////////////////////////////////
void OptimizationProblem::buildGaussNewtonTerms(const StateVector& stateVector,
                                                Eigen::SparseMatrix<double>* approximateHessian,
                                                Eigen::VectorXd* gradientVector) const {

  // Setup Matrices
  std::vector<unsigned int> sqSizes = stateVector.getStateBlockSizes();
  BlockSparseMatrix A_(sqSizes, true);
  BlockVector b_(sqSizes);

  // Add terms from the default dynamic cost terms
  singleCostTerms_.buildGaussNewtonTerms(stateVector, &A_, &b_);

  // Add terms from the custom cost-term collections
  for (unsigned int c = 0; c < parallelizedCostTerms_.size(); c++) {
    parallelizedCostTerms_[c]->buildGaussNewtonTerms(stateVector, &A_, &b_);
  }

  // Convert to Eigen Type - with the block-sparsity pattern
  // ** Note we do not exploit sub-block-sparsity in case it changes at a later iteration
  *approximateHessian = A_.toEigen(false);
  *gradientVector = b_.toEigen();
}

} // steam
