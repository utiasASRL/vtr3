//////////////////////////////////////////////////////////////////////////////////////////////
/// \file ParallelizedCostTermCollection.cpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/problem/ParallelizedCostTermCollection.hpp>

#include <iostream>
#include <steam/common/Timer.hpp>

#include <omp.h>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
ParallelizedCostTermCollection::ParallelizedCostTermCollection(unsigned int numThreads)
  : numThreads_(numThreads) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Add a cost term
//////////////////////////////////////////////////////////////////////////////////////////////
void ParallelizedCostTermCollection::add(const CostTermBase::ConstPtr& costTerm) {

  if (costTerm->isImplParallelized()) {
    throw std::runtime_error("Do not add pre-parallelized cost "
                             "terms to a cost term parallelizer.");
  }
  costTerms_.push_back(costTerm);
}

std::vector<double> ParallelizedCostTermCollection::costs() const {
  std::vector<double> costs;
  for (auto &cost_term : costTerms_) {
    costs.push_back(cost_term->cost());
  }
  return costs;
}
//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Compute the cost from the collection of cost terms
//////////////////////////////////////////////////////////////////////////////////////////////
double ParallelizedCostTermCollection::cost() const {

  // Init
  double cost = 0;

  // Set number of OpenMP threads
  omp_set_num_threads(numThreads_);

  // Parallelize for the cost terms
  #pragma omp parallel
  {
    #pragma omp for reduction(+:cost)
    for(unsigned int i = 0; i < costTerms_.size(); i++) {
      try {
        double cost_i = costTerms_.at(i)->cost();
        if(std::isnan(cost_i)) {
          std::cout << "nan cost term!";
        } else {
          cost += cost_i;
        }
      } catch (const std::exception & e) {
        std::cout << "STEAM exception in parallel cost term:\n" << e.what() << std::endl;
      } catch (...) {
        std::cout << "STEAM exception in parallel cost term: (unknown)" << std::endl;
      }
    }
  }
  return cost;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns the number of cost terms contained by this object
//////////////////////////////////////////////////////////////////////////////////////////////
unsigned int ParallelizedCostTermCollection::numCostTerms() const {
  return costTerms_.size();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not the implementation already uses multi-threading
//////////////////////////////////////////////////////////////////////////////////////////////
bool ParallelizedCostTermCollection::isImplParallelized() const {
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Build the left-hand and right-hand sides of the Gauss-Newton system of equations
///        using the cost terms in this collection.
//////////////////////////////////////////////////////////////////////////////////////////////
void ParallelizedCostTermCollection::buildGaussNewtonTerms(
    const StateVector& stateVector,
    BlockSparseMatrix* approximateHessian,
    BlockVector* gradientVector) const {

  // Locally disable any internal eigen multithreading -- we do our own OpenMP
  Eigen::setNbThreads(1);

  // Set number of OpenMP threads
  omp_set_num_threads(numThreads_);

  // Parallelize for the cost terms
  #pragma omp parallel
  {
    #pragma omp for
    for (unsigned int c = 0 ; c < costTerms_.size(); c++) {
      try {
        costTerms_.at(c)->buildGaussNewtonTerms(stateVector, approximateHessian, gradientVector);
      } catch (const std::exception & e) {
        std::cout << "STEAM exception in parallel cost term:\n" << e.what() << std::endl;
      } catch (...) {
        std::cout << "STEAM exception in parallel cost term: (unknown)" << std::endl;
      }
    } // end cost term loop
  } // end parallel
}

} // steam
