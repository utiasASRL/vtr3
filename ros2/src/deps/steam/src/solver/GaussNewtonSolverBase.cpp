//////////////////////////////////////////////////////////////////////////////////////////////
/// \file GaussNewtonSolverBase.cpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/solver/GaussNewtonSolverBase.hpp>

#include <iostream>
#include <Eigen/Cholesky>

#include <steam/common/Timer.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
GaussNewtonSolverBase::GaussNewtonSolverBase(OptimizationProblem* problem) :
  SolverBase(problem), patternInitialized_(false), factorizedInformationSuccesfully_(false) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Query the covariance related to a single state variable
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd GaussNewtonSolverBase::queryCovariance(const steam::StateKey& key) {

  return queryCovariance(key, key);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Query the covariance relating two state variables
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd GaussNewtonSolverBase::queryCovariance(const steam::StateKey& rowKey,
                                                       const steam::StateKey& colKey) {

  std::vector<steam::StateKey> rkeys; rkeys.push_back(rowKey);
  std::vector<steam::StateKey> ckeys; ckeys.push_back(colKey);
  BlockMatrix m = queryCovarianceBlock(rkeys, ckeys);
  return m.at(0,0);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Query a block of covariances
//////////////////////////////////////////////////////////////////////////////////////////////
BlockMatrix GaussNewtonSolverBase::queryCovarianceBlock(const std::vector<steam::StateKey>& keys) {

  return queryCovarianceBlock(keys, keys);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Query a block of covariances
//////////////////////////////////////////////////////////////////////////////////////////////
BlockMatrix GaussNewtonSolverBase::queryCovarianceBlock(const std::vector<steam::StateKey>& rowKeys,
                                                        const std::vector<steam::StateKey>& colKeys) {

  // Check if the Hessian has been factorized (without augmentation, i.e. the Information matrix)
  if (!factorizedInformationSuccesfully_) {
    throw std::runtime_error("Cannot query covariance, as the plain approximate Hessian "
                             "was not factorized properly. If using LevMarq, you may "
                             "have to call solveCovariances().");
  }

  // Creating indexing
  BlockMatrixIndexing indexing(this->getStateVector().getStateBlockSizes());
  const BlockDimIndexing& blkRowIndexing = indexing.rowIndexing();
  const BlockDimIndexing& blkColIndexing = indexing.colIndexing();

  // Fixed sizes
  unsigned int numRowKeys = rowKeys.size();
  unsigned int numColKeys = colKeys.size();

  // Look up block indexes
  std::vector<unsigned int> blkRowIndices; blkRowIndices.resize(numRowKeys);
  for (unsigned int i = 0; i < numRowKeys; i++) {
    blkRowIndices[i] = this->getStateVector().getStateBlockIndex(rowKeys[i]);
  }
  std::vector<unsigned int> blkColIndices; blkColIndices.resize(numColKeys);
  for (unsigned int i = 0; i < numColKeys; i++) {
    blkColIndices[i] = this->getStateVector().getStateBlockIndex(colKeys[i]);
  }

  // Look up block size of state variables
  std::vector<unsigned int> blkRowSizes; blkRowSizes.resize(numRowKeys);
  for (unsigned int i = 0; i < numRowKeys; i++) {
    blkRowSizes[i] = blkRowIndexing.blkSizeAt(blkRowIndices[i]);
  }
  std::vector<unsigned int> blkColSizes; blkColSizes.resize(numColKeys);
  for (unsigned int i = 0; i < numColKeys; i++) {
    blkColSizes[i] = blkColIndexing.blkSizeAt(blkColIndices[i]);
  }

  // Create result container
  BlockMatrix result(blkRowSizes, blkColSizes);

  // For each column key
  for (unsigned int c = 0; c < numColKeys; c++) {

    // For each scalar column
    Eigen::VectorXd projection(blkRowIndexing.scalarSize()); projection.setZero();
    for (unsigned int j = 0; j < blkColSizes[c]; j++) {

      // Get scalar index
      unsigned int scalarColIndex = blkColIndexing.cumSumAt(blkColIndices[c]) + j;

      // Solve for scalar column of covariance matrix
      projection(scalarColIndex) = 1.0;
      Eigen::VectorXd x = hessianSolver_.solve(projection);
      projection(scalarColIndex) = 0.0;

      // For each block row
      for (unsigned int r = 0; r < numRowKeys; r++) {

        // Get scalar index into solution vector
        unsigned int scalarRowIndex = blkRowIndexing.cumSumAt(blkRowIndices[r]);

        // Do the backward pass, using the Cholesky factorization (fast)
        result.at(r,c).block(0, j, blkRowSizes[r], 1) = x.block(scalarRowIndex, 0, blkRowSizes[r], 1);
      }
    }
  }

  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Build the left-hand and right-hand sides of the Gauss-Newton system of equations
//////////////////////////////////////////////////////////////////////////////////////////////
void GaussNewtonSolverBase::buildGaussNewtonTerms(Eigen::SparseMatrix<double>* approximateHessian,
                                                  Eigen::VectorXd* gradientVector) {
  this->getProblem().buildGaussNewtonTerms(this->getStateVector(), approximateHessian, gradientVector);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Perform the LLT decomposition on the approx. Hessian matrix
//////////////////////////////////////////////////////////////////////////////////////////////
void GaussNewtonSolverBase::factorizeHessian(const Eigen::SparseMatrix<double>& approximateHessian,
                                             bool augmentedHessian) {

  // Check if the pattern has been initialized
  if (!patternInitialized_) {

    // The first time we are solving the problem we need to analyze the sparsity pattern
    // ** Note we use approximate-minimal-degree (AMD) reordering.
    //    Also, this step does not actually use the numerical values in gaussNewtonLHS
    hessianSolver_.analyzePattern(approximateHessian);
    patternInitialized_ = true;
  }

  // Perform a Cholesky factorization of the approximate Hessian matrix
  factorizedInformationSuccesfully_ = false;
  hessianSolver_.factorize(approximateHessian);

  // Check if the factorization succeeded
  if (hessianSolver_.info() != Eigen::Success) {
    throw decomp_failure("During steam solve, Eigen LLT decomposition failed. "
                         "It is possible that the matrix was ill-conditioned, in which case "
                         "adding a prior may help. On the other hand, it is also possible that "
                         "the problem you've constructed is not positive semi-definite.");
  } else {

    // Information matrix was solved successfully, if the hessian was not augmented
    factorizedInformationSuccesfully_ = !augmentedHessian;
  }

  // todo - it would be nice to check the condition number (not just the determinant) of the
  // solved system... need to find a fast way to do this
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Solve the Gauss-Newton system of equations: A*x = b
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd GaussNewtonSolverBase::solveGaussNewton(const Eigen::SparseMatrix<double>& approximateHessian,
                                                        const Eigen::VectorXd& gradientVector,
                                                        bool augmentedHessian) {

  // Perform a Cholesky factorization of the approximate Hessian matrix
  this->factorizeHessian(approximateHessian, augmentedHessian);

  // Do the backward pass, using the Cholesky factorization (fast)
  return hessianSolver_.solve(gradientVector);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Find the Cauchy point (used for the Dogleg method).
///        The cauchy point is the optimal step length in the gradient descent direction.
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd GaussNewtonSolverBase::getCauchyPoint(const Eigen::SparseMatrix<double>& approximateHessian,
                                                      const Eigen::VectorXd& gradientVector) {
  double num = gradientVector.squaredNorm();
  double den = gradientVector.transpose() *
               (approximateHessian.selfadjointView<Eigen::Upper>() * gradientVector);
  return (num/den)*gradientVector;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Get the predicted cost reduction based on the proposed step
//////////////////////////////////////////////////////////////////////////////////////////////
double GaussNewtonSolverBase::predictedReduction(const Eigen::SparseMatrix<double>& approximateHessian,
                                                 const Eigen::VectorXd& gradientVector,
                                                 const Eigen::VectorXd& step) {
  // grad^T * step - 0.5 * step^T * Hessian * step
  double gradTransStep = gradientVector.transpose() * step;
  double stepTransHessianStep = step.transpose()
                                * (approximateHessian.selfadjointView<Eigen::Upper>() * step);
  return gradTransStep - 0.5 * stepTransHessianStep;
}

} // steam

