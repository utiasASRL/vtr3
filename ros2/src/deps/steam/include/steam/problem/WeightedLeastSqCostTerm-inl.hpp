//////////////////////////////////////////////////////////////////////////////////////////////
/// \file WeightedLeastSqCostTerm-inl.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/problem/WeightedLeastSqCostTerm.hpp>

#include <iostream>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
template <int MEAS_DIM, int MAX_STATE_SIZE>
WeightedLeastSqCostTerm<MEAS_DIM,MAX_STATE_SIZE>::WeightedLeastSqCostTerm(
    const typename ErrorEvaluator<MEAS_DIM,MAX_STATE_SIZE>::ConstPtr& errorFunction,
    const typename BaseNoiseModel<MEAS_DIM>::ConstPtr& noiseModel,
    const LossFunctionBase::ConstPtr& lossFunc) :
  errorFunction_(errorFunction), noiseModel_(noiseModel), lossFunc_(lossFunc) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the cost of this term. Error is first whitened by the noise model
///        and then passed through the loss function, as in
///          cost = loss(sqrt(e^T * cov^{-1} * e))
//////////////////////////////////////////////////////////////////////////////////////////////
template <int MEAS_DIM, int MAX_STATE_SIZE>
double WeightedLeastSqCostTerm<MEAS_DIM,MAX_STATE_SIZE>::cost() const
{
  return lossFunc_->cost(noiseModel_->getWhitenedErrorNorm(errorFunction_->evaluate()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns the number of cost terms contained by this object (typically 1)
//////////////////////////////////////////////////////////////////////////////////////////////
template <int MEAS_DIM, int MAX_STATE_SIZE>
unsigned int WeightedLeastSqCostTerm<MEAS_DIM,MAX_STATE_SIZE>::numCostTerms() const {
  return 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not the implementation already uses multi-threading
//////////////////////////////////////////////////////////////////////////////////////////////
template <int MEAS_DIM, int MAX_STATE_SIZE>
bool WeightedLeastSqCostTerm<MEAS_DIM,MAX_STATE_SIZE>::isImplParallelized() const {
  return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Add the contribution of this cost term to the left-hand (Hessian) and right-hand
///        (gradient vector) sides of the Gauss-Newton system of equations.
//////////////////////////////////////////////////////////////////////////////////////////////
template <int MEAS_DIM, int MAX_STATE_SIZE>
void WeightedLeastSqCostTerm<MEAS_DIM,MAX_STATE_SIZE>::buildGaussNewtonTerms(
    const StateVector& stateVector,
    BlockSparseMatrix* approximateHessian,
    BlockVector* gradientVector) const {

  // Get square block indices (we know the hessian is block-symmetric)
  const std::vector<unsigned int>& blkSizes =
      approximateHessian->getIndexing().rowIndexing().blkSizes();

  // Init dynamic matrices
  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,0,MAX_STATE_SIZE,MAX_STATE_SIZE> newHessianTerm;
  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,0,MAX_STATE_SIZE,1> newGradTerm;

  // Compute the weighted and whitened errors and jacobians
  // err = sqrt(w)*sqrt(R^-1)*rawError
  // jac = sqrt(w)*sqrt(R^-1)*rawJacobian
  std::vector<Jacobian<MEAS_DIM,MAX_STATE_SIZE> > jacobians;
  Eigen::Matrix<double,MEAS_DIM,1> error = this->evalWeightedAndWhitened(&jacobians);

  // For each jacobian
  for (unsigned int i = 0; i < jacobians.size(); i++) {

    // Get the key and state range affected
    unsigned int blkIdx1 = stateVector.getStateBlockIndex(jacobians[i].key);

    // Calculate terms needed to update the right-hand-side
    unsigned int size1 = blkSizes.at(blkIdx1);
    newGradTerm = (-1)*jacobians[i].jac.leftCols(size1).transpose()*error;

    // Update the right-hand side (thread critical)
    #pragma omp critical(b_update)
    {
      gradientVector->mapAt(blkIdx1) += newGradTerm;
    }

    // For each jacobian (in upper half)
    for (unsigned int j = i; j < jacobians.size(); j++) {

      // Get the key and state range affected
      unsigned int blkIdx2 = stateVector.getStateBlockIndex(jacobians[j].key);

      // Calculate terms needed to update the Gauss-Newton left-hand side
      unsigned int size2 = blkSizes.at(blkIdx2);
      unsigned int row;
      unsigned int col;
      if (blkIdx1 <= blkIdx2) {
        row = blkIdx1;
        col = blkIdx2;
        newHessianTerm = jacobians[i].jac.leftCols(size1).transpose()*jacobians[j].jac.leftCols(size2);
      } else {
        row = blkIdx2;
        col = blkIdx1;
        newHessianTerm = jacobians[j].jac.leftCols(size2).transpose()*jacobians[i].jac.leftCols(size1);
      }

      // Update the left-hand side (thread critical)
      BlockSparseMatrix::BlockRowEntry& entry = approximateHessian->rowEntryAt(row, col, true);
      omp_set_lock(&entry.lock);
      entry.data += newHessianTerm;
      omp_unset_lock(&entry.lock);

    } // end row loop
  } // end column loop
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the iteratively reweighted error vector and Jacobians. The error and
///        Jacobians are first whitened by the noise model and then weighted by the loss
///        function, as in:
///              error = sqrt(weight)*sqrt(cov^-1)*rawError
///           jacobian = sqrt(weight)*sqrt(cov^-1)*rawJacobian
//////////////////////////////////////////////////////////////////////////////////////////////
template <int MEAS_DIM, int MAX_STATE_SIZE>
Eigen::Matrix<double,MEAS_DIM,1> WeightedLeastSqCostTerm<MEAS_DIM,MAX_STATE_SIZE>::evalWeightedAndWhitened(
    std::vector<Jacobian<MEAS_DIM,MAX_STATE_SIZE> >* outJacobians) const {

  // Check and initialize jacobian array
  if (outJacobians == NULL) {
    throw std::invalid_argument("Null pointer provided to return-input 'jacs' in evaluate");
  }
  outJacobians->clear();

  // Get raw error and Jacobians
  Eigen::Matrix<double,MEAS_DIM,1> rawError =
      errorFunction_->evaluate(noiseModel_->getSqrtInformation(), outJacobians);

  // Get whitened error vector
  Eigen::Matrix<double,MEAS_DIM,1> whiteError = noiseModel_->whitenError(rawError);

  // Get weight from loss function
  double sqrt_w = sqrt(lossFunc_->weight(whiteError.norm()));

  // Weight the white jacobians
  for (unsigned int i = 0; i < outJacobians->size(); i++) {
    outJacobians->at(i).jac *= sqrt_w;
  }

  // Weight the error and return
  return sqrt_w * whiteError;
}

} // steam
