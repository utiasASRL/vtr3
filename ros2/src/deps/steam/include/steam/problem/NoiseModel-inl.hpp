//////////////////////////////////////////////////////////////////////////////////////////////
/// \file StaticNoiseModel-inl.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/problem/NoiseModel.hpp>

#include <iostream>
#include <stdexcept>

#include <Eigen/Cholesky>

namespace steam {


template<int MEAS_DIM>
DynamicNoiseModel<MEAS_DIM>::DynamicNoiseModel(boost::shared_ptr<NoiseEvaluator<MEAS_DIM>> eval) :
eval_(eval) {
  this->setByCovariance(eval_->evaluateCovariance());
}


//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Get a reference to the square root information matrix
//////////////////////////////////////////////////////////////////////////////////////////////
template<int MEAS_DIM>
const Eigen::Matrix<double,MEAS_DIM,MEAS_DIM>& DynamicNoiseModel<MEAS_DIM>::getSqrtInformation() const {
  this->setByCovariance(eval_->evaluateCovariance());
  return this->sqrtInformation_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Get the norm of the whitened error vector, sqrt(rawError^T * info * rawError)
//////////////////////////////////////////////////////////////////////////////////////////////
template<int MEAS_DIM>
double DynamicNoiseModel<MEAS_DIM>::getWhitenedErrorNorm(
    const Eigen::Matrix<double,MEAS_DIM,1>& rawError) const {
  this->setByCovariance(eval_->evaluateCovariance());
  return (this->sqrtInformation_*rawError).norm();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Get the whitened error vector, sqrtInformation*rawError
//////////////////////////////////////////////////////////////////////////////////////////////
template<int MEAS_DIM>
Eigen::Matrix<double,MEAS_DIM,1> DynamicNoiseModel<MEAS_DIM>::whitenError(
    const Eigen::Matrix<double,MEAS_DIM,1>& rawError) const {
  this->setByCovariance(eval_->evaluateCovariance());
  return this->sqrtInformation_*rawError;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Default constructor
//////////////////////////////////////////////////////////////////////////////////////////////
template<int MEAS_DIM>
StaticNoiseModel<MEAS_DIM>::StaticNoiseModel() {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief General constructor
//////////////////////////////////////////////////////////////////////////////////////////////
template<int MEAS_DIM>
BaseNoiseModel<MEAS_DIM>::BaseNoiseModel(const Eigen::Matrix<double,MEAS_DIM,MEAS_DIM>& matrix,
                                 MatrixType type) {

  // Depending on the type of 'matrix', we set the internal storage
  switch(type) {
    case COVARIANCE :
      setByCovariance(matrix);
      break;
    case INFORMATION :
      setByInformation(matrix);
      break;
    case SQRT_INFORMATION :
      setBySqrtInformation(matrix);
      break;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Set by covariance matrix
//////////////////////////////////////////////////////////////////////////////////////////////
template<int MEAS_DIM>
void BaseNoiseModel<MEAS_DIM>::setByCovariance(
    const Eigen::Matrix<double,MEAS_DIM,MEAS_DIM>& matrix) const {

  // Information is the inverse of covariance
  this->setByInformation(matrix.inverse());
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Set by information matrix
//////////////////////////////////////////////////////////////////////////////////////////////
template<int MEAS_DIM>
void BaseNoiseModel<MEAS_DIM>::setByInformation(
    const Eigen::Matrix<double,MEAS_DIM,MEAS_DIM>& matrix) const {

  // Check that the matrix is positive definite
  this->assertPositiveDefiniteMatrix(matrix);

  // Perform an LLT decomposition
  Eigen::LLT<Eigen::Matrix<double,MEAS_DIM,MEAS_DIM> > lltOfInformation(matrix);

  // Store upper triangular matrix (the square root information matrix)
  this->setBySqrtInformation(lltOfInformation.matrixL().transpose());
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Set by square root of information matrix
//////////////////////////////////////////////////////////////////////////////////////////////
template<int MEAS_DIM>
void BaseNoiseModel<MEAS_DIM>::setBySqrtInformation(
    const Eigen::Matrix<double,MEAS_DIM,MEAS_DIM>& matrix)  const {

  // Set internal storage matrix
  sqrtInformation_ = matrix; // todo: check this is upper triangular
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Assert that the matrix is positive definite
//////////////////////////////////////////////////////////////////////////////////////////////
template<int MEAS_DIM>
void BaseNoiseModel<MEAS_DIM>::assertPositiveDefiniteMatrix(
    const Eigen::Matrix<double,MEAS_DIM,MEAS_DIM>& matrix)  const{

  // Initialize an eigen value solver
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,MEAS_DIM,MEAS_DIM> >
      eigsolver(matrix, Eigen::EigenvaluesOnly);

  // Check the minimum eigen value
  if (eigsolver.eigenvalues().minCoeff() <= 0) {
    std::stringstream ss; ss << "Covariance \n" << matrix << "\n must be positive definite. "
                             << "Min. eigenvalue : " << eigsolver.eigenvalues().minCoeff();
    throw std::invalid_argument(ss.str());
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief General constructor
//////////////////////////////////////////////////////////////////////////////////////////////
template<int MEAS_DIM>
StaticNoiseModel<MEAS_DIM>::StaticNoiseModel(const Eigen::Matrix<double,MEAS_DIM,MEAS_DIM>& matrix,
                                             MatrixType type) 
: BaseNoiseModel<MEAS_DIM>::BaseNoiseModel(matrix,type) {

}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Get a reference to the square root information matrix
//////////////////////////////////////////////////////////////////////////////////////////////
template<int MEAS_DIM>
const Eigen::Matrix<double,MEAS_DIM,MEAS_DIM>& StaticNoiseModel<MEAS_DIM>::getSqrtInformation() const {
  return this->sqrtInformation_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Get the norm of the whitened error vector, sqrt(rawError^T * info * rawError)
//////////////////////////////////////////////////////////////////////////////////////////////
template<int MEAS_DIM>
double StaticNoiseModel<MEAS_DIM>::getWhitenedErrorNorm(
    const Eigen::Matrix<double,MEAS_DIM,1>& rawError) const {
  return (this->sqrtInformation_*rawError).norm();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Get the whitened error vector, sqrtInformation*rawError
//////////////////////////////////////////////////////////////////////////////////////////////
template<int MEAS_DIM>
Eigen::Matrix<double,MEAS_DIM,1> StaticNoiseModel<MEAS_DIM>::whitenError(
    const Eigen::Matrix<double,MEAS_DIM,1>& rawError) const {
  return this->sqrtInformation_*rawError;
}


} // steam
