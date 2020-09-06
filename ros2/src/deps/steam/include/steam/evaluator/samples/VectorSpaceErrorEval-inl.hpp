//////////////////////////////////////////////////////////////////////////////////////////////
/// \file VectorSpaceErrorEval-inl.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/evaluator/samples/VectorSpaceErrorEval.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
template<int MEAS_DIM, int MAX_STATE_DIM>
VectorSpaceErrorEval<MEAS_DIM,MAX_STATE_DIM>::VectorSpaceErrorEval(
    const Eigen::Matrix<double,MEAS_DIM,1>& measurement,
    const VectorSpaceStateVar::ConstPtr& stateVec)
  : measurement_(measurement), stateVec_(stateVec) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not an evaluator contains unlocked state variables
//////////////////////////////////////////////////////////////////////////////////////////////
template<int MEAS_DIM, int MAX_STATE_DIM>
bool VectorSpaceErrorEval<MEAS_DIM,MAX_STATE_DIM>::isActive() const {
  return !stateVec_->isLocked();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the measurement error
//////////////////////////////////////////////////////////////////////////////////////////////
template<int MEAS_DIM, int MAX_STATE_DIM>
Eigen::Matrix<double,MEAS_DIM,1> VectorSpaceErrorEval<MEAS_DIM,MAX_STATE_DIM>::evaluate() const {
  return measurement_ - stateVec_->getValue();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the measurement error and relevant Jacobians
//////////////////////////////////////////////////////////////////////////////////////////////
template<int MEAS_DIM, int MAX_STATE_DIM>
Eigen::Matrix<double,MEAS_DIM,1> VectorSpaceErrorEval<MEAS_DIM,MAX_STATE_DIM>::evaluate(
    const Eigen::Matrix<double,MEAS_DIM,MEAS_DIM>& lhs,
    std::vector<Jacobian<MEAS_DIM,MAX_STATE_DIM> >* jacs) const {

  // Check and initialize jacobian array
  if (jacs == NULL) {
    throw std::invalid_argument("Null pointer provided to return-input 'jacs' in evaluate");
  }
  jacs->clear();

  // Check that dimensions match
  if (lhs.cols() != stateVec_->getPerturbDim()) {
    throw std::runtime_error("evaluate had dimension mismatch.");
  }

  // Construct Jacobian
  if(!stateVec_->isLocked()) {
    jacs->resize(1);
    Jacobian<MEAS_DIM,MAX_STATE_DIM>& jacref = jacs->back();
    jacref.key = stateVec_->getKey();
    jacref.jac = -lhs;
  }

  // Return error
  return measurement_ - stateVec_->getValue();
}

} // steam
