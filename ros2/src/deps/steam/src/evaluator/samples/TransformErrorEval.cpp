//////////////////////////////////////////////////////////////////////////////////////////////
/// \file TransformErrorEval.cpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/evaluator/samples/TransformErrorEval.hpp>

#include <steam/evaluator/blockauto/transform/TransformStateEvaluator.hpp>
#include <steam/evaluator/blockauto/transform/FixedTransformEvaluator.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor - error is difference between 'T' and identity (in Lie algebra space)
//////////////////////////////////////////////////////////////////////////////////////////////
TransformErrorEval::TransformErrorEval(const se3::TransformEvaluator::ConstPtr& T) {
  errorEvaluator_ = se3::tran2vec(T);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Convenience constructor - error between meas_T_21 and T_21
//////////////////////////////////////////////////////////////////////////////////////////////
TransformErrorEval::TransformErrorEval(const lgmath::se3::Transformation& meas_T_21,
                                       const se3::TransformEvaluator::ConstPtr& T_21) {

  // Construct the evaluator using the convenient transform evaluators
  se3::FixedTransformEvaluator::ConstPtr meas = se3::FixedTransformEvaluator::MakeShared(meas_T_21);
  errorEvaluator_ = se3::tran2vec(se3::compose(meas, se3::inverse(T_21)));
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Convenience constructor - error between meas_T_21 and T_20*inv(T_10)
//////////////////////////////////////////////////////////////////////////////////////////////
TransformErrorEval::TransformErrorEval(const lgmath::se3::Transformation& meas_T_21,
                                       const se3::TransformStateVar::Ptr& T_20,
                                       const se3::TransformStateVar::Ptr& T_10) {

  // Construct the evaluator using the convenient transform evaluators
  se3::FixedTransformEvaluator::ConstPtr meas = se3::FixedTransformEvaluator::MakeShared(meas_T_21);
  se3::TransformStateEvaluator::ConstPtr t10 = se3::TransformStateEvaluator::MakeShared(T_10);
  se3::TransformStateEvaluator::ConstPtr t20 = se3::TransformStateEvaluator::MakeShared(T_20);
  errorEvaluator_ = se3::tran2vec(se3::compose(se3::compose(meas, t10), se3::inverse(t20)));
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not an evaluator contains unlocked state variables
//////////////////////////////////////////////////////////////////////////////////////////////
bool TransformErrorEval::isActive() const {
  return errorEvaluator_->isActive();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the 6-d measurement error
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,1> TransformErrorEval::evaluate() const {
  return errorEvaluator_->evaluate();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the 6-d measurement error and Jacobians
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,1> TransformErrorEval::evaluate(const Eigen::Matrix<double,6,6>& lhs,
                                                       std::vector<Jacobian<6,6> >* jacs) const {

  // Check and initialize jacobian array
  if (jacs == NULL) {
    throw std::invalid_argument("Null pointer provided to return-input 'jacs' in evaluate");
  }
  jacs->clear();

  // Get evaluation tree
  EvalTreeHandle<Eigen::Matrix<double,6,1> > blkAutoEvalLogOfTransformDiff =
      errorEvaluator_->getBlockAutomaticEvaluation();

  // Get evaluation from tree
  Eigen::Matrix<double,6,1> error = blkAutoEvalLogOfTransformDiff.getValue();

  // Get Jacobians
  errorEvaluator_->appendBlockAutomaticJacobians(lhs,
      blkAutoEvalLogOfTransformDiff.getRoot(), jacs);

  // Return evaluation
  return error;
}

} // steam
