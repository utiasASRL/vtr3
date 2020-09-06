//////////////////////////////////////////////////////////////////////////////////////////////
/// \file PositionErrorEval.cpp
///
/// \author Kai van Es, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/evaluator/samples/PositionErrorEval.hpp>

//#include <steam/evaluator/blockauto/transform/TransformStateEvaluator.hpp>
//#include <steam/evaluator/blockauto/transform/FixedTransformEvaluator.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor - error is difference between 'T' and zero
//////////////////////////////////////////////////////////////////////////////////////////////
PositionErrorEval::PositionErrorEval(const se3::TransformEvaluator::ConstPtr &T) : meas_(0,0,0)  {
  positionEvaluator_.reset(new se3::PositionEvaluator(T));
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Convenience constructor - linear error between meas_r_21_in1 and T_21
/// \param meas_r_21_in1 Position of frame 2 wrt frame 1, expressed in frame 1
/// \param T_21 Transformation from frame 1 to frame 2
//////////////////////////////////////////////////////////////////////////////////////////////
PositionErrorEval::PositionErrorEval(const Eigen::Vector3d &meas_r_21_in1,
                                     const se3::TransformEvaluator::ConstPtr &T_21) : meas_(meas_r_21_in1) {

  positionEvaluator_.reset(new se3::PositionEvaluator(T_21));
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Convenience constructor - error between meas_r_21_in1 and T_20*inv(T_10)
/// \param meas_r_21_in1 Position of frame 2 wrt frame 1, expressed in frame 1
/// \param T_20 Transformation from the world frame (0) to frame 2
/// \param T_10 Transformation from the world frame (0) to frame 1
//////////////////////////////////////////////////////////////////////////////////////////////
PositionErrorEval::PositionErrorEval(const Eigen::Vector3d &meas_r_21_in1,
                                     const se3::TransformStateVar::Ptr &T_20,
                                     const se3::TransformStateVar::Ptr &T_10) : meas_(meas_r_21_in1) {

  se3::TransformStateEvaluator::ConstPtr t10 = se3::TransformStateEvaluator::MakeShared(T_10);
  se3::TransformStateEvaluator::ConstPtr t20 = se3::TransformStateEvaluator::MakeShared(T_20);
  positionEvaluator_.reset(new se3::PositionEvaluator(se3::compose(t20, se3::inverse(t10))));
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not an evaluator contains unlocked state variables
//////////////////////////////////////////////////////////////////////////////////////////////
bool PositionErrorEval::isActive() const {
  return positionEvaluator_->isActive();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the 3-d measurement error
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double, 3, 1> PositionErrorEval::evaluate() const {
  return positionEvaluator_->evaluate() - meas_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the 3-d measurement error and Jacobians
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double, 3, 1> PositionErrorEval::evaluate(const Eigen::Matrix<double, 3, 3> &lhs,
                                                        std::vector<Jacobian<3, 6> > *jacs) const {

  // Check and initialize jacobian array
  if (jacs == NULL) {
    throw std::invalid_argument("Null pointer provided to return-input 'jacs' in evaluate");
  }
  jacs->clear();

  // Get evaluation tree
  EvalTreeHandle<Eigen::Matrix<double, 3, 1> > blkAutoEvalPosOfTransformDiff =
      positionEvaluator_->getBlockAutomaticEvaluation();

  // Get evaluation from tree
  Eigen::Matrix<double, 3, 1> error = blkAutoEvalPosOfTransformDiff.getValue() - meas_;

  // Get Jacobians
  positionEvaluator_->appendBlockAutomaticJacobians(lhs, blkAutoEvalPosOfTransformDiff.getRoot(), jacs);

  // Return evaluation
  return error;
}

} // steam
