//////////////////////////////////////////////////////////////////////////////////////////////
/// \file scale_error_eval.cpp
///
/// \author Michael Warren, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <vtr_steam_extensions/evaluator/scale_error_eval.hpp>

#include <steam/evaluator/blockauto/transform/FixedTransformEvaluator.hpp>
#include <steam/evaluator/blockauto/transform/TransformStateEvaluator.hpp>

namespace vtr {
namespace steam_extensions {

//////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Scale error Jacobian
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double, 1, 6> scaleJacobian(const Eigen::Matrix<double, 6, 1> d) {
  // precache
  const double& x = d(0, 0);
  const double& y = d(1, 0);
  const double& z = d(2, 0);
  double sqrtxyz = std::sqrt(x * x + y * y + z * z);

  // Construct Jacobian with respect to x, y, z, and scalar w
  Eigen::Matrix<double, 1, 6> jac;
  double dx = x / sqrtxyz;
  double dy = y / sqrtxyz;
  double dz = z / sqrtxyz;
  jac << dx, dy, dz, 0.0, 0.0, 0.0;

  return jac;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor - error is difference between 'T' and identity (in Lie
/// algebra space)
//////////////////////////////////////////////////////////////////////////////////////////////
ScaleErrorEval::ScaleErrorEval(
    double meas, const steam::se3::TransformEvaluator::ConstPtr& T) {
  meas_ = meas;
  errorEvaluator_ = steam::se3::tran2vec(T);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not an evaluator contains unlocked state variables
//////////////////////////////////////////////////////////////////////////////////////////////
bool ScaleErrorEval::isActive() const { return errorEvaluator_->isActive(); }

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the 1-d measurement error
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd ScaleErrorEval::evaluate() const {
  Eigen::Matrix<double, 6, 1> d = errorEvaluator_->evaluate();
  Eigen::VectorXd ret(1);
  ret << meas_ - d.topRows(3).norm();
  return ret;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the 1-d measurement error and Jacobians
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd ScaleErrorEval::evaluate(
    const Eigen::MatrixXd& lhs, std::vector<steam::Jacobian<> >* jacs) const {
  // Check and initialize jacobian array
  if (jacs == NULL) {
    throw std::invalid_argument(
        "Null pointer provided to return-input 'jacs' in evaluate");
  }
  jacs->clear();

  // Get evaluation tree
  steam::EvalTreeHandle<Eigen::Matrix<double, 6, 1> >
      blkAutoEvalLogOfTransformDiff =
          errorEvaluator_->getBlockAutomaticEvaluation();

  // Get evaluation from tree
  Eigen::Matrix<double, 6, 1> d = blkAutoEvalLogOfTransformDiff.getValue();

  // Get Jacobians
  Eigen::Matrix<double, 1, 6> t = scaleJacobian(d);
  Eigen::MatrixXd newLhs = (-1) * lhs * t;

  // Get Jacobians
  errorEvaluator_->appendBlockAutomaticJacobians(
      newLhs, blkAutoEvalLogOfTransformDiff.getRoot(), jacs);

  // Return evaluation
  Eigen::VectorXd ret(1);
  ret << meas_ - d.topRows(3).norm();
  return ret;
}

}  // namespace steam_extensions
}  // namespace vtr
