//////////////////////////////////////////////////////////////////////////////////////////////
/// \file range_conditioning_eval.cpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <vtr_steam_extensions/evaluator/range_conditioning_eval.hpp>

namespace vtr {
namespace steam_extensions {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
RangeConditioningEval::RangeConditioningEval(
    const steam::se3::LandmarkStateVar::ConstPtr& landmark)
    : landmark_(landmark) {
  Eigen::Vector4d h = landmark_->getValue();
  Eigen::Vector3d v = (1.0 / h[3]) * h.head<3>();
  meas_ << v.norm();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not an evaluator contains unlocked state variables
//////////////////////////////////////////////////////////////////////////////////////////////
bool RangeConditioningEval::isActive() const { return !landmark_->isLocked(); }

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double, 1, 1> RangeConditioningEval::evaluate() const {
  Eigen::Vector4d h = landmark_->getValue();
  Eigen::Vector3d v = (1.0 / h[3]) * h.head<3>();
  Eigen::Matrix<double, 1, 1> rhs;
  rhs << v.norm();
  return meas_ - rhs;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double, 1, 1> RangeConditioningEval::evaluate(
    const Eigen::Matrix<double, 1, 1>& lhs,
    std::vector<steam::Jacobian<1, 3> >* jacs) const {
  Eigen::Vector4d h = landmark_->getValue();
  Eigen::Vector3d v = (1.0 / h[3]) * h.head<3>();

  if (!landmark_->isLocked()) {
    jacs->push_back(steam::Jacobian<1, 3>());
    steam::Jacobian<1, 3>& jacref = jacs->back();
    jacref.key = landmark_->getKey();
    jacref.jac = (-1.0 / (h[3] * v.norm())) * lhs * v.transpose();
  }

  Eigen::Matrix<double, 1, 1> rhs;
  rhs << v.norm();
  return meas_ - rhs;
}

}  // namespace steam_extensions
}  // namespace vtr
