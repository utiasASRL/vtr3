//////////////////////////////////////////////////////////////////////////////////////////////
/// \file StereoCameraErrorEvalX.cpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/evaluator/samples/StereoCameraErrorEvalX.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
StereoCameraErrorEvalX::StereoCameraErrorEvalX(const Eigen::Vector4d& meas,
                                     const CameraIntrinsics::ConstPtr& intrinsics,
                                     const se3::TransformEvaluator::ConstPtr& T_cam_landmark,
                                     const se3::LandmarkStateVar::Ptr& landmark)
  : meas_(meas), intrinsics_(intrinsics), eval_(se3::compose(T_cam_landmark, landmark)) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not an evaluator contains unlocked state variables
//////////////////////////////////////////////////////////////////////////////////////////////
bool StereoCameraErrorEvalX::isActive() const {
  return eval_->isActive();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the 4-d measurement error (ul vl ur vr)
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd StereoCameraErrorEvalX::evaluate() const {

  // Return error (between measurement and point estimate projected in camera frame)
  return meas_ - cameraModel(eval_->evaluate());
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the 4-d measurement error (ul vl ur vr) and Jacobians
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd StereoCameraErrorEvalX::evaluate(const Eigen::MatrixXd& lhs, std::vector<Jacobian<> >* jacs) const {

  // Check and initialize jacobian array
  if (jacs == NULL) {
    throw std::invalid_argument("Null pointer provided to return-input 'jacs' in evaluate");
  }
  jacs->clear();

  // Get evaluation tree
  EvalTreeHandle<Eigen::Vector4d> blkAutoEvalPointInCameraFrame =
      eval_->getBlockAutomaticEvaluation();

  // Get evaluation from tree
  const Eigen::Vector4d& pointInCamFrame = blkAutoEvalPointInCameraFrame.getValue();

  // Get Jacobians
  Eigen::Matrix4d newLhs = (-1)*lhs*cameraModelJacobian(pointInCamFrame);
  eval_->appendBlockAutomaticJacobians(newLhs, blkAutoEvalPointInCameraFrame.getRoot(), jacs);

  // Return evaluation
  return meas_ - cameraModel(pointInCamFrame);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Camera model
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector4d StereoCameraErrorEvalX::cameraModel(const Eigen::Vector4d& point) const {

  // Precompute values
  const double x = point[0];
  const double y = point[1];
  const double z = point[2];
  const double w = point[3];
  const double xr = x - w * intrinsics_->b;
  const double one_over_z = 1.0/z;

  // Project point into camera coordinates
  Eigen::Vector4d projectedMeas;
  projectedMeas << intrinsics_->fu *  x  * one_over_z + intrinsics_->cu,
                   intrinsics_->fv *  y  * one_over_z + intrinsics_->cv,
                   intrinsics_->fu *  xr * one_over_z + intrinsics_->cu,
                   intrinsics_->fv *  y  * one_over_z + intrinsics_->cv;
  return projectedMeas;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Camera model Jacobian
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4d StereoCameraErrorEvalX::cameraModelJacobian(const Eigen::Vector4d& point) const {

  // Precompute values
  const double x = point[0];
  const double y = point[1];
  const double z = point[2];
  const double w = point[3];
  const double xr = x - w * intrinsics_->b;
  const double one_over_z = 1.0/z;
  const double one_over_z2 = one_over_z*one_over_z;

  // Construct Jacobian with respect to x, y, z, and scalar w
  const double dw = -intrinsics_->fu * intrinsics_->b * one_over_z;
  Eigen::Matrix4d jac;
  jac << intrinsics_->fu*one_over_z, 0.0, -intrinsics_->fu *  x  * one_over_z2, 0.0,
         0.0, intrinsics_->fv*one_over_z, -intrinsics_->fv *  y  * one_over_z2, 0.0,
         intrinsics_->fu*one_over_z, 0.0, -intrinsics_->fu *  xr * one_over_z2,  dw,
         0.0, intrinsics_->fv*one_over_z, -intrinsics_->fv *  y  * one_over_z2, 0.0;
  return jac;
}

} // steam
