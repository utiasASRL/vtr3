//////////////////////////////////////////////////////////////////////////////////////////////
/// \file mono_camera_error_eval.cpp
///
/// \author Michael Warren, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <vtr_steam_extensions/evaluator/mono_camera_error_eval.hpp>

namespace vtr {
namespace steam_extensions {
namespace mono {

//////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Camera model Jacobian
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double, 2, 4> cameraModelJacobian(
    const CameraIntrinsics::ConstPtr& intrinsics,
    const Eigen::Vector4d& point) {
  // Precompute values
  const double x = point[0];
  const double y = point[1];
  const double z = point[2];
  //  const double w = point[3];
  const double one_over_z = 1.0 / z;
  const double one_over_z2 = one_over_z * one_over_z;

  // Construct Jacobian with respect to x, y, z, and scalar w
  Eigen::Matrix<double, 2, 4> jac;
  jac << intrinsics->fu * one_over_z, 0.0, -intrinsics->fu * x * one_over_z2,
      0.0, 0.0, intrinsics->fv * one_over_z, -intrinsics->fv * y * one_over_z2,
      0.0;
  return jac;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
LandmarkNoiseEvaluator::LandmarkNoiseEvaluator(
    const Eigen::Vector4d& landmark_mean, const Eigen::Matrix3d& landmark_cov,
    const Eigen::Matrix2d& meas_noise,
    const CameraIntrinsics::ConstPtr& intrinsics,
    const steam::se3::TransformEvaluator::ConstPtr& T_query_map)
    : intrinsics_(intrinsics),
      meas_noise_(meas_noise),
      mean_(landmark_mean),
      T_query_map_(T_query_map) {
  // compute the dialated phi;
  dialated_phi_.setZero();
  dialated_phi_.block(0, 0, 3, 3) = landmark_cov;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Evaluate covariance
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd LandmarkNoiseEvaluator::evaluate() {
  // TODO: Check to see if we need to recalculate (add a change flag to steam
  // variables.)

  // evaluate the steam transform evaluator
  const auto& T_l_p = T_query_map_->evaluate();

  // compute the camera model jacobian based on the transformed mean.
  camera_jacobian_j_ = mono::cameraModelJacobian(intrinsics_, T_l_p * mean_);

  // Compute the new landmark noise
  auto lm_noise = camera_jacobian_j_ * T_l_p.matrix() * dialated_phi_ *
                  T_l_p.matrix().transpose() * camera_jacobian_j_.transpose();

  // Add the measurement noise.
  last_computed_cov_ = meas_noise_ + lm_noise;

  // return the new noise.
  return last_computed_cov_;
}

}  // end namespace mono

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Constructor
//////////////////////////////////////////////////////////////////////////////////////////////
MonoCameraErrorEval::MonoCameraErrorEval(
    const Eigen::Vector2d& meas,
    const mono::CameraIntrinsics::ConstPtr& intrinsics,
    const steam::se3::TransformEvaluator::ConstPtr& T_cam_landmark,
    const steam::se3::LandmarkStateVar::Ptr& landmark)
    : meas_(meas),
      intrinsics_(intrinsics),
      eval_(steam::se3::compose(T_cam_landmark, landmark)) {}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns whether or not an evaluator contains unlocked state variables
//////////////////////////////////////////////////////////////////////////////////////////////
bool MonoCameraErrorEval::isActive() const { return eval_->isActive(); }

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the 4-d measurement error (ul vl ur vr)
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd MonoCameraErrorEval::evaluate() const {
  // Return error (between measurement and point estimate projected in camera
  // frame)
  return meas_ - cameraModel(eval_->evaluate());
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluate the 2-d measurement error (u v) and Jacobians
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd MonoCameraErrorEval::evaluate(
    const Eigen::MatrixXd& lhs, std::vector<steam::Jacobian<> >* jacs) const {
  // Check and initialize Jacobian array
  if (jacs == NULL) {
    throw std::invalid_argument(
        "Null pointer provided to return-input 'jacs' in evaluate");
  }
  jacs->clear();

  // Get evaluation tree
  steam::EvalTreeHandle<Eigen::Vector4d> blkAutoEvalPointInCameraFrame =
      eval_->getBlockAutomaticEvaluation();

  // Get evaluation from tree
  const Eigen::Vector4d& pointInCamFrame =
      blkAutoEvalPointInCameraFrame.getValue();

  // Get Jacobians
  Eigen::Matrix<double, 2, 4> t =
      mono::cameraModelJacobian(intrinsics_, pointInCamFrame);
  Eigen::MatrixXd newLhs = (-1) * lhs * t;
  eval_->appendBlockAutomaticJacobians(
      newLhs, blkAutoEvalPointInCameraFrame.getRoot(), jacs);

  // Return evaluation
  return meas_ - cameraModel(pointInCamFrame);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Camera model
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector2d MonoCameraErrorEval::cameraModel(
    const Eigen::Vector4d& point) const {
  // Precompute values
  const double x = point[0];
  const double y = point[1];
  const double z = point[2];
  //  const double w = point[3];
  const double one_over_z = 1.0 / z;

  // Project point into camera coordinates
  Eigen::Vector2d projectedMeas;
  projectedMeas << intrinsics_->fu * x * one_over_z + intrinsics_->cu,
      intrinsics_->fv * y * one_over_z + intrinsics_->cv;
  return projectedMeas;
}

}  // namespace steam_extensions
}  // namespace vtr
