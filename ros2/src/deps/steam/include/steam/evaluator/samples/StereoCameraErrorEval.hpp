//////////////////////////////////////////////////////////////////////////////////////////////
/// \file StereoCameraErrorEval.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_STEREO_CAMERA_ERROR_EVALUATOR_HPP
#define STEAM_STEREO_CAMERA_ERROR_EVALUATOR_HPP

#include <steam.hpp>
#include <steam/problem/NoiseModel.hpp>

namespace steam {

namespace stereo {
//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Simple structure to hold the stereo camera intrinsics
//////////////////////////////////////////////////////////////////////////////////////////////
struct CameraIntrinsics {
  /// Convenience typedefs
  typedef boost::shared_ptr<CameraIntrinsics> Ptr;
  typedef boost::shared_ptr<const CameraIntrinsics> ConstPtr;

  /// \brief Stereo baseline
  double b;

  /// \brief Focal length in the u-coordinate (horizontal)
  double fu;

  /// \brief Focal length in the v-coordinate (vertical)
  double fv;

  /// \brief Focal center offset in the u-coordinate (horizontal)
  double cu;

  /// \brief Focal center offset in the v-coordinate (vertical)
  double cv;
};

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Calculates the stereo Camera model Jacobian
/// \param The stereo camera intrinsic properties.
/// \param The homogeneous point the jacobian is being evaluated at.
/// \return the jacobian of the camera model, evaluated at the given point.
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4d cameraModelJacobian(const CameraIntrinsics::ConstPtr &intrinsics, const Eigen::Vector4d& point);


//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Evaluates the noise of an uncertain map landmark, which has been reprojected into the
///        a query coordinate frame using a steam transform evaluator.
//////////////////////////////////////////////////////////////////////////////////////////////
class LandmarkNoiseEvaluator : public NoiseEvaluator<4> {
 public:

  /// \brief Constructor
  /// \param The landmark mean, in the query frame.
  /// \param The landmark covariance, in the query frame.
  /// \param The noise on the landmark measurement.
  /// \param The stereo camera intrinsics.
  /// \param The steam transform evaluator that takes points from the landmark frame
  ///        into the query frame.
  LandmarkNoiseEvaluator(const Eigen::Vector4d& landmark_mean,
                         const Eigen::Matrix3d& landmark_cov,
                         const Eigen::Matrix4d& meas_noise,
                         const CameraIntrinsics::ConstPtr& intrinsics,
                         const se3::TransformEvaluator::ConstPtr& T_query_map);

  /// \brief Default destructor
  ~LandmarkNoiseEvaluator()=default;
  
  /// \brief Evaluates the reprojection covariance 
  /// @return the 4x4 covariance of the landmark reprojected into the query stereo
  ///         camera frame.
  virtual Eigen::Matrix<double,4,4> evaluate();

 private:
  template <int N>
  bool positiveDefinite(const Eigen::Matrix<double,N,N> &matrix) {
  
    // Initialize an eigen value solver
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,N,N>> 
       eigsolver(matrix, Eigen::EigenvaluesOnly);

    // Check the minimum eigen value
    auto positive_definite = eigsolver.eigenvalues().minCoeff() > 0;
    if (!positive_definite) {
      std::cout << "Covariance \n" << matrix << "\n must be positive definite. "
                << "Min. eigenvalue : " << eigsolver.eigenvalues().minCoeff();
    }
    
    return positive_definite;
  }

  /// \brief The stereo camera intrinsics.
  CameraIntrinsics::ConstPtr intrinsics_;

  /// \brief The landmark covariance.
  Eigen::Matrix4d meas_noise_;

  /// \brief the landmark mean.
  Eigen::Vector4d mean_;

  /// \brief The steam transform evaluator that takes points from the landmark frame
  ///        into the query frame.
  se3::TransformEvaluator::ConstPtr T_query_map_;

  /// \brief The 3x3 landmark covariance (phi) dialated into a 3x3 matrix.
  /// @details dialated_phi_ = D*phi*D^T, where D is a 4x3 dialation matrix.
  Eigen::Matrix4d dialated_phi_;

  /// \brief The stereo camarea jacobian, evaluated at landmark mean, j.
  Eigen::Matrix4d camera_jacobian_j_;

  // \brief the last computed covariance
  Eigen::Matrix<double,4,4> last_computed_cov_;
};

} // end namespace stereo

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Stereo camera error function evaluator
///
/// *Note that we fix MAX_STATE_DIM to 6. Typically the performance benefits of fixed size
///  matrices begin to die if larger than 6x6. Size 6 allows for transformation matrices
///  and 6D velocities. If you have a state-type larger than this, consider writing an
///  error evaluator that extends from the dynamically sized ErrorEvaluatorX.
//////////////////////////////////////////////////////////////////////////////////////////////
class StereoCameraErrorEval : public ErrorEvaluator<4,6>::type
{
public:

  /// Convenience typedefs
  typedef boost::shared_ptr<StereoCameraErrorEval> Ptr;
  typedef boost::shared_ptr<const StereoCameraErrorEval> ConstPtr;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  StereoCameraErrorEval(const Eigen::Vector4d& meas,
                        const stereo::CameraIntrinsics::ConstPtr& intrinsics,
                        const se3::TransformEvaluator::ConstPtr& T_cam_landmark,
                        const se3::LandmarkStateVar::Ptr& landmark);

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Returns whether or not an evaluator contains unlocked state variables
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool isActive() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the 4-d measurement error (ul vl ur vr)
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual Eigen::Vector4d evaluate() const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the 4-d measurement error (ul vl ur vr) and Jacobians
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual Eigen::Vector4d evaluate(const Eigen::Matrix4d& lhs,
                                   std::vector<Jacobian<4,6> >* jacs) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Camera model Jacobian
  //////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::Matrix4d cameraModelJacobian() const { return stereo::cameraModelJacobian(intrinsics_, meas_); };
private:

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Camera model
  //////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::Vector4d cameraModel(const Eigen::Vector4d& point) const;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Measurement coordinates extracted from images (ul vl ur vr)
  //////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::Vector4d meas_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Camera instrinsics
  //////////////////////////////////////////////////////////////////////////////////////////////
  stereo::CameraIntrinsics::ConstPtr intrinsics_;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Point evaluator (evaluates the point transformed into the camera frame)
  //////////////////////////////////////////////////////////////////////////////////////////////
  se3::ComposeLandmarkEvaluator::ConstPtr eval_;

};

} // steam

#endif // STEAM_STEREO_CAMERA_ERROR_EVALUATOR_HPP
