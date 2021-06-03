////////////////////////////////////////////////////////////////////////////////
/// @brief StereoTransformModel.cpp Source file for the ASRL vision package
/// @details This file defines the StereoTransformModel class, which provides
///          a solver for stereo pose estimation problems
///
/// @author Kirk MacTavish, ASRL
///////////////////////////////////////////////////////////////////////////////

#include "vtr_vision/sensors/stereo_transform_model.hpp"
#include "vtr_vision/sensors/register_svd.hpp"

// External
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <chrono>

namespace vtr {
namespace vision {

template class CameraModelInterface<Eigen::Matrix3Xd, Eigen::Matrix3Xd>;

bool StereoTransformModel::solveModel(const SimpleMatches& matches,
                                   SolutionType* p_solution,
                                   double threshold) const {
  // Check the input
  if (matches.size() < N || !p_solution) return false;

  // Copy inlier points into an Eigen matrix
  Eigen::Matrix<double, 3, Eigen::Dynamic> inliers_ref (3, matches.size());
  Eigen::Matrix<double, 3, Eigen::Dynamic> inliers_cam (3, matches.size());
  for (unsigned int i = 0; i < matches.size(); ++i) {
    inliers_ref.col(i) = pts_ref_->col(matches[i].first);
    inliers_cam.col(i) = pts_query_->col(matches[i].second);
  }

  // Compute inverse distance weights for the SVD
  Eigen::Array<double,1,Eigen::Dynamic> weights = 1. / inliers_ref.row(2).array();

  // Run SVD and return on failure
  if (!registerSVD<3>(inliers_cam, inliers_ref, p_solution, weights)) {
    return false;
  }

#if 0
   std::cout << "ref:\n" << inliers_ref
   	    << "\ncam:\n" << inliers_cam
   	    << "\nr2c:\n" << (*p_solution)*inliers_ref.colwise().homogeneous()
   	    << std::endl;
#endif

  // Calculate error for the fit, if the points aren't inliers, it's a bad sample
  ErrorList errors;
  computeError(matches, *p_solution, &errors, nullptr);

  if ((errors.colwise().norm().array() >= threshold).count() != 0) {
    return false;
  }

  // Good sample!
  return true;
}

bool StereoTransformModel::verifyMatches(const SimpleMatches& matches) const {
  // Check existence
  if (!pts_ref_ || !pts_query_)
    return false;
  
  // Verify indices to be within points size
  for (const auto & matche : matches)
    if (matche.first >= pts_ref_->size() ||
        matche.second >= pts_query_->size())
      return false;

  return true;
}

bool StereoTransformModel::computeError(const SimpleMatches& matches,
                                     const SolutionType& model,
                                     ErrorList* whitened_errors,
                                     double *robust_error,
                                     double stop_error,
                                     double sigma2) const {

  // Initialize counters
  if (!whitened_errors && !robust_error) return false;
  if (whitened_errors) {
    whitened_errors->resize(matches.size());
  }
  double cum_error = 0.;

  // Convert the model to a transformation
  Eigen::Affine3d tf (model);

  // Calculate the error for each point
  for (unsigned int i = 0; i < matches.size() && cum_error < stop_error; ++i) {

    // Transform the reference point into the camera frame
    Eigen::Vector4d pt3dh_ref = model * pts_ref_->col(matches[i].first).homogeneous();
    Eigen::Vector4d pt3dh_cam = pts_query_->col(matches[i].second).homogeneous();

    // Project the reference and camera points into the camera frames
    Eigen::Vector2d pt2d_l_ref = (projection_l_ * pt3dh_ref).hnormalized();
    Eigen::Vector2d pt2d_l_cam = (projection_l_ * pt3dh_cam).hnormalized();

    Eigen::Vector2d pt2d_r_ref = (projection_r_ * pt3dh_ref).hnormalized();
    Eigen::Vector2d pt2d_r_cam = (projection_r_ * pt3dh_cam).hnormalized();

    // calculate the squared error for left and right cameras, taking into account the covariance
    Eigen::Matrix2d S = Eigen::Matrix2d::Identity();
    if(2*matches[i].first + 1 <= inv_r_list_.cols()) {
      S = inv_r_list_.block(0,2*matches[i].first,2,2);
    }

    Eigen::Vector2d dl = pt2d_l_cam - pt2d_l_ref;
    Eigen::Vector2d dr = pt2d_r_cam - pt2d_r_ref;
    double i_error_l = dl.transpose()*S*dl;
    double i_error_r = dr.transpose()*S*dr;

    // Calculate the mahalanobis error
    double i_mahalanobis2 = i_error_l + i_error_r;

#if 0
    std::cout << "model: " << model << "\ncam: "
          << camera_matrix_ << "\nref: "
          << pt3d_ref.transpose() << "\nref: "
          << pt2d_ref.transpose() << "\ncam: " << pts_cam_->col(matches[i].second).transpose() << "\ncam: "
          << pt2d_cam.transpose() << "\nerr: "
          << i_error << "\n\n" << std::endl;
#endif

    // Add to the error recorders
    if (whitened_errors) {
      (*whitened_errors)(i) = std::sqrt(i_mahalanobis2);
    }
    // sanity check
    if (i_mahalanobis2 != i_mahalanobis2 || std::isinf(i_mahalanobis2)) {
      cum_error++;
    } else {
      cum_error += i_mahalanobis2 / (i_mahalanobis2 + sigma2);
    }
  }

  // Copy to output
  if (robust_error)
    *robust_error = cum_error;

  // Not the best so far
  if (cum_error >= stop_error)
    return false;

  // The best so far!
  return true;
}

} // namespace vision
} // namespace vtr
