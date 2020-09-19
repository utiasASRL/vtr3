////////////////////////////////////////////////////////////////////////////////
/// @brief Header for registerSVD function
/// @details Calculates rigid transformation given two sets of points.
///          Used in stereo_transform_model
///
/// @author Kirk MacTavish, ASRL
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>

namespace vtr {
namespace vision {

// Returns tf s.t. a = tf * b
template <unsigned int N>
bool registerSVD(const Eigen::Matrix<double,N,Eigen::Dynamic>& a,
                 const Eigen::Matrix<double,N,Eigen::Dynamic>& b,
                 Eigen::Matrix<double,N+1,N+1> * tf,
                 const Eigen::Array<double,1,Eigen::Dynamic>& weights = Eigen::Array<double,1,Eigen::Dynamic>(1,0),
                 bool scaling = false) {
  // Types
  //typedef Eigen::Matrix<double,N+1,N+1> Transform;
  typedef Eigen::Matrix<double,N,1> Point;
  typedef Eigen::Matrix<double,N,Eigen::Dynamic> Points;
  typedef Eigen::Matrix<double,N,N> Square;

  // Check output allocation
  if (!tf) return false;

  // Are we weighting points?
  bool use_weights = weights.cols() > 0;

  // Check input (1 pt for 1D, 2 pts for 2D, 3 pts for 3D)
  if (a.cols() < N || a.cols() != b.cols()
      || (use_weights && a.cols() != weights.cols()))
    return false;

  // Switch based on whether we're weighting the points
  // See https://igl.ethz.ch/projects/ARAP/svd_rot.pdf
  Point ca, cb;
  Points a0, b0;
  Square cov;
  if (use_weights) {
    // Weight points
    Points aw = a.array().rowwise() * weights;
    Points bw = b.array().rowwise() * weights;

    // Compute centroids
    double w_sum = weights.sum();
    ca = aw.rowwise().sum() / w_sum;
    cb = bw.rowwise().sum() / w_sum;

    // Demean
    a0 = a.colwise()-ca;
    b0 = b.colwise()- cb;

    // Covariance
    cov = (a0.array().rowwise() * weights).matrix() * b0.transpose(); // a0*W*b0'
  } else {
    // Compute centroids
    ca = a.rowwise().mean();
    cb = b.rowwise().mean();

    // Demean
    a0 = a.colwise()-ca;
    b0 = b.colwise()-cb;

    // Covariance
    cov = a0 * b0.transpose();
  }

  // SVD
  Eigen::JacobiSVD<Square> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Square V = svd.matrixV();
  // Normally V * U', but then we have to use R'.
  Square R = svd.matrixU() * V.transpose();

  // Check for proper right-hand rotation
  double det = R.determinant();
  if (det < 0. || use_weights) {
    // If we use the weights, we have to remove their influence here
    V.col(N-1) *= use_weights ? det : -1.;
    // Normally V * U', but then we have to use R'.
    R = svd.matrixU() * V.transpose();
  }

  // Apply scaling if necessary
  if (scaling) {
    double scale = b0.norm()/a0.norm();
    R /= scale;
  }

  // Translation
  Point t = ca - R*cb;

  // Build final transform
  tf->row(N).setZero();
  (*tf)(N,N) = 1.;
  tf->template topLeftCorner<N,N>() = R;
  tf->template topRightCorner<N,1>() = t;

  // Return
  return true;
}

// Explicit instantiation for common params
extern template bool registerSVD<2>(
  const Eigen::Matrix<double,2,Eigen::Dynamic>& a,
  const Eigen::Matrix<double,2,Eigen::Dynamic>& b,
  Eigen::Matrix<double,2+1,2+1> * tf,
  const Eigen::Array<double,1,Eigen::Dynamic>& weights,
  bool scaling);
extern template bool registerSVD<3>(
  const Eigen::Matrix<double,3,Eigen::Dynamic>& a,
  const Eigen::Matrix<double,3,Eigen::Dynamic>& b,
  Eigen::Matrix<double,3+1,3+1> * tf,
  const Eigen::Array<double,1,Eigen::Dynamic>& weights,
  bool scaling);

} // namespace vision
} // namespace vtr_vision
