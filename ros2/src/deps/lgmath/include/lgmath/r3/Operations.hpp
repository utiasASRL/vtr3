//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Header file for point and point with covariance definitions.
/// \details Mostly typedefs to standardize interfacing points with transformations.
///
/// \author Kirk MacTavish
//////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////
///
/// A note on EIGEN_MAKE_ALIGNED_OPERATOR_NEW (Sean Anderson, as of May 23, 2013)
/// (also see http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html)
///
/// Fortunately, Eigen::Matrix3d and Eigen::Vector3d are NOT 16-byte vectorizable,
/// therefore this class should not require alignment, and can be used normally in STL.
///
/// To inform others of the issue, classes that include *fixed-size vectorizable Eigen types*,
/// see http://eigen.tuxfamily.org/dox-devel/group__TopicFixedSizeVectorizable.html,
/// must include the above macro! Furthermore, special considerations must be taken if
/// you want to use them in STL containers, such as std::vector or std::map.
/// The macro overloads the dynamic "new" operator so that it generates
/// 16-byte-aligned pointers, this MUST be in the public section of the header!
///
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef LGM_R3_OPERATIONS_HPP
#define LGM_R3_OPERATIONS_HPP

#include <lgmath/r3/Types.hpp>
#include <lgmath/se3/TransformationWithCovariance.hpp>
#include <lgmath/se3/Operations.hpp>
#include <Eigen/Dense>
#include <stdexcept>

namespace lgmath {
namespace r3 {

/// The transform covariance is required to be set
static constexpr bool COVARIANCE_REQUIRED = true;
/// The transform covariance is not required to be set
static constexpr bool COVARIANCE_NOT_REQUIRED = false;

//////////////////////////////////////////////////////////////////////////////////////////////
/// Transforms a 3x3 covariance for a 3D point (with an assumed certain transform).
///
/// @note THROW_IF_UNSET Will complain (at compile time) if the transform does not have
///       an explicit covariance. This is always the case for se3::Transformation,
///       so a static_assert prevents compiling without explicitly ignoring the warning
///       (by templating on false).
///
/// @param T_ba The certain transform that will be used to transform the point covariance
/// @param cov_a The covariance is still in the original frame, A.
/// @param p_b The point is unused since the transformation has no uncertainty
///
/// See eq. 12 in clearpath_virtual_roadways/pm2/mel.pdf for more information.
//////////////////////////////////////////////////////////////////////////////////////////////
template<bool THROW_IF_UNSET=COVARIANCE_REQUIRED>
CovarianceMatrix transformCovariance(const se3::Transformation & T_ba,
                                     const CovarianceMatrixConstRef & cov_a,
                                     const HPointConstRef & p_b = HPoint()) {
  (void)&p_b; // unused
  static_assert(!THROW_IF_UNSET, "Error: Transformation never has covariance explicitly set");

  // The component from the point noise
  return T_ba.C_ba()*cov_a*T_ba.C_ba().transpose();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// Transforms a 3x3 covariance for a 3D point (with an uncertain transform).
///
/// @note THROW_IF_UNSET Will complain (at run time) if the transform does not have
///       an explicit covariance by throwing a runtime_error unless explictly told not to.
///
/// @param T_ba The certain transform that will be used to transform the point covariance
/// @param cov_a The covariance is still in the original frame, A.
/// @param p_b Note that the point has already been transformed to frame B: p_b = T_ba*p_a
///
/// See eq. 12 in clearpath_virtual_roadways/pm2/mel.pdf for more information.
//////////////////////////////////////////////////////////////////////////////////////////////
template<bool THROW_IF_UNSET=COVARIANCE_REQUIRED>
CovarianceMatrix transformCovariance(const se3::TransformationWithCovariance & T_ba,
                                     const CovarianceMatrixConstRef & cov_a,
                                     const HPointConstRef & p_b) {
  if (THROW_IF_UNSET && !T_ba.covarianceSet()) {
    throw std::runtime_error("Error: TransformationWithCovariance does not have covariance set");
  }

  // The component from the point noise (reuse the base Transform function)
  const auto & T_ba_base = static_cast<const se3::Transformation &>(T_ba);
  CovarianceMatrix cov_b = transformCovariance<false>(T_ba_base, cov_a, p_b);

  // The component from the transform noise
  if (T_ba.covarianceSet()) {
    auto jacobian = se3::point2fs(p_b.hnormalized()).topRows<3>();
    cov_b += jacobian*T_ba.cov()*jacobian.transpose();
  }

  return cov_b;
}

} // r3
} // lgmath

#endif // LGM_R3_OPERATIONS_HPP
