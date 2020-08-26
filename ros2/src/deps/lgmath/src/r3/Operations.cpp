//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Implementation file for the 3D Euclidean space R3 math functions.
/// \details This is mostly explicit instantiations of template functions
///
/// \author Kirk MacTavish
//////////////////////////////////////////////////////////////////////////////////////////////

#include <lgmath/r3/Operations.hpp>

namespace lgmath {
namespace r3 {

// Explicitly instantiate transformCovariance for se3::Transform
template CovarianceMatrix transformCovariance<false>(
    const se3::Transformation & T_ba,
    const CovarianceMatrixConstRef & cov_a,
    const HPointConstRef & p_b);

// Explicitly instantiate transformCovariance for se3::TransformWithCovariance
template CovarianceMatrix transformCovariance<true>(
    const se3::TransformationWithCovariance & T_ba,
    const CovarianceMatrixConstRef & cov_a,
    const HPointConstRef & p_b);
template CovarianceMatrix transformCovariance<false>(
    const se3::TransformationWithCovariance & T_ba,
    const CovarianceMatrixConstRef & cov_a,
    const HPointConstRef & p_b);

} // r3
} // lgmath
