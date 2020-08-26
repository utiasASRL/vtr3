//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Header file for Euclidean space R3 types.
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

#ifndef LGM_R3_TYPES_HPP
#define LGM_R3_TYPES_HPP

#include <Eigen/Dense>

namespace lgmath {
namespace r3 {

//////////////////////////////////////////////////////////////////////////////////////////////
/// A 3D point
//////////////////////////////////////////////////////////////////////////////////////////////
typedef Eigen::Vector3d Point;
typedef Eigen::Ref<Point> PointRef;
typedef Eigen::Ref<const Point> PointConstRef;

//////////////////////////////////////////////////////////////////////////////////////////////
/// A 3D homogeneous point
//////////////////////////////////////////////////////////////////////////////////////////////
typedef Eigen::Vector4d HPoint;
typedef Eigen::Ref<HPoint> HPointRef;
typedef Eigen::Ref<const HPoint> HPointConstRef;

//////////////////////////////////////////////////////////////////////////////////////////////
/// A 3x3 covariance for a 3D point
//////////////////////////////////////////////////////////////////////////////////////////////
typedef Eigen::Matrix3d CovarianceMatrix;
typedef Eigen::Ref<CovarianceMatrix> CovarianceMatrixRef;
typedef Eigen::Ref<const CovarianceMatrix> CovarianceMatrixConstRef;

} // r3
} // lgmath

#endif // LGM_R3_TYPES_HPP
