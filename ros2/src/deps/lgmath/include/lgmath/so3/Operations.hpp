//////////////////////////////////////////////////////////////////////////////////////////////
/// \file Operations.hpp
/// \brief Header file for the SO3 Lie Group math functions.
/// \details These namespace functions provide implementations of the special orthogonal (SO)
///          Lie group functions that we commonly use in robotics.
///
/// \author Sean Anderson
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef LGM_SO3_PUBLIC_HPP
#define LGM_SO3_PUBLIC_HPP

#include <Eigen/Core>

/////////////////////////////////////////////////////////////////////////////////////////////
/// Lie Group Math - Special Orthogonal Group
/////////////////////////////////////////////////////////////////////////////////////////////
namespace lgmath {
namespace so3 {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Builds the 3x3 skew symmetric matrix
///
/// The hat (^) operator, builds the 3x3 skew symmetric matrix from the 3x1 vector:
///
/// v^ = [0.0  -v3   v2]
///      [ v3  0.0  -v1]
///      [-v2   v1  0.0]
///
/// See eq. 5 in Barfoot-TRO-2014 for more information.
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3d hat(const Eigen::Vector3d& vector);

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Builds a rotation matrix using the exponential map
///
/// This function builds a rotation matrix, C_ab, using the exponential map (from an axis-
/// angle parameterization).
///
///   C_ab = exp(aaxis_ba^),
///
/// where aaxis_ba is a 3x1 axis-angle vector, the magnitude of the angle of rotation
/// can be recovered by finding the norm of the vector, and the axis of rotation is the unit-
/// length vector that arises from normalization. Note that the angle around the axis,
/// aaxis_ba, is a right-hand-rule (counter-clockwise positive) angle from 'a' to 'b'.
///
/// Alternatively, we that note that
///
///   C_ba = exp(-aaxis_ba^) = exp(aaxis_ab^).
///
/// Typical robotics convention has some oddity when it comes using this exponential map in
/// practice. For example, if we wish to integrate the kinematics:
///
///   d/dt C = omega^ * C,
///
/// where omega is the 3x1 angular velocity, we employ the convention:
///
///   C_20 = exp(deltaTime*-omega^) * C_10,
///
/// Noting that omega is negative (left-hand-rule).
/// For more information see eq. 97 in Barfoot-TRO-2014.
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3d vec2rot(const Eigen::Vector3d& aaxis_ba, unsigned int numTerms = 0);

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Builds and returns both the rotation matrix and SO(3) Jacobian
///
/// Similar to the function 'vec2rot', this function builds a rotation matrix, C_ab, using an
/// equivalent expression to the exponential map, but allows us to simultaneously extract
/// the Jacobian of SO(3), which is also required in some cases.
///
///   J_ab = jac(aaxis_ba)
///   C_ab = exp(aaxis_ba^) = identity + aaxis_ba^ * J_ab
///
/// For more information see eq. 97 in Barfoot-TRO-2014.
//////////////////////////////////////////////////////////////////////////////////////////////
void vec2rot(const Eigen::Vector3d& aaxis_ba, Eigen::Matrix3d* out_C_ab,
             Eigen::Matrix3d* out_J_ab);

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Compute the matrix log of a rotation matrix
///
/// Compute the inverse of the exponential map (the logarithmic map). This lets us go from
/// a 3x3 rotation matrix back to a 3x1 axis angle parameterization. In some cases, when the
/// rotation matrix is 'numerically off', this involves some 'projection' back to SO(3).
///
///   aaxis_ba = ln(C_ab)
///
/// where aaxis_ba is a 3x1 axis angle, where the axis is normalized and the magnitude of
/// the rotation can be recovered by finding the norm of the axis angle. Note that the
/// angle around the axis, aaxis_ba, is a right-hand-rule (counter-clockwise positive)
/// angle from 'a' to 'b'.
///
/// Alternatively, we that note that
///
///   aaxis_ab = -aaxis_ba = ln(C_ba) = ln(C_ab^T)
///
/// See Barfoot-TRO-2014 Appendix B2 for more information.
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3d rot2vec(const Eigen::Matrix3d& C_ab);

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Builds the 3x3 Jacobian matrix of SO(3)
///
/// Build the 3x3 left Jacobian of SO(3).
///
/// For the sake of a notation, we assign subscripts consistence with the rotation,
///
///   J_ab = J(aaxis_ba),
///
/// although we note to the SO(3) novice that this Jacobian is not a rotation matrix, and
/// should be used with care.
///
/// For more information see eq. 98 in Barfoot-TRO-2014.
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3d vec2jac(const Eigen::Vector3d& aaxis_ba, unsigned int numTerms = 0);

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Builds the 3x3 inverse Jacobian matrix of SO(3)
///
/// Build the 3x3 inverse left Jacobian of SO(3).
///
/// For the sake of a notation, we assign subscripts consistence with the rotation,
///
///   J_ab_inverse = J(aaxis_ba)^{-1},
///
/// although we note to the SO(3) novice that this Jacobian is not a rotation matrix, and
/// should be used with care. Also, please note that J_ab_inverse is not equivalent to J_ba:
///
///   J(aaxis_ba)^{-1} != J(-aaxis_ba)
///
/// For more information see eq. 99 in Barfoot-TRO-2014.
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3d vec2jacinv(const Eigen::Vector3d& aaxis_ba, unsigned int numTerms = 0);

} // so3
} // lgmath


#endif // LGM_SO3_PUBLIC_HPP
