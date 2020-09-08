//////////////////////////////////////////////////////////////////////////////////////////////
/// \file Operations.cpp
/// \brief Implementation file for the SE3 Lie Group math functions.
/// \details These namespace functions provide implementations of the special Euclidean (SE)
///          Lie group functions that we commonly use in robotics.
///
/// \author Sean Anderson
//////////////////////////////////////////////////////////////////////////////////////////////

#include <lgmath/se3/Operations.hpp>

#include <Eigen/Dense>
#include <stdexcept>
#include <stdio.h>

#include <lgmath/so3/Operations.hpp>

namespace lgmath {
namespace se3 {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Builds the 4x4 "skew symmetric matrix"
///
/// The hat (^) operator, builds the 4x4 skew symmetric matrix from the 3x1 axis angle
/// vector and 3x1 translation vector.
///
/// hat(rho, aaxis) = [aaxis^ rho] = [0.0  -a3   a2  rho1]
///                   [  0^T    0]   [ a3  0.0  -a1  rho2]
///                                  [-a2   a1  0.0  rho3]
///                                  [0.0  0.0  0.0   0.0]
///
/// See eq. 4 in Barfoot-TRO-2014 for more information.
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4d hat(const Eigen::Vector3d& rho, const Eigen::Vector3d& aaxis) {

  Eigen::Matrix4d mat = Eigen::Matrix4d::Zero();
  mat.topLeftCorner<3,3>() = so3::hat(aaxis);
  mat.topRightCorner<3,1>() = rho;
  return mat;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Builds the 4x4 "skew symmetric matrix"
///
/// The hat (^) operator, builds the 4x4 skew symmetric matrix from
/// the 6x1 se3 algebra vector, xi:
///
/// xi^ = [rho  ] = [aaxis^ rho] = [0.0  -a3   a2  rho1]
///       [aaxis]   [  0^T    0]   [ a3  0.0  -a1  rho2]
///                                [-a2   a1  0.0  rho3]
///                                [0.0  0.0  0.0   0.0]
///
/// See eq. 4 in Barfoot-TRO-2014 for more information.
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4d hat(const Eigen::Matrix<double,6,1>& xi) {

  return hat(xi.head<3>(), xi.tail<3>());
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Builds the 6x6 "curly hat" matrix (related to the skew symmetric matrix)
///
/// The curly hat operator builds the 6x6 skew symmetric matrix from the 3x1 axis angle
/// vector and 3x1 translation vector.
///
/// curlyhat(rho, aaxis) = [aaxis^   rho^]
///                        [     0 aaxis^]
///
/// See eq. 12 in Barfoot-TRO-2014 for more information.
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,6> curlyhat(const Eigen::Vector3d& rho, const Eigen::Vector3d& aaxis) {

  Eigen::Matrix<double,6,6> mat = Eigen::Matrix<double,6,6>::Zero();
  mat.topLeftCorner<3,3>() = mat.bottomRightCorner<3,3>() = so3::hat(aaxis);
  mat.topRightCorner<3,3>() = so3::hat(rho);
  return mat;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Builds the 6x6 "curly hat" matrix (related to the skew symmetric matrix)
///
/// The curly hat operator builds the 6x6 skew symmetric matrix
/// from the 6x1 se3 algebra vector, xi:
///
/// curlyhat(xi) = curlyhat([rho  ]) = [aaxis^   rho^]
///                        ([aaxis])   [     0 aaxis^]
///
/// See eq. 12 in Barfoot-TRO-2014 for more information.
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,6> curlyhat(const Eigen::Matrix<double,6,1>& xi) {

  return curlyhat(xi.head<3>(), xi.tail<3>());
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Turns a homogeneous point into a special 4x6 matrix
///
/// See eq. 72 in Barfoot-TRO-2014 for more information.
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,4,6> point2fs(const Eigen::Vector3d& p, double scale) {

  Eigen::Matrix<double,4,6> mat = Eigen::Matrix<double,4,6>::Zero();
  mat.topLeftCorner<3,3>() = scale*Eigen::Matrix3d::Identity();
  mat.topRightCorner<3,3>() = -so3::hat(p);
  return mat;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Turns a homogeneous point into a special 6x4 matrix
///
/// See eq. 72 in Barfoot-TRO-2014 for more information.
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,4> point2sf(const Eigen::Vector3d& p, double scale) {

  Eigen::Matrix<double,6,4> mat = Eigen::Matrix<double,6,4>::Zero();
  mat.bottomLeftCorner<3,3>() = -so3::hat(p);
  mat.topRightCorner<3,1>() = p;
  return mat;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Builds a transformation matrix using the analytical exponential map
///
/// This function builds a transformation matrix, T_ab, using the analytical exponential map,
/// from the se3 algebra vector, xi_ba,
///
///   T_ab = exp(xi_ba^) = [ C_ab r_ba_ina],   xi_ba = [  rho_ba]
///                        [  0^T        1]            [aaxis_ba]
///
/// where C_ab is a 3x3 rotation matrix from 'b' to 'a', r_ba_ina is the 3x1 translation
/// vector from 'a' to 'b' expressed in frame 'a', aaxis_ba is a 3x1 axis-angle vector,
/// the magnitude of the angle of rotation can be recovered by finding the norm of the vector,
/// and the axis of rotation is the unit-length vector that arises from normalization.
/// Note that the angle around the axis, aaxis_ba, is a right-hand-rule (counter-clockwise
/// positive) angle from 'a' to 'b'.
///
/// The parameter, rho_ba, is a special translation-like parameter related to 'twist' theory.
/// It is most inuitively described as being like a constant linear velocity (expressed in
/// the smoothly-moving frame) for a fixed duration; for example, consider the curve of a
/// car driving 'x' meters while turning at a rate of 'y' rad/s.
///
/// For more information see Barfoot-TRO-2014 Appendix B1.
///
/// Alternatively, we that note that
///
///   T_ba = exp(-xi_ba^) = exp(xi_ab^).
///
/// Both the analytical (numTerms = 0) or the numerical (numTerms > 0) may be evaluated.
//////////////////////////////////////////////////////////////////////////////////////////////
void vec2tran_analytical(const Eigen::Vector3d& rho_ba, const Eigen::Vector3d& aaxis_ba,
                         Eigen::Matrix3d* out_C_ab, Eigen::Vector3d* out_r_ba_ina) {

  // Check pointers
  if (out_C_ab == NULL) {
    throw std::invalid_argument("Null pointer out_C_ab in vec2tran_analytical");
  }
  if (out_r_ba_ina == NULL) {
    throw std::invalid_argument("Null pointer out_r_ba_ina in vec2tran_analytical");
  }

  if(aaxis_ba.norm() < 1e-12) {

    // If angle is very small, rotation is Identity
    *out_C_ab = Eigen::Matrix3d::Identity();
    *out_r_ba_ina = rho_ba;
  } else {

    // Normal analytical solution
    Eigen::Matrix3d J_ab;

    // Use rotation identity involving jacobian, as we need it to
    // convert rho_ba to the proper translation
    so3::vec2rot(aaxis_ba, out_C_ab, &J_ab);

    // Convert rho_ba (twist-translation) to r_ba_ina
    *out_r_ba_ina = J_ab*rho_ba;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Builds a transformation matrix using the first N terms of the infinite series
///
/// Builds a transformation matrix numerically using the infinite series evalation
/// of the exponential map.
///
/// For more information see eq. 96 in Barfoot-TRO-2014
//////////////////////////////////////////////////////////////////////////////////////////////
void vec2tran_numerical(const Eigen::Vector3d& rho_ba, const Eigen::Vector3d& aaxis_ba,
                        Eigen::Matrix3d* out_C_ab, Eigen::Vector3d* out_r_ba_ina,
                        unsigned int numTerms) {

  // Check pointers
  if (out_C_ab == NULL) {
    throw std::invalid_argument("Null pointer out_C_ab in vec2tran_numerical");
  }
  if (out_r_ba_ina == NULL) {
    throw std::invalid_argument("Null pointer out_r_ba_ina in vec2tran_numerical");
  }

  // Init 4x4 transformation
  Eigen::Matrix4d T_ab = Eigen::Matrix4d::Identity();

  // Incremental variables
  Eigen::Matrix<double,6,1> xi_ba; xi_ba << rho_ba, aaxis_ba;
  Eigen::Matrix4d x_small = se3::hat(xi_ba);
  Eigen::Matrix4d x_small_n = Eigen::Matrix4d::Identity();

  // Loop over sum up to the specified numTerms
  for (unsigned int n = 1; n <= numTerms; n++) {
    x_small_n = x_small_n*x_small/double(n);
    T_ab += x_small_n;
  }

  // Fill output
  *out_C_ab = T_ab.topLeftCorner<3,3>();
  *out_r_ba_ina = T_ab.topRightCorner<3,1>();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Builds the 3x3 rotation and 3x1 translation using the exponential map, the
///        default parameters (numTerms = 0) use the analytical solution.
//////////////////////////////////////////////////////////////////////////////////////////////
void vec2tran(const Eigen::Matrix<double,6,1>& xi_ba, Eigen::Matrix3d* out_C_ab,
              Eigen::Vector3d* out_r_ba_ina, unsigned int numTerms) {

  if (numTerms == 0) {

    // Analytical solution
    vec2tran_analytical(xi_ba.head<3>(), xi_ba.tail<3>(), out_C_ab, out_r_ba_ina);
  } else {

    // Numerical solution (good for testing the analytical solution)
    vec2tran_numerical(xi_ba.head<3>(), xi_ba.tail<3>(), out_C_ab, out_r_ba_ina, numTerms);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Builds a 4x4 transformation matrix using the exponential map, the
///        default parameters (numTerms = 0) use the analytical solution.
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4d vec2tran(const Eigen::Matrix<double,6,1>& xi_ba, unsigned int numTerms) {

  // Get rotation and translation
  Eigen::Matrix3d C_ab;
  Eigen::Vector3d r_ba_ina;
  vec2tran(xi_ba, &C_ab, &r_ba_ina, numTerms);

  // Fill output
  Eigen::Matrix4d T_ab = Eigen::Matrix4d::Identity();
  T_ab.topLeftCorner<3,3>() = C_ab;
  T_ab.topRightCorner<3,1>() = r_ba_ina;
  return T_ab;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Compute the matrix log of a transformation matrix (from the rotation and trans)
///
/// Compute the inverse of the exponential map (the logarithmic map). This lets us go from
/// a the 3x3 rotation and 3x1 translation vector back to a 6x1 se3 algebra vector (composed
/// of a 3x1 axis-angle vector and 3x1 twist-translation vector). In some cases, when the
/// rotation in the transformation matrix is 'numerically off', this involves some
/// 'projection' back to SE(3).
///
///   xi_ba = ln(T_ab)
///
/// where xi_ba is the 6x1 se3 algebra vector. Alternatively, we that note that
///
///   xi_ab = -xi_ba = ln(T_ba) = ln(T_ab^{-1})
///
/// See Barfoot-TRO-2014 Appendix B2 for more information.
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,1> tran2vec(const Eigen::Matrix3d& C_ab,
                                   const Eigen::Vector3d& r_ba_ina) {

  // Init
  Eigen::Matrix<double,6,1> xi_ba;

  // Get axis angle from rotation matrix
  Eigen::Vector3d aaxis_ba = so3::rot2vec(C_ab);

  // Get twist-translation vector using Jacobian
  Eigen::Vector3d rho_ba = so3::vec2jacinv(aaxis_ba)*r_ba_ina;

  // Return se3 algebra vector
  xi_ba << rho_ba, aaxis_ba;
  return xi_ba;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Compute the matrix log of a transformation matrix
///
/// Compute the inverse of the exponential map (the logarithmic map). This lets us go from
/// a 4x4 transformation matrix back to a 6x1 se3 algebra vector (composed of a 3x1 axis-angle
/// vector and 3x1 twist-translation vector). In some cases, when the rotation in the
/// transformation matrix is 'numerically off', this involves some 'projection' back to SE(3).
///
///   xi_ba = ln(T_ab)
///
/// where xi_ba is the 6x1 se3 algebra vector. Alternatively, we that note that
///
///   xi_ab = -xi_ba = ln(T_ba) = ln(T_ab^{-1})
///
/// See Barfoot-TRO-2014 Appendix B2 for more information.
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,1> tran2vec(const Eigen::Matrix4d& T_ab) {

  return tran2vec(T_ab.topLeftCorner<3,3>(), T_ab.topRightCorner<3,1>());
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Builds the 6x6 adjoint transformation matrix from the 3x3 rotation matrix and 3x1
///        translation vector.
///
/// Builds the 6x6 adjoint transformation matrix from the 3x3 rotation matrix and 3x1
///        translation vector.
///
///  Adjoint(T_ab) = Adjoint([C_ab r_ba_ina]) = [C_ab r_ba_ina^*C_ab] = exp(curlyhat(xi_ba))
///                         ([ 0^T        1])   [   0           C_ab]
///
/// See eq. 101 in Barfoot-TRO-2014 for more information.
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,6> tranAd(const Eigen::Matrix3d& C_ab,
                                 const Eigen::Vector3d& r_ba_ina) {

  Eigen::Matrix<double,6,6> adjoint_T_ab = Eigen::Matrix<double,6,6>::Zero();
  adjoint_T_ab.topLeftCorner<3,3>() = adjoint_T_ab.bottomRightCorner<3,3>() = C_ab;
  adjoint_T_ab.topRightCorner<3,3>() = so3::hat(r_ba_ina)*C_ab;
  return adjoint_T_ab;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Builds the 6x6 adjoint transformation matrix from a 4x4 one
///
/// Builds the 6x6 adjoint transformation matrix from a 4x4 transformation matrix
///
///  Adjoint(T_ab) = Adjoint([C_ab r_ba_ina]) = [C_ab r_ba_ina^*C_ab] = exp(curlyhat(xi_ba))
///                         ([ 0^T        1])   [   0           C_ab]
///
/// See eq. 101 in Barfoot-TRO-2014 for more information.
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,6> tranAd(const Eigen::Matrix4d& T_ab) {

  return tranAd(T_ab.topLeftCorner<3,3>(), T_ab.topRightCorner<3,1>());
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Construction of the 3x3 "Q" matrix, used in the 6x6 Jacobian of SE(3)
///
/// See eq. 102 in Barfoot-TRO-2014 for more information
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3d vec2Q(const Eigen::Vector3d& rho_ba, const Eigen::Vector3d& aaxis_ba) {

  // Construct scalar terms
  const double ang = aaxis_ba.norm();
  const double ang2 = ang*ang;
  const double ang3 = ang2*ang;
  const double ang4 = ang3*ang;
  const double ang5 = ang4*ang;
  const double cang = cos(ang);
  const double sang = sin(ang);
  const double m2 = (ang - sang)/ang3;
  const double m3 = (1.0 - 0.5*ang2 - cang)/ang4;
  const double m4 = 0.5*(m3 - 3*(ang - sang - ang3/6)/ang5);

  // Construct matrix terms
  Eigen::Matrix3d rx = so3::hat(rho_ba);
  Eigen::Matrix3d px = so3::hat(aaxis_ba);
  Eigen::Matrix3d pxrx = px*rx;
  Eigen::Matrix3d rxpx = rx*px;
  Eigen::Matrix3d pxrxpx = pxrx*px;

  // Construct Q matrix
  return 0.5 * rx + m2 * (pxrx + rxpx + pxrxpx)
         - m3 * (px*pxrx + rxpx*px - 3*pxrxpx) - m4 * (pxrxpx*px + px*pxrxpx);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Construction of the 3x3 "Q" matrix, used in the 6x6 Jacobian of SE(3)
///
/// See eq. 102 in Barfoot-TRO-2014 for more information
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3d vec2Q(const Eigen::Matrix<double,6,1>& xi_ba) {
  return vec2Q(xi_ba.head<3>(), xi_ba.tail<3>());
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Builds the 6x6 Jacobian matrix of SE(3) using the analytical expression
///
/// Build the 6x6 left Jacobian of SE(3).
///
/// For the sake of a notation, we assign subscripts consistence with the transformation,
///
///   J_ab = J(xi_ba)
///
/// Where applicable, we also note that
///
///   J(xi_ba) = Adjoint(exp(xi_ba^)) * J(-xi_ba),
///
/// and
///
///   Adjoint(exp(xi_ba^)) = identity + curlyhat(xi_ba) * J(xi_ba).
///
/// For more information see eq. 100 in Barfoot-TRO-2014.
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,6> vec2jac(const Eigen::Vector3d& rho_ba,
                                  const Eigen::Vector3d& aaxis_ba) {

  // Init
  Eigen::Matrix<double,6,6> J_ab = Eigen::Matrix<double,6,6>::Zero();

  if(aaxis_ba.norm() < 1e-12) {

    // If angle is very small, so3 jacobian is Identity
    J_ab.topLeftCorner<3,3>() = J_ab.bottomRightCorner<3,3>() = Eigen::Matrix3d::Identity();
    J_ab.topRightCorner<3,3>() = 0.5*so3::hat(rho_ba);
  } else {

    // General analytical scenario
    J_ab.topLeftCorner<3,3>() = J_ab.bottomRightCorner<3,3>() = so3::vec2jac(aaxis_ba);
    J_ab.topRightCorner<3,3>() = se3::vec2Q(rho_ba, aaxis_ba);
  }
  return J_ab;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Builds the 6x6 Jacobian matrix of SE(3) from the se(3) algebra; note that the
///        default parameter (numTerms = 0) will call the analytical solution, but the
///        numerical solution can also be evaluating to some number of terms.
///
/// For more information see eq. 100 in Barfoot-TRO-2014.
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,6> vec2jac(const Eigen::Matrix<double,6,1>& xi_ba,
                                  unsigned int numTerms) {

  if (numTerms == 0) {

    // Analytical solution
    return vec2jac(xi_ba.head<3>(), xi_ba.tail<3>());
  } else {

    // Numerical solution (good for testing the analytical solution)
    Eigen::Matrix<double,6,6> J_ab = Eigen::Matrix<double,6,6>::Identity();

    // Incremental variables
    Eigen::Matrix<double,6,6> x_small = se3::curlyhat(xi_ba);
    Eigen::Matrix<double,6,6> x_small_n = Eigen::Matrix<double,6,6>::Identity();

    // Loop over sum up to the specified numTerms
    for (unsigned int n = 1; n <= numTerms; n++) {
      x_small_n = x_small_n*x_small/double(n+1);
      J_ab += x_small_n;
    }
    return J_ab;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Builds the 6x6 inverse Jacobian matrix of SE(3) using the analytical expression
///
/// Build the 6x6 inverse left Jacobian of SE(3).
///
/// For the sake of a notation, we assign subscripts consistence with the transformation,
///
///   J_ab_inverse = J(xi_ba)^{-1},
///
/// Please note that J_ab_inverse is not equivalent to J_ba:
///
///   J(xi_ba)^{-1} != J(-xi_ba)
///
/// For more information see eq. 103 in Barfoot-TRO-2014.
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,6> vec2jacinv(const Eigen::Vector3d& rho_ba,
                                     const Eigen::Vector3d& aaxis_ba) {

  // Init
  Eigen::Matrix<double,6,6> J66_ab_inv = Eigen::Matrix<double,6,6>::Zero();

  if(aaxis_ba.norm() < 1e-12) {

    // If angle is very small, so3 jacobian is Identity
    J66_ab_inv.topLeftCorner<3,3>()  = J66_ab_inv.bottomRightCorner<3,3>()
                                     = Eigen::Matrix3d::Identity();
    J66_ab_inv.topRightCorner<3,3>() = -0.5*so3::hat(rho_ba);
  } else {

    // General analytical scenario
    Eigen::Matrix3d J33_ab_inv = so3::vec2jacinv(aaxis_ba);
    J66_ab_inv.topLeftCorner<3,3>() = J66_ab_inv.bottomRightCorner<3,3>() = J33_ab_inv;
    J66_ab_inv.topRightCorner<3,3>() = -J33_ab_inv*se3::vec2Q(rho_ba, aaxis_ba)*J33_ab_inv;
  }
  return J66_ab_inv;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Builds the 6x6 inverse Jacobian matrix of SE(3) from the se(3) algebra; note that
///        the default parameter (numTerms = 0) will call the analytical solution, but the
///        numerical solution can also be evaluating to some number of terms.
///
/// For more information see eq. 103 in Barfoot-TRO-2014.
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,6> vec2jacinv(const Eigen::Matrix<double,6,1>& xi_ba,
                                     unsigned int numTerms) {

  if (numTerms == 0) {

    // Analytical solution
    return vec2jacinv(xi_ba.head<3>(), xi_ba.tail<3>());
  } else {

    // Logic error
    if (numTerms > 20) {
      throw std::invalid_argument("Numerical vec2jacinv does not support numTerms > 20");
    }

    // Numerical solution (good for testing the analytical solution)
    Eigen::Matrix<double,6,6> J_ab = Eigen::Matrix<double,6,6>::Identity();

    // Incremental variables
    Eigen::Matrix<double,6,6> x_small = se3::curlyhat(xi_ba);
    Eigen::Matrix<double,6,6> x_small_n = Eigen::Matrix<double,6,6>::Identity();

    // Boost has a bernoulli package... but we shouldn't need more than 20
    Eigen::Matrix<double,21,1> bernoulli;
    bernoulli << 1.0, -0.5, 1.0/6.0, 0.0, -1.0/30.0, 0.0, 1.0/42.0, 0.0, -1.0/30.0,
                 0.0, 5.0/66.0, 0.0, -691.0/2730.0, 0.0, 7.0/6.0, 0.0, -3617.0/510.0,
                 0.0, 43867.0/798.0, 0.0, -174611.0/330.0;

    // Loop over sum up to the specified numTerms
    for (unsigned int n = 1; n <= numTerms; n++) {
      x_small_n = x_small_n*x_small/double(n);
      J_ab += bernoulli(n)*x_small_n;
    }
    return J_ab;
  }
}

} // se3
} // lgmath
