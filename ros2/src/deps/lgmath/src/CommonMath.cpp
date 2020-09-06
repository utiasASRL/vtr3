//////////////////////////////////////////////////////////////////////////////////////////////
/// \file CommonMath.cpp
/// \brief Implementation file for some common math functions
/// \details defines some constants, angle-based functions, and comparison functions.
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <lgmath/CommonMath.hpp>

#include <math.h>

namespace lgmath {
namespace common {

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief moves a radian value to the range -pi, pi
  //////////////////////////////////////////////////////////////////////////////////////////////
  double angleMod(double radians) {
    return (double)( radians - ( constants::TWO_PI * rint(radians*constants::ONE_DIV_TWO_PI) ) );
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief converts from degrees to radians
  //////////////////////////////////////////////////////////////////////////////////////////////
  double deg2rad(double degrees) {
    return (double)(degrees * constants::DEG2RAD);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief converts from radians to degrees
  //////////////////////////////////////////////////////////////////////////////////////////////
  double rad2deg(double radians) {
    return (double)(radians * constants::RAD2DEG);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief compares if doubles are near equal
  //////////////////////////////////////////////////////////////////////////////////////////////
  bool nearEqual(double a, double b, double tol) {
    return fabs(a-b) <= tol;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief compares if (double) Eigen matrices are near equal
  //////////////////////////////////////////////////////////////////////////////////////////////
  bool nearEqual(Eigen::MatrixXd A, Eigen::MatrixXd B, double tol) {
    bool near = true;
    near = near & (A.cols() == B.cols());
    near = near & (A.rows() == B.rows());
    for (int j = 0; j < A.cols(); j++) {
      for (int i = 0; i < A.rows(); i++) {
        near = near & nearEqual(A(i,j),B(i,j),tol);
      }
    }
    return near;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief compares if radian angles are near equal
  //////////////////////////////////////////////////////////////////////////////////////////////
  bool nearEqualAngle(double radA, double radB, double tol) {
    return nearEqual(angleMod(radA-radB), 0.0, tol);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief compares if axis angles are near equal
  //////////////////////////////////////////////////////////////////////////////////////////////
  bool nearEqualAxisAngle(Eigen::Matrix<double,3,1> aaxis1, Eigen::Matrix<double,3,1> aaxis2, double tol) {
    bool near = true;

    // get angles
    double a1 = aaxis1.norm();
    double a2 = aaxis2.norm();

    // check if both angles are near zero
    if (fabs(a1) < tol && fabs(a2) < tol) {
      return true;
    } else { // otherwise, compare normalized axis

      // compare each element of axis
      Eigen::Matrix<double,3,1> axis1 = aaxis1/a1;
      Eigen::Matrix<double,3,1> axis2 = aaxis1/a2;
      for (int i = 0; i < 3; i++) {
        near = near & nearEqual(axis1(i),axis2(i),tol);
      }

      // compare wrapped angles
      near = near & nearEqualAngle(a1,a2,tol);
      return near;
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief compares if lie algebra is near equal
  //////////////////////////////////////////////////////////////////////////////////////////////
  bool nearEqualLieAlg(Eigen::Matrix<double,6,1> vec1, Eigen::Matrix<double,6,1> vec2, double tol) {
    bool near = true;
    near = near & nearEqualAxisAngle(vec1.tail<3>(), vec2.tail<3>(), tol);
    near = near & nearEqual(vec1.head<3>(), vec2.head<3>(), tol);
    return near;
  }

} // common
} // lgmath
