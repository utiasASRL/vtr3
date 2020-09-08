//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Unit tests for the transformation of points (with and without
/// covariance)
///
/// \author Kirk MacTavish
//////////////////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>

#include <math.h>
#include <iomanip>
#include <ios>
#include <iostream>
#include <typeinfo>

#include <Eigen/Dense>
#include <lgmath.hpp>
#include <lgmath/CommonMath.hpp>

/////////////////////////////////////////////////////////////////////////////////////////////
///
/// UNIT TESTS OF POINTS WITH AND WITHOUT COVARIANCE
///
/////////////////////////////////////////////////////////////////////////////////////////////

using namespace lgmath;

/////////////////////////////////////////////////////////////////////////////////////////////
// HELPER CONSTANTS
/////////////////////////////////////////////////////////////////////////////////////////////

static const so3::RotationMatrix C_z180 =
    so3::Rotation(so3::AxisAngle(0., 0., constants::PI)).matrix();
static const so3::RotationMatrix C_z90 =
    so3::Rotation(so3::AxisAngle(0., 0., constants::PI_DIV_TWO)).matrix();

/////////////////////////////////////////////////////////////////////////////////////////////
// MAIN TESTS
/////////////////////////////////////////////////////////////////////////////////////////////

TEST(Points, PointCovarianceTransform) {
  r3::CovarianceMatrix cov_a;
  cov_a.setZero();
  cov_a.diagonal() << 1., 2., 3.;
  r3::HPoint p_a = (r3::Point() << 2., 3., 4.).finished().homogeneous();

  {
    std::cout << "Given a 180 degree transform, "
              << "when we transform the covariance, "
              << "then the covariance should be unchanged." << std::endl;
    se3::Transformation T_ba(C_z180, se3::TranslationVector::Zero());
    auto cov_b =
        r3::transformCovariance<r3::COVARIANCE_NOT_REQUIRED>(T_ba, cov_a);
    std::cout << cov_a << "\n==\n" << cov_b << std::endl;
    EXPECT_TRUE(cov_a.isApprox(cov_b));
  }
  {
    std::cout << "Given a 90 degree transform, "
              << "when we transform the covariance, "
              << "then the covariance should should have x and y swapped."
              << std::endl;
    se3::Transformation T_ba(C_z90, se3::TranslationVector::Zero());
    auto cov_b =
        r3::transformCovariance<r3::COVARIANCE_NOT_REQUIRED>(T_ba, cov_a);
    cov_a.row(0).swap(cov_a.row(1));
    cov_a.col(0).swap(cov_a.col(1));
    std::cout << cov_a << "\n==\n" << cov_b << std::endl;
    EXPECT_TRUE(cov_a.isApprox(cov_b));
  }
  {
    std::cout << "Given uncertain translation, "
              << "when we translate the point and covariance, "
              << "then the covariance should be additive." << std::endl;
    se3::LieAlgebraCovariance S_ba = se3::LieAlgebraCovariance::Zero();
    S_ba.topLeftCorner<3, 3>() = cov_a;
    se3::TransformationWithCovariance T_ba(
        so3::RotationMatrix::Identity(), se3::TranslationVector::Zero(), S_ba);
    auto p_b = T_ba * p_a;
    r3::CovarianceMatrix cov_b;
    EXPECT_NO_THROW(cov_b = r3::transformCovariance(T_ba, cov_a, p_b));
    r3::CovarianceMatrix cov_b_expect = cov_a * 2.;
    std::cout << cov_b << "\n==\n" << cov_b_expect << std::endl;
    EXPECT_TRUE(cov_b.isApprox(cov_b_expect));
  }
  {
    std::cout << "Given uninitialized uncertain transform, "
              << "when we transform the point without ignoring the 'covariance "
                 "set' flag, "
              << "then it should throw." << std::endl;
    se3::TransformationWithCovariance T_ba;
    r3::CovarianceMatrix cov_b;
    auto p_b = T_ba * p_a;
    EXPECT_THROW(cov_b = r3::transformCovariance(T_ba, cov_a, p_b),
                 std::runtime_error);

    std::cout
        << "when we transform the point but ignore the 'covariance set' flag, "
        << "then it shouldn't throw, and the covariance should be unchanged, "
        << std::endl;
    EXPECT_NO_THROW(cov_b =
                        r3::transformCovariance<r3::COVARIANCE_NOT_REQUIRED>(
                            T_ba, cov_a, p_b));
    std::cout << cov_a << "\n==\n" << cov_b << std::endl;
    EXPECT_TRUE(cov_a.isApprox(cov_b));
  }
  {
    std::cout
        << "Given uncertain rotation, "
        << "when we transform the point and covariance, "
        << "then the covariance should be unchanged in Z, and larger in X and Y"
        << std::endl;
    se3::LieAlgebraCovariance S_ba;
    S_ba.setZero();
    S_ba(5, 5) = 1;
    se3::TransformationWithCovariance T_ba(
        se3::TransformationMatrix(se3::TransformationMatrix::Identity()), S_ba);
    auto p_b = T_ba * p_a;
    auto cov_b = r3::transformCovariance(T_ba, cov_a, p_b);
    EXPECT_TRUE(cov_b(0, 0) > cov_a(0, 0) + 1e-3);
    EXPECT_TRUE(cov_b(1, 1) > cov_a(1, 1) + 1e-3);
    EXPECT_TRUE(cov_b(2, 2) == cov_a(2, 2));  // Approximately
  }
}

TEST(Points, PointTransform) {
  std::cout << "Given a point, "
            << "when we transform it, "
            << "then it should be in the right spot." << std::endl;

  r3::HPoint x;
  x << 1., 2., 3., 1.;
  se3::Transformation T(
      (se3::LieAlgebra() << 1., 0, 0, constants::PI, 0, 0).finished());
  r3::HPoint x_tf;
  x_tf << 2., -2., -3., 1.;
  std::cout << T * x << "\n==\n" << x_tf << std::endl;
  EXPECT_TRUE((T * x).isApprox(x_tf));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
