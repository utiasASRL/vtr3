//////////////////////////////////////////////////////////////////////////////////////////////
/// \file TransformTests.cpp
/// \brief Unit tests for the implementation of the transformation matrix class.
/// \details Unit tests for the various Lie Group functions will test both
/// special cases,
///          and randomly generated cases.
///
/// \author Sean Anderson
//////////////////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>

#include <math.h>
#include <iomanip>
#include <ios>
#include <iostream>

#include <Eigen/Dense>
#include <lgmath/CommonMath.hpp>

#include <lgmath/se3/Operations.hpp>
#include <lgmath/se3/Transformation.hpp>
#include <lgmath/so3/Operations.hpp>

/////////////////////////////////////////////////////////////////////////////////////////////
///
/// UNIT TESTS OF TRANSFORMATION MATRIX
///
/////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
/// \brief General test of transformation constructors
/////////////////////////////////////////////////////////////////////////////////////////////
TEST(LGMath, TransformationConstructors) {
  // Generate random transform from most basic constructor
  Eigen::Matrix<double, 3, 3> C_ba =
      lgmath::so3::vec2rot(Eigen::Matrix<double, 3, 1>::Random());
  Eigen::Matrix<double, 3, 1> r_ba_ina = Eigen::Matrix<double, 3, 1>::Random();
  lgmath::se3::Transformation rand(C_ba, r_ba_ina);

  // Transformation();
  {
    lgmath::se3::Transformation tmatrix;
    Eigen::Matrix4d test = Eigen::Matrix4d::Identity();
    std::cout << "tmat: " << tmatrix.matrix() << std::endl;
    std::cout << "test: " << test << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(tmatrix.matrix(), test, 1e-6));
  }

  // Transformation(const Transformation& T);
  {
    lgmath::se3::Transformation test(rand);
    std::cout << "tmat: " << rand.matrix() << std::endl;
    std::cout << "test: " << test.matrix() << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(rand.matrix(), test.matrix(), 1e-6));
  }

  // Transformation(const Eigen::Matrix4d& T);
  {
    lgmath::se3::Transformation test(rand.matrix());
    std::cout << "tmat: " << rand.matrix() << std::endl;
    std::cout << "test: " << test.matrix() << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(rand.matrix(), test.matrix(), 1e-6));

    // Test forced reprojection (ones to identity)
    Eigen::Matrix4d proj_test = Eigen::Matrix4d::Identity();
    proj_test.topRightCorner<3, 1>() = -r_ba_ina;
    Eigen::Matrix3d notRotation = Eigen::Matrix3d::Ones();
    Eigen::Matrix4d notTransform = Eigen::Matrix4d::Identity();
    notTransform.topLeftCorner<3, 3>() = notRotation;
    notTransform.topRightCorner<3, 1>() = -r_ba_ina;
    lgmath::se3::Transformation test_bad(notTransform);  // force reproj
    std::cout << "cmat: " << proj_test.matrix() << std::endl;
    std::cout << "test: " << test_bad.matrix() << std::endl;
    EXPECT_TRUE(
        lgmath::common::nearEqual(proj_test.matrix(), test_bad.matrix(), 1e-6));
  }

  // Transformation& operator=(Transformation T);
  {
    lgmath::se3::Transformation test = rand;
    std::cout << "tmat: " << rand.matrix() << std::endl;
    std::cout << "test: " << test.matrix() << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(rand.matrix(), test.matrix(), 1e-6));
  }

  // Transformation(const Eigen::Matrix<double,6,1>& vec, unsigned int numTerms
  // = 0);
  {
    Eigen::Matrix<double, 6, 1> vec = Eigen::Matrix<double, 6, 1>::Random();
    Eigen::Matrix4d tmat = lgmath::se3::vec2tran(vec);
    lgmath::se3::Transformation testAnalytical(vec);
    lgmath::se3::Transformation testNumerical(vec, 15);
    std::cout << "tmat: " << tmat << std::endl;
    std::cout << "testAnalytical: " << testAnalytical.matrix() << std::endl;
    std::cout << "testNumerical: " << testNumerical.matrix() << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(tmat, testAnalytical.matrix(), 1e-6));
    EXPECT_TRUE(lgmath::common::nearEqual(tmat, testNumerical.matrix(), 1e-6));
  }

  // Transformation(const Eigen::VectorXd& vec);
  {
    Eigen::VectorXd vec = Eigen::Matrix<double, 6, 1>::Random();
    Eigen::Matrix4d tmat = lgmath::se3::vec2tran(vec);
    lgmath::se3::Transformation test(vec);
    std::cout << "tmat: " << tmat << std::endl;
    std::cout << "test: " << test.matrix() << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(tmat, test.matrix(), 1e-6));
  }

  // Transformation(const Eigen::VectorXd& vec);
  {
    Eigen::VectorXd vec = Eigen::Matrix<double, 6, 1>::Random();
    lgmath::se3::Transformation test(vec);

    // Wrong size vector
    Eigen::VectorXd badvec = Eigen::Matrix<double, 3, 1>::Random();
    lgmath::se3::Transformation testFailure;
    try {
      testFailure = lgmath::se3::Transformation(badvec);
    } catch (const std::invalid_argument& e) {
      testFailure = test;
    }
    std::cout << "tmat: " << testFailure.matrix() << std::endl;
    std::cout << "test: " << test.matrix() << std::endl;
    EXPECT_TRUE(
        lgmath::common::nearEqual(testFailure.matrix(), test.matrix(), 1e-6));
  }

  // Transformation(const Eigen::Matrix3d& C_ba,
  //               const Eigen::Vector3d& r_ba_ina);
  {
    lgmath::se3::Transformation tmat(C_ba, r_ba_ina);
    Eigen::Matrix4d test = Eigen::Matrix4d::Identity();
    test.topLeftCorner<3, 3>() = C_ba;
    test.topRightCorner<3, 1>() = -C_ba * r_ba_ina;
    std::cout << "tmat: " << tmat.matrix() << std::endl;
    std::cout << "test: " << test << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(tmat.matrix(), test, 1e-6));

    // Test forced reprojection (ones to identity)
    Eigen::Matrix4d proj_test = Eigen::Matrix4d::Identity();
    proj_test.topRightCorner<3, 1>() = -Eigen::Matrix3d::Identity() * r_ba_ina;
    Eigen::Matrix3d notRotation = Eigen::Matrix3d::Ones();
    lgmath::se3::Transformation test_bad(notRotation,
                                         r_ba_ina);  // forces reprojection
    std::cout << "cmat: " << proj_test.matrix() << std::endl;
    std::cout << "test: " << test_bad.matrix() << std::endl;
    EXPECT_TRUE(
        lgmath::common::nearEqual(proj_test.matrix(), test_bad.matrix(), 1e-6));
  }

  // Transformation(Transformation&&);
  {
    auto rand2 = rand;
    lgmath::se3::Transformation test(std::move(rand));
    rand = rand2;

    std::cout << "tmat: " << test.matrix() << std::endl;
    std::cout << "test: " << rand.matrix() << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(test.matrix(), rand.matrix(), 1e-6));
  }

  // Transformation = Transformation&&;
  {
    lgmath::se3::Transformation test;
    auto rand2 = rand;
    test = std::move(rand);
    rand = rand2;

    std::cout << "tmat: " << test.matrix() << std::endl;
    std::cout << "test: " << rand.matrix() << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(test.matrix(), rand.matrix(), 1e-6));
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Test some get methods
/////////////////////////////////////////////////////////////////////////////////////////////
TEST(LGMath, TransformationGetMethods) {
  // Generate random transform from most basic constructor
  Eigen::Matrix<double, 3, 3> C_ba =
      lgmath::so3::vec2rot(Eigen::Matrix<double, 3, 1>::Random());
  Eigen::Matrix<double, 3, 1> r_ba_ina = Eigen::Matrix<double, 3, 1>::Random();
  lgmath::se3::Transformation T_ba(C_ba, r_ba_ina);

  // Construct simple eigen matrix from random rotation and translation
  Eigen::Matrix4d test = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 3, 1> r_ab_inb = -C_ba * r_ba_ina;
  test.topLeftCorner<3, 3>() = C_ba;
  test.topRightCorner<3, 1>() = r_ab_inb;

  // Test matrix()
  std::cout << "T_ba: " << T_ba.matrix() << std::endl;
  std::cout << "test: " << test << std::endl;
  EXPECT_TRUE(lgmath::common::nearEqual(T_ba.matrix(), test, 1e-6));

  // Test C_ba()
  std::cout << "T_ba: " << T_ba.C_ba() << std::endl;
  std::cout << "C_ba: " << C_ba << std::endl;
  EXPECT_TRUE(lgmath::common::nearEqual(T_ba.C_ba(), C_ba, 1e-6));

  // Test r_ba_ina()
  std::cout << "T_ba: " << T_ba.r_ba_ina() << std::endl;
  std::cout << "r_ba_ina: " << r_ba_ina << std::endl;
  EXPECT_TRUE(lgmath::common::nearEqual(T_ba.r_ba_ina(), r_ba_ina, 1e-6));

  // Test r_ab_inb()
  std::cout << "T_ba: " << T_ba.r_ab_inb() << std::endl;
  std::cout << "r_ab_inb: " << r_ab_inb << std::endl;
  EXPECT_TRUE(lgmath::common::nearEqual(T_ba.r_ab_inb(), r_ab_inb, 1e-6));
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Test exponential map construction and logarithmic vec() method
/////////////////////////////////////////////////////////////////////////////////////////////
TEST(LGMath, TransformationToFromSE3Algebra) {
  // Add vectors to be tested
  std::vector<Eigen::Matrix<double, 6, 1> > trueVecs;
  Eigen::Matrix<double, 6, 1> temp;
  temp << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  trueVecs.push_back(temp);
  temp << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, 0.0, lgmath::constants::PI, 0.0, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, 0.0, 0.0, lgmath::constants::PI, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, 0.0, 0.0, 0.0, lgmath::constants::PI;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, 0.0, -lgmath::constants::PI, 0.0, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, 0.0, 0.0, -lgmath::constants::PI, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, 0.0, 0.0, 0.0, -lgmath::constants::PI;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, 0.0, 0.5 * lgmath::constants::PI, 0.0, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, 0.0, 0.0, 0.5 * lgmath::constants::PI, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, 0.0, 0.0, 0.0, 0.5 * lgmath::constants::PI;
  trueVecs.push_back(temp);
  const unsigned numRand = 20;
  for (unsigned i = 0; i < numRand; i++) {
    trueVecs.push_back(Eigen::Matrix<double, 6, 1>::Random());
  }

  // Get number of tests
  const unsigned numTests = trueVecs.size();

  // Calc transformation matrices
  std::vector<Eigen::Matrix4d> transMatrices;
  for (unsigned i = 0; i < numTests; i++) {
    transMatrices.push_back(lgmath::se3::vec2tran(trueVecs.at(i)));
  }

  // Calc transformations
  std::vector<lgmath::se3::Transformation> transformations;
  for (unsigned i = 0; i < numTests; i++) {
    transformations.push_back(lgmath::se3::Transformation(trueVecs.at(i)));
  }

  // Compare matrices
  {
    for (unsigned i = 0; i < numTests; i++) {
      std::cout << "matr: " << transMatrices.at(i) << std::endl;
      std::cout << "tran: " << transformations.at(i).matrix() << std::endl;
      EXPECT_TRUE(lgmath::common::nearEqual(
          transMatrices.at(i), transformations.at(i).matrix(), 1e-6));
    }
  }

  // Test logarithmic map
  {
    for (unsigned i = 0; i < numTests; i++) {
      Eigen::Matrix<double, 6, 1> testVec = transformations.at(i).vec();
      std::cout << "true: " << trueVecs.at(i) << std::endl;
      std::cout << "func: " << testVec << std::endl;
      EXPECT_TRUE(
          lgmath::common::nearEqualLieAlg(trueVecs.at(i), testVec, 1e-6));
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Test inverse, adjoint and operatations
/////////////////////////////////////////////////////////////////////////////////////////////
TEST(LGMath, TransformationInverse) {
  // Add vectors to be tested
  std::vector<Eigen::Matrix<double, 6, 1> > trueVecs;
  Eigen::Matrix<double, 6, 1> temp;
  temp << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  trueVecs.push_back(temp);
  temp << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, 0.0, lgmath::constants::PI, 0.0, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, 0.0, 0.0, lgmath::constants::PI, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, 0.0, 0.0, 0.0, lgmath::constants::PI;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, 0.0, -lgmath::constants::PI, 0.0, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, 0.0, 0.0, -lgmath::constants::PI, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, 0.0, 0.0, 0.0, -lgmath::constants::PI;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, 0.0, 0.5 * lgmath::constants::PI, 0.0, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, 0.0, 0.0, 0.5 * lgmath::constants::PI, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, 0.0, 0.0, 0.0, 0.5 * lgmath::constants::PI;
  trueVecs.push_back(temp);
  const unsigned numRand = 20;
  for (unsigned i = 0; i < numRand; i++) {
    trueVecs.push_back(Eigen::Matrix<double, 6, 1>::Random());
  }

  // Get number of tests
  const unsigned numTests = trueVecs.size();

  // Add vectors to be tested - random
  std::vector<Eigen::Matrix<double, 4, 1> > landmarks;
  for (unsigned i = 0; i < numTests; i++) {
    landmarks.push_back(Eigen::Matrix<double, 4, 1>::Random());
  }

  // Calc transformation matrices
  std::vector<Eigen::Matrix4d> transMatrices;
  for (unsigned i = 0; i < numTests; i++) {
    transMatrices.push_back(lgmath::se3::vec2tran(trueVecs.at(i)));
  }

  // Calc transformations
  std::vector<lgmath::se3::Transformation> transformations;
  for (unsigned i = 0; i < numTests; i++) {
    transformations.push_back(lgmath::se3::Transformation(trueVecs.at(i)));
  }

  // Compare inverse to basic matrix inverse
  {
    for (unsigned i = 0; i < numTests; i++) {
      std::cout << "matr: " << transMatrices.at(i).inverse() << std::endl;
      std::cout << "tran: " << transformations.at(i).inverse().matrix()
                << std::endl;
      EXPECT_TRUE(lgmath::common::nearEqual(
          transMatrices.at(i).inverse(),
          transformations.at(i).inverse().matrix(), 1e-6));
    }
  }

  // Test that product of inverse and self make identity
  {
    for (unsigned i = 0; i < numTests; i++) {
      std::cout << "T*Tinv: "
                << transformations.at(i).matrix() *
                       transformations.at(i).inverse().matrix();
      EXPECT_TRUE(lgmath::common::nearEqual(
          transformations.at(i).matrix() *
              transformations.at(i).inverse().matrix(),
          Eigen::Matrix4d::Identity(), 1e-6));
    }
  }

  // Test adjoint
  {
    for (unsigned i = 0; i < numTests; i++) {
      std::cout << "matr: " << lgmath::se3::tranAd(transMatrices.at(i))
                << std::endl;
      std::cout << "tran: " << transformations.at(i).adjoint() << std::endl;
      EXPECT_TRUE(
          lgmath::common::nearEqual(lgmath::se3::tranAd(transMatrices.at(i)),
                                    transformations.at(i).adjoint(), 1e-6));
    }
  }

  // Test self-product
  {
    for (unsigned i = 0; i < numTests - 1; i++) {
      lgmath::se3::Transformation test = transformations.at(i);
      test *= transformations.at(i + 1);
      Eigen::Matrix4d matrix = transMatrices.at(i) * transMatrices.at(i + 1);
      std::cout << "matr: " << matrix << std::endl;
      std::cout << "tran: " << test.matrix() << std::endl;
      EXPECT_TRUE(lgmath::common::nearEqual(matrix, test.matrix(), 1e-6));
    }
  }

  // Test product
  {
    for (unsigned i = 0; i < numTests - 1; i++) {
      lgmath::se3::Transformation test =
          transformations.at(i) * transformations.at(i + 1);
      Eigen::Matrix4d matrix = transMatrices.at(i) * transMatrices.at(i + 1);
      std::cout << "matr: " << matrix << std::endl;
      std::cout << "tran: " << test.matrix() << std::endl;
      EXPECT_TRUE(lgmath::common::nearEqual(matrix, test.matrix(), 1e-6));
    }
  }

  // Test self product with inverse
  {
    for (unsigned i = 0; i < numTests - 1; i++) {
      lgmath::se3::Transformation test = transformations.at(i);
      test /= transformations.at(i + 1);
      Eigen::Matrix4d matrix =
          transMatrices.at(i) * transMatrices.at(i + 1).inverse();
      std::cout << "matr: " << matrix << std::endl;
      std::cout << "tran: " << test.matrix() << std::endl;
      EXPECT_TRUE(lgmath::common::nearEqual(matrix, test.matrix(), 1e-6));
    }
  }

  // Test product with inverse
  {
    for (unsigned i = 0; i < numTests - 1; i++) {
      lgmath::se3::Transformation test =
          transformations.at(i) / transformations.at(i + 1);
      Eigen::Matrix4d matrix =
          transMatrices.at(i) * transMatrices.at(i + 1).inverse();
      std::cout << "matr: " << matrix << std::endl;
      std::cout << "tran: " << test.matrix() << std::endl;
      EXPECT_TRUE(lgmath::common::nearEqual(matrix, test.matrix(), 1e-6));
    }
  }

  // Test product with landmark
  {
    for (unsigned i = 0; i < numTests; i++) {
      Eigen::Matrix<double, 4, 1> mat = transMatrices.at(i) * landmarks.at(i);
      Eigen::Matrix<double, 4, 1> test =
          transformations.at(i) * landmarks.at(i);

      std::cout << "matr: " << mat << std::endl;
      std::cout << "test: " << test << std::endl;
      EXPECT_TRUE(lgmath::common::nearEqual(mat, test, 1e-6));
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
