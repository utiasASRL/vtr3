//////////////////////////////////////////////////////////////////////////////////////////////
/// \file RotationTests.cpp
/// \brief Unit tests for the implementation of the rotation matrix class.
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

#include <lgmath/so3/Operations.hpp>
#include <lgmath/so3/Rotation.hpp>

/////////////////////////////////////////////////////////////////////////////////////////////
///
/// UNIT TESTS OF ROTATION MATRIX
///
/////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
/// \brief General test of rotation constructors
/////////////////////////////////////////////////////////////////////////////////////////////
TEST(LGMath, RotationConstructors) {
  // Generate random transform from most basic constructor
  Eigen::Matrix3d C_ba_rand = lgmath::so3::vec2rot(Eigen::Vector3d::Random());
  lgmath::so3::Rotation rand(C_ba_rand);

  // Rotation();
  {
    lgmath::so3::Rotation cmatrix;
    Eigen::Matrix3d test = Eigen::Matrix3d::Identity();
    std::cout << "cmat: " << cmatrix.matrix() << std::endl;
    std::cout << "test: " << test << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(cmatrix.matrix(), test, 1e-6));
  }

  // Rotation(const Rotation& C);
  {
    lgmath::so3::Rotation test(rand);
    std::cout << "cmat: " << rand.matrix() << std::endl;
    std::cout << "test: " << test.matrix() << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(rand.matrix(), test.matrix(), 1e-6));
  }

  // Rotation(const Eigen::Matrix3d& C);
  {
    lgmath::so3::Rotation test = rand;
    std::cout << "cmat: " << rand.matrix() << std::endl;
    std::cout << "test: " << test.matrix() << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(rand.matrix(), test.matrix(), 1e-6));

    // Test forced reprojection (ones to identity)
    Eigen::Matrix3d notRotation = Eigen::Matrix3d::Ones();
    lgmath::so3::Rotation test_bad(notRotation);  // forces reprojection
    std::cout << "cmat: " << test_bad.matrix() << std::endl;
    std::cout << "test: " << Eigen::Matrix3d::Identity() << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(test_bad.matrix(),
                                          Eigen::Matrix3d::Identity(), 1e-6));
  }

  // Rotation& operator=(Rotation C);
  {
    lgmath::so3::Rotation test(C_ba_rand);
    std::cout << "cmat: " << rand.matrix() << std::endl;
    std::cout << "test: " << test.matrix() << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(rand.matrix(), test.matrix(), 1e-6));
  }

  // Rotation(const Eigen::Vector3d& vec, unsigned int numTerms = 0);
  {
    Eigen::Vector3d vec = Eigen::Vector3d::Random();
    Eigen::Matrix3d cmat = lgmath::so3::vec2rot(vec);
    lgmath::so3::Rotation testAnalytical(vec);
    lgmath::so3::Rotation testNumerical(vec, 15);
    std::cout << "cmat: " << cmat << std::endl;
    std::cout << "testAnalytical: " << testAnalytical.matrix() << std::endl;
    std::cout << "testNumerical: " << testNumerical.matrix() << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(cmat, testAnalytical.matrix(), 1e-6));
    EXPECT_TRUE(lgmath::common::nearEqual(cmat, testNumerical.matrix(), 1e-6));
  }

  // Rotation(const Eigen::VectorXd& vec);
  {
    Eigen::VectorXd vec = Eigen::Vector3d::Random();
    Eigen::Matrix3d tmat = lgmath::so3::vec2rot(vec);
    lgmath::so3::Rotation test(vec);
    std::cout << "tmat: " << tmat << std::endl;
    std::cout << "test: " << test.matrix() << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(tmat, test.matrix(), 1e-6));
  }

  // Rotation(const Eigen::VectorXd& vec);
  {
    Eigen::VectorXd vec = Eigen::Vector3d::Random();
    lgmath::so3::Rotation test(vec);

    Eigen::VectorXd badvec = Eigen::Matrix<double, 6, 1>::Random();
    lgmath::so3::Rotation testFailure;
    try {
      testFailure = lgmath::so3::Rotation(badvec);
    } catch (const std::invalid_argument& e) {
      testFailure = test;
    }
    std::cout << "tmat: " << testFailure.matrix() << std::endl;
    std::cout << "test: " << test.matrix() << std::endl;
    EXPECT_TRUE(
        lgmath::common::nearEqual(testFailure.matrix(), test.matrix(), 1e-6));
  }

  // Rotation(Rotation&&);
  {
    auto rand2 = rand;  // std::move may invalidate the original variable.
    lgmath::so3::Rotation test(std::move(rand));
    rand = rand2;

    std::cout << "tmat: " << test.matrix() << std::endl;
    std::cout << "test: " << rand2.matrix() << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(test.matrix(), rand2.matrix(), 1e-6));
  }

  // Rotation = Rotation&&;
  {
    lgmath::so3::Rotation test;
    auto rand2 = rand;  // std::move may invalidate the original variable.
    test = std::move(rand);
    rand = rand2;

    std::cout << "tmat: " << test.matrix() << std::endl;
    std::cout << "test: " << rand2.matrix() << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(test.matrix(), rand2.matrix(), 1e-6));
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Test exponential map construction and logarithmic vec() method
/////////////////////////////////////////////////////////////////////////////////////////////
TEST(LGMath, RotationToFromSE3Algebra) {
  // Add vectors to be tested
  std::vector<Eigen::Vector3d> trueVecs;
  Eigen::Vector3d temp;
  temp << 0.0, 0.0, 0.0;
  trueVecs.push_back(temp);
  temp << lgmath::constants::PI, 0.0, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, lgmath::constants::PI, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, lgmath::constants::PI;
  trueVecs.push_back(temp);
  temp << -lgmath::constants::PI, 0.0, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, -lgmath::constants::PI, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, -lgmath::constants::PI;
  trueVecs.push_back(temp);
  temp << 0.5 * lgmath::constants::PI, 0.0, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.5 * lgmath::constants::PI, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, 0.5 * lgmath::constants::PI;
  trueVecs.push_back(temp);
  const unsigned numRand = 20;
  for (unsigned i = 0; i < numRand; i++) {
    trueVecs.push_back(Eigen::Vector3d::Random());
  }

  // Get number of tests
  const unsigned numTests = trueVecs.size();

  // Calc rotation matrices
  std::vector<Eigen::Matrix3d> rotMatrices;
  for (unsigned i = 0; i < numTests; i++) {
    rotMatrices.push_back(lgmath::so3::vec2rot(trueVecs.at(i)));
  }

  // Calc rotations
  std::vector<lgmath::so3::Rotation> rotations;
  for (unsigned i = 0; i < numTests; i++) {
    rotations.push_back(lgmath::so3::Rotation(trueVecs.at(i)));
  }

  // Compare matrices
  {
    for (unsigned i = 0; i < numTests; i++) {
      std::cout << "matr: " << rotMatrices.at(i) << std::endl;
      std::cout << "tran: " << rotations.at(i).matrix() << std::endl;
      EXPECT_TRUE(lgmath::common::nearEqual(rotMatrices.at(i),
                                            rotations.at(i).matrix(), 1e-6));
    }
  }

  // Test logarithmic map
  {
    for (unsigned i = 0; i < numTests; i++) {
      Eigen::Vector3d testVec = rotations.at(i).vec();
      std::cout << "true: " << trueVecs.at(i) << std::endl;
      std::cout << "func: " << testVec << std::endl;
      EXPECT_TRUE(
          lgmath::common::nearEqualAxisAngle(trueVecs.at(i), testVec, 1e-6));
    }
  }

}  // TEST

/////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Test inverse and operatations
/////////////////////////////////////////////////////////////////////////////////////////////
TEST(LGMath, RotationInverse) {
  // Add vectors to be tested
  std::vector<Eigen::Vector3d> trueVecs;
  Eigen::Vector3d temp;
  temp << 0.0, 0.0, 0.0;
  trueVecs.push_back(temp);
  temp << lgmath::constants::PI, 0.0, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, lgmath::constants::PI, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, lgmath::constants::PI;
  trueVecs.push_back(temp);
  temp << -lgmath::constants::PI, 0.0, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, -lgmath::constants::PI, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, -lgmath::constants::PI;
  trueVecs.push_back(temp);
  temp << 0.5 * lgmath::constants::PI, 0.0, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.5 * lgmath::constants::PI, 0.0;
  trueVecs.push_back(temp);
  temp << 0.0, 0.0, 0.5 * lgmath::constants::PI;
  trueVecs.push_back(temp);
  const unsigned numRand = 20;
  for (unsigned i = 0; i < numRand; i++) {
    trueVecs.push_back(Eigen::Vector3d::Random());
  }

  // Get number of tests
  const unsigned numTests = trueVecs.size();

  // Add vectors to be tested - random
  std::vector<Eigen::Vector3d> landmarks;
  for (unsigned i = 0; i < numTests; i++) {
    landmarks.push_back(Eigen::Vector3d::Random());
  }

  // Calc rotation matrices
  std::vector<Eigen::Matrix3d> rotMatrices;
  for (unsigned i = 0; i < numTests; i++) {
    rotMatrices.push_back(lgmath::so3::vec2rot(trueVecs.at(i)));
  }

  // Calc rotations
  std::vector<lgmath::so3::Rotation> rotations;
  for (unsigned i = 0; i < numTests; i++) {
    rotations.push_back(lgmath::so3::Rotation(trueVecs.at(i)));
  }

  // Compare inverse to basic matrix inverse
  {
    for (unsigned i = 0; i < numTests; i++) {
      std::cout << "matr: " << rotMatrices.at(i).inverse() << std::endl;
      std::cout << "tran: " << rotations.at(i).inverse().matrix() << std::endl;
      EXPECT_TRUE(lgmath::common::nearEqual(rotMatrices.at(i).inverse(),
                                            rotations.at(i).inverse().matrix(),
                                            1e-6));
    }
  }

  // Test that product of inverse and self make identity
  {
    for (unsigned i = 0; i < numTests; i++) {
      std::cout << "C*Cinv: "
                << rotations.at(i).matrix() * rotations.at(i).inverse().matrix()
                << std::endl;
      EXPECT_TRUE(lgmath::common::nearEqual(
          rotations.at(i).matrix() * rotations.at(i).inverse().matrix(),
          Eigen::Matrix3d::Identity(), 1e-6));
    }
  }

  // Test self-product
  {
    for (unsigned i = 0; i < numTests - 1; i++) {
      lgmath::so3::Rotation test = rotations.at(i);
      test *= rotations.at(i + 1);
      Eigen::Matrix3d matrix = rotMatrices.at(i) * rotMatrices.at(i + 1);
      std::cout << "matr: " << matrix << std::endl;
      std::cout << "tran: " << test.matrix() << std::endl;
      EXPECT_TRUE(lgmath::common::nearEqual(matrix, test.matrix(), 1e-6));
    }
  }

  // Test product
  {
    for (unsigned i = 0; i < numTests - 1; i++) {
      lgmath::so3::Rotation test = rotations.at(i) * rotations.at(i + 1);
      Eigen::Matrix3d matrix = rotMatrices.at(i) * rotMatrices.at(i + 1);
      std::cout << "matr: " << matrix << std::endl;
      std::cout << "tran: " << test.matrix() << std::endl;
      EXPECT_TRUE(lgmath::common::nearEqual(matrix, test.matrix(), 1e-6));
    }
  }

  // Test self product with inverse
  {
    for (unsigned i = 0; i < numTests - 1; i++) {
      lgmath::so3::Rotation test = rotations.at(i);
      test /= rotations.at(i + 1);
      Eigen::Matrix3d matrix =
          rotMatrices.at(i) * rotMatrices.at(i + 1).inverse();
      std::cout << "matr: " << matrix << std::endl;
      std::cout << "tran: " << test.matrix() << std::endl;
      EXPECT_TRUE(lgmath::common::nearEqual(matrix, test.matrix(), 1e-6));
    }
  }

  // Test product with inverse
  {
    for (unsigned i = 0; i < numTests - 1; i++) {
      lgmath::so3::Rotation test = rotations.at(i) / rotations.at(i + 1);
      Eigen::Matrix3d matrix =
          rotMatrices.at(i) * rotMatrices.at(i + 1).inverse();
      std::cout << "matr: " << matrix << std::endl;
      std::cout << "tran: " << test.matrix() << std::endl;
      EXPECT_TRUE(lgmath::common::nearEqual(matrix, test.matrix(), 1e-6));
    }
  }

  // Test product with landmark
  {
    for (unsigned i = 0; i < numTests; i++) {
      Eigen::Vector3d mat = rotMatrices.at(i) * landmarks.at(i);
      Eigen::Vector3d test = rotations.at(i) * landmarks.at(i);

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
