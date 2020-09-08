//////////////////////////////////////////////////////////////////////////////////////////////
/// \file NaiveSO3Tests.cpp
/// \brief Unit tests for the naive implementation of the SO3 Lie Group math.
/// \details Unit tests for the various Lie Group functions will test both special cases,
///          and randomly generated cases.
///
/// \author Sean Anderson
//////////////////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>

#include <math.h>
#include <iostream>
#include <iomanip>
#include <ios>

#include <Eigen/Dense>
#include <lgmath/CommonMath.hpp>
#include <lgmath/so3/Operations.hpp>

/////////////////////////////////////////////////////////////////////////////////////////////
///
/// UNIT TESTS OF SO(3) MATH
///
/////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
/// \brief General test of SO(3) hat function
/////////////////////////////////////////////////////////////////////////////////////////////
TEST(LGMath, test3x3HatFunction) {

  // Init
  std::vector<Eigen::Matrix<double,3,1> > trueVecs;
  std::vector<Eigen::Matrix<double,3,3> > trueMats;

  // Add vectors to be tested - we choose a few
  trueVecs.push_back(Eigen::Matrix<double,3,1>( 0.0,  0.0,  0.0));
  trueVecs.push_back(Eigen::Matrix<double,3,1>( 1.0,  2.0,  3.0));
  trueVecs.push_back(Eigen::Matrix<double,3,1>(-1.0, -2.0, -3.0));
  trueVecs.push_back(Eigen::Matrix<double,3,1>::Random());

  // Get number of tests
  const unsigned numTests = trueVecs.size();

  // Setup truth matrices
  for (unsigned i = 0; i < numTests; i++) {
    Eigen::Matrix<double,3,3> mat;
    mat <<               0.0,  -trueVecs.at(i)[2],   trueVecs.at(i)[1],
           trueVecs.at(i)[2],                 0.0,  -trueVecs.at(i)[0],
          -trueVecs.at(i)[1],   trueVecs.at(i)[0],                 0.0;
    trueMats.push_back(mat);
  }

  // Test the function
  for (unsigned i = 0; i < numTests; i++) {
    Eigen::Matrix<double,3,3> testMat = lgmath::so3::hat(trueVecs.at(i));
    std::cout << "true: " << trueMats.at(i) << std::endl;
    std::cout << "func: " << testMat << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(trueMats.at(i), testMat, 1e-6));
  }

  // Test identity,  hat(v)^T = -hat(v)
  for (unsigned i = 0; i < numTests; i++) {
    Eigen::Matrix<double,3,3> testMat = lgmath::so3::hat(trueVecs.at(i));
    EXPECT_TRUE(lgmath::common::nearEqual(testMat.transpose(), -testMat, 1e-6));
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Test special cases of exponential functions: vec2rot and rot2vec
/////////////////////////////////////////////////////////////////////////////////////////////
TEST(LGMath, testVec2RotSpecialCase) {

  // Init
  std::vector<Eigen::Matrix<double,3,1> > trueVecs;
  std::vector<Eigen::Matrix<double,3,3> > trueMats;
  Eigen::Matrix<double,3,3> temp;

  // Identity
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.0, 0.0, 0.0));
  trueMats.push_back(Eigen::Matrix<double,3,3>::Identity());

  // x-rot by PI
  trueVecs.push_back(Eigen::Matrix<double,3,1>(lgmath::constants::PI, 0.0, 0.0));
  temp <<  1.0,  0.0,  0.0,
           0.0, -1.0,  0.0,
           0.0,  0.0, -1.0;
  trueMats.push_back(temp);

  // y-rot by PI
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.0, lgmath::constants::PI, 0.0));
  temp << -1.0,  0.0,  0.0,
           0.0,  1.0,  0.0,
           0.0,  0.0, -1.0;
  trueMats.push_back(temp);

  // z-rot by PI
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.0, 0.0, lgmath::constants::PI));
  temp << -1.0,  0.0,  0.0,
           0.0, -1.0,  0.0,
           0.0,  0.0,  1.0;
  trueMats.push_back(temp);

  // x-rot by -PI
  trueVecs.push_back(Eigen::Matrix<double,3,1>(-lgmath::constants::PI, 0.0, 0.0));
  temp <<  1.0,  0.0,  0.0,
           0.0, -1.0,  0.0,
           0.0,  0.0, -1.0;
  trueMats.push_back(temp);

  // y-rot by -PI
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.0, -lgmath::constants::PI, 0.0));
  temp << -1.0,  0.0,  0.0,
           0.0,  1.0,  0.0,
           0.0,  0.0, -1.0;
  trueMats.push_back(temp);

  // z-rot by -PI
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.0, 0.0, -lgmath::constants::PI));
  temp << -1.0,  0.0,  0.0,
           0.0, -1.0,  0.0,
           0.0,  0.0,  1.0;
  trueMats.push_back(temp);

  // x-rot by PI/2
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.5*lgmath::constants::PI, 0.0, 0.0));
  temp <<  1.0,  0.0,  0.0,
           0.0,  0.0, -1.0,
           0.0,  1.0,  0.0;
  trueMats.push_back(temp);

  // y-rot by PI/2
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.0, 0.5*lgmath::constants::PI, 0.0));
  temp <<  0.0,  0.0,  1.0,
           0.0,  1.0,  0.0,
          -1.0,  0.0,  0.0;
  trueMats.push_back(temp);

  // z-rot by PI/2
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.0, 0.0, 0.5*lgmath::constants::PI));
  temp <<  0.0, -1.0,  0.0,
           1.0,  0.0,  0.0,
           0.0,  0.0,  1.0;
  trueMats.push_back(temp);

  // Get number of tests
  const unsigned numTests = trueVecs.size();

  // Test vec2rot
  for (unsigned i = 0; i < numTests; i++) {
    Eigen::Matrix<double,3,3> testMat = lgmath::so3::vec2rot(trueVecs.at(i));
    std::cout << "true: " << trueMats.at(i) << std::endl;
    std::cout << "func: " << testMat << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(trueMats.at(i), testMat, 1e-6));
  }

  // Test rot2vec
  for (unsigned i = 0; i < numTests; i++) {
    Eigen::Matrix<double,3,1> testVec = lgmath::so3::rot2vec(trueMats.at(i));
    std::cout << "true: " << trueVecs.at(i) << std::endl;
    std::cout << "func: " << testVec << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqualAxisAngle(trueVecs.at(i), testVec, 1e-6));
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////
/// \brief General test of exponential functions: vec2rot and rot2vec
/////////////////////////////////////////////////////////////////////////////////////////////
TEST(LGMath, CompareAnalyticalAndNumericVec2Rot) {

  // Add vectors to be tested
  std::vector<Eigen::Matrix<double,3,1> > trueVecs;
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.0, 0.0, 0.0));
  trueVecs.push_back(Eigen::Matrix<double,3,1>(lgmath::constants::PI, 0.0, 0.0));
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.0, lgmath::constants::PI, 0.0));
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.0, 0.0, lgmath::constants::PI));
  trueVecs.push_back(Eigen::Matrix<double,3,1>(-lgmath::constants::PI, 0.0, 0.0));
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.0, -lgmath::constants::PI, 0.0));
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.0, 0.0, -lgmath::constants::PI));
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.5*lgmath::constants::PI, 0.0, 0.0));
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.0, 0.5*lgmath::constants::PI, 0.0));
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.0, 0.0, 0.5*lgmath::constants::PI));
  const unsigned numRand = 50;
  for (unsigned i = 0; i < numRand; i++) {
    trueVecs.push_back(Eigen::Matrix<double,3,1>::Random());
  }

  // Get number of tests
  const unsigned numTests = trueVecs.size();

  // Calc matrices
  std::vector<Eigen::Matrix<double,3,3> > analyticRots;
  for (unsigned i = 0; i < numTests; i++) {
    analyticRots.push_back(lgmath::so3::vec2rot(trueVecs.at(i)));
  }

  // Compare analytical and numeric result
  for (unsigned i = 0; i < numTests; i++) {
    Eigen::Matrix<double,3,3> numericRot = lgmath::so3::vec2rot(trueVecs.at(i), 20);
    std::cout << "ana: " << analyticRots.at(i) << std::endl;
    std::cout << "num: " << numericRot << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(analyticRots.at(i), numericRot, 1e-6));
  }

  // Test identity, rot^T = rot^-1
  for (unsigned i = 0; i < numTests; i++) {
    Eigen::Matrix<double,3,3> lhs = analyticRots.at(i).transpose();
    Eigen::Matrix<double,3,3> rhs = analyticRots.at(i).inverse();
    std::cout << "lhs: " << lhs << std::endl;
    std::cout << "rhs: " << rhs << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(lhs, rhs, 1e-6));
  }

  // Test rot2vec
  for (unsigned i = 0; i < numTests; i++) {
    Eigen::Matrix<double,3,1> testVec = lgmath::so3::rot2vec(analyticRots.at(i));
    std::cout << "true: " << trueVecs.at(i) << std::endl;
    std::cout << "func: " << testVec << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqualAxisAngle(trueVecs.at(i), testVec, 1e-6));
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////
/// \brief General test of exponential jacobians: vec2jac and vec2jacinv
/////////////////////////////////////////////////////////////////////////////////////////////
TEST(LGMath, CompareAnalyticalJacobiansInversesAndNumericCounterparts) {

  // Add vectors to be tested
  std::vector<Eigen::Matrix<double,3,1> > trueVecs;
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.0, 0.0, 0.0));
  trueVecs.push_back(Eigen::Matrix<double,3,1>(lgmath::constants::PI, 0.0, 0.0));
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.0, lgmath::constants::PI, 0.0));
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.0, 0.0, lgmath::constants::PI));
  trueVecs.push_back(Eigen::Matrix<double,3,1>(-lgmath::constants::PI, 0.0, 0.0));
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.0, -lgmath::constants::PI, 0.0));
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.0, 0.0, -lgmath::constants::PI));
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.5*lgmath::constants::PI, 0.0, 0.0));
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.0, 0.5*lgmath::constants::PI, 0.0));
  trueVecs.push_back(Eigen::Matrix<double,3,1>(0.0, 0.0, 0.5*lgmath::constants::PI));
  const unsigned numRand = 50;
  for (unsigned i = 0; i < numRand; i++) {
    trueVecs.push_back(Eigen::Matrix<double,3,1>::Random());
  }

  // Get number of tests
  const unsigned numTests = trueVecs.size();

  // Calc analytical matrices
  std::vector<Eigen::Matrix<double,3,3> > analyticJacs;
  std::vector<Eigen::Matrix<double,3,3> > analyticJacInvs;
  for (unsigned i = 0; i < numTests; i++) {
    analyticJacs.push_back(lgmath::so3::vec2jac(trueVecs.at(i)));
    analyticJacInvs.push_back(lgmath::so3::vec2jacinv(trueVecs.at(i)));
  }

  // Compare inversed analytical and analytical inverse
  for (unsigned i = 0; i < numTests; i++) {
    std::cout << "ana: " << analyticJacs.at(i) << std::endl;
    std::cout << "num: " << analyticJacInvs.at(i) << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(analyticJacs.at(i).inverse(), analyticJacInvs.at(i), 1e-6));
  }

  // Compare analytical and 'numerical' jacobian
  for (unsigned i = 0; i < numTests; i++) {
    Eigen::Matrix<double,3,3> numericJac = lgmath::so3::vec2jac(trueVecs.at(i), 20);
    std::cout << "ana: " << analyticJacs.at(i) << std::endl;
    std::cout << "num: " << numericJac << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(analyticJacs.at(i), numericJac, 1e-6));
  }

  // Compare analytical and 'numerical' jacobian inverses
  for (unsigned i = 0; i < numTests; i++) {
    Eigen::Matrix<double,3,3> numericJac = lgmath::so3::vec2jacinv(trueVecs.at(i), 20);
    std::cout << "ana: " << analyticJacInvs.at(i) << std::endl;
    std::cout << "num: " << numericJac << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(analyticJacInvs.at(i), numericJac, 1e-6));
  }

  // Test identity, rot(v) = eye(3) + hat(v)*jac(v), through the 'alternate' vec2rot method
  for (unsigned i = 0; i < numTests; i++) {
    Eigen::Matrix<double,3,3> lhs = lgmath::so3::vec2rot(trueVecs.at(i));
    Eigen::Matrix<double,3,3> rhs, jac;
    // the following vec2rot call uses the identity: rot(v) = eye(3) + hat(v)*jac(v)
    lgmath::so3::vec2rot(trueVecs.at(i), &rhs, &jac);
    std::cout << "lhs: " << lhs << std::endl;
    std::cout << "rhs: " << rhs << std::endl;
    EXPECT_TRUE(lgmath::common::nearEqual(lhs, rhs, 1e-6));
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

