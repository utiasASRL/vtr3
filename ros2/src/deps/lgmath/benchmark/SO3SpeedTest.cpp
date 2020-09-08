#include <gtest/gtest.h>

#include <lgmath/CommonTools.hpp>
#include <lgmath/so3/Operations.hpp>

TEST(LGMath, SO3Benchmark) {
  // Init variables
  double margin = 1.1;  // 10% increase
  unsigned int N = 1000000;
  lgmath::common::Timer timer;
  double time1;
  double recorded;

  // Allocate test memory
  Eigen::Matrix<double, 3, 3> m33;
  Eigen::Matrix<double, 3, 1> v3 = Eigen::Matrix<double, 3, 1>::Random();

  /////////////////////////////////////////////////////////////////////////////////////////////
  /// SO Testing
  /////////////////////////////////////////////////////////////////////////////////////////////
  std::cout << "Starting SO(3) Tests" << std::endl;
  std::cout << "--------------------" << std::endl;
  std::cout << "Comparison timings are to get a ballpark estimate."
            << std::endl;
  std::cout << "Check that it is not an order of magnitude off; you may not be "
               "in release mode."
            << std::endl;
  std::cout << " " << std::endl;

  // test
  std::cout << "Test SO3 hat, over " << N << " iterations." << std::endl;
  timer.reset();
  for (unsigned int i = 0; i < N; i++) {
    m33 = lgmath::so3::hat(v3);
  }
  time1 = timer.milliseconds();
  recorded = 0.00637;
  std::cout << "your speed: " << 1000.0 * time1 / double(N) << "usec per call."
            << std::endl;
  std::cout << "recorded:   " << recorded
            << "usec per call, 2.4 GHz processor, March 2015" << std::endl;
  std::cout << " " << std::endl;
  EXPECT_LT((1000.0 * time1 / double(N)), recorded * margin);

  // test
  std::cout << "Test SO3 vec2rot, over " << N << " iterations." << std::endl;
  timer.reset();
  for (unsigned int i = 0; i < N; i++) {
    m33 = lgmath::so3::vec2rot(v3);
  }
  time1 = timer.milliseconds();
  recorded = 0.077;
  std::cout << "your speed: " << 1000.0 * time1 / double(N) << "usec per call."
            << std::endl;
  std::cout << "recorded:   " << recorded
            << "usec per call, 2.4 GHz processor, March 2015" << std::endl;
  std::cout << " " << std::endl;
  EXPECT_LT((1000.0 * time1 / double(N)), recorded * margin);

  // test
  std::cout << "Test SO3 rot2vec, over " << N << " iterations." << std::endl;
  timer.reset();
  for (unsigned int i = 0; i < N; i++) {
    v3 = lgmath::so3::rot2vec(m33);
  }
  time1 = timer.milliseconds();
  recorded = 0.053;
  std::cout << "your speed: " << 1000.0 * time1 / double(N) << "usec per call."
            << std::endl;
  std::cout << "recorded:   " << recorded
            << "usec per call, 2.4 GHz processor, March 2015" << std::endl;
  std::cout << " " << std::endl;
  EXPECT_LT((1000.0 * time1 / double(N)), recorded * margin);

  // test
  std::cout << "Test SO3 vec2jac, over " << N << " iterations." << std::endl;
  timer.reset();
  for (unsigned int i = 0; i < N; i++) {
    m33 = lgmath::so3::vec2jac(v3);
  }
  time1 = timer.milliseconds();
  recorded = 0.0907;
  std::cout << "your speed: " << 1000.0 * time1 / double(N) << "usec per call."
            << std::endl;
  std::cout << "recorded:   " << recorded
            << "usec per call, 2.4 GHz processor, March 2015" << std::endl;
  std::cout << " " << std::endl;
  EXPECT_LT((1000.0 * time1 / double(N)), recorded * margin);

  // test
  std::cout << "Test SO3 vec2jacinv, over " << N << " iterations." << std::endl;
  timer.reset();
  for (unsigned int i = 0; i < N; i++) {
    m33 = lgmath::so3::vec2jacinv(v3);
  }
  time1 = timer.milliseconds();
  recorded = 0.0623;
  std::cout << "your speed: " << 1000.0 * time1 / double(N) << "usec per call."
            << std::endl;
  std::cout << "recorded:   " << recorded
            << "usec per call, 2.4 GHz processor, March 2015" << std::endl;
  std::cout << " " << std::endl;
  EXPECT_LT((1000.0 * time1 / double(N)), recorded * margin);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
