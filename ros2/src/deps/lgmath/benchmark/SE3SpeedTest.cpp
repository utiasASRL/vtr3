#include <gtest/gtest.h>

#include <lgmath/CommonTools.hpp>
#include <lgmath/se3/Operations.hpp>

TEST(LGMath, SE3Benchmark) {
  // Init variables
  double margin = 1.1;  // 10% increase
  unsigned int N = 1000000;
  lgmath::common::Timer timer;
  double time1;
  double recorded;

  // Allocate test memory
  Eigen::Matrix<double, 3, 3> m33;
  Eigen::Matrix<double, 4, 4> m44;
  Eigen::Matrix<double, 6, 4> m64;
  Eigen::Matrix<double, 4, 6> m46;
  Eigen::Matrix<double, 6, 6> m66;
  Eigen::Matrix<double, 4, 1> v4 = Eigen::Matrix<double, 4, 1>::Random();
  Eigen::Matrix<double, 6, 1> v6 = Eigen::Matrix<double, 6, 1>::Random();

  /////////////////////////////////////////////////////////////////////////////////////////////
  /// SE Testing
  /////////////////////////////////////////////////////////////////////////////////////////////
  std::cout << "Starting SE(3) Tests" << std::endl;
  std::cout << "--------------------" << std::endl;
  std::cout << "Comparison timings are to get a ballpark estimate."
            << std::endl;
  std::cout << "Check that it is not an order of magnitude off; you may not be "
               "in release mode."
            << std::endl;
  std::cout << " " << std::endl;

  // test
  std::cout << "Test SE3 hat, over " << N << " iterations." << std::endl;
  timer.reset();
  for (unsigned int i = 0; i < N; i++) {
    m44 = lgmath::se3::hat(v6);
  }
  time1 = timer.milliseconds();
  recorded = 0.0189;
  std::cout << "your speed: " << 1000.0 * time1 / double(N) << "usec per call."
            << std::endl;
  std::cout << "recorded:   " << recorded
            << "usec per call, 2.4 GHz processor, March 2015" << std::endl;
  std::cout << " " << std::endl;
  EXPECT_LT((1000.0 * time1 / double(N)), recorded * margin);

  // test
  std::cout << "Test SE3 curlyhat, over " << N << " iterations." << std::endl;
  timer.reset();
  for (unsigned int i = 0; i < N; i++) {
    m66 = lgmath::se3::curlyhat(v6);
  }
  time1 = timer.milliseconds();
  recorded = 0.0233;
  std::cout << "your speed: " << 1000.0 * time1 / double(N) << "usec per call."
            << std::endl;
  std::cout << "recorded:   " << recorded
            << "usec per call, 2.4 GHz processor, March 2015" << std::endl;
  std::cout << " " << std::endl;
  EXPECT_LT((1000.0 * time1 / double(N)), recorded * margin);

  // test
  std::cout << "Test SE3 point2fs, over " << N << " iterations." << std::endl;
  timer.reset();
  for (unsigned int i = 0; i < N; i++) {
    m46 = lgmath::se3::point2fs(v4.head<3>());
  }
  time1 = timer.milliseconds();
  recorded = 0.0131;
  std::cout << "your speed: " << 1000.0 * time1 / double(N) << "usec per call."
            << std::endl;
  std::cout << "recorded:   " << recorded
            << "usec per call, 2.4 GHz processor, March 2015" << std::endl;
  std::cout << " " << std::endl;
  EXPECT_LT((1000.0 * time1 / double(N)), recorded * margin);

  // test
  std::cout << "Test SE3 point2sf, over " << N << " iterations." << std::endl;
  timer.reset();
  for (unsigned int i = 0; i < N; i++) {
    m64 = lgmath::se3::point2sf(v4.head<3>());
  }
  time1 = timer.milliseconds();
  recorded = 0.0125;
  std::cout << "your speed: " << 1000.0 * time1 / double(N) << "usec per call."
            << std::endl;
  std::cout << "recorded:   " << recorded
            << "usec per call, 2.4 GHz processor, March 2015" << std::endl;
  std::cout << " " << std::endl;
  EXPECT_LT((1000.0 * time1 / double(N)), recorded * margin);

  // test
  std::cout << "Test SE3 vec2tran, over " << N << " iterations." << std::endl;
  timer.reset();
  for (unsigned int i = 0; i < N; i++) {
    m44 = lgmath::se3::vec2tran(v6);
  }
  time1 = timer.milliseconds();
  recorded = 0.112;
  std::cout << "your speed: " << 1000.0 * time1 / double(N) << "usec per call."
            << std::endl;
  std::cout << "recorded:   " << recorded
            << "usec per call, 2.4 GHz processor, March 2015" << std::endl;
  std::cout << " " << std::endl;
  EXPECT_LT((1000.0 * time1 / double(N)), recorded * margin);

  // test
  std::cout << "Test SE3 tran2vec, over " << N << " iterations." << std::endl;
  timer.reset();
  for (unsigned int i = 0; i < N; i++) {
    v6 = lgmath::se3::tran2vec(m44);
  }
  time1 = timer.milliseconds();
  recorded = 0.129;
  std::cout << "your speed: " << 1000.0 * time1 / double(N) << "usec per call."
            << std::endl;
  std::cout << "recorded:   " << recorded
            << "usec per call, 2.4 GHz processor, March 2015" << std::endl;
  std::cout << " " << std::endl;
  EXPECT_LT((1000.0 * time1 / double(N)), recorded * margin);

  // test
  std::cout << "Test SE3 tranAd, over " << N << " iterations." << std::endl;
  timer.reset();
  for (unsigned int i = 0; i < N; i++) {
    m66 = lgmath::se3::tranAd(m44);
  }
  time1 = timer.milliseconds();
  recorded = 0.027;
  std::cout << "your speed: " << 1000.0 * time1 / double(N) << "usec per call."
            << std::endl;
  std::cout << "recorded:   " << recorded
            << "usec per call, 2.4 GHz processor, March 2015" << std::endl;
  std::cout << " " << std::endl;
  EXPECT_LT((1000.0 * time1 / double(N)), recorded * margin);

  // test
  std::cout << "Test SE3 vec2Q, over " << N << " iterations." << std::endl;
  timer.reset();
  for (unsigned int i = 0; i < N; i++) {
    m33 = lgmath::se3::vec2Q(v6);
  }
  time1 = timer.milliseconds();
  recorded = 0.187;
  std::cout << "your speed: " << 1000.0 * time1 / double(N) << "usec per call."
            << std::endl;
  std::cout << "recorded:   " << recorded
            << "usec per call, 2.4 GHz processor, March 2015" << std::endl;
  std::cout << " " << std::endl;
  EXPECT_LT((1000.0 * time1 / double(N)), recorded * margin);

  // test
  std::cout << "Test SE3 vec2jac, over " << N << " iterations." << std::endl;
  timer.reset();
  for (unsigned int i = 0; i < N; i++) {
    m66 = lgmath::se3::vec2jac(v6);
  }
  time1 = timer.milliseconds();
  recorded = 0.291;
  std::cout << "your speed: " << 1000.0 * time1 / double(N) << "usec per call."
            << std::endl;
  std::cout << "recorded:   " << recorded
            << "usec per call, 2.4 GHz processor, March 2015" << std::endl;
  std::cout << " " << std::endl;
  EXPECT_LT((1000.0 * time1 / double(N)), recorded * margin);

  // test
  std::cout << "Test SE3 vec2jacinv, over " << N << " iterations." << std::endl;
  timer.reset();
  for (unsigned int i = 0; i < N; i++) {
    m66 = lgmath::se3::vec2jacinv(v6);
  }
  time1 = timer.milliseconds();
  recorded = 0.277;
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
