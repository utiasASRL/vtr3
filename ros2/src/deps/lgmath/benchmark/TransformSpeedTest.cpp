#include <gtest/gtest.h>

#include <lgmath/CommonTools.hpp>
#include <lgmath/se3/Operations.hpp>
#include <lgmath/se3/Transformation.hpp>

TEST(LGMath, TransformBenchmark) {
  // Init variables
  double margin = 1.1;
  unsigned int N = 1000000;
  unsigned int L = 1000;
  unsigned int M = 10000;
  lgmath::common::Timer timer;
  double time1, time2;
  double recorded;

  // Allocate test memory
  lgmath::se3::Transformation transform;
  Eigen::Matrix<double, 4, 1> v4 = Eigen::Matrix<double, 4, 1>::Random();
  Eigen::Matrix<double, 6, 1> v6 = Eigen::Matrix<double, 6, 1>::Random();

  /////////////////////////////////////////////////////////////////////////////////////////////
  /// Transformation Testing
  /////////////////////////////////////////////////////////////////////////////////////////////
  std::cout << "Starting Transformation Tests" << std::endl;
  std::cout << "-----------------------------" << std::endl;
  std::cout << "Comparison timings are to get a ballpark estimate."
            << std::endl;
  std::cout << "Check that it is not an order of magnitude off; you may not be "
               "in release mode."
            << std::endl;
  std::cout << " " << std::endl;

  // test
  std::cout << "Test transform vec2tran, over " << N << " iterations."
            << std::endl;
  timer.reset();
  for (unsigned int i = 0; i < N; i++) {
    transform = lgmath::se3::Transformation(v6);
  }
  time1 = timer.nanoseconds();
  recorded = 0.122;
  std::cout << "your speed: " << time1 / double(N) << "nsec per call."
            << std::endl;
  std::cout << "recorded:   " << 1000.0 * recorded
            << "nsec per call, 2.4 GHz processor, March 2015" << std::endl;
  std::cout << " " << std::endl;
  EXPECT_LT((time1 / double(N)), 1000.0 * recorded * margin);

  // test
  std::cout << "Test transform tran2vec, over " << N << " iterations."
            << std::endl;
  timer.reset();
  for (unsigned int i = 0; i < N; i++) {
    v6 = transform.vec();
  }
  time1 = timer.nanoseconds();
  recorded = 0.132;
  std::cout << "your speed: " << time1 / double(N) << "nsec per call."
            << std::endl;
  std::cout << "recorded:   " << 1000.0 * recorded
            << "nsec per call, 2.4 GHz processor, March 2015" << std::endl;
  std::cout << " " << std::endl;
  EXPECT_LT((time1 / double(N)), 1000.0 * recorded * margin);

  // test
  std::cout << "Test lval vs rval assignment, over " << N << " iterations."
            << std::endl;
  lgmath::se3::Transformation tmp(transform);
  lgmath::se3::Transformation tmp2(transform);
  double build_time;
  build_time = time1 = time2 = 0;

  for (unsigned int j = 0; j < L; ++j) {
    timer.reset();
    for (unsigned int i = 0; i < M; i++) {
      tmp = lgmath::se3::Transformation(transform);
    }
    build_time += timer.nanoseconds();

    timer.reset();
    for (unsigned int i = 0; i < M; i++) {
      tmp = lgmath::se3::Transformation(transform);
      tmp2 = tmp;
    }
    time1 += timer.nanoseconds();

    timer.reset();
    for (unsigned int i = 0; i < M; i++) {
      tmp = lgmath::se3::Transformation(transform);
      tmp2 = std::move(tmp);
    }
    time2 += timer.nanoseconds();
  }

  std::cout << "Lval assignment time: " << (time1 - build_time) / double(M * L)
            << "nsec per call." << std::endl;
  std::cout << "Rval assignment time: " << (time2 - build_time) / double(M * L)
            << "nsec per call." << std::endl;
  std::cout << "Difference: " << (time1 - time2) / (time1 - build_time) * 100
            << "%" << std::endl;
  std::cout << " " << std::endl;

  // test
  std::cout << "Test std::swap, over " << N << " iterations." << std::endl;

  timer.reset();
  for (unsigned int i = 0; i < N; i++) {
    using std::swap;
    swap(tmp, tmp2);
  }
  time1 = timer.nanoseconds();

  std::cout << "std::swap time: " << (time1) / double(N) << "nsec per call."
            << std::endl;
  std::cout << " " << std::endl;

  // test
  std::cout << "Test transform vec2tran, over " << N << " iterations."
            << std::endl;
  timer.reset();
  for (unsigned int i = 0; i < N; i++) {
    transform = lgmath::se3::Transformation(v6);
  }
  time1 = timer.nanoseconds();
  recorded = 0.122;
  std::cout << "your speed: " << time1 / double(N) << "nsec per call."
            << std::endl;
  std::cout << "recorded:   " << 1000.0 * recorded
            << "nsec per call, 2.4 GHz processor, March 2015" << std::endl;
  std::cout << " " << std::endl;
  EXPECT_LT((time1 / double(N)), 1000.0 * recorded * margin);

  // test
  std::cout << "Test product over " << N << " iterations." << std::endl;
  timer.reset();
  for (unsigned int i = 0; i < N; i++) {
    transform = transform * transform;
  }
  time1 = timer.nanoseconds();
  recorded = 0.033;
  std::cout << "your speed: " << time1 / double(N) << "nsec per call."
            << std::endl;
  std::cout << "recorded:   " << 1000.0 * recorded
            << "nsec per call, 2.4 GHz processor, March 2015" << std::endl;
  std::cout << " " << std::endl;
  EXPECT_LT((time1 / double(N)), 1000.0 * recorded * margin);

  // test
  std::cout << "Test product with inverse over " << N << " iterations."
            << std::endl;
  timer.reset();
  for (unsigned int i = 0; i < N; i++) {
    transform = transform / transform;
  }
  time1 = timer.nanoseconds();
  recorded = 0.035;
  std::cout << "your speed: " << time1 / double(N) << "nsec per call."
            << std::endl;
  std::cout << "recorded:   " << 1000.0 * recorded
            << "nsec per call, 2.4 GHz processor, March 2015" << std::endl;
  std::cout << " " << std::endl;
  EXPECT_LT((time1 / double(N)), 1000.0 * recorded * margin);

  // test
  std::cout << "Test product with landmark over " << N << " iterations."
            << std::endl;
  timer.reset();
  for (unsigned int i = 0; i < N; i++) {
    v4 = transform * v4;
  }
  time1 = timer.nanoseconds();
  recorded = 0.015;
  std::cout << "your speed: " << time1 / double(N) << "nsec per call."
            << std::endl;
  std::cout << "recorded:   " << 1000.0 * recorded
            << "nsec per call, 2.4 GHz processor, March 2015" << std::endl;
  std::cout << " " << std::endl;
  EXPECT_LT((time1 / double(N)), 1000.0 * recorded * margin);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
