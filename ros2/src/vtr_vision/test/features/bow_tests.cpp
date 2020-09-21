#include <gtest/gtest.h>

#include <vtr_vision/features/bow/incremental_bow_trainer.hpp>
#include <vtr_vision/features/bow/sparse_bow_descriptor.hpp>
#include <vtr_vision/features/bow/sliding_window_bow_descriptor.hpp>
//#include <vtr_vision/types.hpp>
#include <vtr_common/timing/simple_timer.hpp>

#include <array>
//#include <flann/flann.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <vtr_logging/logging_init.hpp>

using namespace vtr::vision;

TEST(Vision, bow_features) {

  cv::FlannBasedMatcher matcher;
  static const int r_sz = 1000*5, q_sz = 400, dims = 64, nn = 2;
  cv::Mat_<float> desc_r(r_sz, dims), desc_q(q_sz, dims), desc_q2(100, dims);

  std::cout << "rand" << std::endl;

  cv::randu(desc_r, cv::Scalar(0), cv::Scalar(1));
  cv::randu(desc_q, cv::Scalar(0), cv::Scalar(1));
  cv::randu(desc_q2, cv::Scalar(0), cv::Scalar(1));

  std::cout << "add" << std::endl;

  matcher.add(std::vector<cv::Mat>(1,desc_r));
  //index.addPoints(desc_r_fl);

  std::cout << "train" << std::endl;

  vtr::common::timing::SimpleTimer timer;
  matcher.train();
  std::cout << "cv: " << timer.elapsedMs() << " ms" << std::endl;

  std::cout << "match" << std::endl;

  std::vector<std::vector<cv::DMatch>> matches;
  std::vector<std::vector<cv::DMatch>> matches_bf;
  timer.reset();
  matcher.knnMatch(desc_q, matches, 2);
  std::cout << "cv: " << timer.elapsedMs() << " ms" << std::endl;

  std::cout << "display" << std::endl;

  for (const auto & a : matches) {
    for (const auto & b : a) {
      const cv::DMatch & c = b;
      std::cout << c.distance << " (" << c.trainIdx << ") ";
    }
    std::cout << std::endl;
  }

  std::cout << "cluster " << std::endl;
  timer.reset();
  IncrementalBOWTrainer trainer(3.);
  trainer.add(desc_r);
  cv::Mat centres = trainer.cluster();
  std::cout << timer.elapsedMs() << " ms" << std::endl;

  std::cout << "train bow matcher" << std::endl;
  timer.reset();
  cv::FlannBasedMatcher bow_matcher;
  bow_matcher.add(std::vector<cv::Mat>(1,centres));
  bow_matcher.train();
  std::cout << timer.elapsedMs() << " ms" << std::endl;

  std::cout << "bow" << std::endl;
  timer.reset();

  typedef SparseBOWDescriptor<unsigned> SBD;
  typedef typename SBD::SparseBOW SB;
  SBD bow_describer(&bow_matcher, 3.);
  SB bow = bow_describer.compute(desc_q);
  std::cout << timer.elapsedMs() << " ms" << std::endl;

  SB bow2 = bow_describer.compute(desc_q2);

  timer.reset();
  double dist = SBD::distance(bow, bow2);
  std::cout << "distance:" << dist << std::endl;
  std::cout << timer.elapsedMs() << " ms" << std::endl;

  std::cout << "Sliding window" << std::endl;
  timer.reset();
  typedef SlidingWindowBOWDescriptor<unsigned> SWBD;
  SWBD sliding(2);
  sliding.push(bow);
  sliding.push(bow);
  dist = SBD::distance(bow, sliding.bow());
  std::cout << "distance: " << dist << std::endl;
  sliding.push(bow2);
  dist = SBD::distance(bow, sliding.bow());
  std::cout << "distance: " << dist << std::endl;
  std::cout << timer.elapsedMs() << " ms" << std::endl;

  EXPECT_EQ(true, true);
}
