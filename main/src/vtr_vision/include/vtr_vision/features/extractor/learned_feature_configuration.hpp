#pragma once

#include <vtr_vision/features/matcher/asrl_feature_matcher.hpp>

namespace vtr {
namespace vision {

struct LearnedFeatureConfiguration {
  std::string model_path;
};

struct LearnedFeatureStereoConfiguration {
  /// The minimum disparity value to consider.
  float stereoDisparityMinimum;
  /// The maximum disparity value to consider.
  float stereoDisparityMaximum;
  /// Configurations for OpenCV SGBM disparity algorithm.
  // int minDisparity;
  // int numDisparities;
  // int blockSize;
  // int preFilterCap;
  // int uniquenessRatio;
  // int P1;
  // int P2;
  // int speckleWindowSize;
  // int speckleRange;
  // int disp12MaxDiff;
  // bool fullDP = false; // whether to use full-scale two-pass dyn. prog. algo.
};

}  // namespace vision
}  // namespace vtr_vision
