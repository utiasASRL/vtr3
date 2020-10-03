#pragma once

#include <vtr_vision/features/matcher/asrl_feature_matcher.hpp>

namespace vtr {
namespace vision {
// ORB Detector params
struct ORBConfiguration {
  bool use_STAR_detector_;
  bool use_GPU_descriptors_;
  int STAR_maxSize_;
  int STAR_responseThreshold_;
  int STAR_lineThresholdProjected_;
  int STAR_lineThresholdBinarized_;
  int STAR_suppressNonmaxSize_;
  int num_detector_features_;
  int num_binned_features_;
  double scaleFactor_;
  int nlevels_;
  int edgeThreshold_;
  int firstLevel_;
  int WTA_K_;
  int scoreType_;
  int patchSize_;
  int fastThreshold_;
  int x_bins_;
  int y_bins_;
  bool upright_;
  bool useHistogramEqualization_;
  double simpleCovarianceScale_;
  int num_threads_;
  // Stereo Matcher Configuration
  ASRLFeatureMatcher::Config stereo_matcher_config_;
};

}  // namespace vision
}  // namespace vtr_vision
