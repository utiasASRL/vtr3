// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file orb_configuration.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
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
}  // namespace vtr
