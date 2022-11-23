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
 * \file asrl_feature_matcher.cpp
 * \brief Source file for the ASRL vision package
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#define _GLIBCXX_USE_NANOSLEEP 1

#include <omp.h>
#include <chrono>
#include <limits>

#include <vtr_logging/logging.hpp>
#include <vtr_vision/features/matcher/asrl_feature_matcher.hpp>

namespace vtr {
namespace vision {

Point transformKeypoint(Point ptin, const Eigen::Matrix3d &H_2_1) {
  // put the point in a homogeneous form
  Eigen::Vector3d point1;
  point1 << ptin.x, ptin.y, 1.0;

  // predict the location of point1 projected into frame2
  Eigen::Vector3d point2_predicted = H_2_1 * point1;

  // normalise
  Point pt;
  pt.x = point2_predicted.hnormalized()(0);
  pt.y = point2_predicted.hnormalized()(1);

  return pt;
}

/////////////////////////////////////////////////////////////////////////
// ASRLFeatureMatcher::ASRLFeatureMatcher
/////////////////////////////////////////////////////////////////////////
ASRLFeatureMatcher::ASRLFeatureMatcher(Config config) {
  config_ = config;
  // sanity check
  config_.num_threads_ = std::max(config_.num_threads_, 1);
}

/////////////////////////////////////////////////////////////////////////
// ASRLFeatureMatcher::~ASRLFeatureMatcher
/////////////////////////////////////////////////////////////////////////
ASRLFeatureMatcher::~ASRLFeatureMatcher() = default;

SimpleMatches ASRLFeatureMatcher::matchFeatures(const Features &frame1,
                                                const Features &frame2) {
  float window_size = std::numeric_limits<float>::max();
  return matchFeatures(frame1, frame2, window_size);
}

SimpleMatches ASRLFeatureMatcher::matchStereoFeatures(const Features &frame1,
                                                      const Features &frame2) {
  Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
  return matchFeatures(frame1, frame2, H, config_.stereo_x_tolerance_min_,
                       config_.stereo_x_tolerance_max_,
                       config_.stereo_y_tolerance_);
}

SimpleMatches ASRLFeatureMatcher::matchFeatures(const Features &frame1,
                                                const Features &frame2,
                                                const float &window_size) {
  Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
  return matchFeatures(frame1, frame2, H, -window_size, window_size,
                       window_size);
}

SimpleMatches ASRLFeatureMatcher::matchFeatures(
    const Features &frame1, const Features &frame2,
    const Eigen::Matrix3d &G,  // H_2_1 or F_2_1
    const float &x_window_min, const float &x_window_max,
    const float &y_window_size, const CheckType type) {
  SimpleMatches matches;

  // sanity check
  if (frame1.feat_type.impl != frame2.feat_type.impl) {
    LOG(ERROR)
        << "ASRLFeatureMatcher::matchfeatures(): feature types do not match!";
    return matches;
  }

  // parallelize
#pragma omp parallel for num_threads(config_.num_threads_)

  // loop through all the keypoints in frame1
  for (unsigned ii = 0; ii < frame1.keypoints.size(); ii++) {
    // copy the keypoint (because we'll overwrite the 2D point)
    Keypoint kp1 = frame1.keypoints[ii];

    // this stores all the required data for checking the epipolar line
    EpipoleHelper eh;

    // now check if we need to precache anything depending on the match type
    if (type == CheckType::HOMOGRAPHY) {
      // transform the keypoint through the homography matrix
      kp1.pt = transformKeypoint(kp1.pt, G);

    } else if (type == CheckType::EPIPOLE) {
      // make the point into a homogeneous Eigen type
      Eigen::Vector3d pt1h;
      pt1h << kp1.pt.x, kp1.pt.y, 1.0;

      // make the line from the input fundamental matrix
      eh.l = G * pt1h;

      // cache some common data
      eh.a2b2 = eh.l(0) * eh.l(0) + eh.l(1) * eh.l(1);
      eh.sqrta2b2 = std::sqrt(eh.a2b2);
    }

    // should we scale the x tolerance by the depth?
    // a bit of a hack, but proves useful for ORB stereo matching
    int x_window_max_mod = x_window_max;
    if (config_.scale_x_tolerance_by_y_) {
      x_window_max_mod *= (kp1.pt.y / config_.x_tolerance_scale_);
    }

    // reset the metrics
    float best_dist = std::numeric_limits<float>::max();
    unsigned match_idx = 0;

    // loop through all the keypoints in frame2
    for (unsigned jj = 0; jj < frame2.keypoints.size(); jj++) {
      // check all the descriptor conditions
      if (checkConditions(kp1, frame1.feat_infos[ii], frame2.keypoints[jj],
                          frame2.feat_infos[jj], eh, x_window_min,
                          x_window_max_mod, y_window_size, type)) {
        // passed!

        // initialise the distance to the worst case
        float dist = std::numeric_limits<float>::max();

        // now check the descriptor distance. 0.0 is perfect, 1.0 is imperfect
        if (frame1.feat_type.impl == FeatureImpl::OPENCV_ORB) {
          dist = briefmatch(&frame1.descriptors.at<unsigned char>(ii, 0),
                            &frame2.descriptors.at<unsigned char>(jj, 0),
                            frame1.feat_type.bytes_per_desc);
        } else if (frame1.feat_type.impl == FeatureImpl::ASRL_GPU_SURF) {
          dist = surfmatch(&frame1.descriptors.at<float>(ii, 0),
                           &frame2.descriptors.at<float>(jj, 0),
                           frame1.feat_type.bytes_per_desc / sizeof(float));
        } else {
          LOG(ERROR)
              << "ASRLFeatureMatcher::matchfeatures(): unknown feature type!";
        }
        if (dist < config_.descriptor_match_thresh_ && dist < best_dist) {
          best_dist = dist;
          match_idx = jj;
        }
      }
    }

    // did we find a good match?
    if (best_dist < std::numeric_limits<float>::max()) {
      SimpleMatch match;
      match.first = ii;
      match.second = match_idx;
#pragma omp critical(updatematch)
      matches.push_back(match);
    }
  }

  return matches;
}

bool ASRLFeatureMatcher::checkConditions(
    const Keypoint &kp1, const FeatureInfo &fi1, const Keypoint &kp2,
    const FeatureInfo &fi2, const EpipoleHelper &eh,
    const float &x_window_size_min, const float &x_window_size_max,
    const float &y_window_size, const CheckType &type) {
  // check that the octave of the two keypoints are roughly similar
  if (config_.check_laplacian_bit_ && fi1.laplacian_bit != fi2.laplacian_bit) {
    return false;
  }

  // check that the octave of the two keypoints are roughly similar
  if (config_.check_octave_ && kp1.octave != kp2.octave) {
    return false;
  }

  // check that the responses of the two keypoints are roughly similar
  if (config_.check_response_) {
    float highest_response = std::max(kp1.response, kp2.response);
    float lowest_response = std::min(kp1.response, kp2.response);
    if (lowest_response / highest_response < config_.min_response_ratio_) {
      return false;
    }
  }

  // if we're using the epipolar check we need to be a bit more involved
  if (type == CheckType::EPIPOLE) {
    return checkEpipole(kp1, fi1, kp2, fi2, eh, x_window_size_min,
                        x_window_size_max, y_window_size);
  } else {
    // since we don't have to or have already transformed the point, just do the
    // normal check
    float x_dist = kp1.pt.x - kp2.pt.x;
    float y_dist = kp1.pt.y - kp2.pt.y;
    // do the two points lie close to each other within conditions?
    if (x_dist < x_window_size_min || x_dist > x_window_size_max ||
        std::fabs(y_dist) > y_window_size) {
      return false;
    }
  }

  return true;
}

bool ASRLFeatureMatcher::checkEpipole(
    const Keypoint &kp1, const FeatureInfo &fi1, const Keypoint &kp2,
    const FeatureInfo &fi2, const EpipoleHelper &eh,
    const float &x_window_size_min, const float &x_window_size_max,
    const float &y_window_size) {
  (void)fi1;  /// \todo unused
  (void)fi2;  /// \todo unused

  // find the point on the line closest to kp1.pt;
  Point lp1;
  lp1.x = (eh.l(1) * (eh.l(1) * kp1.pt.x - eh.l(0) * kp1.pt.y) -
           eh.l(0) * eh.l(2)) /
          eh.a2b2;
  lp1.y = (eh.l(0) * (-eh.l(1) * kp1.pt.x + eh.l(0) * kp1.pt.y) -
           eh.l(1) * eh.l(2)) /
          eh.a2b2;

  // find the point on the line closest to kp2.pt;
  Point lp2;
  lp2.x = (eh.l(1) * (eh.l(1) * kp2.pt.x - eh.l(0) * kp2.pt.y) -
           eh.l(0) * eh.l(2)) /
          eh.a2b2;
  lp2.y = (eh.l(0) * (-eh.l(1) * kp2.pt.x + eh.l(0) * kp2.pt.y) -
           eh.l(1) * eh.l(2)) /
          eh.a2b2;

  // subtract lp2 from lp1 to put lp2 at the origin of the co-ordinate system we
  // are about to rotate
  lp1.x -= lp2.x;
  lp1.y -= lp2.y;

  // rotate point 1 so that the two points can be compared in x space
  float theta = std::atan2(eh.l(0), -eh.l(1));
  Point rp1;
  rp1.x = std::cos(theta) * lp1.x - std::sin(theta) * lp1.y;
  rp1.y = std::sin(theta) * lp1.x + std::cos(theta) * lp1.y;

  // find the distance from the closest line point to the point
  float x_dist = rp1.x;

  // get the distance from kp2.pt to the line
  float y_dist = std::fabs(eh.l(0) * kp2.pt.x + eh.l(1) * kp2.pt.y + eh.l(2)) /
                 eh.sqrta2b2;

  if (x_dist < x_window_size_min || x_dist > x_window_size_max ||
      y_dist > y_window_size) {
    return false;
  }

  // if we got here, everything passed
  return true;
}

float ASRLFeatureMatcher::briefmatch(const unsigned char *d1,
                                     const unsigned char *d2, unsigned size) {
  // size is the number of bytes used for the descriptor
  float score = 0.f;
  uchar hamming = 0;
  int dist = 0;
  for (uint i = 0; i < size; i++) {
    hamming = d1[i] ^ d2[i];  // XOR: 1 if bit is different
    for (uint bit = 1; bit <= 128; bit *= 2) {
      if (hamming & uchar(bit)) {
        dist++;
      }
    }
  }
  // make the score compatible with our standard by subtracting from 1.0. 1.0 is
  // worst match, 0.0 is perfect
  score = (size * 8.0 - dist) / (size * 8.0);
  score = 1.0 - score;
  // return the value
  return score;
}

float ASRLFeatureMatcher::surfmatch(const float *d1, const float *d2,
                                    unsigned size) {
  // todo: check if this flag is/should be defined
#if FAST_BUT_UNREADABLE
  // This code is run so often, that we need to optimize it:
  float score = 0.f;
  for (unsigned i = 0; i < size; ++i) score += d1[i] * d2[i];
#else
  // Practically, it does the same as this more readable version:
  typedef Eigen::Matrix<float, Eigen::Dynamic, 1> MatrixType;
  Eigen::Map<const MatrixType> m1(d1, (int)size);
  Eigen::Map<const MatrixType> m2(d2, (int)size);
  // size is the number of bytes used for the descriptor
  float score = m1.transpose() * m2;
#endif
  // return the value
  return 1.f - score;
}

float ASRLFeatureMatcher::learnedfeaturematch(const float *d1, const float *d2, unsigned size) {

  //todo: check if this flag is/should be defined
#if FAST_BUT_UNREADABLE
  // This code is run so often, that we need to optimize it:
  float score = 0.f;
  for (unsigned i = 0; i < size; ++i)
    score += d1[i]*d2[i];
  #else
  // Practically, it does the same as this more readable version:
  typedef Eigen::Matrix<float,Eigen::Dynamic,1> MatrixType;
  Eigen::Map<const MatrixType> m1(d1,(int)size);
  Eigen::Map<const MatrixType> m2(d2,(int)size);
  // size is the number of bytes used for the descriptor
  float score =  m1.transpose()*m2;
#endif
  // LOG(INFO) << score;
  score = score / size;
  // LOG(INFO) << score;
  //return the value
  return 1.f-score;
}

float ASRLFeatureMatcher::distance(const void *d1, const void *d2,
                                   const FeatureType &feat_type) {
  if (feat_type.impl == vtr::vision::FeatureImpl::OPENCV_ORB) {
    return vision::ASRLFeatureMatcher::briefmatch(
        (unsigned char *)d1, (unsigned char *)d2, feat_type.bytes_per_desc);
  } else if (feat_type.impl == vtr::vision::FeatureImpl::ASRL_GPU_SURF) {
    return vision::ASRLFeatureMatcher::surfmatch(
        (float *)d1, (float *)d2, feat_type.bytes_per_desc / sizeof(float));

  } else if (feat_type.impl == vtr::vision::FeatureImpl::LEARNED_FEATURE){
    return vision::ASRLFeatureMatcher::learnedfeaturematch(
        (float *)d1, (float *)d2, feat_type.bytes_per_desc / sizeof(float));

  } else {
    return -1.0;
  }
}
/////////////////////////////////////////////////////////////////////////
// ASRLFeatureMatcher::addMatchingTask
/////////////////////////////////////////////////////////////////////////
void ASRLFeatureMatcher::addMatchingTask(const Features &frame1,
                                         const Features &frame2) {
  // spawn an asynchronous descriptor extraction call and put the future frame
  // at the end of the queue.
  future_matches.emplace(std::async(std::launch::async,
                                    (SimpleMatches(ASRLFeatureMatcher::*)(
                                        const Features &, const Features &)) &
                                        ASRLFeatureMatcher::matchFeatures,
                                    this, frame1, frame2));
}

/////////////////////////////////////////////////////////////////////////
// ASRLFeatureMatcher::addMatchingTask
/////////////////////////////////////////////////////////////////////////
void ASRLFeatureMatcher::addStereoMatchingTask(const Features &frame1,
                                               const Features &frame2) {
  // spawn an asynchronous descriptor extraction call and put the future frame
  // at the end of the queue.
  future_stereomatches.emplace(
      std::async(std::launch::async,
                 (SimpleMatches(ASRLFeatureMatcher::*)(const Features &,
                                                       const Features &)) &
                     ASRLFeatureMatcher::matchStereoFeatures,
                 this, frame1, frame2));
}

/////////////////////////////////////////////////////////////////////////
// ASRLFeatureMatcher::getOldestFrame
/////////////////////////////////////////////////////////////////////////
SimpleMatches ASRLFeatureMatcher::getOldestMatchTask() {
  while (future_matches.empty()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  future_matches.front().wait();
  auto frame = future_matches.front().get();
  future_matches.pop();
  return frame;
}

/////////////////////////////////////////////////////////////////////////
// ASRLFeatureMatcher::getOldestFrame
/////////////////////////////////////////////////////////////////////////
SimpleMatches ASRLFeatureMatcher::getOldestStereoMatchTask() {
  while (future_stereomatches.empty()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  future_stereomatches.front().wait();
  auto frame = future_stereomatches.front().get();
  future_stereomatches.pop();
  return frame;
}

}  // namespace vision
}  // namespace vtr
