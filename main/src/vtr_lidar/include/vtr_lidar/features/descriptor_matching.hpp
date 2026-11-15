/**
 * \file descriptor_matching.hpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 *
 * Descriptor matching for lidar intensity-image features.
 */
#pragma once

#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

namespace vtr {
namespace lidar {

/**
 * \brief Mutual nearest neighbours of binary (ORB) descriptors: a pair is
 * kept only if each descriptor is the other's nearest neighbour (Hamming)
 * and their distance is below max_distance.
 * \return pairs of (query index, train index)
 */
inline std::vector<std::pair<int, int>> matchMutualNN(
    const cv::Mat& query_descriptors, const cv::Mat& train_descriptors,
    const float max_distance) {
  std::vector<std::pair<int, int>> pairs;
  if (query_descriptors.empty() || train_descriptors.empty()) return pairs;
  const auto matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);
  std::vector<cv::DMatch> matches;
  matcher->match(query_descriptors, train_descriptors, matches);
  pairs.reserve(matches.size());
  for (const auto& m : matches)
    if (m.distance < max_distance) pairs.emplace_back(m.queryIdx, m.trainIdx);
  return pairs;
}

}  // namespace lidar
}  // namespace vtr
