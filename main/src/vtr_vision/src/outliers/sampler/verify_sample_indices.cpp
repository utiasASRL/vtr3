/**
 * \file verify_sample_indices.cpp
 * \brief
 * \details This file defines the BaseSampler class, which provides verified
 * samples for RANSAC
 *
 * \author Kirk MacTavish, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_vision/outliers/sampler/verify_sample_indices.hpp>

namespace vtr {
namespace vision {

bool VerifySampleIndices::checkSubset(const SimpleMatches &matches,
                                      unsigned int n, unsigned int N) const {
  (void)N;
  // Just check the matches after n (the rest were already verified)
  for (unsigned int q = n; q < matches.size(); ++q) {
    // Verify it doesn't share indices with earlier sample matches
    for (unsigned int j = 0; j < q; ++j) {
      if (matches[q].first == matches[j].first ||
          matches[q].second == matches[j].second) {
        return false;
      }
    }
  }

  return true;
}

}  // namespace vision
}  // namespace vtr