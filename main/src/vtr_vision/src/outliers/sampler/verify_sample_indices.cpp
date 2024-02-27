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