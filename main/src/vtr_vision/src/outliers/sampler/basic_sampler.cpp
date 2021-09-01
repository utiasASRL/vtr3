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
 * \file basic_sampler.cpp
 * \brief
 * \details This file defines the BaseSampler class, which provides verified
 * samples for RANSAC
 *
 * \author Kirk MacTavish, Autonomous Space Robotics Lab (ASRL)
 */
#include <random>

#include <vtr_vision/outliers/sampler/basic_sampler.hpp>
#include <vtr_vision/outliers/sampler/verify_sample_indices.hpp>

namespace vtr {
namespace vision {

BasicSampler::BasicSampler() {
  setVerifier(std::make_shared<VerifySampleIndices>());
}

BasicSampler::BasicSampler(const std::shared_ptr<VerifySampleNoOp>& verifier) {
  setVerifier(verifier);
}

void BasicSampler::setInputMatches(const SimpleMatches* matches) {
  // Save a ref to the matches
  matches_ = matches;
  // Re-seed the random engine
#ifdef VTR_DETERMINISTIC
  setSeed(0);
#else
  randomSeed();
#endif
  // Set up the sample distribution
  dist_ = std::uniform_int_distribution<int>(0, matches_->size() - 1);
}

bool BasicSampler::getSample(unsigned int m, SimpleMatches* p_sample,
                             const unsigned int& max_attempts) {
  // Preliminary checks
  if (!precheck(m, p_sample)) return false;

  // References
  SimpleMatches& sample = *p_sample;
  const SimpleMatches& matches = *matches_;

  // Loop over each index in the sample
  unsigned int tries = 0;
  for (unsigned int i = 0; i < m; ++i) {
    // Try to find a unique sample
    for (; tries < max_attempts && sample.size() == i; ++tries) {
      // Get a new sample
      sample.push_back(matches[dist_(eng_)]);

      // Verify the sample to make sure we didn't choose it before
      if (!verifier_->checkSubset(sample, sample.size() - 1, m)) {
        sample.pop_back();
      }
    }
  }

  if (sample.size() != m) {
    // We were unsuccessful in finding a unique sample that met the criteria of
    // the verifier
    sample.clear();
    return false;
  }

  // Done
  return true;
}

bool BasicSampler::precheck(unsigned int m, SimpleMatches* p_sample) {
  // Allocate the sample
  if (!p_sample) {
    LOG(ERROR) << "The sample list was not allocated.";
    return false;
  }
  p_sample->reserve(m);

  // Check the verifier
  if (!verifier_) {
    LOG(ERROR) << "The sample verifier is invalid.";
    return false;
  }

  // Make sure we have enough matches
  if (!matches_) {
    LOG(ERROR) << "Matches have not been set";
    return false;
  }
  if (matches_->size() < m) {
    LOG(WARNING) << "There aren't enough matches to satisfy the model.";
    return false;
  }

  return true;
}

}  // namespace vision
}  // namespace vtr
