////////////////////////////////////////////////////////////////////////////////
/// @brief BasicSampler.cpp Source file for the ASRL vision package
/// @details This file defines the BaseSampler class, which provides
///          verified samples for RANSAC
///
/// @author Kirk MacTavish, ASRL
///////////////////////////////////////////////////////////////////////////////

// ASRL
#include "vtr_vision/outliers/sampler/basic_sampler.hpp"
#include "vtr_vision/outliers/sampler/verify_sample_indices.hpp"

// External
#include <random>

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
#ifdef DETERMINISTIC_VTR
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
