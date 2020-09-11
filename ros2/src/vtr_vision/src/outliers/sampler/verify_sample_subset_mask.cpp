////////////////////////////////////////////////////////////////////////////////
/// @brief Source file for the ASRL vision package
/// @details
///
/// @author Kirk MacTavish, ASRL
///////////////////////////////////////////////////////////////////////////////
#include <iostream>

#include "vtr_vision/outliers/sampler/verify_sample_subset_mask.hpp"

namespace vtr {
namespace vision {

bool VerifySampleSubsetMask::checkSubset(const SimpleMatches& matches,
                                         unsigned int n,
                                         unsigned int N) const {

  // Verify the indices using the parent
  if (!VerifySampleIndices::checkSubset(matches,n,N))
      return false;

  // The number of privileged matches we need to find during this check
  // We only need to meet the minimum criteria. If there are more in
  // the count than the target, that's OK. This also means that if it's
  // possible to meet the criteria in future samples, that criteria isn't
  // required in this sample
  int pseudo_target = privileged_ - N + matches.size();
  // make sure the value is greater than 0
  unsigned int target = std::max(pseudo_target,0);
  // Loop over all matches, and count how many are privileged
  unsigned int count = 0;
  for (unsigned int q = 0; q < matches.size() && count < target; ++q) {
    if (mask_.at(matches[q].second)) ++count;
  }

  // Make sure we found enough privileged matches
  return count >= target;
}

} // namespace vision
} // namespace vtr_vision
