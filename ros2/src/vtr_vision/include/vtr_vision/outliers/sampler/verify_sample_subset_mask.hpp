////////////////////////////////////////////////////////////////////////////////
/// @brief Header file for the ASRL vision package
///
/// @author Kirk MacTavish, ASRL
///////////////////////////////////////////////////////////////////////////////

#pragma once

// Internal
#include "vtr_vision/outliers/sampler/verify_sample_indices.hpp"

// External
#include <memory>

namespace vtr {
namespace vision {

////////////////////////////////////////////////////////////////////
/// @brief This class verifies a sample ensuring no repeated indices,
///        and at least n samples from a privileged subset.
///
/// @details Requires a mask for the privileged subset.
////////////////////////////////////////////////////////////////////
class VerifySampleSubsetMask
    : public VerifySampleIndices {

public:

  ////////////////////////////////////////////////////////////////////
  /// @param [in] p The minimum number of privileged indices in a valid sample
  /// @param [in] N The size of the sample
  /// @param [in] mask The camera feature (MatchPair.second) index mask, with priviledge "true" indices
  ////////////////////////////////////////////////////////////////////
  VerifySampleSubsetMask(int privileged, const std::vector<bool>& mask)
    : privileged_(privileged), mask_(mask) {
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Set the mask (needs to be updated when the RANSAC matches change
  /// @param [in] mask The index mask, with privilege "true" indices
  ////////////////////////////////////////////////////////////////////
  void setMask(const std::vector<bool>& mask) {
    mask_ = mask;
  }

  /// @brief Class shared pointer
  typedef std::shared_ptr<VerifySampleNoOp> Ptr;

  ////////////////////////////////////////////////////////////////////
  /// @brief Are the last n matches valid with the rest of the sample?
  /// @param [in] matches The matches to verify (up to N matches)
  /// @param [in] n The first n matches have previously been verified
  /// @param [in] N The size of the completed sample
  /// @return true if the sample is valid
  ////////////////////////////////////////////////////////////////////
  virtual bool checkSubset(const SimpleMatches& matches,
                           unsigned int n,
                           unsigned int N) const;

private:

  /// The minimum number of privileged indices that must be present in the sample
  int privileged_;

  /// The camera feature (MatchPair.second) index mask, with privileged "true" indices
  std::vector<bool> mask_;
};

} // namespace vision
} // namespace vtr_vision
