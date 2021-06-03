////////////////////////////////////////////////////////////////////////////////
/// @brief Header file for the ASRL vision package
///
/// @author Kirk MacTavish, ASRL
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include "vtr_vision/outliers/sampler/verify_sample_no_op.hpp"

// External
#include <memory>

namespace vtr {
namespace vision {

////////////////////////////////////////////////////////////////////
/// @brief This class verifies a sample ensuring no repeated indices
///
/// @details Only verifies a sample by its indices
////////////////////////////////////////////////////////////////////
class VerifySampleIndices
    : public VerifySampleNoOp {

public:

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

};

} // namespace vision
} // namespace vtr_vision
