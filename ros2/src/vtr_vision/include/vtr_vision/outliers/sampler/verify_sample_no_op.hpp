////////////////////////////////////////////////////////////////////////////////
/// @brief Header file for the ASRL vision package
///
/// @author Kirk MacTavish, ASRL
///////////////////////////////////////////////////////////////////////////////

#pragma once

// Internal
#include "vtr_vision/types.hpp"

// External
#include <memory>

namespace vtr {
namespace vision {

////////////////////////////////////////////////////////////////////
/// @brief This class provides a sample verification interface
///
/// @details Implements a no-op check if no sample check is desired.
////////////////////////////////////////////////////////////////////
class VerifySampleNoOp {

public:

  /// @brief Class shared pointer
  typedef std::shared_ptr<VerifySampleNoOp> Ptr;

  ////////////////////////////////////////////////////////////////////
  /// @brief Are the last n matches valid with the rest of the sample?
  /// @param [in] matches The matches to verify (up to N matches)
  /// @param [in] n The first n matches have previously been verified
  /// @param [in] N The size of the completed sample
  /// @return Always returns true, does not perform any check
  ////////////////////////////////////////////////////////////////////
  virtual bool checkSubset(const SimpleMatches& matches __attribute__((unused)),
                           unsigned int n __attribute__((unused)),
                           unsigned int N __attribute__((unused)) ) const {
    return true;
  }
};

} // namespace vision
} // namespace vtr_vision
