/**
 * \file verify_sample_np_op.hpp
 * \brief Header file for the ASRL vision package
 * \details
 *
 * \author Kirk MacTavish, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <memory>

#include <vtr_vision/types.hpp>

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
  virtual bool checkSubset(const SimpleMatches& matches, unsigned int n,
                           unsigned int N) const {
    (void)matches;
    (void)n;
    (void)N;
    return true;
  }
};

}  // namespace vision
}  // namespace vtr
