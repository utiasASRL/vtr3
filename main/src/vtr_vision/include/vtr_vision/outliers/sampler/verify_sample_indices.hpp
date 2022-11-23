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
 * \file verify_sample_indices.hpp
 * \brief Header file for the ASRL vision package
 * \details
 *
 * \author Kirk MacTavish, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <memory>

#include <vtr_vision/outliers/sampler/verify_sample_no_op.hpp>

namespace vtr {
namespace vision {

////////////////////////////////////////////////////////////////////
/// @brief This class verifies a sample ensuring no repeated indices
///
/// @details Only verifies a sample by its indices
////////////////////////////////////////////////////////////////////
class VerifySampleIndices : public VerifySampleNoOp {
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
  virtual bool checkSubset(const SimpleMatches& matches, unsigned int n,
                           unsigned int N) const;
};

}  // namespace vision
}  // namespace vtr
