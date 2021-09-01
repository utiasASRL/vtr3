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
 * \file camera_model_interface.hpp
 * \brief Header file for the ASRL vision package
 * \details
 *
 * \author Kirk MacTavish, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <iostream>

#include <vtr_vision/sensors/sensor_model_types.hpp>
#include <vtr_vision/types.hpp>

namespace vtr {
namespace vision {

////////////////////////////////////////////////////////////////////
/// @brief This class provides the base interface for vision sensors
///
/// @details
////////////////////////////////////////////////////////////////////
template <typename M, typename C>
class CameraModelInterface {
 public:
  //
  typedef std::shared_ptr<CameraModelInterface> Ptr;

  ////////////////////////////////////////////////////////////////////
  /// @brief Verify the points that will be used for model evaluations
  /// @param [in] matches The matches to verify
  /// @return true if the matches are acceptable
  ////////////////////////////////////////////////////////////////////
  virtual bool verifyMatches(const SimpleMatches& matches) const = 0;

  ////////////////////////////////////////////////////////////////////
  /// @brief Set the points that will be used for model evaluations (owned by
  /// user)
  /// @param [in] pts_ref The points seen at the reference frame
  /// @param [in] pts_query The points seen at the query frame
  ////////////////////////////////////////////////////////////////////
  virtual void setPoints(const M* pts_ref, const C* pts_query) {
    // Save to members
    pts_ref_ = pts_ref;
    pts_query_ = pts_query;
  }

 protected:
  /// @brief The points from the reference frame
  const M* pts_ref_;

  /// @brief The points from frame b
  const C* pts_query_;
};

}  // namespace vision
}  // namespace vtr
