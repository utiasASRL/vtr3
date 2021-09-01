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
 * \file ransac_base.hpp
 * \brief Header file for the VTR vision package.
 * \details This header file declares the base RANSAC class.
 *
 * \author Kirk MacTavish, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <memory>

#include <vtr_vision/sensors/sensor_model_types.hpp>
#include <vtr_vision/types.hpp>

namespace vtr {
namespace vision {

// Forward declarations
class BasicSampler;
template <typename SolutionType>
class SensorModelBase;

////////////////////////////////////////////////////////////////////
/// @brief This class provides a basic RANSAC interface
///
/// @details
////////////////////////////////////////////////////////////////////
template <typename SolutionType>
class RansacBase {
 public:
  /// @brief Class shared pointer
  typedef std::shared_ptr<RansacBase> Ptr;

  // Other helpful typedefs
  typedef std::shared_ptr<SensorModelBase<SolutionType> > SensorModelPtr;
  typedef std::shared_ptr<BasicSampler> SamplerPtr;

  ////////////////////////////////////////////////////////////////////
  /// @brief Constructor
  /// @param [in] sampler The sampler generates random samples of matches
  ////////////////////////////////////////////////////////////////////
  explicit RansacBase(const SamplerPtr& sampler) : sampler_(sampler) {}

  ////////////////////////////////////////////////////////////////////
  /// @brief Register the model callback
  /// @param [in] cb The model (loaded with data) that ransac will use for
  /// outlier rejection
  ////////////////////////////////////////////////////////////////////
  void setCallback(const SensorModelPtr& cb) { cb_ = cb; }

  ////////////////////////////////////////////////////////////////////
  /// @brief Run ransac until termination conditions are met
  /// @param [in] matches The putative matches to be verified. The pair is
  /// (reference_idx, query_idx).
  /// @param [out] model \todo
  /// @param [out] inliers The inliers within the decision boundary using the
  /// hypothesis.
  ////////////////////////////////////////////////////////////////////
  virtual int run(const SimpleMatches& matches, SolutionType* model,
                  SimpleMatches* inliers) const = 0;

  ////////////////////////////////////////////////////////////////////
  /// @brief Find inlier points given a solution
  /// @param [in] matches The putative matches that were evaluated.
  /// @param [in] errors The mahalanobis distance associated with each match.
  /// @param [out] inliers The matches that were considered successful inliers.
  ////////////////////////////////////////////////////////////////////
  virtual int findInliers(const SimpleMatches& matches, const ErrorList& errors,
                          SimpleMatches* inliers) const = 0;

 protected:
  /// @brief The data sampling method
  SamplerPtr sampler_;

  /// @brief The model that is being solved
  SensorModelPtr cb_;

};  // RansacBase

extern template class RansacBase<Eigen::Matrix3d>;
extern template class RansacBase<Eigen::Matrix4d>;

}  // namespace vision
}  // namespace vtr
