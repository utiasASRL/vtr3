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
 * \file feature_extractor_factory.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_vision/features/extractor/base_feature_extractor.hpp>

#include <vtr_vision/features/extractor/extractor_configs.hpp>
#include <vtr_vision/features/extractor/orb_feature_extractor.hpp>
#include <vtr_vision/features/extractor/learned_feature_extractor.hpp>

#ifdef VTR_ENABLE_GPUSURF
#include <vtr_vision/features/extractor/cuda/gpu_surf_feature_extractor.hpp>
#endif

namespace vtr {
namespace vision {
/** \brief Creates a feature extractor based on the input string.
 */
class FeatureExtractorFactory {
 public:
  /**
   * \brief Creates a feature extractor based on the input string.
   * \param[in] type The input_string describing the feature extractor type
   * \return a shared pointer to the instantiated feature extractor
   * \throws std::runtime_error if the type is not found.
   */
  static std::shared_ptr<BaseFeatureExtractor> createExtractor(
      const std::string &type);

 private:
};

}  // namespace vision
}  // namespace vtr
