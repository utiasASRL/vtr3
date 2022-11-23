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
 * \file feature_extractor_factory.cpp
 * \brief Source file for the ASRL vision package
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_vision/features/extractor/feature_extractor_factory.hpp>

namespace vtr {
namespace vision {

/**
 * \brief Creates a feature extractor based on the input string.
 * \param[in] type The input_string describing the feature extractor type
 * \return a shared pointer to the instantiated feature extractor
 * \throws std::runtime_error if the type is not found.
 */
std::shared_ptr<BaseFeatureExtractor> FeatureExtractorFactory::createExtractor(
    const std::string &type) {
  std::shared_ptr<BaseFeatureExtractor> extractor = nullptr;

  if (type == "OPENCV_ORB") {
    // CPU Based Feature Extractors
    extractor.reset(new vtr::vision::OrbFeatureExtractor());
  } else if (type == "ASRL_GPU_SURF") {
    // CUDA Based Feature Extractors
#ifdef VTR_ENABLE_GPUSURF
    extractor.reset(new vtr::vision::GpuSurfFeatureExtractor());
#else
    std::string err_str =
        "Attempted to make extractor of type " + type +
        " when CUDA or GPUSURF isn't enabled." +
        "\nMake sure that CUDA and GPUSURF are installed on your system";
    LOG(ERROR) << err_str;
#endif
  } else if (type == "LEARNED_FEATURE") {
    LOG(INFO) << "LEARNED FEATURE CALLED IN FACTORY";
    extractor.reset(new vtr::vision::LearnedFeatureExtractor());
  } else {
    std::string err_str = "Could not find extractor of type " + type;
    LOG(ERROR) << err_str;
    throw std::runtime_error(err_str);
  }

  return extractor;
}

}  // namespace vision
}  // namespace vtr
