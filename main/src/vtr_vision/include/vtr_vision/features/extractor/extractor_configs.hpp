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
 * \file extractor_configs.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_vision/features/extractor/orb_configuration.hpp>
#include <vtr_vision/features/extractor/learned_feature_configuration.hpp>

#ifdef VTR_ENABLE_GPUSURF
#include <asrl/vision/gpusurf/GpuSurfDetector.hpp>
#include <asrl/vision/gpusurf/GpuSurfStereoDetector.hpp>
#endif
