// Copyright 2023, Autonomous Space Robotics Lab (ASRL)
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
 * \file modules.hpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 * \brief List of all module headers for stereo pipeline for convenience.
 */
#pragma once

#include <vtr_vision/modules/preprocessing/conversion_extraction_module.hpp>
#include <vtr_vision/modules/preprocessing/calibration_module.hpp>
#include <vtr_vision/modules/preprocessing/image_triangulation_module.hpp>
#include <vtr_vision/modules/odometry/landmark_recall_module.hpp>
#include <vtr_vision/modules/odometry/asrl_stereo_matcher_module.hpp>
#include <vtr_vision/modules/odometry/simple_vertex_test_module.hpp>
#include <vtr_vision/modules/ransac/ransac_module.hpp>
#include <vtr_vision/modules/ransac/stereo_ransac_module.hpp>
#include <vtr_vision/modules/optimization/keyframe_optimization_module.hpp>
#include <vtr_vision/modules/localization/sub_map_extraction_module.hpp>
#include <vtr_vision/modules/localization/mel_matcher_module.hpp>
#include <vtr_vision/modules/localization/experience_triage_module.hpp>
#include <vtr_vision/modules/localization/landmark_migration_module.hpp>
#include <vtr_vision/modules/localization/tod_recognition_module.hpp>
#include <vtr_vision/modules/bundle_adjustment/stereo_windowed_recall_module.hpp>