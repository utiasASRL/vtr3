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
 * \file modules.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_vision/modules/conversion/conversion_extraction_module.hpp>
#include <vtr_vision/modules/conversion/image_triangulation_module.hpp>
#include <vtr_vision/modules/localization/experience_triage_module.hpp>
#include <vtr_vision/modules/localization/landmark_migration_module.hpp>
#include <vtr_vision/modules/localization/sub_map_extraction_module.hpp>
#include <vtr_vision/modules/localization/tod_recognition_module.hpp>
#include <vtr_vision/modules/matching/asrl_stereo_matcher_module.hpp>
#include <vtr_vision/modules/matching/mel_matcher_module.hpp>
#include <vtr_vision/modules/miscellaneous/landmark_recall_module.hpp>
#include <vtr_vision/modules/miscellaneous/simple_vertex_test_module.hpp>
#include <vtr_vision/modules/miscellaneous/stereo_windowed_recall_module.hpp>
#include <vtr_vision/modules/miscellaneous/vertex_creation_test_module.hpp>
#include <vtr_vision/modules/optimization/keyframe_optimization_module.hpp>
#include <vtr_vision/modules/optimization/steam_module.hpp>
#include <vtr_vision/modules/optimization/stereo_window_optimization_module.hpp>
#include <vtr_vision/modules/ransac/ransac_module.hpp>
#include <vtr_vision/modules/ransac/stereo_ransac_module.hpp>