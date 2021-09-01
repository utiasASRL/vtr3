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
 * \file outliers.hpp
 * \brief
 * \details Convenience header for outlier rejection (RANSAC)
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_vision/outliers/ransac/vanilla_ransac.hpp>
#include <vtr_vision/outliers/sampler/basic_sampler.hpp>
#include <vtr_vision/outliers/sampler/progressive_sampler.hpp>
#include <vtr_vision/outliers/sampler/verify_sample_indices.hpp>
#include <vtr_vision/outliers/sampler/verify_sample_subset_mask.hpp>
