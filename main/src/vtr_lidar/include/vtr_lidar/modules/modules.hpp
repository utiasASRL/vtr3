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
 * \brief List of all module headers for LiDAR pipeline for convenience.
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_lidar/modules/conversions/honeycomb_conversion_module.hpp>
#include <vtr_lidar/modules/conversions/velodyne_conversion_module.hpp>
#include <vtr_lidar/modules/keyframe_test_module.hpp>
#include <vtr_lidar/modules/localization_icp_module.hpp>
#include <vtr_lidar/modules/map_maintenance_module.hpp>
#include <vtr_lidar/modules/map_recall_module.hpp>
#include <vtr_lidar/modules/odometry_icp_module.hpp>
#include <vtr_lidar/modules/preprocessing_module.hpp>
#include <vtr_lidar/modules/windowed_map_recall_module.hpp>