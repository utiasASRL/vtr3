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
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 * \brief List of all module headers for LiDAR pipeline for convenience.
 */
#pragma once

#include "vtr_lidar/modules/preprocessing/conversions/aeva_conversion_module.hpp"
#include "vtr_lidar/modules/preprocessing/conversions/aeva_conversion_module_v2.hpp"
#include "vtr_lidar/modules/preprocessing/conversions/honeycomb_conversion_module_v2.hpp"
#include "vtr_lidar/modules/preprocessing/conversions/velodyne_conversion_module.hpp"
#include "vtr_lidar/modules/preprocessing/conversions/velodyne_conversion_module_v2.hpp"
#include "vtr_lidar/modules/preprocessing/conversions/ouster_conversion_module.hpp"
#include "vtr_lidar/modules/preprocessing/preprocessing_module.hpp"
#include "vtr_lidar/modules/preprocessing/preprocessing_module_v2.hpp"

#include "vtr_lidar/modules/odometry/odometry_icp_module.hpp"
#include "vtr_lidar/modules/odometry/odometry_map_maintenance_module_v2.hpp"
#include "vtr_lidar/modules/odometry/vertex_test_module.hpp"
// #include "vtr_lidar/modules/odometry/sample_module.hpp"

#include "vtr_lidar/modules/localization/localization_icp_module.hpp"
#include "vtr_lidar/modules/localization/localization_map_recall_module.hpp"

#include "vtr_lidar/modules/pointmap/dynamic_detection_module.hpp"
#include "vtr_lidar/modules/pointmap/inter_exp_merging_module_v2.hpp"
#include "vtr_lidar/modules/pointmap/intra_exp_merging_module_v2.hpp"

#include "vtr_lidar/modules/planning/change_detection_module_v3.hpp"
// #include "vtr_lidar/modules/planning/ground_extraction_module.hpp"
// #include "vtr_lidar/modules/planning/obstacle_detection_module.hpp"
// #include "vtr_lidar/modules/planning/safe_corridor_module.hpp"
// #include "vtr_lidar/modules/planning/terrain_assessment_module.hpp"
// #include "vtr_lidar/modules/planning/diff_generator.hpp"

