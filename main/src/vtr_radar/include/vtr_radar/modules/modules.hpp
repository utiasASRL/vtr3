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
 * \author Yuchen Wu, Keenan Burnett, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_radar/modules/preprocessing/conversions/online_radar_conversion_module.hpp"
#include "vtr_radar/modules/preprocessing/conversions/offline_radar_conversion_module.hpp"
#include "vtr_radar/modules/preprocessing/extraction/radar_extraction_module.hpp"
#include "vtr_radar/modules/preprocessing/extraction/doppler_extraction_module.hpp"
#include "vtr_radar/modules/preprocessing/preprocessing_module.hpp"

#include "vtr_radar/modules/odometry/odometry_gyro_module.hpp"
#include "vtr_radar/modules/odometry/odometry_doppler_module.hpp"

#include "vtr_radar/modules/odometry/odometry_icp_module.hpp"
#include "vtr_radar/modules/odometry/odometry_map_maintenance_module.hpp"
#include "vtr_radar/modules/odometry/vertex_test_module.hpp"

#include "vtr_radar/modules/localization/localization_icp_module.hpp"
#include "vtr_radar/modules/localization/localization_map_recall_module.hpp"