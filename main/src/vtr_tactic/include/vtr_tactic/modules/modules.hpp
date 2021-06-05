#pragma once

#include "vtr_tactic/modules/template_module.hpp"

// lidar
#include "vtr_tactic/modules/lidar/icp_module.hpp"
#include "vtr_tactic/modules/lidar/keyframe_test_module.hpp"
#include "vtr_tactic/modules/lidar/map_maintenance_module.hpp"
#include "vtr_tactic/modules/lidar/map_recall_module.hpp"
#include "vtr_tactic/modules/lidar/preprocessing_module.hpp"
#include "vtr_tactic/modules/lidar/windowed_map_recall_module.hpp"

// stereo
#include "vtr_tactic/modules/stereo/conversion/conversion_extraction_module.hpp"
#include "vtr_tactic/modules/stereo/conversion/image_triangulation_module.hpp"
#include "vtr_tactic/modules/stereo/localization/experience_triage_module.hpp"
#include "vtr_tactic/modules/stereo/localization/landmark_migration_module.hpp"
#include "vtr_tactic/modules/stereo/localization/sub_map_extraction_module.hpp"
#include "vtr_tactic/modules/stereo/localization/tod_recognition_module.hpp"
#include "vtr_tactic/modules/stereo/matching/asrl_stereo_matcher_module.hpp"
#include "vtr_tactic/modules/stereo/matching/mel_matcher_module.hpp"
#include "vtr_tactic/modules/stereo/miscellaneous/landmark_recall_module.hpp"
#include "vtr_tactic/modules/stereo/miscellaneous/simple_vertex_test_module.hpp"
#include "vtr_tactic/modules/stereo/miscellaneous/stereo_windowed_recall_module.hpp"
#include "vtr_tactic/modules/stereo/miscellaneous/vertex_creation_test_module.hpp"
#include "vtr_tactic/modules/stereo/optimization/keyframe_optimization_module.hpp"
#include "vtr_tactic/modules/stereo/optimization/steam_module.hpp"
#include "vtr_tactic/modules/stereo/optimization/stereo_window_optimization_module.hpp"
#include "vtr_tactic/modules/stereo/ransac/ransac_module.hpp"
#include "vtr_tactic/modules/stereo/ransac/stereo_ransac_module.hpp"