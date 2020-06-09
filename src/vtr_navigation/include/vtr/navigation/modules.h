#pragma once

// conversion
#include <vtr/navigation/modules/conversion/conversion_extraction_module.h>
#include <vtr/navigation/modules/conversion/image_triangulation_module.h>
// #include <vtr/navigation/modules/conversion/CVGpuReprojectorModule.hpp>
// #include <vtr/navigation/modules/conversion/CVGpuStereoBMModule.hpp>
// #include <vtr/navigation/modules/conversion/CVReprojectorModule.hpp>
// #include <vtr/navigation/modules/conversion/CVStereoBMModule.hpp>
// #include <vtr/navigation/modules/conversion/CVStereoSgbmModule.hpp>
// #include <vtr/navigation/modules/conversion/ElasModule.hpp>
// #include <vtr/navigation/modules/conversion/FeatureExtractionModule.hpp>
// #include <vtr/navigation/modules/conversion/ImageConversionModule.hpp>

// matching
// #include <vtr/navigation/modules/matching/ASRLMonoMatcherModule.hpp>
#include <vtr/navigation/modules/matching/asrl_stereo_matcher_module.h>
// #include <vtr/navigation/modules/matching/MelMatcherModule.hpp>
// #include <vtr/navigation/modules/matching/MelRecognitionModule.hpp>
// #include <vtr/navigation/modules/matching/OpenCVStereoMatcherModule.hpp>
// #include <vtr/navigation/modules/matching/TodRecognitionModule.hpp>

// ransac
// #include <vtr/navigation/modules/ransac/InitMonoRansacModule.hpp>
// #include <vtr/navigation/modules/ransac/MonoRansacModule.hpp>
// #include <vtr/navigation/modules/ransac/RansacModule.hpp>
#include <vtr/navigation/modules/ransac/stereo_ransac_module.h>

// misc
// #include <vtr/navigation/modules/miscellaneous/GimbalVertexTestModule.hpp>
#include <vtr/navigation/modules/miscellaneous/landmark_recall_module.h>
// #include
// <vtr/navigation/modules/miscellaneous/QuickVORosPublisherModule.hpp>
// #include
// <vtr/navigation/modules/miscellaneous/RefinedVORosPublisherModule.hpp>
// #include <vtr/navigation/modules/miscellaneous/ResultsModule.hpp>
// #include
// <vtr/navigation/modules/miscellaneous/SequentialTriangulationModule.hpp>
#include <vtr/navigation/modules/miscellaneous/simple_vertex_test_module.h>
#include <vtr/navigation/modules/miscellaneous/windowed_recall_module.h>

// localization
// #include <vtr/navigation/modules/localization/CollaborativeLandmarks.hpp>
// #include <vtr/navigation/modules/localization/ExperienceTriage.hpp>
// #include <vtr/navigation/modules/localization/LandmarkMigrationModule.hpp>
// #include
// <vtr/navigation/modules/localization/LocalizationRosPublisherModule.hpp>
// #include <vtr/navigation/modules/localization/RandomExperiences.hpp>
// #include <vtr/navigation/modules/localization/SubMapExtractionModule.hpp>

// optimization
#include <vtr/navigation/modules/optimization/keyframe_optimization_module.h>
#include <vtr/navigation/modules/optimization/steam_module.h>
#include <vtr/navigation/modules/optimization/window_optimization_module.h>

// mono
// #include <vtr/navigation/modules/mono/MonoOdomScalingModule.hpp>
// #include <vtr/navigation/modules/mono/MonoPlanarScalingModule.hpp>
// #include <vtr/navigation/modules/mono/MonoTriangulationModule.hpp>

// lancaster
// #include <vtr/navigation/modules/lancaster/LancasterVertexTestModule.hpp>

// terrain assessment
// #include <vtr/navigation/modules/terrain_assessment/CDGmmModule.hpp>
// #include <vtr/navigation/modules/terrain_assessment/CDGpcModule.hpp>
// #include <vtr/navigation/modules/terrain_assessment/CDMaxMinModule.hpp>
// #include <vtr/navigation/modules/terrain_assessment/CDMinMaxModule.hpp>
// #include <vtr/navigation/modules/terrain_assessment/GpcTrainingModule.hpp>
// #include
// <vtr/navigation/modules/terrain_assessment/LookaheadPatchGenerationModule.hpp>
// #include
// <vtr/navigation/modules/terrain_assessment/UnderfootAggregateModule.hpp>
// #include
// <vtr/navigation/modules/terrain_assessment/UnderfootSeparateModule.hpp>
