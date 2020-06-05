#pragma once

// conversion
#include <asrl/navigation/modules/conversion/conversion_extraction_module.h>
#include <asrl/navigation/modules/conversion/image_triangulation_module.h>
// #include <asrl/navigation/modules/conversion/CVGpuReprojectorModule.hpp>
// #include <asrl/navigation/modules/conversion/CVGpuStereoBMModule.hpp>
// #include <asrl/navigation/modules/conversion/CVReprojectorModule.hpp>
// #include <asrl/navigation/modules/conversion/CVStereoBMModule.hpp>
// #include <asrl/navigation/modules/conversion/CVStereoSgbmModule.hpp>
// #include <asrl/navigation/modules/conversion/ElasModule.hpp>
// #include <asrl/navigation/modules/conversion/FeatureExtractionModule.hpp>
// #include <asrl/navigation/modules/conversion/ImageConversionModule.hpp>

// matching
// #include <asrl/navigation/modules/matching/ASRLMonoMatcherModule.hpp>
#include <asrl/navigation/modules/matching/asrl_stereo_matcher_module.h>
// #include <asrl/navigation/modules/matching/MelMatcherModule.hpp>
// #include <asrl/navigation/modules/matching/MelRecognitionModule.hpp>
// #include <asrl/navigation/modules/matching/OpenCVStereoMatcherModule.hpp>
// #include <asrl/navigation/modules/matching/TodRecognitionModule.hpp>

// ransac
// #include <asrl/navigation/modules/ransac/InitMonoRansacModule.hpp>
// #include <asrl/navigation/modules/ransac/MonoRansacModule.hpp>
// #include <asrl/navigation/modules/ransac/RansacModule.hpp>
#include <asrl/navigation/modules/ransac/stereo_ransac_module.h>

// misc
// #include <asrl/navigation/modules/miscellaneous/GimbalVertexTestModule.hpp>
#include <asrl/navigation/modules/miscellaneous/landmark_recall_module.h>
// #include
// <asrl/navigation/modules/miscellaneous/QuickVORosPublisherModule.hpp>
// #include
// <asrl/navigation/modules/miscellaneous/RefinedVORosPublisherModule.hpp>
// #include <asrl/navigation/modules/miscellaneous/ResultsModule.hpp>
// #include
// <asrl/navigation/modules/miscellaneous/SequentialTriangulationModule.hpp>
#include <asrl/navigation/modules/miscellaneous/simple_vertex_test_module.h>
#include <asrl/navigation/modules/miscellaneous/windowed_recall_module.h>

// localization
// #include <asrl/navigation/modules/localization/CollaborativeLandmarks.hpp>
// #include <asrl/navigation/modules/localization/ExperienceTriage.hpp>
// #include <asrl/navigation/modules/localization/LandmarkMigrationModule.hpp>
// #include
// <asrl/navigation/modules/localization/LocalizationRosPublisherModule.hpp>
// #include <asrl/navigation/modules/localization/RandomExperiences.hpp>
// #include <asrl/navigation/modules/localization/SubMapExtractionModule.hpp>

// optimization
#include <asrl/navigation/modules/optimization/keyframe_optimization_module.h>
#include <asrl/navigation/modules/optimization/steam_module.h>
#include <asrl/navigation/modules/optimization/window_optimization_module.h>

// mono
// #include <asrl/navigation/modules/mono/MonoOdomScalingModule.hpp>
// #include <asrl/navigation/modules/mono/MonoPlanarScalingModule.hpp>
// #include <asrl/navigation/modules/mono/MonoTriangulationModule.hpp>

// lancaster
// #include <asrl/navigation/modules/lancaster/LancasterVertexTestModule.hpp>

// terrain assessment
// #include <asrl/navigation/modules/terrain_assessment/CDGmmModule.hpp>
// #include <asrl/navigation/modules/terrain_assessment/CDGpcModule.hpp>
// #include <asrl/navigation/modules/terrain_assessment/CDMaxMinModule.hpp>
// #include <asrl/navigation/modules/terrain_assessment/CDMinMaxModule.hpp>
// #include <asrl/navigation/modules/terrain_assessment/GpcTrainingModule.hpp>
// #include
// <asrl/navigation/modules/terrain_assessment/LookaheadPatchGenerationModule.hpp>
// #include
// <asrl/navigation/modules/terrain_assessment/UnderfootAggregateModule.hpp>
// #include
// <asrl/navigation/modules/terrain_assessment/UnderfootSeparateModule.hpp>
