#pragma once
// conversion
#include <vtr_navigation/modules/conversion/conversion_extraction_module.hpp>
#include <vtr_navigation/modules/conversion/image_triangulation_module.hpp>
/// #include <vtr_navigation/modules/conversion/CVGpuReprojectorModule.hpp>
/// #include <vtr_navigation/modules/conversion/CVGpuStereoBMModule.hpp>
/// #include <vtr_navigation/modules/conversion/CVReprojectorModule.hpp>
/// #include <vtr_navigation/modules/conversion/CVStereoBMModule.hpp>
/// #include <vtr_navigation/modules/conversion/CVStereoSgbmModule.hpp>
/// #include <vtr_navigation/modules/conversion/ElasModule.hpp>
/// #include <vtr_navigation/modules/conversion/FeatureExtractionModule.hpp>
/// #include <vtr_navigation/modules/conversion/ImageConversionModule.hpp>

// matching
#include <vtr_navigation/modules/matching/asrl_stereo_matcher_module.hpp>
#if false
/// #include <vtr_navigation/modules/matching/ASRLMonoMatcherModule.hpp>
#include <vtr_navigation/modules/matching/mel_matcher_module.hpp>
/// #include <vtr_navigation/modules/matching/MelRecognitionModule.hpp>
/// #include <vtr_navigation/modules/matching/OpenCVStereoMatcherModule.hpp>
#include <vtr_navigation/modules/matching/tod_recognition_module.hpp>
#endif

// ransac
#include <vtr_navigation/modules/ransac/stereo_ransac_module.hpp>
#if false
/// #include <vtr_navigation/modules/ransac/InitMonoRansacModule.hpp>
/// #include <vtr_navigation/modules/ransac/MonoRansacModule.hpp>
/// #include <vtr_navigation/modules/ransac/RansacModule.hpp>
#endif

// misc
#include <vtr_navigation/modules/miscellaneous/landmark_recall_module.hpp>
#include <vtr_navigation/modules/miscellaneous/simple_vertex_test_module.hpp>
#if false
/// #include <vtr_navigation/modules/miscellaneous/GimbalVertexTestModule.hpp>
/// #include <vtr_navigation/modules/miscellaneous/QuickVORosPublisherModule.hpp>
/// #include <vtr_navigation/modules/miscellaneous/RefinedVORosPublisherModule.hpp>
#include <vtr_navigation/modules/miscellaneous/results_module.hpp>
/// #include <vtr_navigation/modules/miscellaneous/SequentialTriangulationModule.hpp>
#include <vtr_navigation/modules/miscellaneous/windowed_recall_module.hpp>
#endif

// localization
#if false
#include <vtr_navigation/modules/localization/collaborative_landmarks.hpp>
#include <vtr_navigation/modules/localization/experience_triage.hpp>
#include <vtr_navigation/modules/localization/landmark_migration_module.hpp>
#include <vtr_navigation/modules/localization/random_experiences.hpp>
#include <vtr_navigation/modules/localization/sub_map_extraction_module.hpp>
/// #include <vtr_navigation/modules/localization/LocalizationRosPublisherModule.hpp>
#endif

// optimization
#include <vtr_navigation/modules/optimization/keyframe_optimization_module.hpp>
#include <vtr_navigation/modules/optimization/steam_module.hpp>
#if false
#include <vtr_navigation/modules/optimization/window_optimization_module.hpp>

// mono
/// #include <vtr_navigation/modules/mono/MonoOdomScalingModule.hpp>
/// #include <vtr_navigation/modules/mono/MonoPlanarScalingModule.hpp>
/// #include <vtr_navigation/modules/mono/MonoTriangulationModule.hpp>

// lancaster
/// #include <vtr_navigation/modules/lancaster/LancasterVertexTestModule.hpp>

// terrain assessment
/// #include <vtr_navigation/modules/terrain_assessment/CDGmmModule.hpp>
/// #include <vtr_navigation/modules/terrain_assessment/CDGpcModule.hpp>
/// #include <vtr_navigation/modules/terrain_assessment/CDMaxMinModule.hpp>
/// #include <vtr_navigation/modules/terrain_assessment/CDMinMaxModule.hpp>
/// #include <vtr_navigation/modules/terrain_assessment/GpcTrainingModule.hpp>
/// #include <vtr_navigation/modules/terrain_assessment/LookaheadPatchGenerationModule.hpp>
/// #include <vtr_navigation/modules/terrain_assessment/UnderfootAggregateModule.hpp>
/// #include <vtr_navigation/modules/terrain_assessment/UnderfootSeparateModule.hpp>
#endif