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
/// #include <vtr_navigation/modules/matching/ASRLMonoMatcherModule.hpp>
#include <vtr_navigation/modules/matching/mel_matcher_module.hpp>
#include <vtr_navigation/modules/localization/mel_recognition_module.hpp>
#if false
/// #include <vtr_navigation/modules/matching/OpenCVStereoMatcherModule.hpp>
#endif

// ransac
#include <vtr_navigation/modules/ransac/stereo_ransac_module.hpp>
/// #include <vtr_navigation/modules/ransac/InitMonoRansacModule.hpp>
/// #include <vtr_navigation/modules/ransac/MonoRansacModule.hpp>
/// #include <vtr_navigation/modules/ransac/RansacModule.hpp>

// misc
#include <vtr_navigation/modules/miscellaneous/landmark_recall_module.hpp>
#include <vtr_navigation/modules/miscellaneous/simple_vertex_test_module.hpp>
#include <vtr_navigation/modules/miscellaneous/windowed_recall_module.hpp>
#if false
/// #include <vtr_navigation/modules/miscellaneous/GimbalVertexTestModule.hpp>
/// #include <vtr_navigation/modules/miscellaneous/QuickVORosPublisherModule.hpp>
/// #include <vtr_navigation/modules/miscellaneous/RefinedVORosPublisherModule.hpp>
#include <vtr_navigation/modules/miscellaneous/results_module.hpp>
/// #include <vtr_navigation/modules/miscellaneous/SequentialTriangulationModule.hpp>
#endif

// localization
#include <vtr_navigation/modules/localization/collaborative_landmarks.hpp>
#include <vtr_navigation/modules/localization/tod_recognition_module.hpp>
#include <vtr_navigation/modules/localization/experience_triage_module.hpp>
#include <vtr_navigation/modules/localization/landmark_migration_module.hpp>
#include <vtr_navigation/modules/localization/random_experiences.hpp>
#include <vtr_navigation/modules/localization/sub_map_extraction_module.hpp>
/// #include <vtr_navigation/modules/localization/LocalizationRosPublisherModule.hpp>

// optimization
#include <vtr_navigation/modules/optimization/keyframe_optimization_module.hpp>
#include <vtr_navigation/modules/optimization/steam_module.hpp>
#include <vtr_navigation/modules/optimization/window_optimization_module.hpp>
