#include <vtr/navigation/factories/module_factory.h>
#include <vtr/navigation/modules.h>

namespace vtr {
namespace navigation {

ModuleFactory::mod_ptr ModuleFactory::make() const {
  /// \todo maybe put this in constructor, because it should only be created
  // once.
  FactoryTypeSwitch<mod_t> type_switch;
  type_switch.add<ConversionExtractionModule>();
  type_switch.add<ImageTriangulationModule>();
  type_switch.add<StereoRansacModule>();
  type_switch.add<KeyframeOptimizationModule>();
  type_switch.add<ASRLStereoMatcherModule>();
  type_switch.add<SimpleVertexTestModule>();
  type_switch.add<WindowOptimizationModule>();
  type_switch.add<WindowedRecallModule>();
  type_switch.add<LandmarkRecallModule>();
  /*
  type_switch.add<NoopModule>();
  type_switch.add<OpenCVStereoMatcherModule>();
  type_switch.add<ASRLMonoMatcherModule>();
  type_switch.add<FeatureExtractionModule>();
  type_switch.add<ImageConversionModule>();
  type_switch.add<CVStereoBMModule>();
  type_switch.add<CVStereoSgbmModule>();
  type_switch.add<CVGpuStereoBMModule>();
  type_switch.add<ElasModule>();
  type_switch.add<CVReprojectorModule>();
  type_switch.add<CVGpuReprojectorModule>();
  type_switch.add<SubMapExtractionModule>();
  type_switch.add<LandmarkMigrationModule>();
  type_switch.add<MelMatcherModule>();
  type_switch.add<SubMapExtractionModule>();
  type_switch.add<ResultsModule>();
  type_switch.add<MelRecognitionModule>();
  type_switch.add<TodRecognitionModule>();
  type_switch.add<CollaborativeLandmarksModule>();
  type_switch.add<RandomExperiencesModule>();
  type_switch.add<ExperienceTriageModule>();
  type_switch.add<QuickVORosPublisherModule>();
  type_switch.add<RefinedVORosPublisherModule>();
  type_switch.add<LocalizationRosPublisherModule>();
  type_switch.add<UnderfootSeparateModule>();
  type_switch.add<UnderfootAggregateModule>();
  type_switch.add<LookaheadPatchGenerationModule>();
  type_switch.add<GpcTrainingModule>();
  type_switch.add<CDMaxMinModule>();
  type_switch.add<CDMinMaxModule>();
  type_switch.add<CDGmmModule>();
  type_switch.add<CDGpcModule>();
  type_switch.add<MonoPlanarScalingModule>();
  type_switch.add<MonoOdomScalingModule>();
  type_switch.add<MonoTriangulationModule>();
  type_switch.add<SequentialTriangulationModule>();
  type_switch.add<InitMonoRansacModule>();
  type_switch.add<MonoRansacModule>();
  type_switch.add<LancasterVertexTestModule>();
  type_switch.add<GimbalVertexTestModule>();
  */
  auto module = type_switch.make(type_str_);
  if (!module) {
    auto msg = "unknown module of type " + type_str_;
    LOG(ERROR) << msg;
    throw std::invalid_argument(msg);
  }
  return module;
}
}  // namespace navigation
}  // namespace vtr
