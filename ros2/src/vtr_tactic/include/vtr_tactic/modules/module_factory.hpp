#pragma once

#include <memory>

#include <vtr_logging/logging.hpp>  // for debugging only
#include <vtr_tactic/factory.hpp>
#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_tactic/modules/modules.hpp>

namespace vtr {
namespace tactic {

/** \brief constructs a module based on a type_str trait */
class ModuleFactory {
 public:
  using Ptr = std::shared_ptr<ModuleFactory>;
  using Module = BaseModule;
  using ModulePtr = std::shared_ptr<BaseModule>;

  /** \brief constructed to build a particular module */
  ModuleFactory() {
    type_switch_.add<TemplateModule>();
    // lidar
    type_switch_.add<PCLPreprocessingModule>();
    type_switch_.add<ICPModule>();
    type_switch_.add<LidarRecallModule>();
    type_switch_.add<KeyframeTestModule>();
    type_switch_.add<MapMaintenanceModule>();
    type_switch_.add<LidarWindowedRecallModule>();
    // stereo
    type_switch_.add<ConversionExtractionModule>();
    type_switch_.add<ImageTriangulationModule>();
    type_switch_.add<LandmarkRecallModule>();
    type_switch_.add<ASRLStereoMatcherModule>();
    type_switch_.add<StereoRansacModule>();
    type_switch_.add<KeyframeOptimizationModule>();
    type_switch_.add<SimpleVertexTestModule>();
    type_switch_.add<StereoWindowedRecallModule>();
    type_switch_.add<StereoWindowOptimizationModule>();
    type_switch_.add<SubMapExtractionModule>();
    type_switch_.add<LandmarkMigrationModule>();
    type_switch_.add<ExperienceTriageModule>();
    type_switch_.add<TodRecognitionModule>();
    type_switch_.add<MelMatcherModule>();
  };

  /**
   * \brief makes the requested module matching the type_str trait
   * \return a base module pointer to the derived class, nullptr if not found
   * \throw invalid_argument if the derived module couldn't be found
   */
  virtual ModulePtr make(const std::string& type_str) const;

 protected:
  /**
   * \brief compare type_str traits, helper for factories who want more control
   * \param[in] type_str the type_str trait that might match derived class D
   * \return true if the trait is a match
   */
  template <class Derived>
  static bool isType(const std::string& type_str) {
    return type_str.compare(Derived::static_name) == 0;
  }

 private:
  FactoryTypeSwitch<Module> type_switch_;
};

}  // namespace tactic
}  // namespace vtr
