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
    type_switch_.add<lidar::PreprocessingModule>();
    type_switch_.add<lidar::ICPModule>();
    type_switch_.add<lidar::MapRecallModule>();
    type_switch_.add<lidar::KeyframeTestModule>();
    type_switch_.add<lidar::MapMaintenanceModule>();
    type_switch_.add<lidar::WindowedMapRecallModule>();
    // stereo
    type_switch_.add<stereo::ConversionExtractionModule>();
    type_switch_.add<stereo::ImageTriangulationModule>();
    type_switch_.add<stereo::LandmarkRecallModule>();
    type_switch_.add<stereo::ASRLStereoMatcherModule>();
    type_switch_.add<stereo::StereoRansacModule>();
    type_switch_.add<stereo::KeyframeOptimizationModule>();
    type_switch_.add<stereo::SimpleVertexTestModule>();
    type_switch_.add<stereo::StereoWindowedRecallModule>();
    type_switch_.add<stereo::StereoWindowOptimizationModule>();
    type_switch_.add<stereo::SubMapExtractionModule>();
    type_switch_.add<stereo::LandmarkMigrationModule>();
    type_switch_.add<stereo::ExperienceTriageModule>();
    type_switch_.add<stereo::TodRecognitionModule>();
    type_switch_.add<stereo::MelMatcherModule>();
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
