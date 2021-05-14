#pragma once

#include "rclcpp/rclcpp.hpp"

#include <vtr_tactic/modules/module_factory.hpp>
#include <vtr_tactic/modules/modules.hpp>

namespace vtr {
namespace tactic {

/** \brief make a module based on ros configuration */
class ROSModuleFactory : public ModuleFactory {
 public:
  using NodePtr = rclcpp::Node::SharedPtr;

  /**
   * \brief constructed with ros param info
   * \param[in] node the ros nodehandle with the params
   */
  ROSModuleFactory(const NodePtr node) : node_(node){};

  /**
   * \brief constructs a module based on ros params
   * \param[in] type_str the type_str trait of the requested module
   */
  ModulePtr make(const std::string &param_prefix) const override;

 private:
  static constexpr auto type_field_ = "type";

  /**
   * \brief configures the module using rosparam
   * \param[in] module pointer to the module
   * \param[in] type_str the type_str trait of the requested module
   */
  void configureModule(ModulePtr &module, const std::string &type_str,
                       const std::string &param_prefix) const;
  // clang-format off
  /// Template
  void configureTemplate(ModulePtr &, const std::string &) const;
  /// Lidar related modules
  void configurePCLPreprocessing(ModulePtr &, const std::string &) const;
  void configureICP(ModulePtr &, const std::string &) const;
  void configurePCMapMaintenance(ModulePtr &, const std::string &) const;
  void configurePCRecall(ModulePtr &, const std::string &) const;
  void configurePCWindowedRecall(ModulePtr &, const std::string &) const;
  void configurePCKeyframeTest(ModulePtr &, const std::string &) const;
  /// Stereo related modules
  void configureConversionExtraction(ModulePtr &, const std::string &) const;
  void configureORBDetector(vision::ORBConfiguration &config, const std::string &) const;
#if GPUSURF_ENABLED
  void configureSURFDetector(asrl::GpuSurfConfiguration &config, const std::string &) const;
  void configureSURFStereoDetector(asrl::GpuSurfStereoConfiguration &config, const std::string &) const;
#endif
  void configureImageTriangulation(ModulePtr &, const std::string &) const;
  void configureLandmarkRecall(ModulePtr &, const std::string &) const;
  void configureASRLStereoMatcher(ModulePtr &, const std::string &) const;
  void configureStereoRANSAC(ModulePtr &, const std::string &) const;
  void configureRANSAC(std::shared_ptr<RansacModule::Config> &, const std::string &) const;
  void configureKeyframeOptimization(ModulePtr &, const std::string &) const;
  void configureSteam(std::shared_ptr<SteamModule::Config> &, const std::string &) const;
  void configureSimpleVertexTest(ModulePtr &, const std::string &) const;
  void configureStereoWindowedRecallModule(ModulePtr &, const std::string &) const;
  void configureStereoWindowOptimization(ModulePtr &, const std::string &) const;

  void configureSubMapExtraction(ModulePtr &, const std::string &) const;
  void configureExperienceTriage(ModulePtr &, const std::string &) const;
  void configureLandmarkMigration(ModulePtr &, const std::string &) const;
  void configureTodRecog(ModulePtr &, const std::string &) const;
  void configureMelMatcher(ModulePtr &, const std::string &) const;
  // clang-format on
  const NodePtr node_;
};

}  // namespace tactic
}  // namespace vtr
