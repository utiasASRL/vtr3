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

  // clang-format off
  /// \todo remove these functions
  /// Stereo related modules
  void configureImageTriangulation(ModulePtr &, const std::string &) const;
  void configureLandmarkRecall(ModulePtr &, const std::string &) const;
  void configureASRLStereoMatcher(ModulePtr &, const std::string &) const;
  void configureStereoRANSAC(ModulePtr &, const std::string &) const;
  void configureRANSAC(std::shared_ptr<stereo::RansacModule::Config> &, const std::string &) const;
  void configureKeyframeOptimization(ModulePtr &, const std::string &) const;
  void configureSteam(std::shared_ptr<stereo::SteamModule::Config> &, const std::string &) const;
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
