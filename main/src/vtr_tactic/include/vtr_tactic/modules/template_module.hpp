#pragma once

#include <vtr_tactic/modules/base_module.hpp>

namespace vtr {
namespace tactic {

/** \brief Reject outliers and estimate a preliminary transform */
class TemplateModule : public BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "template";

  /** \brief Collection of config parameters */
  struct Config {
    std::string parameter = "default value";
  };

  TemplateModule(const std::string &name = static_name)
      : BaseModule{name}, config_(std::make_shared<Config>()){};

  void setConfig(std::shared_ptr<Config> &config) { config_ = config; }

 private:
  void runImpl(QueryCache &, MapCache &, const Graph::ConstPtr &) override {
    LOG(INFO) << "This is a template module with parameter: "
              << config_->parameter;
  }

  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;
};

}  // namespace tactic
}  // namespace vtr