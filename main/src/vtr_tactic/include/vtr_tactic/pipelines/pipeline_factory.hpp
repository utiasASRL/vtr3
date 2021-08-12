#pragma once

#include <memory>

#include <vtr_logging/logging.hpp>  // for debugging only
#include <vtr_tactic/factory.hpp>
#include <vtr_tactic/pipelines/base_pipeline.hpp>
#include <vtr_tactic/pipelines/template_pipeline.hpp>

namespace vtr {
namespace tactic {

/** \brief constructs a pipeline based on a static_name trait */
class PipelineFactory {
 public:
  using Ptr = std::shared_ptr<PipelineFactory>;

  /** \brief constructed to build a particular pipeline */
  PipelineFactory() { type_switch_.add<TemplatePipeline>(); }

  template <class DerivedModule>
  void add() {
    type_switch_.add<DerivedModule>();
  }

  /**
   * \brief makes the requested pipeline matching the static_name trait
   * \return a base pipeline pointer to the derived class, nullptr if not found
   * \throw invalid_argument if the derived pipeline couldn't be found
   */
  virtual BasePipeline::Ptr make(const std::string &static_name) const {
    LOG(DEBUG) << "Constructing pipeline with static name: " << static_name;
    auto pipeline = type_switch_.make(static_name);
    if (!pipeline) {
      auto msg = "Unknown pipeline of static name: " + static_name;
      LOG(ERROR) << msg;
      throw std::invalid_argument(msg);
    }
    return pipeline;
  }

 private:
  FactoryTypeSwitch<BasePipeline> type_switch_;
};

/** \brief make a pipeline based on ros configuration */
class ROSPipelineFactory : public PipelineFactory {
 public:
  using NodePtr = rclcpp::Node::SharedPtr;

  /**
   * \brief constructed with ros param info
   * \param[in] node the ros nodehandle with the params
   */
  ROSPipelineFactory(const NodePtr node) : node_(node) {}

  /** \brief constructs a module based on ros params */
  BasePipeline::Ptr make(const std::string &param_prefix) const override {
    std::string param_name{param_prefix + "." + type_field_};
    auto type_str = node_->declare_parameter<std::string>(param_name, "");
    if (type_str.empty()) {
      auto msg = "No field: '" + param_name + "' in the parameter list.";
      LOG(ERROR) << msg;
      throw std::runtime_error(msg);
    }
    auto pipeline = PipelineFactory::make(type_str);
    pipeline->configFromROS(node_, param_prefix);
    return pipeline;
  }

 private:
  static constexpr auto type_field_ = "type";

  const NodePtr node_;
};

}  // namespace tactic
}  // namespace vtr
