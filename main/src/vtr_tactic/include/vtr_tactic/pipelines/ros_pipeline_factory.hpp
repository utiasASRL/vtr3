#pragma once

#include "rclcpp/rclcpp.hpp"

#include <vtr_tactic/pipelines/pipeline_factory.hpp>
#include <vtr_tactic/pipelines/pipelines.hpp>

namespace vtr {
namespace tactic {

/** \brief make a pipeline based on ros configuration */
class ROSPipelineFactory : public PipelineFactory {
 public:
  using NodePtr = rclcpp::Node::SharedPtr;

  /**
   * \brief constructed with ros param info
   * \param[in] node the ros nodehandle with the params
   */
  ROSPipelineFactory(const NodePtr node) : node_(node){};

  /**
   * \brief constructs a module based on ros params
   * \param[in] type_str the type_str trait of the requested module
   */
  BasePipeline::Ptr make(const std::string &param_prefix) const override;

 private:
  static constexpr auto type_field_ = "type";

  void configureStereo(BasePipeline::Ptr &, const std::string &) const;
  void configureLidar(BasePipeline::Ptr &, const std::string &) const;

  const NodePtr node_;
};

}  // namespace tactic
}  // namespace vtr
