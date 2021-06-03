#pragma once

#include <memory>

#include <vtr_logging/logging.hpp>  // for debugging only
#include <vtr_tactic/factory.hpp>
#include <vtr_tactic/pipelines/base_pipeline.hpp>
#include <vtr_tactic/pipelines/pipelines.hpp>

namespace vtr {
namespace tactic {

/** \brief constructs a pipeline based on a static_name trait */
class PipelineFactory {
 public:
  using Ptr = std::shared_ptr<PipelineFactory>;

  /** \brief constructed to build a particular pipeline */
  PipelineFactory() {
    type_switch_.add<TemplatePipeline>();
    type_switch_.add<StereoPipeline>();
    type_switch_.add<LidarPipeline>();
  };

  /**
   * \brief makes the requested pipeline matching the static_name trait
   * \return a base pipeline pointer to the derived class, nullptr if not found
   * \throw invalid_argument if the derived pipeline couldn't be found
   */
  virtual BasePipeline::Ptr make(const std::string& static_name) const;

 protected:
  /**
   * \brief compare static_name traits, helper for factories who want more
   * control
   * \param[in] static_name the static name trait that might match
   * derived class D \return true if the trait is a match
   */
  template <class Derived>
  static bool isType(const std::string& static_name) {
    return static_name.compare(Derived::static_name) == 0;
  }

 private:
  FactoryTypeSwitch<BasePipeline> type_switch_;
};

}  // namespace tactic
}  // namespace vtr
