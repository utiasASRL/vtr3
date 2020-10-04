#pragma once

#include <stdexcept>

#include <vtr_mission_planning/state_machine_interface.hpp>
#include <vtr_navigation/pipelines.hpp>

namespace vtr {
namespace navigation {

class BasicTactic;

class PipelineFactory {
 public:
  static inline std::shared_ptr<BasePipeline> make(
      mission_planning::PipelineType pipeline, BasicTactic* tactic) {
    using mission_planning::PipelineType;
#pragma GCC diagnostic ignored "-Wswitch"
    switch (pipeline) {
      case PipelineType::Idle:
        return std::make_shared<IdlePipeline>(tactic);
      case PipelineType::VisualOdometry:
        return std::make_shared<BranchPipeline>(tactic);
      case PipelineType::MetricLocalization:
        return std::make_shared<MetricLocalizationPipeline>(tactic);
#if 0
      case PipelineType::LocalizationSearch:
        return std::make_shared<LocalizationSearchPipeline>(tactic);
      case PipelineType::Merge:
        return std::make_shared<MergePipeline>(tactic);
      case PipelineType::Transition:
        return std::make_shared<TransitionPipeline>(tactic);
#endif
      default:
        throw std::invalid_argument("Unknown pipeline type.");
    }
#pragma GCC diagnostic pop

    // GCC throws a warning without this line, despite the fact the we handle
    // all values of an Enum Class above...
    return nullptr;
  }
};

}  // namespace navigation
}  // namespace vtr
