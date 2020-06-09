#pragma once

#include <vtr/navigation/pipelines.h>

#include <vtr/navigation/tactics/state_machine_interface.h>

namespace asrl {
namespace navigation {

class BasicTactic;

class PipelineFactory {
 public:
  static inline std::shared_ptr<BasePipeline> make(
      planning::PipelineType pipeline, BasicTactic* tactic) {
    using asrl::planning::PipelineType;
#pragma GCC diagnostic ignored "-Wswitch"
    switch (pipeline) {
      case PipelineType::VisualOdometry:
        return std::make_shared<BranchPipeline>(tactic);
#if 0
      case PipelineType::Idle:
        return std::make_shared<IdlePipeline>(tactic);
      case PipelineType::MetricLocalization:
        return std::make_shared<MetricLocalizationPipeline>(tactic);
      case PipelineType::LocalizationSearch:
        return std::make_shared<LocalizationSearchPipeline>(tactic);
      case PipelineType::Merge:
        return std::make_shared<MergePipeline>(tactic);
      case PipelineType::Transition:
        return std::make_shared<TransitionPipeline>(tactic);
#endif
    }
#pragma GCC diagnostic pop

    // GCC throws a warning without this line, despite the fact the we handle
    // all values of an Enum Class above...
    return nullptr;
  }
};

}  // namespace navigation
}  // namespace asrl
