#pragma once

#import <vtr_navigation/pipelines/merge_pipeline.hpp>

namespace vtr {
namespace navigation {

class LocalizationSearchPipeline : public MergePipeline {
 public:
  PTR_TYPEDEFS(LocalizationSearchPipeline)

  LocalizationSearchPipeline(BasicTactic *_tactic)
      : MergePipeline(_tactic),
        searchStep_(_tactic->config().pipeline_config.loc_search_step),
        searchWidth_(2 * searchStep_),
        lastOffset_(0) {}

 protected:
  /// Where to move the trunk vertex when we encounter a localization failure
  virtual uint32_t _getNextTrunkSeq();

  /// Update the localization with respect to the current run
  virtual void _updateRunLoc(QueryCachePtr q_data, MapCachePtr m_data);

  /// Update the localization with respect to the privileged chain
  virtual void _updateTrunkLoc();

  const uint32_t searchStep_;
  uint32_t searchWidth_;
  int32_t lastOffset_;
};

}  // namespace navigation
}  // namespace vtr
