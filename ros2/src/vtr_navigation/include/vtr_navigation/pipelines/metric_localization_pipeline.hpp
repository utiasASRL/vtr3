#pragma once

#include <vtr_navigation/pipelines/branch_pipeline.hpp>

namespace vtr {
namespace navigation {

class MetricLocalizationPipeline : public BranchPipeline {
 public:
  PTR_TYPEDEFS(MetricLocalizationPipeline)

  MetricLocalizationPipeline(BasicTactic* _tactic)
      : localization_parallelization_(
            _tactic->config().localization_parallelization),
        localization_skippable_(_tactic->config().localization_skippable),
        BranchPipeline(_tactic) {}
#if false
  void convertData(QueryCachePtr q_data, MapCachePtr m_data) override;
  BasePipeline::KeyframeRequest processData(QueryCachePtr q_data,
                                            MapCachePtr m_data,
                                            bool first_frame) override;
  void localizeKeyFrame(QueryCachePtr q_data, MapCachePtr m_data,
                        bool first_frame) override;
  void processKeyFrame(QueryCachePtr q_data, MapCachePtr m_data,
                       bool first_frame) override;
  void processPetiole(QueryCachePtr q_data, MapCachePtr m_data,
                      bool first_frame) override;
  void makeKeyFrame(QueryCachePtr q_data, MapCachePtr m_data,
                    bool first_frame) override;
  void wait() override;
#endif
 private:
  bool localization_parallelization_;
  bool localization_skippable_;
  std::future<void> keyframe_thread_future_;
};

}  // namespace navigation
}  // namespace vtr
