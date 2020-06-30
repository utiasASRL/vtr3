#pragma once

#include <vtr/navigation/pipelines/branch_pipeline.h>

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

  virtual void convertData(QueryCachePtr q_data, MapCachePtr m_data);
  virtual BasePipeline::KeyframeRequest processData(QueryCachePtr q_data,
                                                    MapCachePtr m_data,
                                                    bool first_frame);
  virtual void localizeKeyFrame(QueryCachePtr q_data, MapCachePtr m_data,
                                bool first_frame);
  virtual void processKeyFrame(QueryCachePtr q_data, MapCachePtr m_data,
                               bool first_frame);
  virtual void processPetiole(QueryCachePtr q_data, MapCachePtr m_data,
                              bool first_frame);
  virtual void makeKeyFrame(QueryCachePtr q_data, MapCachePtr m_data,
                            bool first_frame);
  virtual void wait();

 private:
  bool localization_parallelization_;
  bool localization_skippable_;
  std::future<void> keyframe_thread_future_;
};

}  // namespace navigation
}  // namespace vtr
