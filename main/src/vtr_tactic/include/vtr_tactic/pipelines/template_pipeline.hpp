#pragma once

#include <vtr_tactic/pipelines/base_pipeline.hpp>

namespace vtr {
namespace tactic {

class TemplatePipeline : public BasePipeline {
 public:
  using Ptr = std::shared_ptr<TemplatePipeline>;

  /** \brief Static pipeline identifier. */
  static constexpr auto static_name = "template";

  TemplatePipeline(const std::string &name = static_name)
      : BasePipeline{name} {}

  virtual ~TemplatePipeline() {}

  /** \brief initializes the pipeline data */
  void initialize(const Graph::Ptr &) override {}

  void preprocess(QueryCache::Ptr &, const Graph::Ptr &) override {}

  void runOdometry(QueryCache::Ptr &, const Graph::Ptr &) override {}
  void visualizeOdometry(QueryCache::Ptr &, const Graph::Ptr &) override {}

  void runLocalization(QueryCache::Ptr &, const Graph::Ptr &) override {}

  void visualizeLocalization(QueryCache::Ptr &, const Graph::Ptr &) override {}

  void processKeyframe(QueryCache::Ptr &, const Graph::Ptr &,
                       VertexId) override {}
};

}  // namespace tactic
}  // namespace vtr
