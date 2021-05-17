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
  void initialize(const Graph::Ptr &graph) override {}

  void preprocess(QueryCache::Ptr &qdata, const Graph::Ptr &graph) override {}

  void runOdometry(QueryCache::Ptr &qdata, const Graph::Ptr &graph) override {}
  void visualizeOdometry(QueryCache::Ptr &qdata,
                         const Graph::Ptr &graph) override {}

  void runLocalization(QueryCache::Ptr &qdata,
                       const Graph::Ptr &graph) override {}

  void visualizeLocalization(QueryCache::Ptr &qdata,
                             const Graph::Ptr &graph) override {}

  void processKeyframe(QueryCache::Ptr &qdata, const Graph::Ptr &graph,
                       VertexId live_id) override {}
};

}  // namespace tactic
}  // namespace vtr
