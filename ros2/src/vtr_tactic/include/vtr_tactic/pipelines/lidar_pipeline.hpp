#pragma once

#include <vtr_logging/logging.hpp>  // for debugging only
#include <vtr_tactic/caches.hpp>
#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_tactic/modules/module_factory.hpp>
#include <vtr_tactic/pipelines/base_pipeline.hpp>
#include <vtr_tactic/types.hpp>

namespace vtr {
namespace tactic {

class LidarPipeline : public BasePipeline {
 public:
  using Ptr = std::shared_ptr<LidarPipeline>;

  /** \brief Static pipeline identifier. */
  static constexpr auto static_name = "lidar";

  LidarPipeline(const std::string &name = static_name) : BasePipeline{name} {}

  virtual ~LidarPipeline() {}

  void initialize(MapCache::Ptr &mdata, const Graph::Ptr &graph) override;

  void preprocess(QueryCache::Ptr &qdata, MapCache::Ptr &mdata,
                  const Graph::Ptr &graph) override;

  void runOdometry(QueryCache::Ptr &qdata, MapCache::Ptr &mdata,
                   const Graph::Ptr &graph) override;

  void visualizeOdometry(QueryCache::Ptr &qdata, MapCache::Ptr &mdata,
                         const Graph::Ptr &graph) override;

  void runLocalization(QueryCache::Ptr &qdata, MapCache::Ptr &mdata,
                       const Graph::Ptr &graph) override;

  void visualizeLocalization(QueryCache::Ptr &qdata, MapCache::Ptr &mdata,
                             const Graph::Ptr &graph) override;

  void processKeyframe(QueryCache::Ptr &qdata, MapCache::Ptr &mdata,
                        const Graph::Ptr &graph, VertexId live_id) override;

 private:
  BaseModule::Ptr preprocessing_module_;
  BaseModule::Ptr recall_module_;
  BaseModule::Ptr odometry_icp_module_;
  BaseModule::Ptr keyframe_test_module_;
  BaseModule::Ptr map_maintenance_module_;
  BaseModule::Ptr windowed_recall_module_;
  BaseModule::Ptr localization_icp_module_;
};

}  // namespace tactic
}  // namespace vtr
