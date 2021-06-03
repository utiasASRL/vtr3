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

  /** \brief Collection of config parameters */
  struct Config {
    std::vector<std::string> preprocessing;
    std::vector<std::string> odometry;
    std::vector<std::string> localization;
  };

  LidarPipeline(const std::string &name = static_name) : BasePipeline{name} {}

  virtual ~LidarPipeline() {}

  void setConfig(std::shared_ptr<Config> &config) { config_ = config; }

  void initialize(const Graph::Ptr &graph) override;

  void preprocess(QueryCache::Ptr &qdata, const Graph::Ptr &graph) override;

  void runOdometry(QueryCache::Ptr &qdata, const Graph::Ptr &graph) override;

  void visualizeOdometry(QueryCache::Ptr &qdata,
                         const Graph::Ptr &graph) override;

  void runLocalization(QueryCache::Ptr &qdata,
                       const Graph::Ptr &graph) override;

  void visualizeLocalization(QueryCache::Ptr &qdata,
                             const Graph::Ptr &graph) override;

  void processKeyframe(QueryCache::Ptr &qdata, const Graph::Ptr &graph,
                       VertexId live_id) override;

 private:
  /** \brief Pipeline configuration */
  std::shared_ptr<Config> config_ = std::make_shared<Config>();

  std::vector<BaseModule::Ptr> preprocessing_;
  std::vector<BaseModule::Ptr> odometry_;
  std::vector<BaseModule::Ptr> localization_;
};

}  // namespace tactic
}  // namespace vtr
