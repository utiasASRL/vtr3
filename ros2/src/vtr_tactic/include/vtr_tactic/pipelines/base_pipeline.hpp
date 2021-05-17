#pragma once

#include <mutex>

#include <vtr_logging/logging.hpp>  // for debugging only
#include <vtr_tactic/caches.hpp>
#include <vtr_tactic/modules/module_factory.hpp>
#include <vtr_tactic/types.hpp>

namespace vtr {
namespace tactic {

class BasePipeline {
 public:
  using Ptr = std::shared_ptr<BasePipeline>;

  /** \brief An unique identifier. Subclass should overwrite this. */
  static constexpr auto static_name = "base";

  BasePipeline(const std::string &name = static_name) : name_{name} {}

  virtual ~BasePipeline() {}

  /**
   * \brief Get the identifier of the pipeline instance at runtime.
   * \details The identifier is the string passed to the BasePipeline
   * constructor.
   */
  const std::string &name() const { return name_; };

  void setModuleFactory(const ModuleFactory::Ptr &module_factory) {
    module_factory_ = module_factory;
  }

  /** \brief initializes the pipeline data */
  virtual void initialize(const Graph::Ptr &graph) = 0;

  virtual void preprocess(QueryCache::Ptr &qdata, const Graph::Ptr &graph) = 0;

  virtual void runOdometry(QueryCache::Ptr &qdata, const Graph::Ptr &graph) = 0;
  virtual void visualizeOdometry(QueryCache::Ptr &qdata,
                                 const Graph::Ptr &graph) = 0;

  virtual void runLocalization(QueryCache::Ptr &qdata,
                               const Graph::Ptr &graph) = 0;

  virtual void visualizeLocalization(QueryCache::Ptr &qdata,

                                     const Graph::Ptr &graph) = 0;

  virtual void processKeyframe(QueryCache::Ptr &qdata, const Graph::Ptr &graph,
                               VertexId live_id) = 0;

  virtual void waitForKeyframeJob() {}

 protected:
  /** \brief Module factory instance to help modularize code. */
  ModuleFactory::Ptr module_factory_;

 private:
  /** \brief Name of the module assigned at runtime. */
  const std::string name_;
};

}  // namespace tactic
}  // namespace vtr
