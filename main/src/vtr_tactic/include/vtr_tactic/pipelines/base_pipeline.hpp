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

  virtual void configFromROS(const rclcpp::Node::SharedPtr &,
                             const std::string &) {}

  /** \brief initializes the pipeline data */
  virtual void initialize(const Graph::Ptr &) = 0;

  virtual void preprocess(QueryCache::Ptr &, const Graph::Ptr &) = 0;
  virtual void visualizePreprocess(QueryCache::Ptr &, const Graph::Ptr &) {}

  virtual void runOdometry(QueryCache::Ptr &, const Graph::Ptr &) = 0;
  virtual void visualizeOdometry(QueryCache::Ptr &, const Graph::Ptr &) {}

  virtual void runLocalization(QueryCache::Ptr &, const Graph::Ptr &) = 0;
  virtual void visualizeLocalization(QueryCache::Ptr &, const Graph::Ptr &) {}

  virtual void processKeyframe(QueryCache::Ptr &, const Graph::Ptr &,
                               VertexId) = 0;

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
