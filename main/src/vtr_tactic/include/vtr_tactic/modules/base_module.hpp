// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file base_module.hpp
 * \brief BaseModule class definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <boost/uuid/uuid.hpp>

#include "vtr_common/timing/simple_timer.hpp"
#include "vtr_logging/logging.hpp"
#include "vtr_tactic/cache.hpp"
#include "vtr_tactic/types.hpp"

namespace vtr {
namespace tactic {

class ModuleFactory;
class TaskExecutor;

class BaseModule : public std::enable_shared_from_this<BaseModule> {
 public:
  using Ptr = std::shared_ptr<BaseModule>;

  /** \brief An unique identifier. Subclass should overwrite this. */
  static constexpr auto static_name = "module";

  BaseModule(const std::string &name = static_name) : name_{name} {}

  virtual ~BaseModule() {}

  /**
   * \brief Gets the identifier of the module instance at runtime.
   * \details The identifier is the string passed to the BaseModule constructor.
   */
  const std::string &getName() const { return name_; }

  /** \brief Initializes the module with timing. */
  void initialize(const Graph::ConstPtr &graph) {
    CLOG(DEBUG, "tactic.module")
        << "\033[1;31mInitializing module: " << getName() << "\033[0m";
    timer.reset();
    initializeImpl(graph);
    CLOG(DEBUG, "tactic.module")
        << "Finished initializing module: " << getName() << ", which takes "
        << timer;
  }

  /** \brief Runs the module with timing. */
  void run(QueryCache &qdata, const Graph::Ptr &graph,
           const std::shared_ptr<TaskExecutor> &executor) {
    CLOG(DEBUG, "tactic.module")
        << "\033[1;31mRunning module: " << getName() << "\033[0m";
    timer.reset();
    runImpl(qdata, graph, executor);
    CLOG(DEBUG, "tactic.module") << "Finished running module: " << getName()
                                 << ", which takes " << timer;
  }

  /** \brief Runs the module asynchronously with timing. */
  void runAsync(QueryCache &qdata, const Graph::Ptr &graph,
                const std::shared_ptr<TaskExecutor> &executor,
                const size_t &priority, const boost::uuids::uuid &dep_id) {
    CLOG(DEBUG, "tactic.module")
        << "\033[1;31mRunning module (async): " << getName() << "\033[0m";
    timer.reset();
    runAsyncImpl(qdata, graph, executor, priority, dep_id);
    CLOG(DEBUG, "tactic.module")
        << "Finished running module (async): " << getName() << ", which takes "
        << timer;
  }

  /** \brief Updates the live vertex in pose graph with timing. */
  void updateGraph(QueryCache &qdata, const Graph::Ptr &graph,
                   VertexId live_id) {
    CLOG(DEBUG, "tactic.module")
        << "\033[1;32mUpdating graph module: " << getName() << "\033[0m";
    timer.reset();
    updateGraphImpl(qdata, graph, live_id);
    CLOG(DEBUG, "tactic.module")
        << "Finished updating graph module: " << getName() << ", which takes "
        << timer;
  }

  /** \brief Visualizes data in this module. */
  void visualize(QueryCache &qdata, const Graph::ConstPtr &graph) {
    CLOG(DEBUG, "tactic.module")
        << "\033[1;33mVisualizing module: " << getName() << "\033[0m";
    timer.reset();
    visualizeImpl(qdata, graph);
    CLOG(DEBUG, "tactic.module") << "Finished visualizing module: " << getName()
                                 << ", which takes " << timer;
  }

  /** \brief Visualizes data in this module. */
  virtual void configFromROS(const rclcpp::Node::SharedPtr &,
                             const std::string) {}

  void setFactory(const std::shared_ptr<ModuleFactory> &factory) {
    factory_ = factory;
  }

  const std::shared_ptr<ModuleFactory> &getFactory() const {
    if (factory_ == nullptr)
      throw std::runtime_error{"Module factory is a nullptr."};
    return factory_;
  }
#if false
 protected:
  template <typename Derived>
  std::shared_ptr<Derived> shared_from_base() {
    return std::static_pointer_cast<Derived>(shared_from_this());
  }
#endif
 private:
  /** \brief Initializes the module. */
  virtual void initializeImpl(const Graph::ConstPtr &) {}

  /** \brief Runs the module. */
  virtual void runImpl(QueryCache &, const Graph::Ptr &,
                       const std::shared_ptr<TaskExecutor> &) = 0;

  /** \brief Runs the module asynchronously. */
  virtual void runAsyncImpl(QueryCache &, const Graph::Ptr &,
                            const std::shared_ptr<TaskExecutor> &,
                            const size_t &, const boost::uuids::uuid &) {}

  /**
   * \brief Updates the live vertex in pose graph.
   * \note DEPRECATED: avoid using this function - use runImpl/runAsyncImpl
   */
  virtual void updateGraphImpl(QueryCache &, const Graph::Ptr &, VertexId) {}

  /**
   * \brief Visualization
   * \note DEPRECATED: avoid using this function - use runImpl/runAsyncImpl
   */
  virtual void visualizeImpl(QueryCache &, const Graph::ConstPtr &) {}

 protected:
  /** \brief Pointer to a module factory used to get other modules. */
  std::shared_ptr<ModuleFactory> factory_ = nullptr;

 private:
  /** \brief Name of the module assigned at runtime. */
  const std::string name_;

  /** \brief A timer that times execution of each module */
  common::timing::SimpleTimer timer;
};

}  // namespace tactic
}  // namespace vtr
