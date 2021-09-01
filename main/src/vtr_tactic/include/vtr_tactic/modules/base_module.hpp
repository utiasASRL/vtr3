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
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <mutex>

#include <vtr_common/timing/simple_timer.hpp>
#include <vtr_logging/logging.hpp>
#include <vtr_tactic/cache.hpp>
#include <vtr_tactic/types.hpp>

namespace vtr {
namespace tactic {

class BaseModule {
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
  void run(QueryCache &qdata, const Graph::ConstPtr &graph) {
    CLOG(DEBUG, "tactic.module")
        << "\033[1;31mRunning module: " << getName() << "\033[0m";
    timer.reset();
    runImpl(qdata, graph);
    CLOG(DEBUG, "tactic.module") << "Finished running module: " << getName()
                                 << ", which takes " << timer;
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

 private:
  /** \brief Initializes the module. */
  virtual void initializeImpl(const Graph::ConstPtr &) {}

  /** \brief Runs the module. */
  virtual void runImpl(QueryCache &qdata, const Graph::ConstPtr &graph) = 0;

  /** \brief Updates the live vertex in pose graph. */
  virtual void updateGraphImpl(QueryCache &, const Graph::Ptr &, VertexId) {}

  /** \brief Visualization */
  virtual void visualizeImpl(QueryCache &, const Graph::ConstPtr &) {}

 private:
  /** \brief Name of the module assigned at runtime. */
  const std::string name_;

  /** \brief A timer that times execution of each module */
  common::timing::SimpleTimer timer;
};

}  // namespace tactic
}  // namespace vtr
