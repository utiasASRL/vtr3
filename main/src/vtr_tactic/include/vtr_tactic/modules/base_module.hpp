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

#include "rclcpp/rclcpp.hpp"

#include "vtr_common/timing/simple_timer.hpp"
#include "vtr_logging/logging.hpp"
#include "vtr_tactic/cache.hpp"
#include "vtr_tactic/types.hpp"

namespace vtr {
namespace tactic {

class ModuleFactoryV2;
class TaskExecutor;

class BaseModule : public std::enable_shared_from_this<BaseModule> {
 public:
  using Ptr = std::shared_ptr<BaseModule>;

  /** \brief An unique identifier. Subclass should overwrite this. */
  static constexpr auto static_name = "base_module";

  struct Config {
    using Ptr = std::shared_ptr<Config>;
    using ConstPtr = std::shared_ptr<const Config>;

    virtual ~Config() = default;  // for polymorphism

    /// sub-module must implement this function
    static Ptr fromROS(const rclcpp::Node::SharedPtr &, const std::string &);
  };

  BaseModule(const std::shared_ptr<ModuleFactoryV2> &module_factory = nullptr,
             const std::string &name = static_name)
      : module_factory_{module_factory}, name_{name} {}

  virtual ~BaseModule() {}

  /**
   * \brief Gets the identifier of the module instance at runtime.
   * \details The identifier is the string passed to the BaseModule constructor.
   */
  const std::string &name() const { return name_; }

  /** \brief Initializes the module with timing. */
  void initialize(OutputCache &output, const Graph::ConstPtr &graph) {
    CLOG(DEBUG, "tactic.module")
        << "\033[1;31mInitializing module: " << name() << "\033[0m";
    timer.reset();
    initializeImpl(output, graph);
    CLOG(DEBUG, "tactic.module") << "Finished initializing module: " << name()
                                 << ", which takes " << timer;
  }

  /** \brief Runs the module with timing. */
  void run(QueryCache &qdata, OutputCache &output, const Graph::Ptr &graph,
           const std::shared_ptr<TaskExecutor> &executor) {
    CLOG(DEBUG, "tactic.module")
        << "\033[1;31mRunning module: " << name() << "\033[0m";
    timer.reset();
    runImpl(qdata, output, graph, executor);
    CLOG(DEBUG, "tactic.module")
        << "Finished running module: " << name() << ", which takes " << timer;
  }

  /** \brief Runs the module asynchronously with timing. */
  void runAsync(QueryCache &qdata, OutputCache &output, const Graph::Ptr &graph,
                const std::shared_ptr<TaskExecutor> &executor,
                const size_t &priority, const boost::uuids::uuid &dep_id) {
    CLOG(DEBUG, "tactic.module")
        << "\033[1;31mRunning module (async): " << name() << "\033[0m";
    runAsyncImpl(qdata, output, graph, executor, priority, dep_id);
    CLOG(DEBUG, "tactic.module")
        << "Finished running module (async): " << name();
  }

  /** \brief Updates the live vertex in pose graph with timing. */
  void updateGraph(QueryCache &qdata, OutputCache &output,
                   const Graph::Ptr &graph) {
    CLOG(DEBUG, "tactic.module")
        << "\033[1;32mUpdating graph module: " << name() << "\033[0m";
    timer.reset();
    updateGraphImpl(qdata, output, graph);
    CLOG(DEBUG, "tactic.module") << "Finished updating graph module: " << name()
                                 << ", which takes " << timer;
  }

  /** \brief Visualizes data in this module. */
  void visualize(QueryCache &qdata, OutputCache &output,
                 const Graph::ConstPtr &graph) {
    CLOG(DEBUG, "tactic.module")
        << "\033[1;33mVisualizing module: " << name() << "\033[0m";
    timer.reset();
    visualizeImpl(qdata, output, graph);
    CLOG(DEBUG, "tactic.module") << "Finished visualizing module: " << name()
                                 << ", which takes " << timer;
  }

 protected:
  const std::shared_ptr<ModuleFactoryV2> &factory() const {
    if (module_factory_ == nullptr)
      throw std::runtime_error{"Module factory is a nullptr."};
    return module_factory_;
  }

 private:
  /** \brief Initializes the module. */
  virtual void initializeImpl(OutputCache &, const Graph::ConstPtr &) {}

  /** \brief Runs the module. */
  virtual void runImpl(QueryCache &, OutputCache &, const Graph::Ptr &,
                       const std::shared_ptr<TaskExecutor> &) = 0;

  /** \brief Runs the module asynchronously. */
  virtual void runAsyncImpl(QueryCache &, OutputCache &, const Graph::Ptr &,
                            const std::shared_ptr<TaskExecutor> &,
                            const size_t &, const boost::uuids::uuid &) {}

  /**
   * \brief Updates the live vertex in pose graph.
   * \note DEPRECATED: avoid using this function - use runImpl/runAsyncImpl
   */
  virtual void updateGraphImpl(QueryCache &, OutputCache &,
                               const Graph::Ptr &) {}

  /**
   * \brief Visualization
   * \note DEPRECATED: avoid using this function - use runImpl/runAsyncImpl
   */
  virtual void visualizeImpl(QueryCache &, OutputCache &,
                             const Graph::ConstPtr &) {}

 private:
  const std::shared_ptr<ModuleFactoryV2> module_factory_;

  /** \brief Name of the module assigned at runtime. */
  const std::string name_;

  /** \brief A timer that times execution of each module */
  common::timing::SimpleTimer timer;

  /// factory handlers (note: local static variable constructed on first use)
 private:
  /** \brief a map from type_str trait to a constructor function */
  using CtorFunc = std::function<Ptr(const Config::ConstPtr &,
                                     const std::shared_ptr<ModuleFactoryV2> &)>;
  using Name2Ctor = std::unordered_map<std::string, CtorFunc>;
  static Name2Ctor &name2Ctor() {
    static Name2Ctor name2ctor;
    return name2ctor;
  }

  /** \brief a map from type_str trait to a config from ROS function */
  using CfROSFunc = std::function<Config::ConstPtr(
      const rclcpp::Node::SharedPtr &, const std::string &)>;
  using Name2CfROS = std::unordered_map<std::string, CfROSFunc>;
  static Name2CfROS &name2Cfros() {
    static Name2CfROS name2cfros;
    return name2cfros;
  }

  template <typename T>
  friend class ModuleRegister;
  friend class ModuleFactoryV2;
  friend class ROSModuleFactoryV2;
};

template <typename T>
struct ModuleRegister {
  ModuleRegister() {
    bool success = true;
    success &=
        BaseModule::name2Ctor()
            .try_emplace(
                T::static_name,
                BaseModule::CtorFunc(
                    [](const BaseModule::Config::ConstPtr &config,
                       const std::shared_ptr<ModuleFactoryV2> &factory) {
                      const auto &config_typed =
                          (config == nullptr
                               ? std::make_shared<const typename T::Config>()
                               : std::dynamic_pointer_cast<
                                     const typename T::Config>(config));
                      return std::make_shared<T>(config_typed, factory);
                    }))
            .second;
    success &=
        BaseModule::name2Cfros()
            .try_emplace(
                T::static_name,
                BaseModule::CfROSFunc([](const rclcpp::Node::SharedPtr &node,
                                         const std::string &prefix) {
                  return T::Config::fromROS(node, prefix);
                }))
            .second;
    if (!success)
      throw std::runtime_error{"ModuleRegister failed - duplicated name"};
  }
};

/// \brief Register a module
#define VTR_REGISTER_MODULE_DEC_TYPE(NAME) \
  inline static vtr::tactic::ModuleRegister<NAME> reg_

}  // namespace tactic
}  // namespace vtr
