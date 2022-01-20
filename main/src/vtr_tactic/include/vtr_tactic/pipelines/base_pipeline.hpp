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
 * \file base_pipeline.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "rclcpp/rclcpp.hpp"

#include "vtr_common/timing/stopwatch.hpp"
#include "vtr_logging/logging.hpp"
#include "vtr_tactic/cache.hpp"
#include "vtr_tactic/types.hpp"

namespace vtr {
namespace tactic {

class ModuleFactory;
class TaskExecutor;

class BasePipeline {
 public:
  PTR_TYPEDEFS(BasePipeline);

  /** \brief An unique identifier. Subclass should overwrite this. */
  static constexpr auto static_name = "base_pipeline";

  struct Config {
    PTR_TYPEDEFS(Config);

    virtual ~Config() = default;  // for polymorphism

    /// sub-module must implement this function
    static Ptr fromROS(const rclcpp::Node::SharedPtr &, const std::string &);
  };

  BasePipeline(const std::shared_ptr<ModuleFactory> &module_factory = nullptr,
               const std::string &name = static_name);

  virtual ~BasePipeline() = default;

  /**
   * \brief Get the identifier of the pipeline instance at runtime.
   * \details The identifier is the string passed to the BasePipeline
   * constructor.
   */
  const std::string &name() const { return name_; }

  virtual OutputCache::Ptr createOutputCache() const;

  void initialize(const OutputCache::Ptr &output, const Graph::Ptr &graph);
  void preprocess(const QueryCache::Ptr &qdata, const OutputCache::Ptr &output,
                  const Graph::Ptr &graph,
                  const std::shared_ptr<TaskExecutor> &executor);
  void runOdometry(const QueryCache::Ptr &qdata, const OutputCache::Ptr &output,
                   const Graph::Ptr &graph,
                   const std::shared_ptr<TaskExecutor> &executor);
  void runLocalization(const QueryCache::Ptr &qdata,
                       const OutputCache::Ptr &output, const Graph::Ptr &graph,
                       const std::shared_ptr<TaskExecutor> &executor);
  void processKeyframe(const QueryCache::Ptr &qdata,
                       const OutputCache::Ptr &output, const Graph::Ptr &graph,
                       const std::shared_ptr<TaskExecutor> &executor);

  /** \brief Waits until all internal threads of a pipeline finishes. */
  virtual void wait() {}

  /** \brief Resets internal state of a pipeline when a new run starts. */
  virtual void reset() {}

 private:
  virtual void initialize_(const OutputCache::Ptr &, const Graph::Ptr &) {}
  virtual void preprocess_(const QueryCache::Ptr &, const OutputCache::Ptr &,
                           const Graph::Ptr &,
                           const std::shared_ptr<TaskExecutor> &) = 0;
  virtual void runOdometry_(const QueryCache::Ptr &, const OutputCache::Ptr &,
                            const Graph::Ptr &,
                            const std::shared_ptr<TaskExecutor> &) = 0;
  virtual void runLocalization_(const QueryCache::Ptr &,
                                const OutputCache::Ptr &, const Graph::Ptr &,
                                const std::shared_ptr<TaskExecutor> &) = 0;
  virtual void processKeyframe_(const QueryCache::Ptr &,
                                const OutputCache::Ptr &, const Graph::Ptr &,
                                const std::shared_ptr<TaskExecutor> &) = 0;

 protected:
  std::shared_ptr<ModuleFactory> factory() const;

 private:
  const std::shared_ptr<ModuleFactory> module_factory_;

  /** \brief Name of the module assigned at runtime. */
  const std::string name_;

  /// factory handlers (note: local static variable constructed on first use)
 private:
  /** \brief a map from type_str trait to a constructor function */
  using CtorFunc = std::function<Ptr(const Config::ConstPtr &,
                                     const std::shared_ptr<ModuleFactory> &)>;
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
  friend class PipelineRegister;
  friend class PipelineFactory;
  friend class ROSPipelineFactory;
};

template <typename T>
struct PipelineRegister {
  PipelineRegister() {
    bool success = true;
    success &=
        BasePipeline::name2Ctor()
            .try_emplace(
                T::static_name,
                BasePipeline::CtorFunc(
                    [](const BasePipeline::Config::ConstPtr &config,
                       const std::shared_ptr<ModuleFactory> &factory) {
                      const auto &config_typed =
                          (config == nullptr
                               ? std::make_shared<const typename T::Config>()
                               : std::dynamic_pointer_cast<
                                     const typename T::Config>(config));
                      return std::make_shared<T>(config_typed, factory);
                    }))
            .second;
    success &=
        BasePipeline::name2Cfros()
            .try_emplace(
                T::static_name,
                BasePipeline::CfROSFunc([](const rclcpp::Node::SharedPtr &node,
                                           const std::string &prefix) {
                  return T::Config::fromROS(node, prefix);
                }))
            .second;
    if (!success)
      throw std::runtime_error{"PipelineRegister failed - duplicated name"};
  }
};

/// \brief Register a pipeline
/// \todo probably need to add a dummy use of this variable for initialization
#define VTR_REGISTER_PIPELINE_DEC_TYPE(NAME) \
  inline static vtr::tactic::PipelineRegister<NAME> reg_
}  // namespace tactic
}  // namespace vtr
