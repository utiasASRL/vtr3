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
 * \file factory.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 * \brief PipelineFactory & ROSPipelineFactory class definition
 */
#pragma once

#include "vtr_logging/logging.hpp"
#include "vtr_tactic/modules/factory.hpp"
#include "vtr_tactic/pipelines/base_pipeline.hpp"

namespace vtr {
namespace tactic {

/** \brief constructs a pipeline based on a type_str trait */
class PipelineFactoryV2
    : public std::enable_shared_from_this<PipelineFactoryV2> {
 public:
  using Ptr = std::shared_ptr<PipelineFactoryV2>;

  PipelineFactoryV2(const ModuleFactoryV2::Ptr& module_factory =
                        std::make_shared<ModuleFactoryV2>())
      : module_factory_(module_factory) {}

  /**
   * \brief constructs a new or gets a cached pipeline
   * \param token the token used to get the type_str trait (static name) of the
   * pipeline to construct
   * \return a shared_ptr to the constructed pipeline
   */
  virtual BasePipeline::Ptr get(
      const std::string& token,
      const BasePipeline::Config::ConstPtr& config = nullptr) {
    CLOG(DEBUG, "tactic.pipeline") << "Getting pipeline with token: " << token;
    auto iter = cached_pipelines_.find(token);
    if (iter != cached_pipelines_.end()) {
      return iter->second;
    } else {
      auto pipeline = make(token, config);
      cached_pipelines_.emplace(std::make_pair(token, pipeline));
      return pipeline;
    }
  }

  /**
   * \brief makes the requested pipeline matching the type_str trait
   * \return a base pipeline pointer to the derived class, nullptr if not found
   * \throw invalid_argument if the derived pipeline couldn't be found
   */
  virtual BasePipeline::Ptr make(
      const std::string& token,
      const BasePipeline::Config::ConstPtr& config = nullptr) {
    const auto type_str = getTypeStr(token);
    CLOG(DEBUG, "tactic.pipeline")
        << "Constructing pipeline with static name: " << type_str;
    if (!BasePipeline::name2Ctor().count(type_str))
      throw std::invalid_argument(
          "PipelineFactoryV2::make: pipeline type_str not found: " + type_str);

    return BasePipeline::name2Ctor().at(type_str)(config, module_factory_);
  }

 private:
  virtual std::string getTypeStr(const std::string& token) const {
    return token;
  }

 private:
  /** \brief a module factory for pipeline to construct modules */
  const ModuleFactoryV2::Ptr module_factory_;

  /** \brief a map from type_str trait to a pipeline */
  std::unordered_map<std::string, BasePipeline::Ptr> cached_pipelines_;
};

/** \brief make a pipeline based on ros configuration */
class ROSPipelineFactoryV2 : public PipelineFactoryV2 {
 public:
  /** \brief constructed with ros param info */
  ROSPipelineFactoryV2(const rclcpp::Node::SharedPtr& node)
      : PipelineFactoryV2(std::make_shared<ROSModuleFactoryV2>(node)),
        node_(node) {}

  BasePipeline::Ptr make(
      const std::string& param_prefix,
      const BasePipeline::Config::ConstPtr& config = nullptr) override {
    const auto& type_str = getTypeStr(param_prefix);
    if (!BasePipeline::name2Ctor().count(type_str))
      throw std::invalid_argument(
          "PipelineFactoryV2::make: pipeline type_str not found: " + type_str);

    const auto& config_typed =
        config == nullptr
            ? BasePipeline::name2Cfros().at(type_str)(node_, param_prefix)
            : config;
    return PipelineFactoryV2::make(param_prefix, config_typed);
  }

 private:
  std::string getTypeStr(const std::string& param_prefix) const override {
    std::string param_name{param_prefix + "." + type_field_};
    auto type_str =
        node_->has_parameter(param_name)
            ? node_->get_parameter(param_name).get_value<std::string>()
            : node_->declare_parameter<std::string>(param_name, "");
    if (type_str.empty()) {
      auto msg = "No field: '" + param_name + "'";
      LOG(ERROR) << msg;
      throw std::runtime_error(msg);
    }
    return type_str;
  }

 private:
  static constexpr auto type_field_ = "type";
  const rclcpp::Node::SharedPtr node_;
};

}  // namespace tactic
}  // namespace vtr
