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
 * \file module_factory.hpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <memory>

#include "vtr_logging/logging.hpp"  // for debugging only
#include "vtr_tactic/factory.hpp"
#include "vtr_tactic/modules/base_module.hpp"
#if false
#include "vtr_tactic/modules/modules.hpp"
#endif

namespace vtr {
namespace tactic {

/** \brief constructs a module based on a type_str trait */
class ModuleFactory : public std::enable_shared_from_this<ModuleFactory> {
 public:
  using Ptr = std::shared_ptr<ModuleFactory>;

  /** \brief constructed to build a particular module */
  ModuleFactory() {
    // add generic modules
#if false
    type_switch_.add<TemplateModule>();
    type_switch_.add<LiveMemManagerModule>();
    type_switch_.add<GraphMemManagerModule>();
#endif
  }

  template <class DerivedModule>
  void add() {
    type_switch_.add<DerivedModule>();
  }

  /**
   * \brief constructs a new or gets a cached module
   * \param token the token used to get the type_str trait (static name) of the
   * module to construct
   * \return a shared_ptr to the constructed module
   */
  virtual BaseModule::Ptr get(const std::string& token) {
    CLOG(DEBUG, "tactic.module") << "Getting module with token: " << token;
    auto iter = cached_modules_.find(token);
    if (iter != cached_modules_.end()) {
      return iter->second;
    } else {
      auto module = make(token);
      cached_modules_.emplace(std::make_pair(token, module));
      return module;
    }
  }

  /**
   * \brief makes the requested module matching the type_str trait
   * \return a base module pointer to the derived class, nullptr if not
   * found \throw invalid_argument if the derived module couldn't be found
   */
  virtual BaseModule::Ptr make(const std::string& token) {
    const auto type_str = getTypeStr(token);
    CLOG(DEBUG, "tactic.module")
        << "Constructing module with static name: " << type_str;
    auto module = type_switch_.make(type_str);
    if (module == nullptr) {
      auto msg = "Unknown module of type: " + type_str;
      CLOG(ERROR, "module") << msg;
      throw std::invalid_argument(msg);
    }
    module->setFactory(shared_from_this());
    return module;
  }

 private:
  virtual std::string getTypeStr(const std::string& token) const {
    return token;
  }

 private:
  /** \brief a factory for modules (default constructs) */
  FactoryTypeSwitch<BaseModule> type_switch_;
  /** \brief a map from type_str trait to a module */
  std::unordered_map<std::string, BaseModule::Ptr> cached_modules_;
};

/** \brief make a module based on ros configuration */
class ROSModuleFactory : public ModuleFactory {
 public:
  /** \brief constructed with ros param info */
  ROSModuleFactory(const rclcpp::Node::SharedPtr& node) : node_(node) {}

  BaseModule::Ptr make(const std::string& param_prefix) override {
    auto module = ModuleFactory::make(param_prefix);
    module->configFromROS(node_, param_prefix);
    return module;
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
