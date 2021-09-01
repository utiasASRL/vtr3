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
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <memory>

#include <vtr_logging/logging.hpp>  // for debugging only
#include <vtr_tactic/factory.hpp>
#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_tactic/modules/template_module.hpp>

namespace vtr {
namespace tactic {

/** \brief constructs a module based on a type_str trait */
class ModuleFactory {
 public:
  using Ptr = std::shared_ptr<ModuleFactory>;
  using Module = BaseModule;
  using ModulePtr = std::shared_ptr<BaseModule>;

  /** \brief constructed to build a particular module */
  ModuleFactory() { type_switch_.add<TemplateModule>(); }

  template <class DerivedModule>
  void add() {
    type_switch_.add<DerivedModule>();
  }

  /**
   * \brief makes the requested module matching the type_str trait
   * \return a base module pointer to the derived class, nullptr if not found
   * \throw invalid_argument if the derived module couldn't be found
   */
  virtual ModulePtr make(const std::string& type_str) const {
    LOG(DEBUG) << "Constructing module with static name: " << type_str;
    auto module = type_switch_.make(type_str);
    if (!module) {
      auto msg = "Unknown module of type: " + type_str;
      LOG(ERROR) << msg;
      throw std::invalid_argument(msg);
    }
    return module;
  }

 private:
  FactoryTypeSwitch<Module> type_switch_;
};

/** \brief make a module based on ros configuration */
class ROSModuleFactory : public ModuleFactory {
 public:
  using NodePtr = rclcpp::Node::SharedPtr;

  /**
   * \brief constructed with ros param info
   * \param[in] node the ros nodehandle with the params
   */
  ROSModuleFactory(const NodePtr node) : node_(node){};

  /** \brief constructs a module based on ros params */
  ModulePtr make(const std::string& param_prefix) const override {
    std::string param_name{param_prefix + "." + type_field_};
    auto type_str = node_->declare_parameter<std::string>(param_name, "");
    if (type_str.empty()) {
      auto msg = "No field: '" + param_name + "'";
      LOG(ERROR) << msg;
      throw std::runtime_error(msg);
    }
    auto module = ModuleFactory::make(type_str);
    module->configFromROS(node_, param_prefix);

    return module;
  }

 private:
  static constexpr auto type_field_ = "type";
  const NodePtr node_;
};

}  // namespace tactic
}  // namespace vtr
