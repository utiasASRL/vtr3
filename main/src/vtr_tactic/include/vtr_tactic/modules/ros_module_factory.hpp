#pragma once

#include "rclcpp/rclcpp.hpp"

#include <vtr_tactic/modules/module_factory.hpp>
#include <vtr_tactic/modules/modules.hpp>

namespace vtr {
namespace tactic {

/** \brief make a module based on ros configuration */
class ROSModuleFactory : public ModuleFactory {
 public:
  using NodePtr = rclcpp::Node::SharedPtr;

  /**
   * \brief constructed with ros param info
   * \param[in] node the ros nodehandle with the params
   */
  ROSModuleFactory(const NodePtr node) : node_(node){};

  /**
   * \brief constructs a module based on ros params
   * \param[in] type_str the type_str trait of the requested module
   */
  ModulePtr make(const std::string &param_prefix) const override;

 private:
  static constexpr auto type_field_ = "type";
  const NodePtr node_;
};

}  // namespace tactic
}  // namespace vtr
