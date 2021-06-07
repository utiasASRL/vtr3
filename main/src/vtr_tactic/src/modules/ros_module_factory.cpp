#include <vtr_tactic/modules/ros_module_factory.hpp>

namespace vtr {
namespace tactic {

ROSModuleFactory::ModulePtr ROSModuleFactory::make(
    const std::string &param_prefix) const {
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

}  // namespace tactic
}  // namespace vtr