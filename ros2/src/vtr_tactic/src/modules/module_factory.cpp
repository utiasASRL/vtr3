#include <vtr_tactic/modules/module_factory.hpp>

namespace vtr {
namespace tactic {

ModuleFactory::ModulePtr ModuleFactory::make(
    const std::string& type_str) const {
  LOG(DEBUG) << "Constructing module with static name: " << type_str;
  auto module = type_switch_.make(type_str);
  if (!module) {
    auto msg = "Unknown module of type: " + type_str;
    LOG(ERROR) << msg;
    throw std::invalid_argument(msg);
  }
  return module;
}

}  // namespace tactic
}  // namespace vtr
