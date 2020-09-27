#pragma once

#include <vtr_navigation/factories/assembly_factory.hpp>
#include <vtr_navigation/factories/ros_base_factory.hpp>
#include <vtr_navigation/factories/ros_module_factory.hpp>

namespace vtr {
namespace navigation {

/** \brief makes an assembly (and inner modules) based on ros params */
class ROSAssemblyFactory : public ROSBaseFactory<BaseAssembly> {
 public:
  using base_t = ROSBaseFactory<BaseAssembly>;
  using assy_ptr = base_t::T_ptr;

  /** \brief constructor with ros info detailing the assembly */
  ROSAssemblyFactory(const NodePtr node, const std::string& param_prefix)
      : base_t(node, param_prefix) {}

 private:
  /** \brief builder that constructs an assembly, which is a collection of
   * modules
   */
  assy_ptr make_str(const std::string& type_str) const {
    // make an empty assembly using the default builder
    auto new_assembly = AssemblyFactory{type_str}.make();
    if (!new_assembly) return new_assembly;
    // get the list of requested modules from ros param
    // std::vector<std::string> modules_str;
    std::string param_name{param_prefix_ + ".modules"};
    auto modules_str = node_->declare_parameter<std::vector<std::string>>(
        param_name, std::vector<std::string>{});
    if (modules_str.empty()) {
      auto msg = "No field: '" + param_prefix_ + modules_field_ + "'";
      LOG(ERROR) << msg;
      throw std::runtime_error(msg);
    }

    // construct the requested modules and add them to the assembly
    for (auto module_str : modules_str) {
      auto module_ptr =
          ROSModuleFactory{node_, param_prefix_ + "." + module_str}.make();
      if (!module_ptr) throw std::runtime_error("Module creation failed.");
      // should not try to access private properties
      // new_assembly->modules_.push_back(module_ptr);
      new_assembly->add_module(module_ptr);
    }

    return new_assembly;
  }

  static constexpr auto modules_field_ = "type";
};

}  // namespace navigation
}  // namespace vtr
