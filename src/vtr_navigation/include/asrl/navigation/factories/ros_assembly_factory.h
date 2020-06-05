#pragma once

#include <ros/ros.h>

#include <asrl/navigation/factories/assembly_factory.h>
#include <asrl/navigation/factories/ros_base_factory.h>
#include <asrl/navigation/factories/ros_module_factory.h>

namespace asrl {
namespace navigation {

/** \brief makes an assembly (and inner modules) based on ros params
 */
class ROSAssemblyFactory : public ROSBaseFactory<BaseAssembly> {
 public:
  using base_t = ROSBaseFactory<BaseAssembly>;
  using assy_ptr = base_t::T_ptr;

  /** \brief constructor with ros info detailing the assembly
   */
  ROSAssemblyFactory(nh_t* nh, const std::string& param_prefix)
      : base_t(nh, param_prefix) {}

 private:
  static constexpr auto modules_field_ = "type";

  /** \brief builder that constructs an assembly, which is a collection of
   * modules
   */
  assy_ptr make_str(const std::string& type_str) const {
    // make an empty assembly using the default builder
    auto new_assembly = AssemblyFactory(type_str).make();
    if (!new_assembly) return new_assembly;

    // get the list of requested modules from ros param
    std::vector<std::string> modules_str;
    if (!nh_->getParam(param_prefix_ + "modules", modules_str)) {
      auto msg = "no field: '" + param_prefix_ + modules_field_ + "'";
      LOG(ERROR) << msg;
      throw std::runtime_error(msg);
    }

    // construct the requested modules and add them to the assembly
    for (auto i_module : modules_str) {
      auto module_ptr =
          ROSModuleFactory(nh_, param_prefix_ + "/" + i_module).make();
      if (!module_ptr) {
        throw std::runtime_error("");
      }
      // should not try to access private properties
      // new_assembly->modules_.push_back(module_ptr);
      new_assembly->add_module(module_ptr);
    }

    return new_assembly;
  }
};

}  // namespace navigation
}  // namespace asrl
