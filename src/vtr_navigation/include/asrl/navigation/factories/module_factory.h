#pragma once

#include <memory>

#include <asrl/navigation/factories/base_factory.h>
#include <asrl/navigation/modules/base_module.h>

namespace asrl {
namespace navigation {

/** \brief constructs a module based on a type_str trait
 */
class ModuleFactory : public BaseFactory<BaseModule> {
 public:
  using base_t = BaseFactory<BaseModule>;
  using mod_t = BaseModule;
  using mod_ptr = std::shared_ptr<mod_t>;

  /** \brief constructed to build a particular module
   * \param[in] type_str the trait of the derived module that should be made
   */
  ModuleFactory(const std::string& type_str) : type_str_(type_str) {}

 private:
  /** \brief makes the requested module matching the type_str trait
   * \return a base module pointer to the derived class, nullptr if not found
   * \throws invalid_argument if the derived module couldn't be found
   */
  mod_ptr make() const;

  /** \brief the requested module type_str trait
   */
  const std::string type_str_;

  friend class ROSModuleFactory;
};

}  // namespace navigation
}  // namespace asrl
