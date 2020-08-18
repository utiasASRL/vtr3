#pragma once

#include <asrl/common/logging.hpp>
#include <memory>
#include <stdexcept>

#include <vtr/navigation/assemblies/base_assembly.h>
#include <vtr/navigation/factories/base_factory.h>

namespace vtr {
namespace navigation {

/** \brief uses the default constructor to build an assembly by string typename
 */
class AssemblyFactory : BaseFactory<BaseAssembly> {
 public:
  using base_t = BaseFactory<BaseAssembly>;
  using assy_t = BaseAssembly;
  using assy_ptr = std::shared_ptr<assy_t>;

  /** \brief requires the string typename of the assembly being built
   *
   * \param[in] type_str the trait of the derived module that should be made
   */
  AssemblyFactory(const std::string& type_str) : type_str_(type_str) {}

 private:
  /** \brief the requested assembly type_str trait
   */
  const std::string type_str_;

  /** \brief creates the requested assembly based on the string typename
   *
   * \return base assembly pointer, or nullptr on failure
   * \throws runtime_error if the assembly typename is not known
   */
  assy_ptr make() const;

  friend class ROSAssemblyFactory;
};

}  // namespace navigation
}  // namespace vtr
