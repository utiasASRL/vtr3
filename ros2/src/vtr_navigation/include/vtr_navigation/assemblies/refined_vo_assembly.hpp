#pragma once

#include <vtr_navigation/assemblies/base_assembly.hpp>

namespace vtr {
namespace navigation {

class RefinedVoAssembly : public BaseAssembly {
 public:
  /** \brief An unique identifier for creating this assembly. */
  static constexpr auto type_str_ = "refined_vo";

  RefinedVoAssembly() : BaseAssembly{type_str_} {}

  bool verify() const { return true; }

 protected:
 private:
};

}  // namespace navigation
}  // namespace vtr
