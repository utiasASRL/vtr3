#pragma once

#include <vtr/navigation/assemblies/base_assembly.h>

namespace asrl {
namespace navigation {

class RefinedVoAssembly : public BaseAssembly {
 public:
  /** \brief An unique identifier for creating this assembly.
   */
  static constexpr auto type_str_ = "refined_vo";

  RefinedVoAssembly() : BaseAssembly{type_str_} {}

  bool verify() const { return true; }

 protected:
 private:
};

}  // namespace navigation
}  // namespace asrl
