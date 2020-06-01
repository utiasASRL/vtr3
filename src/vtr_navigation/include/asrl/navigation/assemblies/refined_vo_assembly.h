#pragma once

#include <asrl/navigation/assemblies/base_assembly.h>

namespace asrl {
namespace navigation {

class RefinedVoAssembly : public BaseAssembly {
 public:
  static constexpr auto type_str_ = "refined_vo";

  bool verify() const { return true; }

 protected:
 private:
};

}  // namespace navigation
}  // namespace asrl
