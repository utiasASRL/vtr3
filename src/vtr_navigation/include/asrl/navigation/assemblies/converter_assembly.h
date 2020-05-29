#pragma once

#include <asrl/navigation/assemblies/base_assembly.h>

namespace asrl {
namespace navigation {

/** \brief Run on every new pair of caches pushed through the pipeline.
 * Pre-processes every image by converting colour spaces and extracting
 * features.
 */
class ConverterAssembly : public BaseAssembly {
 public:
  static constexpr auto type_str_ = "converter";

  /** \brief Call after creating the converter assembly.
   *
   *  \todo Should not this be included in the constructor?
   */
  bool verify() const {
    /// \todo Make sure that there are modules.
    return true;
  }

 private:
};

}  // namespace navigation
}  // namespace asrl
